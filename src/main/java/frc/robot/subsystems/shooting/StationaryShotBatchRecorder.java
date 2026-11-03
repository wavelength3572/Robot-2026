package frc.robot.subsystems.shooting;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

/**
 * Records LUT data collection sessions. Writes two files:
 *
 * <ul>
 *   <li><b>lut-data.json</b> — Clean LUT entries: distance, RPM, hood angle, TOF. Only successful
 *       batches. Review these after practice and promote good values to {@link
 *       ShotTableConstants#BASELINE_TABLE}.
 *   <li><b>shot-log.json</b> — Full batch records for analysis: timestamp, position, params, fuel
 *       count, success/fail. All batches included.
 * </ul>
 *
 * <p>A "batch" is one data collection attempt: drive to a spot, fire a hopper of fuel, then mark
 * success or miss. The batch captures the shot parameters once (they're constant for a position)
 * and counts how many fuel were fired.
 */
public class StationaryShotBatchRecorder {

  private static final String DATA_DIR = "shot-data";
  private static final String LUT_FILE = "lut-data.json";
  private static final String LOG_FILE = "shot-log.json";

  /** Clean LUT entry — only the fields the lookup table needs. */
  public record LUTEntry(
      double distanceM,
      double rpm,
      double hoodAngleDeg,
      double timeOfFlightTheoreticalS,
      double timeOfFlightMeasuredS,
      double timestamp) {

    /** Returns measured TOF if available (> 0), otherwise theoretical. */
    public double getEffectiveTOF() {
      return timeOfFlightMeasuredS > 0 ? timeOfFlightMeasuredS : timeOfFlightTheoreticalS;
    }
  }

  /** Full batch record for analysis — includes context and diagnostics. */
  public record BatchRecord(
      double timestamp,
      double robotX,
      double robotY,
      double distanceM,
      double rpm,
      double hoodAngleDeg,
      double turretAngleDeg,
      double timeOfFlightTheoreticalS,
      double timeOfFlightMeasuredS,
      int fuelFired,
      boolean successful) {}

  private final List<LUTEntry> lutEntries = new ArrayList<>();
  private final List<BatchRecord> batchRecords = new ArrayList<>();
  private final File lutFile;
  private final File logFile;

  // Batch tracking state
  private int fuelCountAtBatchStart = -1;
  private boolean batchActive = false;

  // Cached shot parameters — updated during firing so they survive SmartLaunch cleanup
  private double cachedRPM = 0.0;
  private double cachedHoodAngleDeg = 0.0;

  public StationaryShotBatchRecorder() {
    File dir = new File(Filesystem.getOperatingDirectory(), DATA_DIR);
    if (!dir.exists()) {
      dir.mkdirs();
    }
    this.lutFile = new File(dir, LUT_FILE);
    this.logFile = new File(dir, LOG_FILE);
    // Don't reload previous session data — each session starts fresh.
    // Files are still written to disk for offline review / manual promotion to ShotTableConstants.
  }

  // ========== Batch Tracking ==========

  /**
   * Call when a shooting sequence starts (smart launch begins). Captures the fuel count so we can
   * calculate how many were fired when the batch ends.
   */
  /**
   * Cache the current shot parameters during firing. Call this every cycle while SmartLaunch is
   * active so that the values survive after the command ends.
   */
  public void cacheParams(double rpm, double hoodAngleDeg) {
    this.cachedRPM = rpm;
    this.cachedHoodAngleDeg = hoodAngleDeg;
  }

  public double getCachedRPM() {
    return cachedRPM;
  }

  public double getCachedHoodAngleDeg() {
    return cachedHoodAngleDeg;
  }

  public void startBatch(int currentFuelCount) {
    fuelCountAtBatchStart = currentFuelCount;
    batchActive = true;
  }

  /**
   * Call when the operator marks the batch as success or miss. Records both a LUT entry (if
   * successful) and a batch log entry (always).
   *
   * @param currentFuelCount Current fuel count (after firing)
   * @return The number of fuel fired in this batch, or 0 if no batch was active
   */
  public int endBatch(
      double timestamp,
      double robotX,
      double robotY,
      double distanceM,
      double rpm,
      double hoodAngleDeg,
      double turretAngleDeg,
      double timeOfFlightTheoreticalS,
      double timeOfFlightMeasuredS,
      int currentFuelCount,
      boolean successful) {

    int fuelFired = 0;
    if (batchActive && fuelCountAtBatchStart >= 0) {
      fuelFired = fuelCountAtBatchStart - currentFuelCount;
    }
    batchActive = false;
    fuelCountAtBatchStart = -1;

    // Always record to the batch log
    BatchRecord record =
        new BatchRecord(
            timestamp,
            robotX,
            robotY,
            distanceM,
            rpm,
            hoodAngleDeg,
            turretAngleDeg,
            timeOfFlightTheoreticalS,
            timeOfFlightMeasuredS,
            fuelFired,
            successful);
    batchRecords.add(record);
    saveLogToFile();

    // Only add to LUT if successful
    if (successful) {
      lutEntries.add(
          new LUTEntry(
              distanceM,
              rpm,
              hoodAngleDeg,
              timeOfFlightTheoreticalS,
              timeOfFlightMeasuredS,
              timestamp));
      saveLUTToFile();
    }

    return fuelFired;
  }

  /** Whether a batch is currently in progress (started but not yet marked). */
  public boolean isBatchActive() {
    return batchActive;
  }

  // ========== Accessors ==========

  public List<LUTEntry> getLUTEntries() {
    return new ArrayList<>(lutEntries);
  }

  public List<BatchRecord> getBatchRecords() {
    return new ArrayList<>(batchRecords);
  }

  public int getLUTEntryCount() {
    return lutEntries.size();
  }

  public int getBatchCount() {
    return batchRecords.size();
  }

  public int getSuccessCount() {
    return (int) batchRecords.stream().filter(BatchRecord::successful).count();
  }

  public int getMissCount() {
    return (int) batchRecords.stream().filter(b -> !b.successful()).count();
  }

  /**
   * Get human-readable summaries of all LUT entries (successful recordings only).
   *
   * @return Array of summary strings, sorted by distance
   */
  public String[] getLUTSummaries() {
    // Build index-tagged list so user can identify entries for removal
    List<int[]> indexMap = new ArrayList<>();
    for (int i = 0; i < lutEntries.size(); i++) {
      indexMap.add(new int[] {i});
    }
    // Sort by distance but keep original index
    indexMap.sort(
        (a, b) ->
            Double.compare(lutEntries.get(a[0]).distanceM(), lutEntries.get(b[0]).distanceM()));

    String[] summaries = new String[indexMap.size()];
    for (int i = 0; i < indexMap.size(); i++) {
      int origIdx = indexMap.get(i)[0];
      LUTEntry e = lutEntries.get(origIdx);
      summaries[i] =
          String.format(
              "[%d] %.2fm | RPM=%.0f | Hood=%.1f° | TOF=%.3fs",
              origIdx, e.distanceM(), e.rpm(), e.hoodAngleDeg(), e.getEffectiveTOF());
    }
    return summaries;
  }

  /**
   * Get human-readable summaries of all batches for AdvantageKit display.
   *
   * @return Array of summary strings
   */
  public String[] getBatchSummaries() {
    String[] summaries = new String[batchRecords.size()];
    for (int i = 0; i < batchRecords.size(); i++) {
      BatchRecord b = batchRecords.get(i);
      summaries[i] =
          String.format(
              "#%d  %.2fm | RPM=%.0f | Hood=%.1f° | %d fuel | %s",
              i + 1,
              b.distanceM(),
              b.rpm(),
              b.hoodAngleDeg(),
              b.fuelFired(),
              b.successful() ? "HIT" : "MISS");
    }
    return summaries;
  }

  /**
   * Remove the last LUT entry and re-save. Returns a description of the removed entry, or null if
   * empty.
   */
  public String removeLastLUTEntry() {
    if (lutEntries.isEmpty()) {
      return null;
    }
    LUTEntry removed = lutEntries.remove(lutEntries.size() - 1);
    saveLUTToFile();
    return String.format(
        "%.2fm | RPM=%.0f | Hood=%.1f°",
        removed.distanceM(), removed.rpm(), removed.hoodAngleDeg());
  }

  /**
   * Remove a LUT entry by index. Returns a description of the removed entry, or null if invalid.
   */
  public String removeLUTEntry(int index) {
    if (index < 0 || index >= lutEntries.size()) {
      return null;
    }
    LUTEntry removed = lutEntries.remove(index);
    saveLUTToFile();
    return String.format(
        "[%d] %.2fm | RPM=%.0f | Hood=%.1f°",
        index, removed.distanceM(), removed.rpm(), removed.hoodAngleDeg());
  }

  /** Clear all data and delete both files. */
  public void clearAll() {
    lutEntries.clear();
    batchRecords.clear();
    deleteFile(lutFile);
    deleteFile(logFile);
  }

  /** Clear only LUT entries (keeps the batch log for historical analysis). */
  public void clearLUT() {
    lutEntries.clear();
    deleteFile(lutFile);
  }

  // ========== LUT File I/O ==========

  private void saveLUTToFile() {
    try (PrintWriter w = new PrintWriter(new FileWriter(lutFile))) {
      w.println("[");
      for (int i = 0; i < lutEntries.size(); i++) {
        LUTEntry e = lutEntries.get(i);
        w.print("  {");
        w.print("\"distanceM\":" + e.distanceM());
        w.print(",\"rpm\":" + e.rpm());
        w.print(",\"hoodAngleDeg\":" + e.hoodAngleDeg());
        w.print(",\"timeOfFlightTheoreticalS\":" + e.timeOfFlightTheoreticalS());
        w.print(",\"timeOfFlightMeasuredS\":" + e.timeOfFlightMeasuredS());
        w.print(",\"timestamp\":" + e.timestamp());
        w.print("}");
        if (i < lutEntries.size() - 1) w.print(",");
        w.println();
      }
      w.println("]");
    } catch (IOException e) {
      System.err.println("[StationaryShotBatchRecorder] Failed to save LUT: " + e.getMessage());
    }
  }

  private void loadLUTFromFile() {
    if (!lutFile.exists()) return;
    try (BufferedReader reader = new BufferedReader(new FileReader(lutFile))) {
      StringBuilder sb = new StringBuilder();
      String line;
      while ((line = reader.readLine()) != null) sb.append(line);
      parseLUTJson(sb.toString());
      frc.robot.util.StartupLogger.log(
          "[StationaryShotBatchRecorder] Loaded " + lutEntries.size() + " LUT entries");
    } catch (Exception e) {
      System.err.println("[StationaryShotBatchRecorder] Failed to load LUT: " + e.getMessage());
    }
  }

  private void parseLUTJson(String json) {
    json = stripArray(json);
    if (json.isEmpty()) return;
    for (String obj : splitObjects(json)) {
      try {
        // Try new field names first, fall back to old "timeOfFlightS" for backward compat
        double theoreticalTOF = getDouble(obj, "timeOfFlightTheoreticalS");
        double measuredTOF = getDouble(obj, "timeOfFlightMeasuredS");
        if (theoreticalTOF == 0.0) {
          // Old format: single "timeOfFlightS" field → treat as theoretical
          theoreticalTOF = getDouble(obj, "timeOfFlightS");
        }
        lutEntries.add(
            new LUTEntry(
                getDouble(obj, "distanceM"),
                getDouble(obj, "rpm"),
                getDouble(obj, "hoodAngleDeg"),
                theoreticalTOF,
                measuredTOF,
                getDouble(obj, "timestamp")));
      } catch (Exception e) {
        System.err.println(
            "[StationaryShotBatchRecorder] Skipping malformed LUT entry: " + e.getMessage());
      }
    }
  }

  // ========== Log File I/O ==========

  private void saveLogToFile() {
    try (PrintWriter w = new PrintWriter(new FileWriter(logFile))) {
      w.println("[");
      for (int i = 0; i < batchRecords.size(); i++) {
        BatchRecord b = batchRecords.get(i);
        w.print("  {");
        w.print("\"timestamp\":" + b.timestamp());
        w.print(",\"robotX\":" + b.robotX());
        w.print(",\"robotY\":" + b.robotY());
        w.print(",\"distanceM\":" + b.distanceM());
        w.print(",\"rpm\":" + b.rpm());
        w.print(",\"hoodAngleDeg\":" + b.hoodAngleDeg());
        w.print(",\"turretAngleDeg\":" + b.turretAngleDeg());
        w.print(",\"timeOfFlightTheoreticalS\":" + b.timeOfFlightTheoreticalS());
        w.print(",\"timeOfFlightMeasuredS\":" + b.timeOfFlightMeasuredS());
        w.print(",\"fuelFired\":" + b.fuelFired());
        w.print(",\"successful\":" + b.successful());
        w.print("}");
        if (i < batchRecords.size() - 1) w.print(",");
        w.println();
      }
      w.println("]");
    } catch (IOException e) {
      System.err.println("[StationaryShotBatchRecorder] Failed to save log: " + e.getMessage());
    }
  }

  private void loadLogFromFile() {
    if (!logFile.exists()) return;
    try (BufferedReader reader = new BufferedReader(new FileReader(logFile))) {
      StringBuilder sb = new StringBuilder();
      String line;
      while ((line = reader.readLine()) != null) sb.append(line);
      parseLogJson(sb.toString());
      frc.robot.util.StartupLogger.log(
          "[StationaryShotBatchRecorder] Loaded " + batchRecords.size() + " batch records");
    } catch (Exception e) {
      System.err.println("[StationaryShotBatchRecorder] Failed to load log: " + e.getMessage());
    }
  }

  private void parseLogJson(String json) {
    json = stripArray(json);
    if (json.isEmpty()) return;
    for (String obj : splitObjects(json)) {
      try {
        // Try new field names first, fall back to old "timeOfFlightS" for backward compat
        double theoreticalTOF = getDouble(obj, "timeOfFlightTheoreticalS");
        double measuredTOF = getDouble(obj, "timeOfFlightMeasuredS");
        if (theoreticalTOF == 0.0) {
          theoreticalTOF = getDouble(obj, "timeOfFlightS");
        }
        batchRecords.add(
            new BatchRecord(
                getDouble(obj, "timestamp"),
                getDouble(obj, "robotX"),
                getDouble(obj, "robotY"),
                getDouble(obj, "distanceM"),
                getDouble(obj, "rpm"),
                getDouble(obj, "hoodAngleDeg"),
                getDouble(obj, "turretAngleDeg"),
                theoreticalTOF,
                measuredTOF,
                getInt(obj, "fuelFired"),
                getBoolean(obj, "successful")));
      } catch (Exception e) {
        System.err.println(
            "[StationaryShotBatchRecorder] Skipping malformed log entry: " + e.getMessage());
      }
    }
  }

  // ========== JSON Helpers ==========

  private static String stripArray(String json) {
    json = json.trim();
    if (json.startsWith("[")) json = json.substring(1);
    if (json.endsWith("]")) json = json.substring(0, json.length() - 1);
    return json.trim();
  }

  private static String[] splitObjects(String json) {
    String[] objects = json.split("\\},\\s*\\{");
    for (int i = 0; i < objects.length; i++) {
      String obj = objects[i].trim();
      if (obj.startsWith("{")) obj = obj.substring(1);
      if (obj.endsWith("}")) obj = obj.substring(0, obj.length() - 1);
      objects[i] = obj;
    }
    return objects;
  }

  private static double getDouble(String obj, String key) {
    String pattern = "\"" + key + "\":";
    int idx = obj.indexOf(pattern);
    if (idx < 0) return 0.0;
    int start = idx + pattern.length();
    int end = findValueEnd(obj, start);
    try {
      return Double.parseDouble(obj.substring(start, end).trim());
    } catch (NumberFormatException e) {
      return 0.0;
    }
  }

  private static int getInt(String obj, String key) {
    return (int) getDouble(obj, key);
  }

  private static boolean getBoolean(String obj, String key) {
    String pattern = "\"" + key + "\":";
    int idx = obj.indexOf(pattern);
    if (idx < 0) return false;
    int start = idx + pattern.length();
    int end = findValueEnd(obj, start);
    return Boolean.parseBoolean(obj.substring(start, end).trim());
  }

  private static int findValueEnd(String obj, int start) {
    int end = obj.indexOf(',', start);
    if (end < 0) end = obj.indexOf('}', start);
    if (end < 0) end = obj.length();
    return end;
  }

  private static void deleteFile(File file) {
    try {
      if (file.exists()) file.delete();
    } catch (Exception e) {
      System.err.println(
          "[StationaryShotBatchRecorder] Failed to delete "
              + file.getName()
              + ": "
              + e.getMessage());
    }
  }
}
