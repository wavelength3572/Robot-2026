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
import java.util.stream.Collectors;

/**
 * Records and persists shot data points to a JSON file on the roboRIO filesystem. Data survives
 * robot reboots. All file I/O is wrapped in try-catch to never crash the robot.
 *
 * <p>File format is a simple JSON array of objects with named fields — no external JSON library
 * needed.
 */
public class ShotDataRecorder {

  private static final String DATA_DIR = "shot-data";
  private static final String DATA_FILE = "lut-data.json";

  private final List<ShotDataPoint> dataPoints = new ArrayList<>();
  private final File dataFile;

  public ShotDataRecorder() {
    File dir = new File(Filesystem.getOperatingDirectory(), DATA_DIR);
    if (!dir.exists()) {
      dir.mkdirs();
    }
    this.dataFile = new File(dir, DATA_FILE);
    loadFromFile();
  }

  /** Record a new shot data point and persist to disk. */
  public void recordShot(ShotDataPoint point) {
    dataPoints.add(point);
    saveToFile();
  }

  /** Get all data points that were marked as successful. */
  public List<ShotDataPoint> getSuccessfulShots() {
    return dataPoints.stream().filter(ShotDataPoint::successful).collect(Collectors.toList());
  }

  /** Get all recorded data points. */
  public List<ShotDataPoint> getAllShots() {
    return new ArrayList<>(dataPoints);
  }

  /** Get the number of recorded data points. */
  public int getCount() {
    return dataPoints.size();
  }

  /** Get the number of successful data points. */
  public int getSuccessCount() {
    return (int) dataPoints.stream().filter(ShotDataPoint::successful).count();
  }

  /** Clear all data and delete the file. */
  public void clearData() {
    dataPoints.clear();
    try {
      if (dataFile.exists()) {
        dataFile.delete();
      }
    } catch (Exception e) {
      System.err.println("[ShotDataRecorder] Failed to delete file: " + e.getMessage());
    }
  }

  /**
   * Get a human-readable summary of all recorded shots as a string array. Each entry is one line
   * like: "3.45m | RPM=2650 | Hood=24.3° | OK" — viewable in AdvantageKit as a string array.
   */
  public String[] getShotSummaries() {
    String[] summaries = new String[dataPoints.size()];
    for (int i = 0; i < dataPoints.size(); i++) {
      ShotDataPoint p = dataPoints.get(i);
      summaries[i] =
          String.format(
              "#%d  %.2fm | tgtRPM=%.0f (act=%.0f) | Hood=%.1f° | Mot=%.0f | Spin=%.0f | %s",
              i + 1,
              p.distanceM(),
              p.targetRPM(),
              p.actualRPM(),
              p.targetHoodAngleDeg(),
              p.targetMotivatorRPM(),
              p.targetSpindexerRPM(),
              p.successful() ? "HIT" : "MISS");
    }
    return summaries;
  }

  /** Get the last recorded distance, or 0 if no data. */
  public double getLastRecordedDistance() {
    if (dataPoints.isEmpty()) return 0.0;
    return dataPoints.get(dataPoints.size() - 1).distanceM();
  }

  // ========== JSON Persistence (manual, no library) ==========

  private void saveToFile() {
    try (PrintWriter writer = new PrintWriter(new FileWriter(dataFile))) {
      writer.println("[");
      for (int i = 0; i < dataPoints.size(); i++) {
        ShotDataPoint p = dataPoints.get(i);
        writer.print("  {");
        writer.print("\"timestamp\":" + p.timestamp());
        writer.print(",\"robotX\":" + p.robotX());
        writer.print(",\"robotY\":" + p.robotY());
        writer.print(",\"distanceM\":" + p.distanceM());
        // Target/commanded values (used by LUT)
        writer.print(",\"targetRPM\":" + p.targetRPM());
        writer.print(",\"targetHoodAngleDeg\":" + p.targetHoodAngleDeg());
        writer.print(",\"turretAngleDeg\":" + p.turretAngleDeg());
        writer.print(",\"targetMotivatorRPM\":" + p.targetMotivatorRPM());
        writer.print(",\"targetSpindexerRPM\":" + p.targetSpindexerRPM());
        // Actual/measured values (diagnostic)
        writer.print(",\"actualRPM\":" + p.actualRPM());
        writer.print(",\"actualHoodAngleDeg\":" + p.actualHoodAngleDeg());
        writer.print(",\"actualMotivatorRPM\":" + p.actualMotivatorRPM());
        writer.print(",\"actualSpindexerRPM\":" + p.actualSpindexerRPM());
        // Derived values
        writer.print(",\"exitVelocityMps\":" + p.exitVelocityMps());
        writer.print(",\"timeOfFlightS\":" + p.timeOfFlightS());
        writer.print(",\"successful\":" + p.successful());
        writer.print("}");
        if (i < dataPoints.size() - 1) writer.print(",");
        writer.println();
      }
      writer.println("]");
    } catch (IOException e) {
      System.err.println("[ShotDataRecorder] Failed to save: " + e.getMessage());
    }
  }

  private void loadFromFile() {
    if (!dataFile.exists()) return;
    try (BufferedReader reader = new BufferedReader(new FileReader(dataFile))) {
      StringBuilder sb = new StringBuilder();
      String line;
      while ((line = reader.readLine()) != null) {
        sb.append(line);
      }
      parseJson(sb.toString());
      System.out.println("[ShotDataRecorder] Loaded " + dataPoints.size() + " data points");
    } catch (Exception e) {
      System.err.println("[ShotDataRecorder] Failed to load: " + e.getMessage());
    }
  }

  private void parseJson(String json) {
    // Simple parser: split on "},{" to find objects, then extract fields
    json = json.trim();
    if (json.startsWith("[")) json = json.substring(1);
    if (json.endsWith("]")) json = json.substring(0, json.length() - 1);
    json = json.trim();
    if (json.isEmpty()) return;

    // Split objects — handle the fact that first/last won't have leading/trailing brace
    String[] objects = json.split("\\},\\s*\\{");
    for (String obj : objects) {
      obj = obj.trim();
      if (obj.startsWith("{")) obj = obj.substring(1);
      if (obj.endsWith("}")) obj = obj.substring(0, obj.length() - 1);
      try {
        ShotDataPoint point = parseObject(obj);
        if (point != null) {
          dataPoints.add(point);
        }
      } catch (Exception e) {
        System.err.println("[ShotDataRecorder] Skipping malformed entry: " + e.getMessage());
      }
    }
  }

  private ShotDataPoint parseObject(String obj) {
    double timestamp = getDouble(obj, "timestamp");
    double robotX = getDouble(obj, "robotX");
    double robotY = getDouble(obj, "robotY");
    double distanceM = getDouble(obj, "distanceM");
    // Target/commanded
    double targetRPM = getDouble(obj, "targetRPM");
    double targetHoodAngleDeg = getDouble(obj, "targetHoodAngleDeg");
    double turretAngleDeg = getDouble(obj, "turretAngleDeg");
    double targetMotivatorRPM = getDouble(obj, "targetMotivatorRPM");
    double targetSpindexerRPM = getDouble(obj, "targetSpindexerRPM");
    // Actual/measured
    double actualRPM = getDouble(obj, "actualRPM");
    double actualHoodAngleDeg = getDouble(obj, "actualHoodAngleDeg");
    double actualMotivatorRPM = getDouble(obj, "actualMotivatorRPM");
    double actualSpindexerRPM = getDouble(obj, "actualSpindexerRPM");
    // Derived
    double exitVelocityMps = getDouble(obj, "exitVelocityMps");
    double timeOfFlightS = getDouble(obj, "timeOfFlightS");
    boolean successful = getBoolean(obj, "successful");

    return new ShotDataPoint(
        timestamp,
        robotX,
        robotY,
        distanceM,
        targetRPM,
        targetHoodAngleDeg,
        turretAngleDeg,
        targetMotivatorRPM,
        targetSpindexerRPM,
        actualRPM,
        actualHoodAngleDeg,
        actualMotivatorRPM,
        actualSpindexerRPM,
        exitVelocityMps,
        timeOfFlightS,
        successful);
  }

  private static double getDouble(String obj, String key) {
    String pattern = "\"" + key + "\":";
    int idx = obj.indexOf(pattern);
    if (idx < 0) return 0.0;
    int start = idx + pattern.length();
    int end = obj.indexOf(',', start);
    if (end < 0) end = obj.indexOf('}', start);
    if (end < 0) end = obj.length();
    try {
      return Double.parseDouble(obj.substring(start, end).trim());
    } catch (NumberFormatException e) {
      return 0.0;
    }
  }

  private static boolean getBoolean(String obj, String key) {
    String pattern = "\"" + key + "\":";
    int idx = obj.indexOf(pattern);
    if (idx < 0) return false;
    int start = idx + pattern.length();
    int end = obj.indexOf(',', start);
    if (end < 0) end = obj.indexOf('}', start);
    if (end < 0) end = obj.length();
    return Boolean.parseBoolean(obj.substring(start, end).trim());
  }
}
