package frc.robot.subsystems.shooting;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;

/**
 * Interpolating lookup table for shot parameters. Maps distance (meters) to shot parameters (RPM,
 * hood angle, TOF, motivator RPM, spindexer RPM). Uses linear interpolation between entries.
 *
 * <p>Usage: populate with empirically measured shots at known distances, then query for any
 * distance. Entries outside the table range clamp to the nearest entry.
 *
 * <p>This provides a "ground-truth" alternative (or blend partner) to the physics-based
 * TrajectoryOptimizer, especially useful for shoot-on-the-move where small errors compound.
 */
public class ShotLookupTable {

  /**
   * A single entry in the lookup table: all shot parameters at a known distance. TOF is critical
   * for accurate velocity compensation (captures real-world drag that physics ignores).
   */
  public record ShotEntry(
      double rpm,
      double hoodAngleDeg,
      double timeOfFlightTheoreticalS,
      double timeOfFlightMeasuredS,
      double motivatorRPM,
      double spindexerRPM) {

    /** Returns measured TOF if available (> 0), otherwise theoretical. */
    public double getEffectiveTOF() {
      return timeOfFlightMeasuredS > 0 ? timeOfFlightMeasuredS : timeOfFlightTheoreticalS;
    }

    /** Compatibility alias — all existing callers automatically prefer measured TOF. */
    public double timeOfFlightS() {
      return getEffectiveTOF();
    }
  }

  private final TreeMap<Double, ShotEntry> table = new TreeMap<>();

  /** Create an empty lookup table. Use {@link #addEntry} to populate. */
  public ShotLookupTable() {}

  /**
   * Add a measured shot entry at a known distance (full parameters).
   *
   * @param distanceMeters Horizontal distance to the target in meters
   * @param rpm Actual launcher RPM that worked at this distance
   * @param hoodAngleDeg Hood angle that worked at this distance
   * @param timeOfFlightTheoreticalS Theoretical (physics-based) time of flight in seconds
   * @param timeOfFlightMeasuredS Measured time of flight in seconds (0.0 if not measured)
   * @param motivatorRPM Motivator RPM used for this shot
   * @param spindexerRPM Spindexer RPM used for this shot
   * @return this (for chaining)
   */
  public ShotLookupTable addEntry(
      double distanceMeters,
      double rpm,
      double hoodAngleDeg,
      double timeOfFlightTheoreticalS,
      double timeOfFlightMeasuredS,
      double motivatorRPM,
      double spindexerRPM) {
    table.put(
        distanceMeters,
        new ShotEntry(
            rpm,
            hoodAngleDeg,
            timeOfFlightTheoreticalS,
            timeOfFlightMeasuredS,
            motivatorRPM,
            spindexerRPM));
    return this;
  }

  /**
   * Add a measured shot entry (RPM + hood angle only, with default feed speeds and estimated TOF).
   *
   * @param distanceMeters Horizontal distance to the target in meters
   * @param rpm Launcher RPM that worked at this distance
   * @param hoodAngleDeg Hood angle that worked at this distance
   * @return this (for chaining)
   */
  public ShotLookupTable addEntry(double distanceMeters, double rpm, double hoodAngleDeg) {
    // Estimate TOF from distance assuming ~10 m/s horizontal velocity
    double estimatedTOF = distanceMeters / 10.0;
    return addEntry(distanceMeters, rpm, hoodAngleDeg, estimatedTOF, 0.0, 1800.0, 325.0);
  }

  /**
   * Look up interpolated shot parameters for a given distance.
   *
   * @param distanceMeters Horizontal distance to the target
   * @return Interpolated ShotEntry, or null if table is empty
   */
  public ShotEntry lookup(double distanceMeters) {
    if (table.isEmpty()) return null;

    // Exact match
    ShotEntry exact = table.get(distanceMeters);
    if (exact != null) return exact;

    Map.Entry<Double, ShotEntry> floor = table.floorEntry(distanceMeters);
    Map.Entry<Double, ShotEntry> ceil = table.ceilingEntry(distanceMeters);

    // Clamp to nearest if outside range
    if (floor == null) return ceil.getValue();
    if (ceil == null) return floor.getValue();

    // Linear interpolation
    double t = (distanceMeters - floor.getKey()) / (ceil.getKey() - floor.getKey());
    ShotEntry lo = floor.getValue();
    ShotEntry hi = ceil.getValue();

    // Interpolate theoretical TOF normally
    double theoreticalTOF = lerp(lo.timeOfFlightTheoreticalS(), hi.timeOfFlightTheoreticalS(), t);

    // Only interpolate measured TOF if both neighbors have measured values
    double measuredTOF = 0.0;
    if (lo.timeOfFlightMeasuredS() > 0 && hi.timeOfFlightMeasuredS() > 0) {
      measuredTOF = lerp(lo.timeOfFlightMeasuredS(), hi.timeOfFlightMeasuredS(), t);
    }

    return new ShotEntry(
        lerp(lo.rpm(), hi.rpm(), t),
        lerp(lo.hoodAngleDeg(), hi.hoodAngleDeg(), t),
        theoreticalTOF,
        measuredTOF,
        lerp(lo.motivatorRPM(), hi.motivatorRPM(), t),
        lerp(lo.spindexerRPM(), hi.spindexerRPM(), t));
  }

  /**
   * Look up only the time of flight for a given distance. Used by the Hybrid strategy for velocity
   * compensation with real-world drag.
   *
   * @param distanceMeters Horizontal distance to the target
   * @return Interpolated TOF in seconds, or -1 if table is empty
   */
  public double lookupTOF(double distanceMeters) {
    ShotEntry entry = lookup(distanceMeters);
    return entry != null ? entry.timeOfFlightS() : -1.0;
  }

  /**
   * @return true if the table has at least 2 entries (minimum for interpolation)
   */
  public boolean hasEnoughData() {
    return table.size() >= 2;
  }

  /**
   * Check if a distance falls within the range of empirical data. Returns false if outside the
   * min/max recorded distances, indicating the LUT would clamp rather than interpolate.
   *
   * @param distanceMeters Distance to check
   * @return true if the distance is within the LUT's data range
   */
  public boolean isInRange(double distanceMeters) {
    if (table.size() < 2) return false;
    return distanceMeters >= table.firstKey() && distanceMeters <= table.lastKey();
  }

  /** Get the minimum distance in the table, or -1 if empty. */
  public double getMinDistance() {
    return table.isEmpty() ? -1.0 : table.firstKey();
  }

  /** Get the maximum distance in the table, or -1 if empty. */
  public double getMaxDistance() {
    return table.isEmpty() ? -1.0 : table.lastKey();
  }

  /**
   * @return true if the table has at least one entry
   */
  public boolean hasEntries() {
    return !table.isEmpty();
  }

  /**
   * @return the number of entries in the table
   */
  public int size() {
    return table.size();
  }

  /** Clear all entries from the table. */
  public void clear() {
    table.clear();
  }

  /**
   * Load data from a list of clean LUT entries. Replaces any existing entries.
   *
   * @param entries LUT entries from {@link StationaryShotBatchRecorder}
   */
  public void loadFromLUTEntries(List<StationaryShotBatchRecorder.LUTEntry> entries) {
    table.clear();
    for (StationaryShotBatchRecorder.LUTEntry e : entries) {
      addEntry(
          e.distanceM(),
          e.rpm(),
          e.hoodAngleDeg(),
          e.timeOfFlightTheoreticalS(),
          e.timeOfFlightMeasuredS(),
          e.motivatorRPM(),
          e.spindexerRPM());
    }
  }

  private static double lerp(double a, double b, double t) {
    return a + t * (b - a);
  }

  /**
   * Add entries from a list of LUT entries without clearing the table first. Empirical entries at
   * the same distance will overwrite any existing (e.g. parametric) entry.
   *
   * @param entries LUT entries from {@link StationaryShotBatchRecorder}
   */
  public void addFromLUTEntries(List<StationaryShotBatchRecorder.LUTEntry> entries) {
    for (StationaryShotBatchRecorder.LUTEntry e : entries) {
      addEntry(
          e.distanceM(),
          e.rpm(),
          e.hoodAngleDeg(),
          e.timeOfFlightTheoreticalS(),
          e.timeOfFlightMeasuredS(),
          e.motivatorRPM(),
          e.spindexerRPM());
    }
  }


  /**
   * Log the full table contents to AdvantageKit as parallel arrays. Call after any table
   * modification (reload, seed, add) to keep the dashboard view in sync.
   *
   * @param prefix AdvantageKit key prefix (e.g. "LUTDev/Table")
   */
  public void logTable(String prefix) {
    int n = table.size();
    double[] distances = new double[n];
    double[] rpms = new double[n];
    double[] hoodAngles = new double[n];
    double[] theoreticalTOFs = new double[n];
    double[] measuredTOFs = new double[n];
    double[] effectiveTOFs = new double[n];
    double[] motivatorRPMs = new double[n];
    double[] spindexerRPMs = new double[n];

    int i = 0;
    for (Map.Entry<Double, ShotEntry> e : table.entrySet()) {
      distances[i] = e.getKey();
      ShotEntry v = e.getValue();
      rpms[i] = v.rpm();
      hoodAngles[i] = v.hoodAngleDeg();
      theoreticalTOFs[i] = v.timeOfFlightTheoreticalS();
      measuredTOFs[i] = v.timeOfFlightMeasuredS();
      effectiveTOFs[i] = v.getEffectiveTOF();
      motivatorRPMs[i] = v.motivatorRPM();
      spindexerRPMs[i] = v.spindexerRPM();
      i++;
    }

    Logger.recordOutput(prefix + "/Distances", distances);
    Logger.recordOutput(prefix + "/RPMs", rpms);
    Logger.recordOutput(prefix + "/HoodAngles", hoodAngles);
    Logger.recordOutput(prefix + "/TheoreticalTOFs", theoreticalTOFs);
    Logger.recordOutput(prefix + "/MeasuredTOFs", measuredTOFs);
    Logger.recordOutput(prefix + "/EffectiveTOFs", effectiveTOFs);
    Logger.recordOutput(prefix + "/MotivatorRPMs", motivatorRPMs);
    Logger.recordOutput(prefix + "/SpindexerRPMs", spindexerRPMs);
    Logger.recordOutput(prefix + "/EntryCount", n);
  }

}
