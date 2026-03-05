package frc.robot.subsystems.shooting;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;

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
      double timeOfFlightS,
      double motivatorRPM,
      double spindexerRPM) {}

  private final TreeMap<Double, ShotEntry> table = new TreeMap<>();

  /** Create an empty lookup table. Use {@link #addEntry} to populate. */
  public ShotLookupTable() {}

  /**
   * Add a measured shot entry at a known distance (full parameters).
   *
   * @param distanceMeters Horizontal distance to the target in meters
   * @param rpm Actual launcher RPM that worked at this distance
   * @param hoodAngleDeg Hood angle that worked at this distance
   * @param timeOfFlightS Measured or calculated time of flight in seconds
   * @param motivatorRPM Motivator RPM used for this shot
   * @param spindexerRPM Spindexer RPM used for this shot
   * @return this (for chaining)
   */
  public ShotLookupTable addEntry(
      double distanceMeters,
      double rpm,
      double hoodAngleDeg,
      double timeOfFlightS,
      double motivatorRPM,
      double spindexerRPM) {
    table.put(distanceMeters, new ShotEntry(rpm, hoodAngleDeg, timeOfFlightS, motivatorRPM, spindexerRPM));
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
    return addEntry(distanceMeters, rpm, hoodAngleDeg, estimatedTOF, 1800.0, 325.0);
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

    return new ShotEntry(
        lerp(lo.rpm(), hi.rpm(), t),
        lerp(lo.hoodAngleDeg(), hi.hoodAngleDeg(), t),
        lerp(lo.timeOfFlightS(), hi.timeOfFlightS(), t),
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

  /** @return true if the table has at least 2 entries (minimum for interpolation) */
  public boolean hasEnoughData() {
    return table.size() >= 2;
  }

  /** @return true if the table has at least one entry */
  public boolean hasEntries() {
    return !table.isEmpty();
  }

  /** @return the number of entries in the table */
  public int size() {
    return table.size();
  }

  /** Clear all entries from the table. */
  public void clear() {
    table.clear();
  }

  /**
   * Load data from a list of successful shot data points. Replaces any existing entries. Only
   * successful shots are used.
   */
  public void loadFromDataPoints(List<ShotDataPoint> dataPoints) {
    table.clear();
    for (ShotDataPoint p : dataPoints) {
      if (p.successful()) {
        addEntry(
            p.distanceM(),
            p.targetRPM(),
            p.targetHoodAngleDeg(),
            p.timeOfFlightS(),
            p.targetMotivatorRPM(),
            p.targetSpindexerRPM());
      }
    }
  }

  private static double lerp(double a, double b, double t) {
    return a + t * (b - a);
  }

  /**
   * Build the default hub shot lookup table seeded from the known-good fixed shot presets. These are
   * the HubShot, LeftTrench, and RightTrench values from ShootingCommands tunables.
   *
   * <p>Replace or supplement with empirical measurements from practice.
   */
  public static ShotLookupTable buildDefaultHubTable() {
    return new ShotLookupTable()
        // Close range (placeholder)
        .addEntry(1.5, 2000, 35.0)
        .addEntry(2.0, 2200, 32.0)
        .addEntry(2.5, 2400, 29.0)
        // Mid range
        .addEntry(3.0, 2600, 26.0)
        .addEntry(3.5, 2800, 24.0)
        .addEntry(4.0, 3000, 22.0)
        // Long range
        .addEntry(4.5, 3200, 20.0)
        .addEntry(5.0, 3400, 18.0)
        .addEntry(5.5, 3600, 17.0);
  }
}
