package frc.robot.subsystems.shooting;

import java.util.Map;
import java.util.TreeMap;

/**
 * Interpolating lookup table for shot parameters. Maps distance (meters) to shot parameters
 * (launcher RPM, hood angle). Uses linear interpolation between entries for smooth transitions.
 *
 * <p>Usage: populate with empirically measured shots at known distances, then query for any
 * distance. Entries outside the table range clamp to the nearest entry.
 *
 * <p>This provides a "ground-truth" alternative (or blend partner) to the physics-based
 * TrajectoryOptimizer, especially useful for shoot-on-the-move where small errors compound.
 */
public class ShotLookupTable {

  /** A single entry in the lookup table: RPM + hood angle at a known distance. */
  public record ShotEntry(double rpm, double hoodAngleDeg) {}

  private final TreeMap<Double, ShotEntry> table = new TreeMap<>();

  /** Create an empty lookup table. Use {@link #addEntry} to populate. */
  public ShotLookupTable() {}

  /**
   * Add a measured shot entry at a known distance.
   *
   * @param distanceMeters Horizontal distance to the target in meters
   * @param rpm Launcher RPM that worked at this distance
   * @param hoodAngleDeg Hood angle that worked at this distance
   * @return this (for chaining)
   */
  public ShotLookupTable addEntry(double distanceMeters, double rpm, double hoodAngleDeg) {
    table.put(distanceMeters, new ShotEntry(rpm, hoodAngleDeg));
    return this;
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
    double rpm = lo.rpm() + t * (hi.rpm() - lo.rpm());
    double hoodAngle = lo.hoodAngleDeg() + t * (hi.hoodAngleDeg() - lo.hoodAngleDeg());

    return new ShotEntry(rpm, hoodAngle);
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

  /**
   * Build the default hub shot lookup table with placeholder entries. Replace these values with
   * empirical measurements from the actual robot.
   *
   * <p>Entry format: distance (m) → (RPM, hood angle deg)
   */
  public static ShotLookupTable buildDefaultHubTable() {
    return new ShotLookupTable()
        // Close range
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
