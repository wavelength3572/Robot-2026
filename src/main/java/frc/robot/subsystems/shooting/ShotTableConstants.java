package frc.robot.subsystems.shooting;

/**
 * Hardcoded baseline LUT entries — the "known-good" shot parameters for each distance.
 *
 * <p>Edit this file directly to tune shots. Each row is one distance:
 *
 * <pre>{distance_m, rpm, hood_angle_deg, motivator_rpm, spindexer_rpm}</pre>
 *
 * <p>At startup, these are loaded into the LUT first. Field-recorded entries (from the batch
 * recorder) then overlay on top — if a field entry is within 0.15m of a baseline entry, it
 * replaces it; otherwise it's added as a new point.
 *
 * <p>After a good practice session, promote your field-recorded values here so they persist across
 * code deploys. The JSON file on the robot is ephemeral; this file is the source of truth.
 *
 * <p>Hood angle reference: lower number = more vertical launch, higher = flatter. Our range is
 * roughly 16-46 degrees mechanical.
 */
public final class ShotTableConstants {

  private ShotTableConstants() {}

  // ===== BASELINE SHOT TABLE =====
  // Sorted by distance. Edit these values, push code, and they take effect immediately.
  //
  // Format: {distance_m, rpm, hood_angle_deg, motivator_rpm, spindexer_rpm}
  //
  // clang-format off
  public static final double[][] BASELINE_TABLE = {
      // Close range
      {1.15, 2300, 13.0, 1300, 325},
      {1.75, 2550, 14.0, 1350, 325},

      // Mid range
      {2.50, 2650, 22.0, 1400, 325},
      {2.75, 2700, 22.0, 1500, 325},
      {3.00, 2650, 23.5, 1500, 325},
      {3.25, 2700, 19.0, 1550, 325},

      // Long range
      {3.60, 3170, 18.0, 1800, 325},
      {3.85, 2950, 20.0, 1550, 325},
      {5.35, 3700, 39.0, 1800, 225},
  };
  // clang-format on

  /**
   * Load baseline entries into a lookup table. Call this before overlaying field-recorded data.
   *
   * @param table The lookup table to populate
   * @return Number of entries added
   */
  public static int loadBaseline(ShotLookupTable table) {
    for (double[] row : BASELINE_TABLE) {
      double distance = row[0];
      double rpm = row[1];
      double hoodAngle = row[2];
      double motivatorRPM = row[3];
      double spindexerRPM = row[4];

      // Estimate TOF from distance (rough approximation for velocity compensation)
      double estimatedTOF = distance / 10.0;
      table.addEntry(distance, rpm, hoodAngle, estimatedTOF, 0.0, motivatorRPM, spindexerRPM);
    }
    return BASELINE_TABLE.length;
  }
}
