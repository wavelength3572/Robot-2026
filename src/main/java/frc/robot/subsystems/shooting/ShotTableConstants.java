package frc.robot.subsystems.shooting;

/**
 * Hardcoded baseline LUT entries — the "known-good" shot parameters for each distance.
 *
 * <p>Edit this file directly to tune shots. Each row is one distance:
 *
 * <pre>{distance_m, rpm, hood_angle_deg, measured_tof_s}</pre>
 *
 * <p>At startup, these are loaded into the LUT as the sole source of shot data. To update shots,
 * use the batch recorder during practice, review the data offline, then promote good values here
 * and redeploy.
 *
 * <p>TOF (time of flight) values are critical for accurate velocity compensation
 * (shoot-on-the-move). These should be measured from real shots when possible — the batch recorder
 * captures them.
 *
 * <p>Hood angle reference: lower number = more vertical launch, higher = flatter. Our range is
 * roughly 16-46 degrees mechanical.
 */
public final class ShotTableConstants {

  private ShotTableConstants() {}

  // ===== BASELINE SHOT TABLE =====
  // Sorted by distance. Edit these values, push code, and they take effect immediately.
  //
  // Format: {distance_m, rpm, hood_angle_deg, measured_tof_s}
  //
  // clang-format off
  public static final double[][] BASELINE_TABLE = {
    // Close range                              // TOF source
    {1.16, 2372, 13.0, 1.05}, // measured
    {1.25, 2372, 13.0, 1.05}, // measured
    {1.75, 2550, 13.5, 1.15}, // smoothed (measured 1.25)
    {1.88, 2550, 14.5, 1.15}, // smoothed (measured 1.25)

    // Mid range
    {1.963, 2575, 15.5, 1.15},
    {2.200, 2600, 15.5, 1.161},
    {2.500, 2750, 15.5, 1.175},
    {2.818, 2850, 15.5, 1.19}, // interpolated
    {3.111, 2875, 15.5, 1.22}, // measured
    {3.25, 2984, 16.5, 1.23}, // smoothed (measured 1.133)

    // Long range
    {3.63, 3200, 17.0, 1.26}, // smoothed (measured 1.509)
    {3.9, 2950, 20.0, 1.28}, // smoothed (measured 1.127)
    {4.28, 2950, 25.0, 1.28}, // smoothed (measured 1.127)
    {4.615, 3000, 29.0, 1.28}, // smoothed (measured 1.127)
    {5.025, 3000, 35.0, 1.28}, // smoothed (measured 1.127)
    {5.355, 3000, 38.0, 1.41}, // measured
  };
  // clang-format on

  // ===== PRACTICE ROOM TABLE (lower ceiling) =====
  // Copy of baseline — customize these values for practice room with low roof.
  // Lower hood angles and/or RPM may be needed to keep shots below ceiling.
  //
  // Format: {distance_m, rpm, hood_angle_deg, measured_tof_s}
  //
  // clang-format off
  public static final double[][] PRACTICE_ROOM_TABLE = {
    // Close range                              // TOF source
    {1.16, 2372, 13.0, 1.05}, // measured
    {1.75, 2558, 14.1, 1.15}, // smoothed (measured 1.25)

    // Mid range
    {2.818, 2894, 15.5, 1.19}, // interpolated
    {3.07, 2974, 16.0, 1.22}, // measured
    {3.25, 3031, 16.5, 1.23}, // smoothed (measured 1.133)

    // Long range
    {3.63, 3150, 18.0, 1.26}, // smoothed (measured 1.509)
    {3.85, 3220, 20.0, 1.28}, // smoothed (measured 1.127)
    {5.347, 3692, 39.0, 1.41}, // measured
  };
  // clang-format on

  /**
   * Load baseline entries into a lookup table. Call this before overlaying field-recorded data.
   *
   * @param table The lookup table to populate
   * @return Number of entries added
   */
  public static int loadBaseline(ShotLookupTable table) {
    return loadTable(table, BASELINE_TABLE);
  }

  /**
   * Load practice room entries into a lookup table. Tuned for low-ceiling practice space.
   *
   * @param table The lookup table to populate
   * @return Number of entries added
   */
  public static int loadPracticeRoom(ShotLookupTable table) {
    return loadTable(table, PRACTICE_ROOM_TABLE);
  }

  private static int loadTable(ShotLookupTable table, double[][] data) {
    for (double[] row : data) {
      double distance = row[0];
      double rpm = row[1];
      double hoodAngle = row[2];
      double measuredTOF = row[3];

      table.addEntry(distance, rpm, hoodAngle, 0.0, measuredTOF);
    }
    return data.length;
  }
}
