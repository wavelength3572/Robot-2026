package frc.robot.subsystems.shooting;

/**
 * Hardcoded baseline LUT entries — the "known-good" shot parameters for each distance.
 *
 * <p>Edit this file directly to tune shots. Each row is one distance:
 */
public final class ShotTableConstants {

  private ShotTableConstants() {}

  // ===== BASELINE SHOT TABLE =====
  // Format: {distance_m, rpm, hood_angle_deg, measured_tof_s}
  public static final double[][] BASELINE_TABLE = {
    // Close range
    {1.300, 2400, 15.0, 1.070},
    {1.750, 2500, 16.0, 1.150},
    {1.880, 2550, 17.0, 1.160},

    // Mid range
    {1.963, 2575, 15.5, 1.160},
    {2.200, 2600, 15.5, 1.161},
    {2.500, 2750, 15.5, 1.175},
    {2.818, 2850, 15.5, 1.190},
    // {2.900, 2625, 18.0, 1.210},
    {3.008, 2900, 18.0, 1.220}, // was L2650, short when trimmed to 2811, so going to 2900
    {3.300, 3100, 18.0, 1.220}, // new point to edge us above linear
    {3.586, 3175, 18.0, 1.230},
    {3.700, 3350, 18.0, 1.240}, // was 3225, perfect distance with 125 trim so upping permanently
    {3.750, 3375, 18.0, 1.240}, // was 3225
    // Long range
    {
      3.900, 3300, 22.0, 1.28
    }, // was tested at 3050, 20, but practice match 1 it fell really short at this exact distance,
    // bumping 250 and 2 degrees
    {4.152, 3300, 34.0, 1.33}, // tested - okay
    {4.903, 3550, 34.0, 1.38}, // tried to make this a little lobbier +1deg +100RPM untested
  };

  // ===== ALTERNATE LUT TABLE =====
  // Format: {distance_m, rpm, hood_angle_deg, measured_tof_s}
  public static final double[][] ALTERNATE_TABLE = {
    // Close range
    {1.160, 2372, 13.0, 1.050},
    {1.250, 2372, 13.0, 1.070},
    {1.750, 2550, 13.5, 1.150},
    {1.880, 2550, 14.5, 1.160},

    // Mid range
    {1.963, 2575, 15.5, 1.160},
    {2.200, 2600, 15.5, 1.161},
    {2.500, 2750, 15.5, 1.175},
    {2.818, 2850, 15.5, 1.190},
    {3.111, 2875, 15.5, 1.220},
    {3.250, 2984, 16.5, 1.230},

    // Long range
    {3.630, 3200, 17.0, 1.26},
    {3.900, 2950, 20.0, 1.28},
    {4.280, 2950, 25.0, 1.33},
    {4.615, 3000, 29.0, 1.35},
    {5.025, 3000, 35.0, 1.38},
    {5.355, 3000, 38.0, 1.41},
  };

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
   * Load alternate LUT entries into a lookup table. Use for venue-specific tuning.
   *
   * @param table The lookup table to populate
   * @return Number of entries added
   */
  public static int loadAlternate(ShotLookupTable table) {
    return loadTable(table, ALTERNATE_TABLE);
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
