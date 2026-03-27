// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;

  /**
   * Pit mode: runs on the real roboRIO but uses simulated drive (physics sim) while all other
   * subsystems (turret, launcher, hood, vision, intake, etc.) use real hardware. This lets the team
   * "drive around" the field virtually in the pits and observe real targeting, shot calculations,
   * and vision data at arbitrary field positions — e.g. see what happens at 7 m from the hub.
   *
   * <p>Enable by setting the "PitMode" preference to true on the roboRIO (via Preferences widget on
   * the dashboard) and restarting robot code. As a safety measure, pit mode is automatically
   * disabled when FMS is connected so the robot drives normally if you forget to turn it off.
   */
  public static final boolean pitMode = RobotBase.isReal() && detectPitMode();

  public static final Mode currentMode =
      pitMode ? Mode.PIT : (RobotBase.isReal() ? Mode.REAL : simMode);

  /**
   * Robot type for simulation mode. Change this to test different configurations. On real hardware,
   * this is ignored and auto-detection is used.
   */
  public static final RobotType simRobotType = RobotType.MAINBOT;

  // swerve modules
  public static final RobotType currentRobot = detectRobotType();

  private static RobotConfig robotConfig = null;

  /**
   * Detects the robot type based on RobotPreferences. In simulation mode, uses the configured
   * simRobotType instead.
   */
  private static RobotType detectRobotType() {
    // In simulation, use the configured type
    if (!RobotBase.isReal()) {
      System.out.println(
          "[RobotConfig] Simulation mode - using configured robot type: " + simRobotType);
      return simRobotType;
    }

    System.out.println("[RobotConfig] Starting robot type detection...");

    try {
      // Read RobotPreferences from RoboRIO for the RobotName
      if (Preferences.getString("RobotName", "nullBot").equals("SquareBot")) {
        System.out.println("[RobotConfig] Detected SquareBot");
        return RobotType.SQUAREBOT;
      } else {
        System.out.println("[RobotConfig] Assuming MainBot");
        return RobotType.MAINBOT;
      }
    } catch (Exception e) {
      System.out.println(
          "[RobotConfig] Error during detection, defaulting to MainBot: " + e.getMessage());
      return RobotType.MAINBOT;
    }
  }

  public static RobotConfig getRobotConfig() {
    if (robotConfig == null) {
      switch (currentRobot) {
        case SQUAREBOT:
          robotConfig = new SquareBotConfig();
          break;
        case MAINBOT:
          robotConfig = new MainBotConfig();
          break;
      }
    }
    return robotConfig;
  }

  /**
   * Checks if pit mode is enabled via RobotPreferences. Set "PitMode" to true on the dashboard
   * Preferences widget to enable.
   */
  private static boolean detectPitMode() {
    try {
      boolean enabled = Preferences.getBoolean("PitMode", false);
      if (enabled && DriverStation.isFMSAttached()) {
        System.out.println(
            "[RobotConfig] PitMode preference is ON but FMS is connected — ignoring pit mode for safety.");
        return false;
      }
      if (enabled) {
        System.out.println(
            "[RobotConfig] *** PIT MODE ENABLED *** Drive is simulated, all other subsystems are real.");
      }
      return enabled;
    } catch (Exception e) {
      return false;
    }
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /**
     * Pit mode: real robot hardware for all subsystems EXCEPT drive, which uses physics simulation.
     * Allows virtual field driving in the pits with real targeting and vision.
     */
    PIT,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum RobotType {
    /** SquareBot2026 - 21.25" chassis with NEO drive motors */
    SQUAREBOT,

    /** MainBot2026 - 23.5" x 31" chassis with NEO Vortex drive motors */
    MAINBOT
  }

  /**
   * Universal turret constants that apply to all robots. Per-robot values (offsets, height, CAN
   * IDs, gear ratios, PID gains) live in the RobotConfig implementations.
   */
  public static final class TurretConstants {
    // Tolerances
    public static final double ANGLE_TOLERANCE_DEGREES = 2.0;

    // Motion constraints (used for trajectory generation if needed)
    public static final double MAX_VELOCITY_DEG_PER_SEC = 360.0;
    public static final double MAX_ACCELERATION_DEG_PER_SEC_SQUARED = 720.0;
  }

  /** Team-specific strategy constants (pass targets, trench positions). */
  public static final class StrategyConstants {
    /** Pass target X positions (closer to neutral zone for achievable pass RPMs). */
    public static final double BLUE_PASS_TARGET_X = 2.0;

    public static final double RED_PASS_TARGET_X = FieldConstants.fieldLength - BLUE_PASS_TARGET_X;

    /** Pass target Y positions (offset from field center toward each outer wall). */
    public static final double PASS_TARGET_Y_OFFSET = 2.66;

    public static final double RIGHT_PASS_TARGET_Y =
        FieldConstants.fieldWidth / 2 - PASS_TARGET_Y_OFFSET;
    public static final double LEFT_PASS_TARGET_Y =
        FieldConstants.fieldWidth / 2 + PASS_TARGET_Y_OFFSET;

    /**
     * Lob pass target Y positions for driver-station strategy. Station 1/2 target is far from the
     * outpost; station 3 target is near the outpost.
     *
     * <p>Both targets are 1.5m from their respective walls. Station 1 is near the high-Y wall for
     * blue, station 3 is near the low-Y wall for blue.
     */
    public static final double LOB_STATION_1_TARGET_Y = 1.5;

    public static final double LOB_STATION_3_TARGET_Y = 1.5;
  }
}
