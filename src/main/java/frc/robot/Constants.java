// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /**
   * Robot type for simulation mode. Change this to test different configurations. On real hardware,
   * this is ignored and auto-detection is used.
   */
  public static final RobotType simRobotType = RobotType.MAINBOT;

  /**
   * The detected or configured robot type. Set FORCE_ROBOT_TYPE to override auto-detection. Set to
   * null to use auto-detection.
   */
  public static final RobotType FORCE_ROBOT_TYPE = null;
  // RobotType.TURRETBOT; // TODO: Remove this and add TurretBot to auto-detect logic

  public static final RobotType currentRobot = initRobotType();

  private static RobotType initRobotType() {
    if (FORCE_ROBOT_TYPE != null) {
      System.out.println(
          "[RobotConfig] *** FORCED ROBOT TYPE: "
              + FORCE_ROBOT_TYPE
              + " *** (auto-detect disabled)");
      return FORCE_ROBOT_TYPE;
    }
    return detectRobotType();
  }

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
        case TURRETBOT:
          robotConfig = new TurretBotConfig();
          break;
      }
    }
    return robotConfig;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum RobotType {
    /** SquareBot2026 - 21.25" chassis with NEO drive motors */
    SQUAREBOT,

    /** MainBot2026 - 23.5" x 31" chassis with NEO Vortex drive motors */
    MAINBOT,

    /** TurretBot - turret-only test platform with no drivetrain */
    TURRETBOT
  }

  /** Field positions for targeting (in meters). */
  public static final class FieldPositions {
    /** Field dimensions. */
    public static final double FIELD_LENGTH = 16.54;

    public static final double FIELD_WIDTH = 8.07; // 317.69 inches (welded field)

    /** Zone boundaries. */
    public static final double ALLIANCE_ZONE_DEPTH = 4.03;

    /** Hub center height - the smallest opening where balls should land (below the lip). */
    public static final double HUB_HEIGHT = 1.43; // ~56.4 inches - center of hub opening

    /** Pass target X positions (1/3 into alliance zone from wall). */
    public static final double BLUE_PASS_TARGET_X = 1.34;

    public static final double RED_PASS_TARGET_X = 15.2;

    /** Trench Y positions for dynamic pass targeting. */
    public static final double LEFT_TRENCH_Y = 1.5;

    public static final double RIGHT_TRENCH_Y = 6.73;

    /** Blue alliance speaker position. */
    public static final double BLUE_HUB_X = 4.625594; // 182.11 = 4.625594 OLD 4.575

    public static final double BLUE_HUB_Y = 4.034536; // 158.84 = 4.034536 OLD 4.115
    /** Red alliance speaker position. */
    public static final double RED_HUB_X = 11.915394; // 469.11 = 11.915394 OLD 11.9865

    public static final double RED_HUB_Y = 4.034536;
  }
}
