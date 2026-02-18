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
  // RobotType.TURRETBOT; // Bench testing: force TURRETBOT to avoid CAN errors from disconnected
  // swerve modules

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

  /** Team-specific strategy constants (pass targets, trench positions). */
  public static final class StrategyConstants {
    /** Pass target X positions (1/3 into alliance zone from wall). */
    public static final double BLUE_PASS_TARGET_X = 2.0;

    public static final double RED_PASS_TARGET_X = FieldConstants.fieldLength- BLUE_PASS_TARGET_X;

    /** Pass target Y positions (offset from field center toward each trench). */
    public static final double PASS_TARGET_Y_OFFSET = 1.75;
    public static final double RIGHT_PASS_TARGET_Y = FieldConstants.fieldWidth/2-PASS_TARGET_Y_OFFSET;
    public static final double LEFT_PASS_TARGET_Y = FieldConstants.fieldWidth/2+PASS_TARGET_Y_OFFSET;
  }
}
