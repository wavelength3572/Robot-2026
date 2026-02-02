// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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
  public static final RobotType FORCE_ROBOT_TYPE =
      null; // Set to a RobotType to override auto-detect

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
   * Detects the robot type based on connected CAN hardware. In simulation mode, uses the configured
   * simRobotType instead.
   *
   * <p>Detection strategy: Probe SparkMax drive motor CAN IDs unique to each robot. SquareBot uses
   * drive CAN IDs 3, 5. MainBot uses drive CAN IDs 11, 21. TurretBot has no drive motors but has a
   * turret SparkMax at CAN ID 60. If at least 2 drive IDs respond for a robot, that's the match.
   * Otherwise check for TurretBot, and fall back to MAINBOT.
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
      // Give CAN bus time to settle
      try {
        Thread.sleep(200);
      } catch (InterruptedException e) {
        // Ignore
      }

      // Check for SquareBot drive motors (CAN IDs 3, 5)
      int squareBotCount = 0;
      if (isSparkMaxPresent(3)) squareBotCount++;
      if (isSparkMaxPresent(5)) squareBotCount++;
      System.out.println("[RobotConfig] SquareBot drive motors found: " + squareBotCount + "/2");

      if (squareBotCount >= 2) {
        System.out.println("[RobotConfig] *** DETECTED SQUAREBOT ***");
        return RobotType.SQUAREBOT;
      }

      // Check for MainBot drive motors (CAN IDs 11, 21)
      int mainBotCount = 0;
      if (isSparkMaxPresent(11)) mainBotCount++;
      if (isSparkMaxPresent(21)) mainBotCount++;
      System.out.println("[RobotConfig] MainBot drive motors found: " + mainBotCount + "/2");

      if (mainBotCount >= 2) {
        System.out.println("[RobotConfig] *** DETECTED MAINBOT ***");
        return RobotType.MAINBOT;
      }

      // Check for TurretBot turret motor (CAN ID 60)
      if (isSparkMaxPresent(60)) {
        System.out.println("[RobotConfig] *** DETECTED TURRETBOT *** (CAN ID 60 responded)");
        return RobotType.TURRETBOT;
      }

      // Nothing matched, default to MAINBOT
      System.out.println("[RobotConfig] No robot detected, defaulting to MAINBOT");
      return RobotType.MAINBOT;
    } catch (Exception e) {
      System.out.println(
          "[RobotConfig] Error during detection, defaulting to MAINBOT: " + e.getMessage());
      return RobotType.MAINBOT;
    }
  }

  /** Probe a SparkMax at the given CAN ID. Returns true if a motor responds with valid firmware. */
  private static boolean isSparkMaxPresent(int canId) {
    try {
      SparkMax motor = new SparkMax(canId, MotorType.kBrushless);
      int firmware = motor.getFirmwareVersion();
      motor.close();
      System.out.println("[RobotConfig] CAN ID " + canId + " firmware: " + firmware);
      return firmware != 0;
    } catch (Exception e) {
      System.out.println("[RobotConfig] CAN ID " + canId + " error: " + e.getMessage());
      return false;
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
