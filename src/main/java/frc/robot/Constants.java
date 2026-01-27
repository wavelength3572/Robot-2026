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
  public static final RobotType simRobotType = RobotType.SQUAREBOT;

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
   * <p>Detection strategy:
   *
   * <ul>
   *   <li>TurretBot uses turret motor on CAN ID 60
   *   <li>SquareBot uses drive motors on CAN IDs 3, 5, 6, 8
   *   <li>RectangleBot uses drive motors on CAN IDs 11, 21, 31, 41
   * </ul>
   *
   * We check CAN ID 60 first (TurretBot), then CAN ID 5 (SquareBot), then default to RectangleBot.
   */
  private static RobotType detectRobotType() {
    // In simulation, use the configured type
    if (!RobotBase.isReal()) {
      System.out.println(
          "[RobotConfig] Simulation mode - using configured robot type: " + simRobotType);
      return simRobotType;
    }

    System.out.println("[RobotConfig] Starting robot type detection...");

    // On real hardware, auto-detect based on CAN devices
    // Check for TurretBot first (CAN ID 60)
    try {
      System.out.println("[RobotConfig] Checking for TurretBot (CAN ID 60)...");
      SparkMax turretMotor = new SparkMax(60, MotorType.kBrushless);

      // Give CAN bus time to respond
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        // Ignore
      }

      int turretFirmware = turretMotor.getFirmwareVersion();
      System.out.println("[RobotConfig] CAN ID 60 firmware version: " + turretFirmware);
      turretMotor.close();

      if (turretFirmware != 0) {
        System.out.println(
            "[RobotConfig] *** DETECTED TURRETBOT *** (CAN ID 60 responded with firmware "
                + turretFirmware
                + ")");
        return RobotType.TURRETBOT;
      } else {
        System.out.println("[RobotConfig] CAN ID 60 returned firmware 0 - not TurretBot");
      }
    } catch (Exception e) {
      // TurretBot detection failed, continue to SquareBot check
      System.out.println(
          "[RobotConfig] TurretBot detection exception: "
              + e.getClass().getSimpleName()
              + " - "
              + e.getMessage());
    }

    // Check for SquareBot (CAN ID 5)
    try {
      System.out.println("[RobotConfig] Checking for SquareBot (CAN ID 5)...");
      SparkMax testMotor = new SparkMax(5, MotorType.kBrushless);

      // Give CAN bus time to respond
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        // Ignore
      }

      int firmwareVersion = testMotor.getFirmwareVersion();
      System.out.println("[RobotConfig] CAN ID 5 firmware version: " + firmwareVersion);
      testMotor.close();

      if (firmwareVersion != 0) {
        System.out.println(
            "[RobotConfig] *** DETECTED SQUAREBOT *** (CAN ID 5 responded with firmware "
                + firmwareVersion
                + ")");
        return RobotType.SQUAREBOT;
      } else {
        System.out.println("[RobotConfig] *** DETECTED RECTANGLEBOT *** (CAN ID 5 not responding)");
        return RobotType.RECTANGLEBOT;
      }
    } catch (Exception e) {
      // If any error occurs during detection, default to RectangleBot (safest option)
      System.out.println(
          "[RobotConfig] SquareBot detection exception: "
              + e.getClass().getSimpleName()
              + " - "
              + e.getMessage());
      System.out.println("[RobotConfig] *** DEFAULTING TO RECTANGLEBOT ***");
      return RobotType.RECTANGLEBOT;
    }
  }

  public static RobotConfig getRobotConfig() {
    if (robotConfig == null) {
      switch (currentRobot) {
        case SQUAREBOT:
          robotConfig = new SquareBotConfig();
          break;
        case RECTANGLEBOT:
          robotConfig = new RectangleBotConfig();
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

    /** RectangleBot2026 - 14" chassis with NEO Vortex drive motors */
    RECTANGLEBOT,

    /** TurretBot - Minimal test rig with only turret + basic electronics (no drive base) */
    TURRETBOT
  }

  /** Field positions for targeting (in meters). */
  public static final class FieldPositions {
    /** Field dimensions. */
    public static final double FIELD_LENGTH = 16.54;

    public static final double FIELD_WIDTH = 8.23;

    /** Zone boundaries. */
    public static final double ALLIANCE_ZONE_DEPTH = 4.03;

    /** Blue alliance hub position. */
    public static final double BLUE_HUB_X = 4.575; // 4.03

    public static final double BLUE_HUB_Y = 4.115;

    /** Red alliance hub position. */
    public static final double RED_HUB_X = 11.9865; // 12.51

    public static final double RED_HUB_Y = 4.115;

    /** Hub height (scoring target height in meters). */
    public static final double HUB_HEIGHT = 1.43; // ~56.4 inches

    /** Pass target X positions (1/3 into alliance zone from wall). */
    public static final double BLUE_PASS_TARGET_X = 1.34;

    public static final double RED_PASS_TARGET_X = 15.2;

    /** Trench Y positions for dynamic pass targeting. */
    public static final double LEFT_TRENCH_Y = 1.5;

    public static final double RIGHT_TRENCH_Y = 6.73;
  }
}
