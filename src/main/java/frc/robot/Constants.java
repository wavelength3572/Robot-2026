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

  /** The detected or configured robot type. */
  public static final RobotType currentRobot = detectRobotType();

  private static RobotConfig robotConfig = null;

  /**
   * Detects the robot type based on connected CAN hardware. In simulation mode, uses the configured
   * simRobotType instead.
   *
   * <p>Detection strategy: MainBot uses drive motors on CAN IDs 3, 5, 6, 8. MiniBot uses drive
   * motors on CAN IDs 11, 21, 31, 41. We check for a MainBot-specific CAN ID (5) and if it
   * responds, we're on MainBot.
   */
  private static RobotType detectRobotType() {
    // In simulation, use the configured type
    if (!RobotBase.isReal()) {
      System.out.println(
          "[RobotConfig] Simulation mode - using configured robot type: " + simRobotType);
      return simRobotType;
    }

    // On real hardware, auto-detect based on CAN devices
    try {
      // Try to create a SparkMax at MainBot's front-left drive CAN ID (5)
      // If this device exists and responds, we're on MainBot
      SparkMax testMotor = new SparkMax(5, MotorType.kBrushless);

      // Check if the device is actually connected by reading firmware version
      // A disconnected device will have firmware version 0
      int firmwareVersion = testMotor.getFirmwareVersion();
      testMotor.close();

      if (firmwareVersion != 0) {
        System.out.println(
            "[RobotConfig] Detected MainBot (CAN ID 5 responded with firmware "
                + firmwareVersion
                + ")");
        return RobotType.MAINBOT;
      } else {
        System.out.println("[RobotConfig] CAN ID 5 not responding - assuming MiniBot");
        return RobotType.MINIBOT;
      }
    } catch (Exception e) {
      // If any error occurs during detection, default to MainBot
      System.out.println(
          "[RobotConfig] Error during detection, defaulting to MainBot: " + e.getMessage());
      return RobotType.MAINBOT;
    }
  }

  public static RobotConfig getRobotConfig() {
    if (robotConfig == null) {
      switch (currentRobot) {
        case MAINBOT:
          robotConfig = new MainBotConfig();
          break;
        case MINIBOT:
          robotConfig = new MiniBotConfig();
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
    /** MainBot2026 - 21.25" chassis with NEO drive motors */
    MAINBOT,

    /** MiniBot2026 - 14" chassis with NEO Vortex drive motors */
    MINIBOT
  }

  /** Field positions for targeting (in meters). */
  public static final class FieldPositions {
    /** Field dimensions. */
    public static final double FIELD_LENGTH = 16.54;

    public static final double FIELD_WIDTH = 8.23;

    /** Zone boundaries. */
    public static final double ALLIANCE_ZONE_DEPTH = 4.03;

    /** Blue alliance hub position. */
    public static final double BLUE_HUB_X = 4.03;

    public static final double BLUE_HUB_Y = 4.115;

    /** Red alliance hub position. */
    public static final double RED_HUB_X = 12.51;

    public static final double RED_HUB_Y = 4.115;

    /** Pass target X positions (1/3 into alliance zone from wall). */
    public static final double BLUE_PASS_TARGET_X = 1.34;

    public static final double RED_PASS_TARGET_X = 15.2;

    /** Trench Y positions for dynamic pass targeting. */
    public static final double LEFT_TRENCH_Y = 1.5;

    public static final double RIGHT_TRENCH_Y = 6.73;
  }
}
