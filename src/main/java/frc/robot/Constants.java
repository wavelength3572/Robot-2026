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

  /** Field positions for targeting (in meters) - 2026 Rebuilt game. */
  public static final class FieldPositions {
    // Field dimensions
    public static final double FIELD_LENGTH = 16.54; // meters
    public static final double FIELD_WIDTH = 8.23; // meters

    // Hub positions (2026 Rebuilt - 158.6" = 4.03m from alliance wall, centered on field width)
    public static final double BLUE_HUB_X = 4.03; // 158.6 inches from Blue wall
    public static final double BLUE_HUB_Y = 4.115; // Centered on field width
    public static final double RED_HUB_X = 12.51; // 16.54 - 4.03 = 12.51m from Blue wall
    public static final double RED_HUB_Y = 4.115; // Centered on field width

    // Zone boundaries for shoot vs pass logic
    public static final double BLUE_SHOOTING_ZONE_MAX_X = 5.5; // Blue can shoot when X < this
    public static final double RED_SHOOTING_ZONE_MIN_X = 11.0; // Red can shoot when X > this

    // Passing target X positions (aim back into own alliance zone)
    // Alliance zone is 158.6" (4.03m) deep, so target ~2m into zone
    public static final double BLUE_PASS_TARGET_X = 2.0; // ~2m from Blue wall (inside Blue zone)
    public static final double RED_PASS_TARGET_X = 14.54; // ~2m from Red wall (inside Red zone)

    // Passing target Y positions (1/3 distance from wall to center, to avoid hub net structure)
    public static final double LEFT_HALF_PASS_Y = FIELD_WIDTH / 6.0; // ~1.37m from left wall
    public static final double RIGHT_HALF_PASS_Y =
        FIELD_WIDTH * 5.0 / 6.0; // ~6.86m (1.37m from right wall)
  }
}
