// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim), "replay" (log
 * replay from a file), or "pit" (simulated drive with real subsystems for pit testing).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = resolveMode();

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

    System.out.println("[RobotConfig] Assuming MainBot");
    return RobotType.MAINBOT;
  }

  public static RobotConfig getRobotConfig() {
    if (robotConfig == null) {
      robotConfig = new MainBotConfig();
    }
    return robotConfig;
  }

  /**
   * Resolves the runtime mode. On real hardware, normally REAL. When simMode is set to PIT and
   * deployed to the roboRIO, enables pit mode (simulated drive, real everything else). As a safety
   * measure, PIT mode falls back to REAL when FMS is connected.
   */
  private static Mode resolveMode() {
    if (RobotBase.isReal()) {
      if (simMode == Mode.PIT) {
        if (DriverStation.isFMSAttached()) {
          System.out.println(
              "[RobotConfig] simMode is PIT but FMS is connected — falling back to REAL for safety.");
          return Mode.REAL;
        }
        System.out.println(
            "[RobotConfig] *** PIT MODE ENABLED *** Drive is simulated, all other subsystems are real.");
        return Mode.PIT;
      }
      return Mode.REAL;
    }
    return simMode;
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
