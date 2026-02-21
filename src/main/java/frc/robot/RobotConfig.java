// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Interface defining robot-specific configuration values. Implement this interface for each
 * physical robot (MainBot, SquareBot, etc.)
 */
public interface RobotConfig {

  // Physical dimensions (bumper-to-bumper, from PathPlanner settings)
  /** Robot length with bumpers, front to back (meters). */
  double getBumperLength();

  /** Robot width with bumpers, side to side (meters). */
  double getBumperWidth();

  double getTrackWidth();

  double getWheelBase();

  double getDriveBaseRadius();

  Translation2d[] getModuleTranslations();

  // Module zero rotations (calibration values)
  Rotation2d getFrontLeftZeroRotation();

  Rotation2d getFrontRightZeroRotation();

  Rotation2d getBackLeftZeroRotation();

  Rotation2d getBackRightZeroRotation();

  // CAN IDs
  int getPigeonCanId();

  int getFrontLeftDriveCanId();

  int getBackLeftDriveCanId();

  int getFrontRightDriveCanId();

  int getBackRightDriveCanId();

  int getFrontLeftTurnCanId();

  int getBackLeftTurnCanId();

  int getFrontRightTurnCanId();

  int getBackRightTurnCanId();

  // CANCoder CAN IDs (for SquareBot external absolute encoders)
  int getFrontLeftCANCoderCanId();

  int getFrontRightCANCoderCanId();

  int getBackLeftCANCoderCanId();

  int getBackRightCANCoderCanId();

  // Drive motor configuration
  int getDriveMotorCurrentLimit();

  double getWheelRadiusMeters();

  double getDriveMotorReduction();

  DCMotor getDriveGearbox();

  // Drive encoder configuration
  double getDriveEncoderPositionFactor();

  double getDriveEncoderVelocityFactor();

  // Drive PID configuration
  double getDriveKp();

  double getDriveKd();

  double getDriveKs();

  double getDriveKv();

  double getDriveSimP();

  double getDriveSimD();

  double getDriveSimKs();

  double getDriveSimKv();

  // Turn motor configuration
  boolean getTurnInverted();

  int getTurnMotorCurrentLimit();

  double getTurnMotorReduction();

  DCMotor getTurnGearbox();

  // Turn encoder configuration
  boolean getTurnEncoderInverted();

  double getTurnEncoderPositionFactor();

  double getTurnEncoderVelocityFactor();

  // Turn PID configuration
  double getTurnKp();

  double getTurnKd();

  double getTurnSimP();

  double getTurnSimD();

  double getTurnPIDMinInput();

  double getTurnPIDMaxInput();

  // PathPlanner configuration
  double getRobotMassKg();

  double getRobotMOI();

  double getWheelCOF();

  com.pathplanner.lib.config.RobotConfig getPPConfig();

  // Performance
  double getMaxSpeedMetersPerSec();

  double getOdometryFrequency();

  // ========== Turret Configuration ==========
  // Default implementations return 0/false for robots without turrets (e.g.,
  // RectangleBot)

  /** Whether this robot has a turret. */
  default boolean hasTurret() {
    return false;
  }

  /** Turret motor CAN ID. */
  default int getTurretMotorCanId() {
    return 0;
  }

  /** Turret gear ratio (motor rotations per turret rotation). */
  default double getTurretGearRatio() {
    return 1.0;
  }

  // Only used when absolute encoder is after motor gearbox but before external
  // gearing
  default double getTurretMotorGearRatio() {
    return 1.0;
  }

  /** Turret height above ground in meters. */
  default double getTurretHeightMeters() {
    return 0.0;
  }

  /** Turret X offset from robot center (positive = forward). */
  default double getTurretOffsetX() {
    return 0.0;
  }

  /** Turret Y offset from robot center (positive = left). */
  default double getTurretOffsetY() {
    return 0.0;
  }

  /** Maximum turret angle in degrees (forward soft limit). */
  default double getTurretOutsideMaxAngleDeg() {
    return 0.0;
  }

  /** Minimum turret angle in degrees (reverse soft limit). */
  default double getTurretOutsideMinAngleDeg() {
    return 0.0;
  }

  /** Offset from Turret pointing to Robot Up to actual turret 0 */
  default double getTurretZeroOffset() {
    return 0.0;
  }

  /** Turret motor current limit in amps. */
  default int getTurretCurrentLimitAmps() {
    return 0;
  }

  /** Turret PID proportional gain. */
  default double getTurretKp() {
    return 0.0;
  }

  /** Turret PID derivative gain. */
  default double getTurretKd() {
    return 0.0;
  }

  /** Turret PID integral gain. */
  default double getTurretKi() {
    return 0.0;
  }

  /** Turret feedforward gain. */
  default double getTurretKff() {
    return 0.0;
  }

  // Only used when absolute encoder is after motor gearbox but before external gearing
  default double getTurretExternalGearRatio() {
    return 0.0;
  }

  /** Whether turret motor is inverted. */
  default boolean getTurretMotorInverted() {
    return false;
  }

  /** Absolute encoder zero offset in raw rotations (0.0 to 1.0) for turret calibration. */
  default double getTurretAbsoluteEncoderOffset() {
    return 0.0;
  }

  // ========== Launcher Configuration ==========
  // Default implementations return 0/false for robots without launchers

  /** Whether this robot has a launcher. */
  default boolean hasLauncher() {
    return false;
  }

  /** Launcher leader motor CAN ID. */
  default int getLauncherLeaderCanId() {
    return 0;
  }

  /** Launcher follower motor CAN ID. */
  default int getLauncherFollowerCanId() {
    return 0;
  }

  /**
   * Launcher gear ratio: motor rotations to wheel rotations. Example: 1.5 means 1 motor rotation =
   * 1.5 wheel rotations (wheel spins faster).
   */
  default double getLauncherGearRatio() {
    return 0.0;
  }

  /** Launcher motor current limit in amps. */
  default int getLauncherCurrentLimitAmps() {
    return 0;
  }

  /** Launcher PID proportional gain. */
  default double getLauncherKp() {
    return 0.0;
  }

  /** Launcher PID integral gain. */
  default double getLauncherKi() {
    return 0.0;
  }

  /** Launcher PID derivative gain. */
  default double getLauncherKd() {
    return 0.0;
  }

  /** Launcher velocity feedforward gain. */
  default double getLauncherKv() {
    return 0.0;
  }

  /** Launcher velocity feedforward gain. */
  default double getLauncherKs() {
    return 0.0;
  }

  // ========== Hood Configuration ==========
  // Default implementations return 0/false for robots without hoods

  /** Whether this robot has a hood (launch angle adjustment). */
  default boolean hasHood() {
    return false;
  }

  /** Hood motor CAN ID. */
  default int getHoodCanId() {
    return 0;
  }

  /** Hood motor current limit in amps. */
  default int getHoodCurrentLimitAmps() {
    return 0;
  }

  /** Hood Kp */
  default double getHoodKp() {
    return 0.0;
  }

  /** Hood Kd */
  default double getHoodKd() {
    return 0.0;
  }

  /** Hood motor Invert */
  default boolean getHoodMotorInverted() {
    return false;
  }

  /** Hood max angle */
  default double getHoodMaxAngleDegrees() {
    return 0.0;
  }

  /** Hood min angle */
  default double getHoodMinAngleDegrees() {
    return 0.0;
  }

  // ========== Motivator Configuration ==========
  // Default implementations return 0/false for robots without motivators

  /** Whether this robot has a motivator (ball feeder to launcher). */
  default boolean hasMotivator() {
    return false;
  }

  /** Launcher PID proportional gain. */
  default double getMotivatorKp() {
    return 0.0;
  }

  /** Launcher PID integral gain. */
  default double getMotivatorKi() {
    return 0.0;
  }

  /** Launcher PID derivative gain. */
  default double getMotivatorKd() {
    return 0.0;
  }

  /** Launcher PID derivative gain. */
  default double getMotivatorKv() {
    return 0.0;
  }

  /** Launcher PID derivative gain. */
  default double getMotivatorKs() {
    return 0.0;
  }

  /** Motivator leader motor CAN ID. */
  default int getMotivatorCanId() {
    return 0;
  }

  /** Motivator motor current limit in amps. */
  default int getMotivatorCurrentLimit() {
    return 0;
  }

  public default double getMotivatorGearRatio() {
    return 0;
  }

  // ========== Spindexer Configuration ==========
  // Default implementations return 0/false for robots without a spindexer

  /** Whether this robot has a spindexer (ball feeder to motivator). */
  default boolean hasSpindexer() {
    return false;
  }

  /** Spindexer PID proportional gain. */
  default double getSpindexerKp() {
    return 0.0;
  }

  /** Spindexer PID integral gain. */
  default double getSpindexerKi() {
    return 0.0;
  }

  /** Spindexer PID derivative gain. */
  default double getSpindexerKd() {
    return 0.0;
  }

  /** Spindexer feedforward velocity gain. */
  default double getSpindexerKv() {
    return 0.0;
  }

  /** Spindexer feedforward static gain. */
  default double getSpindexerKs() {
    return 0.0;
  }

  /** Spindexer motor CAN ID. */
  default int getSpindexerCanId() {
    return 0;
  }

  /** Spindexer motor current limit in amps. */
  default int getSpindexerCurrentLimit() {
    return 0;
  }

  /** Spindexer gear ratio (output/input). */
  public default double getSpindexerGearRatio() {
    return 0;
  }

  // ========== Intake Configuration ==========
  // Default implementations return 0/false for robots without intakes

  /** Whether this robot has an intake. */
  default boolean hasIntake() {
    return false;
  }

  /** Intake deploy motor CAN ID. */
  default int getIntakeDeployMotorCanId() {
    return 0;
  }

  /** Intake roller motor CAN ID. */
  default int getIntakeRollerMotorCanId() {
    return 0;
  }

  /** Intake deploy gear ratio (motor rotations per mechanism rotation). */
  default double getIntakeDeployGearRatio() {
    return 1.0;
  }

  /** Intake roller gear ratio (motor rotations per roller rotation). */
  default double getIntakeRollerGearRatio() {
    return 1.0;
  }

  /** Whether the intake deploy motor is inverted. */
  default boolean getIntakeDeployMotorInverted() {
    return false;
  }

  /** Whether the intake roller motor is inverted. */
  default boolean getIntakeRollerMotorInverted() {
    return false;
  }

  /** Intake deploy motor current limit in amps. */
  default int getIntakeDeployCurrentLimit() {
    return 0;
  }

  /** Intake roller motor current limit in amps. */
  default int getIntakeRollerCurrentLimit() {
    return 0;
  }

  /** Intake deploy retracted position in rotations. */
  default double getIntakeDeployRetractedPosition() {
    return 0.0;
  }

  /** Intake deploy extended position in rotations. */
  default double getIntakeDeployExtendedPosition() {
    return 0.0;
  }

  /** Intake deploy PID proportional gain. */
  default double getIntakeDeployKp() {
    return 0.0;
  }

  /** Intake deploy PID integral gain. */
  default double getIntakeDeployKi() {
    return 0.0;
  }

  /** Intake deploy PID derivative gain. */
  default double getIntakeDeployKd() {
    return 0.0;
  }

  /** Intake roller PID proportional gain. */
  default double getIntakeRollerKp() {
    return 0.0;
  }

  /** Intake roller PID integral gain. */
  default double getIntakeRollerKi() {
    return 0.0;
  }

  /** Intake roller PID derivative gain. */
  default double getIntakeRollerKd() {
    return 0.0;
  }

  /** Intake roller feedforward gain. */
  default double getIntakeRollerKff() {
    return 0.0;
  }
}
