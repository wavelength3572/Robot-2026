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
 * physical robot (SquareBot, RectangleBot, etc.)
 */
public interface RobotConfig {

  // Physical dimensions
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
  // Default implementations return 0/false for robots without turrets (e.g., RectangleBot)

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

  /** Turret height above ground in meters. */
  default double getTurretHeightMeters() {
    return 0.0;
  }

  /** Maximum turret angle in degrees (forward soft limit). */
  default double getTurretMaxAngleDegrees() {
    return 0.0;
  }

  /** Minimum turret angle in degrees (reverse soft limit). */
  default double getTurretMinAngleDegrees() {
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

  // TurretBot-specific: external gear ratio (encoder to turret)
  // Only used when absolute encoder is after motor gearbox but before external gearing
  default double getTurretExternalGearRatio() {
    return 1.0;
  }

  /** Whether turret motor is inverted. */
  default boolean getTurretMotorInverted() {
    return false;
  }

  /** Whether turret encoder is inverted. */
  default boolean getTurretEncoderInverted() {
    return false;
  }
}
