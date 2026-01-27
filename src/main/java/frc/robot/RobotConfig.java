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

  // CANCoder CAN IDs (for MainBot external absolute encoders)
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
}
