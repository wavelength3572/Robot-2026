// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;

/**
 * DriveConstants provides a unified interface to robot-specific drive configuration. All values are
 * retrieved from the currently selected RobotConfig (MainBot or MiniBot).
 */
public class DriveConstants {
  private static final frc.robot.RobotConfig config = Constants.getRobotConfig();

  // Performance
  public static final double maxSpeedMetersPerSec = config.getMaxSpeedMetersPerSec();
  public static final double odometryFrequency = config.getOdometryFrequency();

  // Physical dimensions
  public static final double trackWidth = config.getTrackWidth();
  public static final double wheelBase = config.getWheelBase();
  public static final double driveBaseRadius = config.getDriveBaseRadius();
  public static final Translation2d[] moduleTranslations = config.getModuleTranslations();

  // Zeroed rotation values for each module
  public static final Rotation2d frontLeftZeroRotation = config.getFrontLeftZeroRotation();
  public static final Rotation2d frontRightZeroRotation = config.getFrontRightZeroRotation();
  public static final Rotation2d backLeftZeroRotation = config.getBackLeftZeroRotation();
  public static final Rotation2d backRightZeroRotation = config.getBackRightZeroRotation();

  // Device CAN IDs
  public static final int pigeonCanId = config.getPigeonCanId();
  public static final int frontLeftDriveCanId = config.getFrontLeftDriveCanId();
  public static final int backLeftDriveCanId = config.getBackLeftDriveCanId();
  public static final int frontRightDriveCanId = config.getFrontRightDriveCanId();
  public static final int backRightDriveCanId = config.getBackRightDriveCanId();
  public static final int frontLeftTurnCanId = config.getFrontLeftTurnCanId();
  public static final int backLeftTurnCanId = config.getBackLeftTurnCanId();
  public static final int frontRightTurnCanId = config.getFrontRightTurnCanId();
  public static final int backRightTurnCanId = config.getBackRightTurnCanId();

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = config.getDriveMotorCurrentLimit();
  public static final double wheelRadiusMeters = config.getWheelRadiusMeters();
  public static final double driveMotorReduction = config.getDriveMotorReduction();
  public static final DCMotor driveGearbox = config.getDriveGearbox();

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor = config.getDriveEncoderPositionFactor();
  public static final double driveEncoderVelocityFactor = config.getDriveEncoderVelocityFactor();

  // Drive PID configuration
  public static final double driveKp = config.getDriveKp();
  public static final double driveKd = config.getDriveKd();
  public static final double driveKs = config.getDriveKs();
  public static final double driveKv = config.getDriveKv();
  public static final double driveSimP = config.getDriveSimP();
  public static final double driveSimD = config.getDriveSimD();
  public static final double driveSimKs = config.getDriveSimKs();
  public static final double driveSimKv = config.getDriveSimKv();

  // Turn motor configuration
  public static final boolean turnInverted = config.getTurnInverted();
  public static final int turnMotorCurrentLimit = config.getTurnMotorCurrentLimit();
  public static final double turnMotorReduction = config.getTurnMotorReduction();
  public static final DCMotor turnGearbox = config.getTurnGearbox();

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = config.getTurnEncoderInverted();
  public static final double turnEncoderPositionFactor = config.getTurnEncoderPositionFactor();
  public static final double turnEncoderVelocityFactor = config.getTurnEncoderVelocityFactor();

  // Turn PID configuration
  public static final double turnKp = config.getTurnKp();
  public static final double turnKd = config.getTurnKd();
  public static final double turnSimP = config.getTurnSimP();
  public static final double turnSimD = config.getTurnSimD();
  public static final double turnPIDMinInput = config.getTurnPIDMinInput();
  public static final double turnPIDMaxInput = config.getTurnPIDMaxInput();

  // PathPlanner configuration
  public static final double robotMassKg = config.getRobotMassKg();
  public static final double robotMOI = config.getRobotMOI();
  public static final double wheelCOF = config.getWheelCOF();
  public static final RobotConfig ppConfig = config.getPPConfig();
}
