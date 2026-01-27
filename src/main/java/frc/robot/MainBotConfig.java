// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * Configuration for MainBot2026 - 23.5" x 31" chassis with NEO Vortex drive motors Based on
 * MainBot-2025 configuration from wavelength3572/MainBot-2025
 */
public class MainBotConfig implements RobotConfig {

  // Physical dimensions
  private static final double trackWidth = Units.inchesToMeters(31.0);
  private static final double wheelBase = Units.inchesToMeters(23.5);
  private static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  private static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // Front Left
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // Front Right
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // Back Left
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // Back Right
      };

  // Module zero rotations (needs calibration for MainBot)
  private static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
  private static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  private static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
  private static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

  // CAN IDs
  private static final int pigeonCanId = 19;
  private static final int frontLeftDriveCanId = 11;
  private static final int frontRightDriveCanId = 21;
  private static final int backLeftDriveCanId = 31;
  private static final int backRightDriveCanId = 41;
  private static final int frontLeftTurnCanId = 12;
  private static final int frontRightTurnCanId = 22;
  private static final int backLeftTurnCanId = 32;
  private static final int backRightTurnCanId = 42;

  // Drive motor configuration
  private static final int driveMotorCurrentLimit = 50;
  private static final double wheelRadiusMeters = Units.inchesToMeters(2.0);
  private static final double driveMotorReduction =
      (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // SDS MK4i L2
  private static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  private static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction;
  private static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction;

  // Drive PID configuration
  private static final double driveKp = 0.0;
  private static final double driveKd = 0.0;
  private static final double driveKs = 0.0;
  private static final double driveKv = 0.1;
  private static final double driveSimP = 0.05;
  private static final double driveSimD = 0.0;
  private static final double driveSimKs = 0.04307;
  private static final double driveSimKv = 0.21126;

  // Turn motor configuration
  private static final boolean turnInverted = true;
  private static final int turnMotorCurrentLimit = 20;
  private static final double turnMotorReduction = 150.0 / 7.0; // SDS MK4i L2
  private static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  private static final boolean turnEncoderInverted = true;
  private static final double turnEncoderPositionFactor = 2 * Math.PI / turnMotorReduction;
  private static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0 / turnMotorReduction;

  // Turn PID configuration
  private static final double turnKp = 1.0;
  private static final double turnKd = 0.0;
  private static final double turnSimP = 8.0;
  private static final double turnSimD = 0.0;
  private static final double turnPIDMinInput = 0;
  private static final double turnPIDMaxInput = 2 * Math.PI;

  // PathPlanner configuration
  private static final double robotMassKg = 74.088;
  private static final double robotMOI = 6.883;
  private static final double wheelCOF = 1.2;

  // Performance
  private static final double maxSpeedMetersPerSec =
      5676.0 / 60.0 / driveMotorReduction * 2.0 * Math.PI * wheelRadiusMeters * 0.95;
  private static final double odometryFrequency = 100.0;

  // PathPlanner RobotConfig (computed)
  private final com.pathplanner.lib.config.RobotConfig ppConfig =
      new com.pathplanner.lib.config.RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  @Override
  public double getTrackWidth() {
    return trackWidth;
  }

  @Override
  public double getWheelBase() {
    return wheelBase;
  }

  @Override
  public double getDriveBaseRadius() {
    return driveBaseRadius;
  }

  @Override
  public Translation2d[] getModuleTranslations() {
    return moduleTranslations;
  }

  @Override
  public Rotation2d getFrontLeftZeroRotation() {
    return frontLeftZeroRotation;
  }

  @Override
  public Rotation2d getFrontRightZeroRotation() {
    return frontRightZeroRotation;
  }

  @Override
  public Rotation2d getBackLeftZeroRotation() {
    return backLeftZeroRotation;
  }

  @Override
  public Rotation2d getBackRightZeroRotation() {
    return backRightZeroRotation;
  }

  @Override
  public int getPigeonCanId() {
    return pigeonCanId;
  }

  @Override
  public int getFrontLeftDriveCanId() {
    return frontLeftDriveCanId;
  }

  @Override
  public int getBackLeftDriveCanId() {
    return backLeftDriveCanId;
  }

  @Override
  public int getFrontRightDriveCanId() {
    return frontRightDriveCanId;
  }

  @Override
  public int getBackRightDriveCanId() {
    return backRightDriveCanId;
  }

  @Override
  public int getFrontLeftTurnCanId() {
    return frontLeftTurnCanId;
  }

  @Override
  public int getBackLeftTurnCanId() {
    return backLeftTurnCanId;
  }

  @Override
  public int getFrontRightTurnCanId() {
    return frontRightTurnCanId;
  }

  @Override
  public int getBackRightTurnCanId() {
    return backRightTurnCanId;
  }

  @Override
  public int getFrontLeftCANCoderCanId() {
    return 0;
  }

  @Override
  public int getFrontRightCANCoderCanId() {
    return 0;
  }

  @Override
  public int getBackLeftCANCoderCanId() {
    return 0;
  }

  @Override
  public int getBackRightCANCoderCanId() {
    return 0;
  }

  @Override
  public int getDriveMotorCurrentLimit() {
    return driveMotorCurrentLimit;
  }

  @Override
  public double getWheelRadiusMeters() {
    return wheelRadiusMeters;
  }

  @Override
  public double getDriveMotorReduction() {
    return driveMotorReduction;
  }

  @Override
  public DCMotor getDriveGearbox() {
    return driveGearbox;
  }

  @Override
  public double getDriveEncoderPositionFactor() {
    return driveEncoderPositionFactor;
  }

  @Override
  public double getDriveEncoderVelocityFactor() {
    return driveEncoderVelocityFactor;
  }

  @Override
  public double getDriveKp() {
    return driveKp;
  }

  @Override
  public double getDriveKd() {
    return driveKd;
  }

  @Override
  public double getDriveKs() {
    return driveKs;
  }

  @Override
  public double getDriveKv() {
    return driveKv;
  }

  @Override
  public double getDriveSimP() {
    return driveSimP;
  }

  @Override
  public double getDriveSimD() {
    return driveSimD;
  }

  @Override
  public double getDriveSimKs() {
    return driveSimKs;
  }

  @Override
  public double getDriveSimKv() {
    return driveSimKv;
  }

  @Override
  public boolean getTurnInverted() {
    return turnInverted;
  }

  @Override
  public int getTurnMotorCurrentLimit() {
    return turnMotorCurrentLimit;
  }

  @Override
  public double getTurnMotorReduction() {
    return turnMotorReduction;
  }

  @Override
  public DCMotor getTurnGearbox() {
    return turnGearbox;
  }

  @Override
  public boolean getTurnEncoderInverted() {
    return turnEncoderInverted;
  }

  @Override
  public double getTurnEncoderPositionFactor() {
    return turnEncoderPositionFactor;
  }

  @Override
  public double getTurnEncoderVelocityFactor() {
    return turnEncoderVelocityFactor;
  }

  @Override
  public double getTurnKp() {
    return turnKp;
  }

  @Override
  public double getTurnKd() {
    return turnKd;
  }

  @Override
  public double getTurnSimP() {
    return turnSimP;
  }

  @Override
  public double getTurnSimD() {
    return turnSimD;
  }

  @Override
  public double getTurnPIDMinInput() {
    return turnPIDMinInput;
  }

  @Override
  public double getTurnPIDMaxInput() {
    return turnPIDMaxInput;
  }

  @Override
  public double getRobotMassKg() {
    return robotMassKg;
  }

  @Override
  public double getRobotMOI() {
    return robotMOI;
  }

  @Override
  public double getWheelCOF() {
    return wheelCOF;
  }

  @Override
  public com.pathplanner.lib.config.RobotConfig getPPConfig() {
    return ppConfig;
  }

  @Override
  public double getMaxSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  @Override
  public double getOdometryFrequency() {
    return odometryFrequency;
  }
}
