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
 * Configuration for TurretBot - A minimal test rig with only turret + basic electronics (no drive
 * base). This configuration provides stub/placeholder values for all drive-related methods since
 * TurretBot has no drivetrain.
 */
public class TurretBotConfig implements RobotConfig {

  // Placeholder physical dimensions (minimal values since no drive base)
  private static final double trackWidth = Units.inchesToMeters(12.0);
  private static final double wheelBase = Units.inchesToMeters(12.0);
  private static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  private static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Module zero rotations (placeholder - no swerve modules)
  private static final Rotation2d zeroRotation = new Rotation2d(0);

  // CAN IDs - Only Pigeon IMU is real, drive motors are stub values (0 = not present)
  private static final int pigeonCanId = 19;

  // Placeholder drive motor configuration (no actual motors)
  private static final int driveMotorCurrentLimit = 0;
  private static final double wheelRadiusMeters = Units.inchesToMeters(2.0);
  private static final double driveMotorReduction = 1.0;
  private static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Placeholder drive encoder configuration
  private static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction;
  private static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction;

  // Placeholder PID configuration
  private static final double driveKp = 0.0;
  private static final double driveKd = 0.0;
  private static final double driveKs = 0.0;
  private static final double driveKv = 0.0;

  // Placeholder turn motor configuration
  private static final boolean turnInverted = false;
  private static final int turnMotorCurrentLimit = 0;
  private static final double turnMotorReduction = 1.0;
  private static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Placeholder turn encoder configuration
  private static final boolean turnEncoderInverted = false;
  private static final double turnEncoderPositionFactor = 2 * Math.PI / turnMotorReduction;
  private static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0 / turnMotorReduction;

  // Placeholder turn PID configuration
  private static final double turnKp = 0.0;
  private static final double turnKd = 0.0;

  // Robot configuration (minimal mass since only turret + electronics)
  private static final double robotMassKg = Units.lbsToKilograms(25.0); // ~25 lbs
  private static final double robotMOI = 1.0;
  private static final double wheelCOF = 1.0;

  // Performance - Virtual driving for turret testing
  private static final double maxSpeedMetersPerSec = 4.5; // Reasonable speed for virtual driving
  private static final double odometryFrequency = 100.0;

  // ========== Turret Configuration (NEO 550 + Spark Max) ==========
  // Through Bore Encoder connects via data port (Gadgeteer) on the same Spark Max
  private static final int turretMotorCanId = 60;
  private static final double turretHeightMeters = 0.4826; // 19 inches

  // Gear ratios:
  // - NEO 550 internal gearbox: 10:1
  // - External gearing from encoder to turret: 66:12 (~5.5:1)
  // - Total ratio: 55:1 (motor rotations per turret rotation)
  // The absolute encoder sits AFTER the 10:1 gearbox but BEFORE the 66:12 external gearing
  private static final double turretGearRatio = 55.0; // Total motor rotations per turret rotation
  private static final double turretExternalGearRatio = 66.0 / 12.0; // ~5.5 (encoder to turret)

  // Travel limits: ±200° = 400° total travel
  private static final double turretMaxAngleDegrees = 200.0;
  private static final double turretMinAngleDegrees = -200.0;

  // NEO 550-specific settings
  private static final int turretCurrentLimitAmps = 20; // NEO 550 is smaller than NEO/Falcon
  private static final double turretKp =
      0.0065; // PID operates in motor rotations (scaled from 0.001 in degrees)
  private static final double turretKi = 0.0;
  private static final double turretKd = 0.0;

  // Absolute encoder zero offset (raw rotations 0.0-1.0)
  // To calibrate: position turret at physical 0°, read the "AbsRaw" value from startup diagnostics,
  // and set this to that value.
  private static final double turretAbsoluteEncoderOffset = 0.25;

  // ========== Launcher Configuration (Two NEO Vortex + SparkFlex) ==========
  // Two motors coupled to same shaft, facing opposite directions
  // Gear ratio: 1 motor rotation = 1.5 wheel rotations (wheel spins faster)
  private static final int launcherLeaderCanId = 58;
  private static final int launcherFollowerCanId = 59;
  private static final double launcherGearRatio = 1.5; // 1 motor rot = 1.5 wheel rot
  private static final int launcherCurrentLimitAmps = 60;
  private static final double launcherKp = 0.000001;
  private static final double launcherKi = 0.0;
  private static final double launcherKd = 0.00001;
  private static final double launcherKff = 0.0; // Disabled - using SysId feedforward instead

  // PathPlanner RobotConfig (placeholder - no autonomous driving)
  private final com.pathplanner.lib.config.RobotConfig ppConfig =
      new com.pathplanner.lib.config.RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              1.0, // Minimal max speed
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
    return zeroRotation;
  }

  @Override
  public Rotation2d getFrontRightZeroRotation() {
    return zeroRotation;
  }

  @Override
  public Rotation2d getBackLeftZeroRotation() {
    return zeroRotation;
  }

  @Override
  public Rotation2d getBackRightZeroRotation() {
    return zeroRotation;
  }

  @Override
  public int getPigeonCanId() {
    return pigeonCanId;
  }

  // All drive motor CAN IDs return 0 (signals "not present")
  @Override
  public int getFrontLeftDriveCanId() {
    return 0;
  }

  @Override
  public int getBackLeftDriveCanId() {
    return 0;
  }

  @Override
  public int getFrontRightDriveCanId() {
    return 0;
  }

  @Override
  public int getBackRightDriveCanId() {
    return 0;
  }

  @Override
  public int getFrontLeftTurnCanId() {
    return 0;
  }

  @Override
  public int getBackLeftTurnCanId() {
    return 0;
  }

  @Override
  public int getFrontRightTurnCanId() {
    return 0;
  }

  @Override
  public int getBackRightTurnCanId() {
    return 0;
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
    return 0.0;
  }

  @Override
  public double getDriveSimD() {
    return 0.0;
  }

  @Override
  public double getDriveSimKs() {
    return 0.0;
  }

  @Override
  public double getDriveSimKv() {
    return 0.0;
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
    return 0.0;
  }

  @Override
  public double getTurnSimD() {
    return 0.0;
  }

  @Override
  public double getTurnPIDMinInput() {
    return 0;
  }

  @Override
  public double getTurnPIDMaxInput() {
    return 2 * Math.PI;
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

  // ========== Turret Configuration Overrides ==========

  @Override
  public boolean hasTurret() {
    return true;
  }

  @Override
  public int getTurretMotorCanId() {
    return turretMotorCanId;
  }

  @Override
  public double getTurretGearRatio() {
    return turretGearRatio;
  }

  @Override
  public double getTurretHeightMeters() {
    return turretHeightMeters;
  }

  @Override
  public double getTurretMaxAngleDegrees() {
    return turretMaxAngleDegrees;
  }

  @Override
  public double getTurretMinAngleDegrees() {
    return turretMinAngleDegrees;
  }

  @Override
  public int getTurretCurrentLimitAmps() {
    return turretCurrentLimitAmps;
  }

  @Override
  public double getTurretKp() {
    return turretKp;
  }

  @Override
  public double getTurretKi() {
    return turretKi;
  }

  @Override
  public double getTurretKd() {
    return turretKd;
  }

  @Override
  public double getTurretExternalGearRatio() {
    return turretExternalGearRatio;
  }

  @Override
  public double getTurretAbsoluteEncoderOffset() {
    return turretAbsoluteEncoderOffset;
  }

  // ========== Launcher Configuration Overrides ==========

  @Override
  public boolean hasLauncher() {
    return true;
  }

  @Override
  public int getLauncherLeaderCanId() {
    return launcherLeaderCanId;
  }

  @Override
  public int getLauncherFollowerCanId() {
    return launcherFollowerCanId;
  }

  @Override
  public double getLauncherGearRatio() {
    return launcherGearRatio;
  }

  @Override
  public int getLauncherCurrentLimitAmps() {
    return launcherCurrentLimitAmps;
  }

  @Override
  public double getLauncherKp() {
    return launcherKp;
  }

  @Override
  public double getLauncherKi() {
    return launcherKi;
  }

  @Override
  public double getLauncherKd() {
    return launcherKd;
  }

  @Override
  public double getLauncherKff() {
    return launcherKff;
  }
}
