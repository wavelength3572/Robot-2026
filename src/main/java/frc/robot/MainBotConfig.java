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
 * Configuration for MainBot2026 - 31" wide x 23.5" deep chassis with NEO Vortex drive motors.
 * Intake is on the 31" (wide) front edge. Based on MainBot-2025 configuration from
 * wavelength3572/MainBot-2025
 */
public class MainBotConfig implements RobotConfig {

  // Physical dimensions (bumper-to-bumper, from PathPlanner settings.json)
  private static final double bumperLength = 0.787; // meters, front to back with bumpers
  private static final double bumperWidth = 0.978; // meters, side to side with bumpers

  private static final double trackWidth = Units.inchesToMeters(31.0);
  private static final double wheelBase = Units.inchesToMeters(23.5);
  private static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  private static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // Front Left
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // Front Right
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // Back Left
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0) // Back Right
      };

  // Module zero rotations (needs calibration for MainBot)
  private static final Rotation2d frontLeftZeroRotation = new Rotation2d(1.77948);
  private static final Rotation2d frontRightZeroRotation = new Rotation2d(1.290078);
  private static final Rotation2d backLeftZeroRotation = new Rotation2d(1.377515);
  private static final Rotation2d backRightZeroRotation = new Rotation2d(1.994175);

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

  private static final int frontleftCanCoderId = 13;
  private static final int frontRightCanCoderId = 23;
  private static final int backLeftCanCoderId = 33;
  private static final int backRightCanCoderId = 43;

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

  // Turret configuration
  private static final int turretMotorCanId = 50;
  private static final double turretHeightMeters = 0.3597275;
  private static final double turretMaxAngleDegrees = 180.0;
  private static final double turretMinAngleDegrees = -180.0;
  private static final double turretAbsoluteEncoderOffset = 0.078061;
  private static final int turretCurrentLimitAmps = 10;
  private static final double turretKp = 0.15;
  private static final double turretKd = 0.0;
  private static final boolean turretMotorInverted = true;
  private static final double turretZeroOffset = 63.873;

  // Gear ratios:
  // - NEO 550 internal gearbox: 10:1
  // - External gearing from encoder to turret: 66:12 (~5.5:1)
  // - Total ratio: 55:1 (motor rotations per turret rotation)
  // The absolute encoder sits AFTER the 10:1 gearbox but BEFORE the 66:12 external gearing
  private static final double turretExternalGearRatio = 66.0 / 12.0; // ~5.5 (encoder to turret)
  private static final double turretMotorGearRatio = 10.0;
  private static final double turretGearRatio = turretExternalGearRatio * turretMotorGearRatio;

  // Physical dimensions
  // Turret offset from robot center (in robot-relative coordinates)
  // Positive X = forward from robot center
  // Positive Y = left from robot center
  private static final double TURRET_X_OFFSET = -0.085211539; // meters
  private static final double TURRET_Y_OFFSET = 0.1819604184; // meters

  // Launcher configuration (same as TurretBot for now)
  private static final int launcherLeaderCanId = 58;
  private static final int launcherFollowerCanId = 59;
  private static final double launcherGearRatio = 1.5; // 1 motor rot = 1.5 wheel rot
  private static final int launcherCurrentLimitAmps = 80;
  private static final double launcherKp = 0.00004;
  private static final double launcherKi = 0.0;
  private static final double launcherKd = 0.003;
  private static final double launcherKv = 0.001742;
  private static final double launcherKs = 0.31;

  // Hood Configuration
  private static final int hoodMotorCanId = 60;
  private static final double hoodMinAngleDegrees = 16;
  private static final double hoodMaxAngleDegrees = 46;
  private static final int hoodCurrentLimitAmps = 40;
  private static final double hoodKp = 0.06;
  private static final double hoodKd = 0;

  // Motivator Configuration
  private static final int motivatorMotorCanId = 56;
  private static final int motivatorCurrentLimitAmps = 40;
  private static final double motivatorKp = 0.00001;
  private static final double motivatorKd = 0.0;
  private static final double motivatorKs = 0.23368;
  private static final double motivatorKv = 0.0021;
  private static final double motivatorGearRatio = 1.0 / 3.0;

  // Spindexer Configuration
  private static final int spindexerMotorCanId = 55;
  private static final int spindexerCurrentLimitAmps = 40;
  private static final double spindexerKp = 0.00001;
  private static final double spindexerKi = 0.0;
  private static final double spindexerKd = 0.0;

  private static final double spindexerKs = 0.23368;
  private static final double spindexerKv = 0.0021;
  private static final double spindexerGearRatio = 1.0 / 3.0;

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
  public double getBumperLength() {
    return bumperLength;
  }

  @Override
  public double getBumperWidth() {
    return bumperWidth;
  }

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
    return frontleftCanCoderId;
  }

  @Override
  public int getFrontRightCANCoderCanId() {
    return frontRightCanCoderId;
  }

  @Override
  public int getBackLeftCANCoderCanId() {
    return backLeftCanCoderId;
  }

  @Override
  public int getBackRightCANCoderCanId() {
    return backRightCanCoderId;
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

  @Override
  public int getTurretMotorCanId() {
    return turretMotorCanId;
  }

  @Override
  public boolean getTurretMotorInverted() {
    return turretMotorInverted;
  }

  @Override
  public double getTurretGearRatio() {
    return turretGearRatio;
  }

  @Override
  public double getTurretExternalGearRatio() {
    return turretExternalGearRatio;
  }

  @Override
  public double getTurretMotorGearRatio() {
    return turretMotorGearRatio;
  }

  @Override
  public double getTurretOffsetX() {
    return TURRET_X_OFFSET;
  }

  @Override
  public double getTurretOffsetY() {
    return TURRET_Y_OFFSET;
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
  public double getTurretAbsoluteEncoderOffset() {
    return turretAbsoluteEncoderOffset;
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
  public double getTurretKd() {
    return turretKd;
  }

  @Override
  public double getTurretZeroOffset() {
    return turretZeroOffset;
  }

  // ========== Launcher Configuration ==========

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
  public double getLauncherKv() {
    return launcherKv;
  }

  @Override
  public double getLauncherKs() {
    return launcherKs;
  }

  // ========== Hood Configuration ==========

  @Override
  public boolean hasHood() {
    return true; // MainBot has hood for hybrid trajectory control
  }

  @Override
  public int getHoodCanId() {
    return hoodMotorCanId; // TODO: Confirm CAN ID when hardware is ready
  }

  @Override
  public double getHoodMinAngleDegrees() {
    return hoodMinAngleDegrees;
  }

  @Override
  public double getHoodMaxAngleDegrees() {
    return hoodMaxAngleDegrees;
  }

  /** Hood motor current limit in amps. */
  @Override
  public int getHoodCurrentLimitAmps() {
    return hoodCurrentLimitAmps;
  }

  /** Hood kp */
  @Override
  public double getHoodKp() {
    return hoodKp;
  }

  /** Hood kd */
  @Override
  public double getHoodKd() {
    return hoodKd;
  }

  /** Hood motor Invert */
  @Override
  public boolean getHoodMotorInverted() {
    return true;
  }
  // ========== Motivator Configuration ==========

  @Override
  public boolean hasMotivator() {
    return true; // MainBot has motivator for feeding balls to launcher
  }

  /** Launcher PID proportional gain. */
  @Override
  public double getMotivatorKp() {
    return motivatorKp;
  }

  /** Launcher PID integral gain. */
  @Override
  public double getMotivatorKi() {
    return 0.0;
  }

  /** Launcher PID derivative gain. */
  @Override
  public double getMotivatorKd() {
    return motivatorKd;
  }

  /** Launcher kv gain. */
  @Override
  public double getMotivatorKv() {
    return motivatorKv;
  }

  /** Launcher ks gain. */
  @Override
  public double getMotivatorKs() {
    return motivatorKs;
  }

  @Override
  public int getMotivatorCanId() {
    return motivatorMotorCanId;
  }

  @Override
  public double getMotivatorGearRatio() {
    return motivatorGearRatio;
  }

  @Override
  public int getMotivatorCurrentLimit() {
    return motivatorCurrentLimitAmps;
  }

  // ========== Spindexer Configuration ==========

  @Override
  public boolean hasSpindexer() {
    return true; // MainBot has spindexer for feeding balls to motivator
  }

  /** Launcher PID proportional gain. */
  @Override
  public double getSpindexerKp() {
    return spindexerKp;
  }

  /** Launcher PID integral gain. */
  @Override
  public double getSpindexerKi() {
    return spindexerKi;
  }

  /** Launcher PID derivative gain. */
  @Override
  public double getSpindexerKd() {
    return spindexerKd;
  }

  /** Launcher kv gain. */
  @Override
  public double getSpindexerKv() {
    return spindexerKv;
  }

  /** Launcher ks gain. */
  @Override
  public double getSpindexerKs() {
    return spindexerKs;
  }

  @Override
  public int getSpindexerCanId() {
    return spindexerMotorCanId;
  }

  @Override
  public double getSpindexerGearRatio() {
    return spindexerGearRatio;
  }

  @Override
  public int getSpindexerCurrentLimit() {
    return spindexerCurrentLimitAmps;
  }
}
