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
 * Configuration for MainBot2026 - 31" wide x 23.5" deep chassis with NEO drive motors. Intake is on
 * the 31" (wide) front edge.
 */
public class MainBotConfig implements RobotConfig {

  // Physical dimensions (bumper-to-bumper, from PathPlanner settings.json)
  private static final double bumperLength = 0.787; // meters, front to back with bumpers
  private static final double bumperWidth = 0.978; // meters, side to side with bumpers

  private static final double trackWidth = Units.inchesToMeters(25.75);
  private static final double wheelBase = Units.inchesToMeters(18.25);
  private static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  private static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // Front Left
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // Front Right
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // Back Left
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0) // Back Right
      };

  // Module zero rotations (needs calibration for MainBot)
  private static final Rotation2d frontLeftZeroRotation = new Rotation2d(1.756408);
  private static final Rotation2d frontRightZeroRotation = new Rotation2d(1.282408);
  private static final Rotation2d backLeftZeroRotation = new Rotation2d(1.337631);
  private static final Rotation2d backRightZeroRotation = new Rotation2d(1.969631);

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
  private static final int driveMotorCurrentLimit = 45;
  private static final double wheelRadiusMeters = Units.inchesToMeters(1.983);
  private static final double driveMotorReduction =
      (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // SDS MK4i L2
  private static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  private static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction;
  private static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction;

  // Drive PID configuration
  private static final double driveKp = 0.0;
  private static final double driveKd = 0.0;
  private static final double driveKs = 0.15679;
  private static final double driveKv = 0.13069;
  private static final double driveSimP = 0.05;
  private static final double driveSimD = 0.0;
  private static final double driveSimKs = 0.04307;
  private static final double driveSimKv = 0.112;

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
  private static final double robotMassKg = 60.3;
  private static final double robotMOI = 6.883;
  private static final double wheelCOF = 1.2;

  // Turret configuration
  // Gear ratios:
  // - NEO 550 internal gearbox: 10:1
  // - External gearing from encoder to turret: 66:12 (~5.5:1)
  // - Total ratio: 55:1 (motor rotations per turret rotation)
  // The absolute encoder sits AFTER the 10:1 gearbox but BEFORE the 66:12
  // external gearing
  private static final double turretExternalGearRatio = 66.0 / 12.0; // ~5.5 (encoder to turret)
  private static final double turretMotorGearRatio = 10.0;
  private static final double turretGearRatio = turretExternalGearRatio * turretMotorGearRatio;

  private static final int turretMotorCanId = 50;
  private static final double turretHeightMeters = 0.3597275;
  private static final double turretInsideMaxAngleDeg = 180.0;
  private static final double turretInsideMinAngleDeg = -180.0;
  private static final double turretZeroOffset = 63.873;
  private static final double turretOutsideMaxAngleDeg = turretInsideMaxAngleDeg + turretZeroOffset;
  private static final double turretOutsideMinAngleDeg = turretInsideMinAngleDeg + turretZeroOffset;
  private static final double turretAbsoluteEncoderOffsetTweak =
      1.5; // 2.827154; // This is in degrees. + is CCW
  private static final double turretAbsoluteEncoderOffset =
      0.30148 + (turretAbsoluteEncoderOffsetTweak / (360.0 / turretExternalGearRatio));

  private static final int turretCurrentLimitAmps = 20;
  private static final double turretKp = 0.12;
  private static final double turretKd = 2.0;
  private static final double turretToleranceAngleDeg = 4.0;

  private static final boolean turretMotorInverted = true;

  // Physical dimensions
  // Turret offset from robot center (in robot-relative coordinates)
  // Positive X = forward from robot center
  // Positive Y = left from robot center
  private static final double TURRET_X_OFFSET = -0.085211539; // meters
  private static final double TURRET_Y_OFFSET = 0.1819604184; // meters

  // Launcher configuration
  private static final int launcherLeaderCanId = 58;
  private static final int launcherFollowerCanId = 59;
  private static final double launcherGearRatio = 1.5; // 1 motor rot = 1.5 wheel rot
  private static final int launcherCurrentLimitAmps = 80;
  private static final double launcherKp = 0.000023; // 0.00014;
  private static final double launcherKi = 0.0;
  private static final double launcherKd = 0.0; // 0.006;
  private static final double launcherKv = 0.00183;
  private static final double launcherKs = 0.198;
  private static final double launcherIZone = 100.0; // motor RPM - integral only below this error

  // Hood Configuration
  private static final int hoodMotorCanId = 60;
  private static final double hoodMinAngleDegrees = 13;
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
  private static final double motivatorKv = 0.00175;
  private static final double motivatorGearRatio = 1.0 / 3.0;

  // Spindexer Configuration
  private static final int spindexerMotorCanId = 55;
  private static final int spindexerCurrentLimitAmps = 40;
  private static final double spindexerKp = 0.00001;
  private static final double spindexerKi = 0.0;
  private static final double spindexerKd = 0.0;

  private static final double spindexerKs = 0.23368;
  private static final double spindexerKv = 0.0021;
  private static final double spindexerGearRatio = 1.0 / 9.0;

  // Intake Configuration
  private static final int intakeDeployMotorCanId = 45;
  private static final int intakeRollerMotorCanId = 46;
  private static final double intakeDeployGearRatio = 25.0;
  private static final double intakeRollerGearRatio = 2.625;
  private static final boolean intakeDeployMotorInverted = false;
  private static final boolean intakeRollerMotorInverted = true;
  // 100A is safe for short bursts (agitation UP phase is capped at 0.4s by timeout).
  // NEO stall current is ~105A; 40A breaker won't trip on sub-second spikes.
  private static final int intakeDeployCurrentLimit = 100;
  private static final int intakeRollerCurrentLimit = 40;
  private static final double intakeDeployStowedPosition = 0.0;
  private static final double intakeDeployRetractedPosition = 0.0;
  private static final double intakeDeployExtendedPosition = 0.085;
  // kP=100 caused massive oscillation because it saturates output for any error > 0.01 rotations
  // (total travel is only 0.13 rotations). REV recommends starting at kP=0.01 for rotations.
  // kP=5 gives 65% duty cycle at full travel error - strong but not saturated.
  // kD provides damping to prevent oscillation near setpoint.
  private static final double intakeDeployKp = 7.0;
  private static final double intakeDeployKi = 0.0;
  private static final double intakeDeployKd = 0.0;
  private static final double intakeDeployKs = 0.0; // Static feedforward (tune first)
  private static final double intakeDeployKv = 0.08; // Velocity feedforward (tune first)
  private static final double intakeDeployMaxVelocity = 80.0; // RPM output shaft
  private static final double intakeDeployMaxAcceleration = 80.0; // RPM/s output shaft

  private static final double intakeRollerKp = 0.00005;
  private static final double intakeRollerKi = 0.0;
  private static final double intakeRollerKd = 0.000001;
  private static final double intakeRollerKff = 0.004644;

  // Climb Configuration:
  private static final int climberMotorCanId = 30;

  // ========== Turret Tuning ==========
  private static final double turretWarningZoneDeg = 20.0;
  private static final double turretEncoderWarningThresholdDeg = 5.0;
  private static final double turretEncoderErrorThresholdDeg = 15.0;

  // ========== Launcher Tuning ==========
  private static final double launcherRecoveryArbFFPct = 0.0;
  private static final double launcherReadyToleranceRPM = 100.0;
  private static final double launcherRecoveryBoostThresholdRPM = 40.0;

  // ========== Hood Tuning ==========
  private static final double hoodReadyToleranceAngleDeg = 1.0;

  // ========== Motivator Tuning ==========
  private static final double motivatorReadyToleranceRPM = 100.0;

  // ========== Spindexer Tuning ==========
  private static final double spindexerUnclogRPM = 650.0;
  private static final double spindexerAutoUnclogStallCurrentAmps = 10.0;
  private static final double spindexerAutoUnclogStallRPMError = 20.0;
  private static final double spindexerAutoUnclogStallDurationSec = 0.2;
  private static final double spindexerAutoUnclogReverseDurationSec = 0.50;
  private static final double spindexerAutoUnclogMaxAttempts = 30;
  private static final double spindexerReciprocateRPM = 50.0;
  private static final double spindexerReciprocateIntervalSec = 1.0;
  private static final double spindexerReadyToleranceRPM = 100.0;

  // ========== Intake Tuning ==========
  private static final double intakeDeployTolerance = 0.02;
  private static final double intakeDeployHoldTolerance = 0.0005;
  private static final double intakeDeployOutputLimit = 0.75;
  private static final double intakeDeployBrakeTimeSec = 0.5;
  private static final double intakeRetractMaxVelocity = 30.0;
  private static final double intakeRetractMaxAcceleration = 50.0;
  private static final double intakeRetractOutputLimit = 0.5;
  private static final double intakeAgitationMaxVelocity = 25.0;
  private static final double intakeAgitationMaxAcceleration = 40.0;
  private static final double intakeAgitationRetractOutputLimit = 1.0;
  private static final double intakeAgitationRetractTarget = 0.035;
  private static final double intakeAgitationTimeoutSec = 0.6;
  private static final double intakeAgitationCoastTimeSec = 0.1;
  private static final double intakeAgitationFallTimeSec = 0.2;
  private static final double intakeAgitationSpeedThresholdMps = 0.05;
  private static final double intakeAgitationStationaryDwellSec = 2.0;
  private static final double intakeRollerMinDeployPosition = 0.05;

  // ========== Shot Calculation ==========
  // Two-roller efficiency (main wheel + hood roller only).
  // The motivator's fixed-speed contribution is modeled separately via
  // shotMotivatorVelocityMps so these values reflect only the roller slip.
  private static final double shotEfficiencyClose = 0.73;
  private static final double shotEfficiencyMid = 0.72;
  private static final double shotEfficiencyFar = 0.71;
  private static final double shotEfficiencyCorner = 0.691;
  // Fixed velocity added by the motivator wheel (m/s) regardless of launcher RPM.
  // Derived from practice 4-1-26: arcs peaked ~24" above the 96" constraint, meaning
  // the motivator added ~1.0 m/s to the exit speed at mid-range (θ ≈ 58°).
  // Tune on robot: if arcs are still high, increase this value; if short, decrease it.
  private static final double shotMotivatorVelocityMps = .55;
  private static final double shotHoodAngleFudgeClose = 0.0;
  private static final double shotHoodAngleFudgeMid = 0.0;
  private static final double shotHoodAngleFudgeFar = 0.0;
  private static final double shotHoodAngleFudgeCorner = 0.0;
  private static final double shotVelocityCompX = 1.0;
  private static final double shotVelocityCompY = 1.0;

  // ========== Zone Boundaries ==========
  private static final double zoneTrenchAllianceBufferM = 0.3;
  private static final double zoneCloseDist = 1.9;
  private static final double zoneMidDist = 3.5;
  private static final double zoneFarDist = 4.1;
  private static final double zoneCornerDist = 5.1;

  // ========== Fixed Height Shot Strategy ==========
  private static final double fixedHeightPeakHeightIn = 96.0;
  private static final double fixedHeightPassThroughHeightIn = 78.0;
  private static final double fixedHeightHorizontalOffsetIn = 8.0;
  private static final double fixedHeightMinRPM = 1500.0;
  private static final double fixedHeightMaxRPM = 4000.0;

  // ========== Fixed Height Pass Strategy ==========
  private static final double fixedHeightPassPeakHeightIn = 65.0;
  private static final double fixedHeightPassMinRPM = 1000.0;
  private static final double fixedHeightPassMaxRPM = 4500.0;
  private static final double fixedHeightPassHoodMinDeg = 18.0;

  // ========== Shooting Coordinator — Trench Safety ==========
  private static final double trenchHoodMaxDeg = 18.0;
  private static final double trenchSafetySpeedLimitMps = 2.0;
  private static final double trenchMovingThresholdMps = 0.6;
  private static final double trenchHoodClampSpeedMps = 0.3;
  private static final double trenchHoodUnclampSpeedMps = 0.5;

  // ========== Shooting Coordinator — Smart Launch ==========
  private static final double smartLaunchReadyTimeoutSec = 3.0;
  private static final double shootOnTheMoveSpeedMps = 1.25;
  private static final double passSpeedMps = 3.0;
  private static final double autoPassSpeedMps = 1.1;
  private static final double passMaxRPM = 4000.0;

  // ========== Shooting Coordinator — Pass Adjustments ==========
  private static final double passLeftAdjustX = 0.0;
  private static final double passLeftAdjustY = 0.0;
  private static final double passRightAdjustX = 0.0;
  private static final double passRightAdjustY = 0.0;
  private static final double symmetricArcPeakHeightMinIn = 50.0;
  private static final double symmetricArcPeakHeightMaxIn = 58.0;
  private static final double symmetricArcDistMinM = 4.0;
  private static final double symmetricArcDistMaxM = 12.0;
  private static final double passRpmPerDegCompensation = 50.0;
  private static final double passMaxRpmCompensation = 200.0;
  private static final double lobNetClearanceMarginM = 0.3;
  private static final double lobMaxPeakHeightM = 5.0;
  private static final double lobMinHubDistM = 4.0;
  private static final double lobStation1AdjustY = 0.0;
  private static final double lobStation3AdjustY = 0.0;

  // ========== Shooting Commands — Preset Shots ==========
  private static final double hubShotLauncherRPM = 2450.0;
  private static final double hubShotHoodAngleDeg = 15.0;
  private static final double hubShotTurretAngleDeg = -90.0;
  private static final double hubShotMotivatorRPM = 1300.0;
  private static final double hubShotSpindexerRPM = 325.0;
  private static final double leftTrenchLauncherRPM = 2650.0;
  private static final double leftTrenchHoodAngleDeg = 18.0;
  private static final double leftTrenchTurretAngleDeg = 186.5;
  private static final double leftTrenchMotivatorRPM = 1800.0;
  private static final double leftTrenchSpindexerRPM = 325.0;
  private static final double rightTrenchLauncherRPM = 3169.0;
  private static final double rightTrenchHoodAngleDeg = 18.0;
  private static final double rightTrenchTurretAngleDeg = -4.84;
  private static final double rightTrenchMotivatorRPM = 1800.0;
  private static final double rightTrenchSpindexerRPM = 325.0;

  // ========== Shooting Commands — Feed Ratios ==========
  private static final double motivatorLauncherRatio = 0.2;
  private static final double passingMotivatorRPM = 1650.0;
  private static final double spindexerCloseRPM = 400.0;
  private static final double spindexerFarRPM = 400.0;
  private static final double spindexerPassRPM = 550.0;

  // ========== Shooting Commands — Override Defaults ==========
  private static final double overrideLauncherRPM = 2500.0;
  private static final double overrideHoodDeg = 25.0;
  private static final double overrideMotivatorRPM = 500.0;
  private static final double overrideSpindexerRPM = 300.0;

  // ========== Dashboard Tuning Defaults ==========
  private static final double tuningLauncherVelocity = 1700.0;
  private static final double tuningMotivatorVelocity = 1000.0;
  private static final double tuningSpindexerVelocity = 1000.0;
  private static final double tuningHoodAngle = 15.0;
  private static final double tuningTurretOutsideAngle = 0.0;
  private static final double tuningIntakeDeployedVelocity = 2000.0;
  private static final double tuningIntakeRetractRollerVelocity = 1000.0;

  // ========== Drive Tuning ==========
  private static final double driveSpeedLimitRampRateMps2 = 4.0;

  // ========== Hub Shift ==========
  private static final double hubShiftPreActiveCutoffSec = 10.0;

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
  public double getTurretOutsideMaxAngleDeg() {
    return turretOutsideMaxAngleDeg;
  }

  @Override
  public double getTurretOutsideMinAngleDeg() {
    return turretOutsideMinAngleDeg;
  }

  @Override
  public double getTurretZeroOffset() {
    return turretZeroOffset;
  }

  @Override
  public double getTurretAbsoluteEncoderOffset() {
    return turretAbsoluteEncoderOffset;
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
  public double getTurretToleranceAngleDeg() {
    return turretToleranceAngleDeg;
  }

  // ========== Drive Configuration ==========

  @Override
  public boolean hasDrive() {
    return true;
  }

  // ========== Vision Configuration ==========

  @Override
  public boolean hasVision() {
    return true;
  }

  // ========== Turret Configuration ==========

  @Override
  public boolean hasTurret() {
    return true;
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

  @Override
  public double getLauncherIZone() {
    return launcherIZone;
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

  // ========== Intake Configuration ==========

  @Override
  public boolean hasIntake() {
    return true;
  }

  @Override
  public int getIntakeDeployMotorCanId() {
    return intakeDeployMotorCanId;
  }

  @Override
  public int getIntakeRollerMotorCanId() {
    return intakeRollerMotorCanId;
  }

  @Override
  public double getIntakeDeployGearRatio() {
    return intakeDeployGearRatio;
  }

  @Override
  public double getIntakeRollerGearRatio() {
    return intakeRollerGearRatio;
  }

  @Override
  public boolean getIntakeDeployMotorInverted() {
    return intakeDeployMotorInverted;
  }

  @Override
  public boolean getIntakeRollerMotorInverted() {
    return intakeRollerMotorInverted;
  }

  @Override
  public int getIntakeDeployCurrentLimit() {
    return intakeDeployCurrentLimit;
  }

  @Override
  public int getIntakeRollerCurrentLimit() {
    return intakeRollerCurrentLimit;
  }

  @Override
  public double getIntakeDeployStowedPosition() {
    return intakeDeployStowedPosition;
  }

  @Override
  public double getIntakeDeployRetractedPosition() {
    return intakeDeployRetractedPosition;
  }

  @Override
  public double getIntakeDeployExtendedPosition() {
    return intakeDeployExtendedPosition;
  }

  @Override
  public double getIntakeDeployKp() {
    return intakeDeployKp;
  }

  @Override
  public double getIntakeDeployKi() {
    return intakeDeployKi;
  }

  @Override
  public double getIntakeDeployKd() {
    return intakeDeployKd;
  }

  @Override
  public double getIntakeRollerKp() {
    return intakeRollerKp;
  }

  @Override
  public double getIntakeRollerKi() {
    return intakeRollerKi;
  }

  @Override
  public double getIntakeRollerKd() {
    return intakeRollerKd;
  }

  @Override
  public double getIntakeRollerKff() {
    return intakeRollerKff;
  }

  @Override
  public double getIntakeDeployKs() {
    return intakeDeployKs;
  }

  @Override
  public double getIntakeDeployKv() {
    return intakeDeployKv;
  }

  @Override
  public double getIntakeDeployMaxVelocity() {
    return intakeDeployMaxVelocity;
  }

  @Override
  public double getIntakeDeployMaxAcceleration() {
    return intakeDeployMaxAcceleration;
  }

  @Override
  public int getClimberCanId() {
    return climberMotorCanId;
  }

  // ========== Turret Tuning ==========

  @Override
  public double getTurretWarningZoneDeg() {
    return turretWarningZoneDeg;
  }

  @Override
  public double getTurretEncoderWarningThresholdDeg() {
    return turretEncoderWarningThresholdDeg;
  }

  @Override
  public double getTurretEncoderErrorThresholdDeg() {
    return turretEncoderErrorThresholdDeg;
  }

  // ========== Launcher Tuning ==========

  @Override
  public double getLauncherRecoveryArbFFPct() {
    return launcherRecoveryArbFFPct;
  }

  @Override
  public double getLauncherReadyToleranceRPM() {
    return launcherReadyToleranceRPM;
  }

  @Override
  public double getLauncherRecoveryBoostThresholdRPM() {
    return launcherRecoveryBoostThresholdRPM;
  }

  // ========== Hood Tuning ==========

  @Override
  public double getHoodReadyToleranceAngleDeg() {
    return hoodReadyToleranceAngleDeg;
  }

  // ========== Motivator Tuning ==========

  @Override
  public double getMotivatorReadyToleranceRPM() {
    return motivatorReadyToleranceRPM;
  }

  // ========== Spindexer Tuning ==========

  @Override
  public double getSpindexerUnclogRPM() {
    return spindexerUnclogRPM;
  }

  @Override
  public double getSpindexerAutoUnclogStallCurrentAmps() {
    return spindexerAutoUnclogStallCurrentAmps;
  }

  @Override
  public double getSpindexerAutoUnclogStallRPMError() {
    return spindexerAutoUnclogStallRPMError;
  }

  @Override
  public double getSpindexerAutoUnclogStallDurationSec() {
    return spindexerAutoUnclogStallDurationSec;
  }

  @Override
  public double getSpindexerAutoUnclogReverseDurationSec() {
    return spindexerAutoUnclogReverseDurationSec;
  }

  @Override
  public double getSpindexerAutoUnclogMaxAttempts() {
    return spindexerAutoUnclogMaxAttempts;
  }

  @Override
  public double getSpindexerReciprocateRPM() {
    return spindexerReciprocateRPM;
  }

  @Override
  public double getSpindexerReciprocateIntervalSec() {
    return spindexerReciprocateIntervalSec;
  }

  @Override
  public double getSpindexerReadyToleranceRPM() {
    return spindexerReadyToleranceRPM;
  }

  // ========== Intake Tuning ==========

  @Override
  public double getIntakeDeployTolerance() {
    return intakeDeployTolerance;
  }

  @Override
  public double getIntakeDeployHoldTolerance() {
    return intakeDeployHoldTolerance;
  }

  @Override
  public double getIntakeDeployOutputLimit() {
    return intakeDeployOutputLimit;
  }

  @Override
  public double getIntakeDeployBrakeTimeSec() {
    return intakeDeployBrakeTimeSec;
  }

  @Override
  public double getIntakeRetractMaxVelocity() {
    return intakeRetractMaxVelocity;
  }

  @Override
  public double getIntakeRetractMaxAcceleration() {
    return intakeRetractMaxAcceleration;
  }

  @Override
  public double getIntakeRetractOutputLimit() {
    return intakeRetractOutputLimit;
  }

  @Override
  public double getIntakeAgitationMaxVelocity() {
    return intakeAgitationMaxVelocity;
  }

  @Override
  public double getIntakeAgitationMaxAcceleration() {
    return intakeAgitationMaxAcceleration;
  }

  @Override
  public double getIntakeAgitationRetractOutputLimit() {
    return intakeAgitationRetractOutputLimit;
  }

  @Override
  public double getIntakeAgitationRetractTarget() {
    return intakeAgitationRetractTarget;
  }

  @Override
  public double getIntakeAgitationTimeoutSec() {
    return intakeAgitationTimeoutSec;
  }

  @Override
  public double getIntakeAgitationCoastTimeSec() {
    return intakeAgitationCoastTimeSec;
  }

  @Override
  public double getIntakeAgitationFallTimeSec() {
    return intakeAgitationFallTimeSec;
  }

  @Override
  public double getIntakeAgitationSpeedThresholdMps() {
    return intakeAgitationSpeedThresholdMps;
  }

  @Override
  public double getIntakeAgitationStationaryDwellSec() {
    return intakeAgitationStationaryDwellSec;
  }

  @Override
  public double getIntakeRollerMinDeployPosition() {
    return intakeRollerMinDeployPosition;
  }

  // ========== Shot Calculation ==========

  @Override
  public double getShotEfficiencyClose() {
    return shotEfficiencyClose;
  }

  @Override
  public double getShotEfficiencyMid() {
    return shotEfficiencyMid;
  }

  @Override
  public double getShotEfficiencyFar() {
    return shotEfficiencyFar;
  }

  @Override
  public double getShotEfficiencyCorner() {
    return shotEfficiencyCorner;
  }

  @Override
  public double getShotMotivatorVelocityMps() {
    return shotMotivatorVelocityMps;
  }

  @Override
  public double getShotHoodAngleFudgeClose() {
    return shotHoodAngleFudgeClose;
  }

  @Override
  public double getShotHoodAngleFudgeMid() {
    return shotHoodAngleFudgeMid;
  }

  @Override
  public double getShotHoodAngleFudgeFar() {
    return shotHoodAngleFudgeFar;
  }

  @Override
  public double getShotHoodAngleFudgeCorner() {
    return shotHoodAngleFudgeCorner;
  }

  @Override
  public double getShotVelocityCompX() {
    return shotVelocityCompX;
  }

  @Override
  public double getShotVelocityCompY() {
    return shotVelocityCompY;
  }

  // ========== Zone Boundaries ==========

  @Override
  public double getZoneTrenchAllianceBufferM() {
    return zoneTrenchAllianceBufferM;
  }

  @Override
  public double getZoneCloseDist() {
    return zoneCloseDist;
  }

  @Override
  public double getZoneMidDist() {
    return zoneMidDist;
  }

  @Override
  public double getZoneFarDist() {
    return zoneFarDist;
  }

  @Override
  public double getZoneCornerDist() {
    return zoneCornerDist;
  }

  // ========== Fixed Height Shot Strategy ==========

  @Override
  public double getFixedHeightPeakHeightIn() {
    return fixedHeightPeakHeightIn;
  }

  @Override
  public double getFixedHeightPassThroughHeightIn() {
    return fixedHeightPassThroughHeightIn;
  }

  @Override
  public double getFixedHeightHorizontalOffsetIn() {
    return fixedHeightHorizontalOffsetIn;
  }

  @Override
  public double getFixedHeightMinRPM() {
    return fixedHeightMinRPM;
  }

  @Override
  public double getFixedHeightMaxRPM() {
    return fixedHeightMaxRPM;
  }

  // ========== Fixed Height Pass Strategy ==========

  @Override
  public double getFixedHeightPassPeakHeightIn() {
    return fixedHeightPassPeakHeightIn;
  }

  @Override
  public double getFixedHeightPassMinRPM() {
    return fixedHeightPassMinRPM;
  }

  @Override
  public double getFixedHeightPassMaxRPM() {
    return fixedHeightPassMaxRPM;
  }

  @Override
  public double getFixedHeightPassHoodMinDeg() {
    return fixedHeightPassHoodMinDeg;
  }

  // ========== Shooting Coordinator — Trench Safety ==========

  @Override
  public double getTrenchHoodMaxDeg() {
    return trenchHoodMaxDeg;
  }

  @Override
  public double getTrenchSafetySpeedLimitMps() {
    return trenchSafetySpeedLimitMps;
  }

  @Override
  public double getTrenchMovingThresholdMps() {
    return trenchMovingThresholdMps;
  }

  @Override
  public double getTrenchHoodClampSpeedMps() {
    return trenchHoodClampSpeedMps;
  }

  @Override
  public double getTrenchHoodUnclampSpeedMps() {
    return trenchHoodUnclampSpeedMps;
  }

  // ========== Shooting Coordinator — Smart Launch ==========

  @Override
  public double getSmartLaunchReadyTimeoutSec() {
    return smartLaunchReadyTimeoutSec;
  }

  @Override
  public double getShootOnTheMoveSpeedMps() {
    return shootOnTheMoveSpeedMps;
  }

  @Override
  public double getPassSpeedMps() {
    return passSpeedMps;
  }

  @Override
  public double getAutoPassSpeedMps() {
    return autoPassSpeedMps;
  }

  @Override
  public double getPassMaxRPM() {
    return passMaxRPM;
  }

  // ========== Shooting Coordinator — Pass Adjustments ==========

  @Override
  public double getPassLeftAdjustX() {
    return passLeftAdjustX;
  }

  @Override
  public double getPassLeftAdjustY() {
    return passLeftAdjustY;
  }

  @Override
  public double getPassRightAdjustX() {
    return passRightAdjustX;
  }

  @Override
  public double getPassRightAdjustY() {
    return passRightAdjustY;
  }

  @Override
  public double getSymmetricArcPeakHeightMinIn() {
    return symmetricArcPeakHeightMinIn;
  }

  @Override
  public double getSymmetricArcPeakHeightMaxIn() {
    return symmetricArcPeakHeightMaxIn;
  }

  @Override
  public double getSymmetricArcDistMinM() {
    return symmetricArcDistMinM;
  }

  @Override
  public double getSymmetricArcDistMaxM() {
    return symmetricArcDistMaxM;
  }

  @Override
  public double getPassRpmPerDegCompensation() {
    return passRpmPerDegCompensation;
  }

  @Override
  public double getPassMaxRpmCompensation() {
    return passMaxRpmCompensation;
  }

  @Override
  public double getLobNetClearanceMarginM() {
    return lobNetClearanceMarginM;
  }

  @Override
  public double getLobMaxPeakHeightM() {
    return lobMaxPeakHeightM;
  }

  @Override
  public double getLobMinHubDistM() {
    return lobMinHubDistM;
  }

  @Override
  public double getLobStation1AdjustY() {
    return lobStation1AdjustY;
  }

  @Override
  public double getLobStation3AdjustY() {
    return lobStation3AdjustY;
  }

  // ========== Shooting Commands — Preset Shots ==========

  @Override
  public double getHubShotLauncherRPM() {
    return hubShotLauncherRPM;
  }

  @Override
  public double getHubShotHoodAngleDeg() {
    return hubShotHoodAngleDeg;
  }

  @Override
  public double getHubShotTurretAngleDeg() {
    return hubShotTurretAngleDeg;
  }

  @Override
  public double getHubShotMotivatorRPM() {
    return hubShotMotivatorRPM;
  }

  @Override
  public double getHubShotSpindexerRPM() {
    return hubShotSpindexerRPM;
  }

  @Override
  public double getLeftTrenchLauncherRPM() {
    return leftTrenchLauncherRPM;
  }

  @Override
  public double getLeftTrenchHoodAngleDeg() {
    return leftTrenchHoodAngleDeg;
  }

  @Override
  public double getLeftTrenchTurretAngleDeg() {
    return leftTrenchTurretAngleDeg;
  }

  @Override
  public double getLeftTrenchMotivatorRPM() {
    return leftTrenchMotivatorRPM;
  }

  @Override
  public double getLeftTrenchSpindexerRPM() {
    return leftTrenchSpindexerRPM;
  }

  @Override
  public double getRightTrenchLauncherRPM() {
    return rightTrenchLauncherRPM;
  }

  @Override
  public double getRightTrenchHoodAngleDeg() {
    return rightTrenchHoodAngleDeg;
  }

  @Override
  public double getRightTrenchTurretAngleDeg() {
    return rightTrenchTurretAngleDeg;
  }

  @Override
  public double getRightTrenchMotivatorRPM() {
    return rightTrenchMotivatorRPM;
  }

  @Override
  public double getRightTrenchSpindexerRPM() {
    return rightTrenchSpindexerRPM;
  }

  // ========== Shooting Commands — Feed Ratios ==========

  @Override
  public double getMotivatorLauncherRatio() {
    return motivatorLauncherRatio;
  }

  @Override
  public double getPassingMotivatorRPM() {
    return passingMotivatorRPM;
  }

  @Override
  public double getSpindexerCloseRPM() {
    return spindexerCloseRPM;
  }

  @Override
  public double getSpindexerFarRPM() {
    return spindexerFarRPM;
  }

  @Override
  public double getSpindexerPassRPM() {
    return spindexerPassRPM;
  }

  // ========== Shooting Commands — Override Defaults ==========

  @Override
  public double getOverrideLauncherRPM() {
    return overrideLauncherRPM;
  }

  @Override
  public double getOverrideHoodDeg() {
    return overrideHoodDeg;
  }

  @Override
  public double getOverrideMotivatorRPM() {
    return overrideMotivatorRPM;
  }

  @Override
  public double getOverrideSpindexerRPM() {
    return overrideSpindexerRPM;
  }

  // ========== Dashboard Tuning Defaults ==========

  @Override
  public double getTuningLauncherVelocity() {
    return tuningLauncherVelocity;
  }

  @Override
  public double getTuningMotivatorVelocity() {
    return tuningMotivatorVelocity;
  }

  @Override
  public double getTuningSpindexerVelocity() {
    return tuningSpindexerVelocity;
  }

  @Override
  public double getTuningHoodAngle() {
    return tuningHoodAngle;
  }

  @Override
  public double getTuningTurretOutsideAngle() {
    return tuningTurretOutsideAngle;
  }

  @Override
  public double getTuningIntakeDeployedVelocity() {
    return tuningIntakeDeployedVelocity;
  }

  @Override
  public double getTuningIntakeRetractRollerVelocity() {
    return tuningIntakeRetractRollerVelocity;
  }

  // ========== Drive Tuning ==========

  @Override
  public double getDriveSpeedLimitRampRateMps2() {
    return driveSpeedLimitRampRateMps2;
  }

  // ========== Hub Shift ==========

  @Override
  public double getHubShiftPreActiveCutoffSec() {
    return hubShiftPreActiveCutoffSec;
  }
}
