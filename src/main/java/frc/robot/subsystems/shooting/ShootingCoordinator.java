package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TurretAimingHelper;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Orchestrates the shooting system: turret rotation, hood angle, launcher RPM, visualization, and
 * auto-shoot logic. This is the "shooting brain" that coordinates all shot-related subsystems.
 *
 * <p>The turret subsystem only handles physical rotation. ShootingCoordinator decides WHAT to aim
 * at, WHEN to fire, and commands turret + hood + launcher accordingly.
 */
public class ShootingCoordinator extends SubsystemBase {

  // Subsystem references
  private final Turret turret;
  private final Hood hood;
  private final Launcher launcher;

  // Turret geometry config (immutable)
  private final ShotCalculator.TurretConfig turretConfig;

  // Visualizer (created during initialize)
  private ShotVisualizer visualizer = null;
  private Supplier<Pose2d> robotPoseSupplier = null;
  private Supplier<ChassisSpeeds> fieldSpeedsSupplier = null;

  // Current shot data
  private ShotCalculator.ShotResult currentShot = null;

  // Pass target offset tunables — separate for left and right trench
  private final LoggedTunableNumber passLeftAdjustX =
      new LoggedTunableNumber("Tuning/Turret/Pass/Left/AdjustX", 0.0);
  private final LoggedTunableNumber passLeftAdjustY =
      new LoggedTunableNumber("Tuning/Turret/Pass/Left/AdjustY", 0.0);
  private final LoggedTunableNumber passRightAdjustX =
      new LoggedTunableNumber("Tuning/Turret/Pass/Right/AdjustX", 0.0);
  private final LoggedTunableNumber passRightAdjustY =
      new LoggedTunableNumber("Tuning/Turret/Pass/Right/AdjustY", 0.0);

  // Pass shot launch angle (tunable for adjusting pass arc)
  private final LoggedTunableNumber passLaunchAngleDeg =
      new LoggedTunableNumber("Tuning/Turret/Pass/LaunchAngleDeg", 44.0);

  // Minimum fuel % before auto-shoot fires in PASS mode (0.0-1.0, default 0.8 = 80%)
  private final LoggedTunableNumber passFuelThreshold =
      new LoggedTunableNumber("Tuning/Turret/Pass/FuelThresholdPct", 0.8);

  // Auto-shoot: fires automatically when conditions are met (for autonomous)
  private boolean autoShootEnabled = false;
  private Runnable onShotFiredCallback = null;
  private double lastShotTimestamp = 0.0;

  // Match shot tracking (counts persist across auto→teleop transition)
  private int totalShots = 0;
  private int autoShots = 0;
  private int teleopShots = 0;
  private final LoggedTunableNumber autoShootMinInterval =
      new LoggedTunableNumber("Tuning/Turret/AutoShootMinInterval", 0.15);
  private final LoggedTunableNumber autoShootMaxSpeedMps =
      new LoggedTunableNumber("Tuning/Turret/AutoShootMaxSpeedMps", 0.01);

  /**
   * Creates a new ShootingCoordinator.
   *
   * @param turret The turret subsystem (rotation control only)
   * @param hood The hood subsystem (launch angle control), may be null
   * @param launcher The launcher subsystem (RPM control), may be null
   */
  public ShootingCoordinator(Turret turret, Hood hood, Launcher launcher) {
    this.turret = turret;
    this.hood = hood;
    this.launcher = launcher;

    var config = Constants.getRobotConfig();
    this.turretConfig =
        new ShotCalculator.TurretConfig(
            config.getTurretHeightMeters(), config.getTurretOffsetX(), config.getTurretOffsetY());
  }

  /**
   * Initialize the coordinator with robot pose and speed suppliers. Creates the visualizer and
   * configures auto-shoot callback.
   *
   * @param poseSupplier Supplier for robot's 2D pose
   * @param speedsSupplier Supplier for field-relative chassis speeds
   */
  public void initialize(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
    this.robotPoseSupplier = poseSupplier;
    this.fieldSpeedsSupplier = speedsSupplier;

    // Create 3D pose supplier from 2D pose
    Supplier<Pose3d> pose3dSupplier =
        () -> {
          Pose2d pose2d = poseSupplier.get();
          return new Pose3d(
              pose2d.getX(),
              pose2d.getY(),
              0,
              new Rotation3d(0, 0, pose2d.getRotation().getRadians()));
        };

    this.visualizer =
        new ShotVisualizer(
            pose3dSupplier,
            speedsSupplier,
            turretConfig.heightMeters(),
            turretConfig.xOffset(),
            turretConfig.yOffset());

    // Configure auto-shoot callback (launcher recovery notification)
    if (launcher != null) {
      this.onShotFiredCallback = launcher::notifyBallFired;
    }
  }

  @Override
  public void periodic() {
    DriverStation.Alliance alliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    boolean isBlueAlliance = (alliance == DriverStation.Alliance.Blue);

    // Never command actuators while disabled — only run visualization
    if (DriverStation.isDisabled()) {
      if (visualizer != null && robotPoseSupplier != null) {
        visualizer.update(createVisualizerSnapshot(isBlueAlliance));
      }
      return;
    }

    updateShotCalculation(alliance, isBlueAlliance);
    runAutoShoot(alliance);

    // Visualization runs AFTER all control logic (passive observer)
    if (visualizer != null && robotPoseSupplier != null) {
      visualizer.update(createVisualizerSnapshot(isBlueAlliance));
    }
  }

  // ========== Shot Calculation Dispatch ==========

  /** Dispatch shot calculation (hub or pass). */
  private void updateShotCalculation(DriverStation.Alliance alliance, boolean isBlueAlliance) {
    // Only auto-calculate shot in COMPETITION mode
    if (robotPoseSupplier != null
        && fieldSpeedsSupplier != null
        && !frc.robot.commands.ShootingCommands.isTestMode()) {
      Pose2d robotPose = robotPoseSupplier.get();
      ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
      TurretAimingHelper.AimResult aimResult =
          TurretAimingHelper.getAimTarget(robotPose.getX(), robotPose.getY(), alliance);
      Logger.recordOutput("Turret/Aim/Mode", aimResult.mode().name());

      // Always compute and log both pass targets so they're visible in AdvantageScope
      double allianceZoneX = FieldConstants.LinesVertical.allianceZone;
      double fieldW = FieldConstants.fieldWidth;
      double minX, maxX;
      if (alliance == DriverStation.Alliance.Blue) {
        minX = 0.5;
        maxX = allianceZoneX - 0.5;
      } else {
        minX = FieldConstants.fieldLength - allianceZoneX + 0.5;
        maxX = FieldConstants.fieldLength - 0.5;
      }
      double minY = 0.5;
      double maxY = fieldW - 0.5;

      double baseX =
          isBlueAlliance
              ? Constants.StrategyConstants.BLUE_PASS_TARGET_X
              : Constants.StrategyConstants.RED_PASS_TARGET_X;

      // Left trench target (with offsets, clamped to alliance zone)
      double leftRawX = baseX + passLeftAdjustX.get() * (maxX - minX) / 2.0;
      double leftRawY =
          Constants.StrategyConstants.RIGHT_PASS_TARGET_Y
              + passLeftAdjustY.get() * (maxY - minY) / 2.0;
      Translation3d leftTarget =
          new Translation3d(
              Math.max(minX, Math.min(maxX, leftRawX)),
              Math.max(minY, Math.min(maxY, leftRawY)),
              0.0);
      Logger.recordOutput("Turret/Pass/Left/Target", new Pose3d(leftTarget, new Rotation3d()));

      // Right trench target (with offsets, clamped to alliance zone)
      double rightRawX = baseX + passRightAdjustX.get() * (maxX - minX) / 2.0;
      double rightRawY =
          Constants.StrategyConstants.LEFT_PASS_TARGET_Y
              + passRightAdjustY.get() * (maxY - minY) / 2.0;
      Translation3d rightTarget =
          new Translation3d(
              Math.max(minX, Math.min(maxX, rightRawX)),
              Math.max(minY, Math.min(maxY, rightRawY)),
              0.0);
      Logger.recordOutput("Turret/Pass/Right/Target", new Pose3d(rightTarget, new Rotation3d()));

      if (aimResult.mode() == TurretAimingHelper.AimMode.SHOOT) {
        calculateShotToHub(robotPose, fieldSpeeds, isBlueAlliance);
      } else {
        boolean isLeftTrench = robotPose.getY() < fieldW / 2.0;
        Translation3d activeTarget = isLeftTrench ? leftTarget : rightTarget;
        Logger.recordOutput("Turret/Pass/Active", isLeftTrench ? "LEFT" : "RIGHT");
        calculatePassToTarget(robotPose, fieldSpeeds, activeTarget);
      }
    }
  }

  /** Calculate and apply hub shot. */
  private void calculateShotToHub(
      Pose2d robotPose, ChassisSpeeds fieldSpeeds, boolean isBlueAlliance) {
    Translation3d hubTarget =
        isBlueAlliance
            ? FieldConstants.Hub.innerCenterPoint
            : FieldConstants.Hub.oppInnerCenterPoint;
    calculateShotToTarget(robotPose, fieldSpeeds, hubTarget);
  }

  /** Calculate shot to a specific target, command turret + hood, and log results. */
  private void calculateShotToTarget(
      Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d target) {
    ShotCalculator.ShotResult result =
        ShotCalculator.calculateHubShot(
            robotPose,
            fieldSpeeds,
            target,
            turretConfig,
            turret.getOutsideCurrentAngle(),
            turret.getEffectiveMinAngle(),
            turret.getEffectiveMaxAngle());

    currentShot = result;

    // Shot parameters are calculated only — commands use getCurrentShot() to drive actuators

    // Log shot data
    double robotHeadingRad = robotPose.getRotation().getRadians();
    double[] turretFieldPos =
        ShotCalculator.getTurretFieldPosition(
            robotPose.getX(), robotPose.getY(), robotHeadingRad, turretConfig);
    double turretX = turretFieldPos[0];
    double turretY = turretFieldPos[1];

    double azimuthDeg =
        Math.toDegrees(
            Math.atan2(result.aimTarget().getY() - turretY, result.aimTarget().getX() - turretX));

    Logger.recordOutput("Turret/Shot/ExitVelocityMps", result.exitVelocityMps());
    Logger.recordOutput("Turret/Shot/HoodAngleDeg", result.hoodAngleDeg());
    Logger.recordOutput("Turret/Shot/LaunchAngleDeg", result.getLaunchAngleDegrees());
    Logger.recordOutput(
        "Turret/Shot/IdealRPM", ShotCalculator.calculateRPMForVelocity(result.exitVelocityMps()));
    Logger.recordOutput("Turret/Shot/Achievable", result.achievable());
    Logger.recordOutput("Turret/Shot/AzimuthDeg", azimuthDeg);
    Logger.recordOutput("Turret/Shot/RelativeAngleDeg", result.turretAngleDeg());

    // Log velocity compensation
    double aimOffsetM =
        Math.sqrt(
            Math.pow(result.aimTarget().getX() - target.getX(), 2)
                + Math.pow(result.aimTarget().getY() - target.getY(), 2));
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    Logger.recordOutput("Turret/Shot/VelocityCompensation/AimOffsetM", aimOffsetM);
    Logger.recordOutput("Turret/Shot/VelocityCompensation/RobotSpeedMps", robotSpeed);

    // Log real target position
    Logger.recordOutput("Turret/Shot/Hub", new Pose3d(target, new Rotation3d()));

    // Log distance to target
    double distanceToTarget =
        Math.sqrt(Math.pow(target.getX() - turretX, 2) + Math.pow(target.getY() - turretY, 2));
    Logger.recordOutput("Turret/Shot/DistanceToTargetM", distanceToTarget);
  }

  /** Calculate and apply pass shot. */
  private void calculatePassToTarget(
      Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d target) {
    ShotCalculator.ShotResult result =
        ShotCalculator.calculatePassShot(
            robotPose,
            fieldSpeeds,
            target,
            turretConfig,
            passLaunchAngleDeg.get(),
            turret.getOutsideCurrentAngle(),
            turret.getEffectiveMinAngle(),
            turret.getEffectiveMaxAngle());

    currentShot = result;

    // Command turret to aim
    // turret.setTurretAngle(result.turretAngleDeg());

    // Log pass shot data
    double robotHeadingRad = robotPose.getRotation().getRadians();
    double[] turretFieldPos =
        ShotCalculator.getTurretFieldPosition(
            robotPose.getX(), robotPose.getY(), robotHeadingRad, turretConfig);
    double turretX = turretFieldPos[0];
    double turretY = turretFieldPos[1];

    double azimuthDeg =
        Math.toDegrees(
            Math.atan2(result.aimTarget().getY() - turretY, result.aimTarget().getX() - turretX));

    Logger.recordOutput("Turret/Shot/ExitVelocityMps", result.exitVelocityMps());
    Logger.recordOutput("Turret/Shot/LaunchAngleDeg", result.getLaunchAngleDegrees());
    Logger.recordOutput("Turret/Shot/Achievable", true);
    Logger.recordOutput("Turret/Shot/AzimuthDeg", azimuthDeg);
    Logger.recordOutput("Turret/Shot/RelativeAngleDeg", result.turretAngleDeg());

    double horizontalDist =
        Math.sqrt(Math.pow(target.getX() - turretX, 2) + Math.pow(target.getY() - turretY, 2));
    Logger.recordOutput("Turret/Shot/DistanceToTargetM", horizontalDist);

    // Log pass target marker
    Logger.recordOutput("Turret/Shot/Hub", new Pose3d(target, new Rotation3d()));

    // Log velocity compensation
    double aimOffsetM =
        Math.sqrt(
            Math.pow(result.aimTarget().getX() - target.getX(), 2)
                + Math.pow(result.aimTarget().getY() - target.getY(), 2));
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    Logger.recordOutput("Turret/Shot/VelocityCompensation/AimOffsetM", aimOffsetM);
    Logger.recordOutput("Turret/Shot/VelocityCompensation/RobotSpeedMps", robotSpeed);
  }

  // ========== Auto-Shoot ==========

  /** Check auto-shoot conditions and fire when all criteria are met. */
  private void runAutoShoot(DriverStation.Alliance alliance) {
    Logger.recordOutput("Turret/AutoShootEnabled", autoShootEnabled);
    if (!autoShootEnabled || launcher == null || robotPoseSupplier == null) return;

    boolean launcherReady = launcher.atSetpoint();
    boolean hasShot = currentShot != null && currentShot.exitVelocityMps() > 0;
    boolean aimed = turret.atTarget();
    boolean hasFuel =
        Constants.currentMode != Constants.Mode.SIM
            || (visualizer != null && visualizer.getFuelCount() > 0);
    double now = Timer.getFPGATimestamp();
    boolean intervalElapsed = (now - lastShotTimestamp) >= autoShootMinInterval.get();

    // Zone check: don't fire in the transition zone
    Pose2d robotPose = robotPoseSupplier.get();
    TurretAimingHelper.AimResult aimResult =
        TurretAimingHelper.getAimTarget(robotPose.getX(), robotPose.getY(), alliance);
    boolean inAllianceZone = aimResult.mode() == TurretAimingHelper.AimMode.SHOOT;
    boolean zoneOk = aimResult.mode() != TurretAimingHelper.AimMode.HOLDFIRE;

    boolean fuelThresholdMet = true;

    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
    double robotSpeedMps = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    boolean robotSlow = !inAllianceZone || robotSpeedMps <= autoShootMaxSpeedMps.get();

    Logger.recordOutput("Turret/AutoShoot/LauncherReady", launcherReady);
    Logger.recordOutput("Turret/AutoShoot/Aimed", aimed);
    Logger.recordOutput("Turret/AutoShoot/HasFuel", hasFuel);
    Logger.recordOutput("Turret/AutoShoot/ZoneOk", zoneOk);
    Logger.recordOutput("Turret/AutoShoot/AimMode", aimResult.mode().name());
    Logger.recordOutput("Turret/AutoShoot/FuelThresholdMet", fuelThresholdMet);
    Logger.recordOutput("Turret/AutoShoot/RobotSpeedMps", robotSpeedMps);
    Logger.recordOutput("Turret/AutoShoot/RobotSlow", robotSlow);
    Logger.recordOutput(
        "Turret/AutoShoot/FuelRemaining", visualizer != null ? visualizer.getFuelCount() : 0);

    if (launcherReady
        && hasShot
        && aimed
        && hasFuel
        && intervalElapsed
        && fuelThresholdMet
        && robotSlow
        && zoneOk) {
      // Snapshot key calibration data at the instant of firing
      double distAtFire =
          Math.sqrt(
              Math.pow(currentShot.aimTarget().getX() - robotPose.getX(), 2)
                  + Math.pow(currentShot.aimTarget().getY() - robotPose.getY(), 2));
      Logger.recordOutput("Match/ShotLog/Timestamp", now);
      Logger.recordOutput("Match/ShotLog/DistanceM", distAtFire);
      Logger.recordOutput("Match/ShotLog/RobotSpeedMps", robotSpeedMps);
      Logger.recordOutput("Match/ShotLog/ExitVelocityMps", currentShot.exitVelocityMps());
      Logger.recordOutput("Match/ShotLog/LaunchAngleDeg", currentShot.getLaunchAngleDegrees());
      Logger.recordOutput("Match/ShotLog/TurretAngleDeg", turret.getOutsideCurrentAngle());

      launchFuel();
      lastShotTimestamp = now;
      if (onShotFiredCallback != null) {
        onShotFiredCallback.run();
      }
      Logger.recordOutput("Turret/AutoShoot/Fired", true);
    } else {
      Logger.recordOutput("Turret/AutoShoot/Fired", false);
    }
  }

  // ========== Launch / Fuel Management ==========

  /** Launch a fuel ball using the current shot parameters. */
  public void launchFuel() {
    if (visualizer != null && currentShot != null && robotPoseSupplier != null) {
      Pose2d robotPose = robotPoseSupplier.get();
      double robotHeadingRad = robotPose.getRotation().getRadians();

      double[] turretFieldPos =
          ShotCalculator.getTurretFieldPosition(
              robotPose.getX(), robotPose.getY(), robotHeadingRad, turretConfig);
      double turretX = turretFieldPos[0];
      double turretY = turretFieldPos[1];

      Translation3d target = currentShot.aimTarget();
      double azimuthAngle = Math.atan2(target.getY() - turretY, target.getX() - turretX);

      visualizer.launchFuel(
          currentShot.exitVelocityMps(), currentShot.launchAngleRad(), azimuthAngle);

      // Track shot counts (auto vs teleop)
      totalShots++;
      if (DriverStation.isAutonomous()) {
        autoShots++;
      } else {
        teleopShots++;
      }
      Logger.recordOutput("Match/ShotLog/TotalShots", totalShots);
      Logger.recordOutput("Match/ShotLog/AutoShots", autoShots);
      Logger.recordOutput("Match/ShotLog/TeleopShots", teleopShots);
    }
  }

  /** Reset match shot counters. */
  public void resetShotCounts() {
    totalShots = 0;
    autoShots = 0;
    teleopShots = 0;
    Logger.recordOutput("Match/ShotLog/TotalShots", 0);
    Logger.recordOutput("Match/ShotLog/AutoShots", 0);
    Logger.recordOutput("Match/ShotLog/TeleopShots", 0);
  }

  /**
   * Get a command that repeatedly launches fuel at the current target.
   *
   * @return Command that launches fuel every 0.25 seconds
   */
  public Command repeatedlyLaunchFuelCommand() {
    if (visualizer == null) {
      return runOnce(() -> {}); // No-op if visualizer not initialized
    }
    return visualizer.repeatedlyLaunchFuel(
        () -> currentShot != null ? currentShot.exitVelocityMps() : 0.0,
        () -> currentShot != null ? currentShot.launchAngleRad() : 0.0,
        this);
  }

  // ========== Shot State ==========

  /**
   * Get the current shot result.
   *
   * @return Current shot parameters, or null if not calculated
   */
  public ShotCalculator.ShotResult getCurrentShot() {
    return currentShot;
  }

  /**
   * Set manual shot parameters for test mode. Overrides auto-calculated shot.
   *
   * @param launcherRPM Launcher wheel RPM
   * @param hoodAngleDeg Hood/launch angle in degrees
   * @param targetTurretAngleDeg Target turret angle in degrees
   */
  public void setManualShotParameters(
      double launcherRPM, double hoodAngleDeg, double targetTurretAngleDeg) {
    if (robotPoseSupplier != null) {
      Pose2d robotPose = robotPoseSupplier.get();
      currentShot =
          ShotCalculator.calculateManualShot(
              robotPose, turretConfig, launcherRPM, hoodAngleDeg, targetTurretAngleDeg);

      // Update the ShotCalculator's target RPM for consistency
      ShotCalculator.setTargetLauncherRPM(launcherRPM);

      Logger.recordOutput("Turret/ManualShot/LauncherRPM", launcherRPM);
      Logger.recordOutput("Turret/ManualShot/HoodAngleDeg", hoodAngleDeg);
      Logger.recordOutput("Turret/ManualShot/ExitVelocityMps", currentShot.exitVelocityMps());
    }
  }

  /** Clear manual shot parameters and return to auto-calculated shots. */
  public void clearManualShotParameters() {
    currentShot = null;
    Logger.recordOutput("Turret/ManualShot/LauncherRPM", 0.0);
    Logger.recordOutput("Turret/ManualShot/HoodAngleDeg", 0.0);
  }

  // ========== Auto-Shoot Mode ==========

  /** Enable auto-shoot mode. Turret will fire automatically when all conditions are met. */
  public void enableAutoShoot() {
    autoShootEnabled = true;
    turret.enableLaunchMode();
  }

  /** Disable auto-shoot mode. Returns turret to normal tracking range. */
  public void disableAutoShoot() {
    autoShootEnabled = false;
    turret.disableLaunchMode();
  }

  /**
   * Check if auto-shoot is currently enabled.
   *
   * @return True if auto-shoot is active
   */
  public boolean isAutoShootEnabled() {
    return autoShootEnabled;
  }

  // ========== Facade Methods (delegate to turret) ==========

  /** Enable launch mode (full turret range). */
  public void enableLaunchMode() {
    turret.enableLaunchMode();
  }

  /** Disable launch mode (narrower tracking range). */
  public void disableLaunchMode() {
    turret.disableLaunchMode();
  }

  /**
   * Set the turret angle directly (for test mode).
   *
   * @param angleDeg Turret angle in degrees
   */
  public void setTurretAngle(double angleDeg) {
    // turret.setTurretAngle(angleDeg);
  }

  /**
   * Check if the turret is at the target angle.
   *
   * @return True if at target
   */
  public boolean turretAtTarget() {
    return turret.atTarget();
  }

  /**
   * Calculate shot to the alliance hub (for one-shot button presses).
   *
   * @param isBlueAlliance True if targeting blue hub
   */
  public void calculateShotToHub(boolean isBlueAlliance) {
    if (robotPoseSupplier == null || fieldSpeedsSupplier == null) return;
    Pose2d robotPose = robotPoseSupplier.get();
    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
    calculateShotToHub(robotPose, fieldSpeeds, isBlueAlliance);
  }

  // ========== Visualizer Access ==========

  /**
   * Get the visualizer instance.
   *
   * @return ShotVisualizer or null if not initialized
   */
  public ShotVisualizer getVisualizer() {
    return visualizer;
  }

  // ========== Snapshot Creation ==========

  /**
   * Build a read-only snapshot of current shooting state for the visualizer.
   *
   * @param isBlueAlliance True if on blue alliance
   * @return Snapshot containing all state the visualizer needs
   */
  private ShotSnapshot createVisualizerSnapshot(boolean isBlueAlliance) {
    return new ShotSnapshot(
        robotPoseSupplier.get(),
        fieldSpeedsSupplier.get(),
        isBlueAlliance,
        turret.getOutsideCurrentAngle(),
        turret.getOutsideTargetAngle(),
        turret.getEffectiveMinAngle(),
        turret.getEffectiveMaxAngle(),
        turret.getOutsideCenterDeg(),
        turret.getWarningZoneDeg(),
        currentShot,
        turretConfig.heightMeters(),
        turretConfig.xOffset(),
        turretConfig.yOffset());
  }
}
