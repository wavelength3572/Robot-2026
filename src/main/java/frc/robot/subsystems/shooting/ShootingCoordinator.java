package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.motivator.Motivator;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RobotStatus;
import frc.robot.util.TurretAimingHelper;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Orchestrates the shooting system: turret rotation, hood angle, launcher RPM, visualization, and
 * auto-shoot logic. This is the "shooting brain" that coordinates all shot-related subsystems.
 *
 * <p>The turret subsystem only handles physical rotation. ShootingCoordinator decides WHAT to aim
 * at, WHEN to fire, and provides shot parameters for commands to act on.
 */
public class ShootingCoordinator extends SubsystemBase {

  // Subsystem references
  private final Turret turret;
  private final Hood hood;
  private final Launcher launcher;
  private final Motivator motivator;

  // Turret geometry config (immutable)
  private final ShotCalculator.TurretConfig turretConfig;

  // Shot strategy system (LUT / Parametric / Hybrid) — dropdown on dashboard
  private final SendableChooser<String> strategyChooser = new SendableChooser<>();
  private final ShotLookupTable lookupTable = new ShotLookupTable();
  private final StationaryShotBatchRecorder batchRecorder = new StationaryShotBatchRecorder();
  private final ParametricShotStrategy parametricStrategy = new ParametricShotStrategy();
  private final ParametricShotStrategy rawParametricStrategy = new ParametricShotStrategy(true);
  private final LUTShotStrategy lutStrategy;
  private final HybridShotStrategy hybridStrategy;
  private ShotStrategy activeStrategy;

  // RPM-dependent efficiency model — derived from LUT data, used by parametric & hybrid
  private final LaunchEfficiencyModel efficiencyModel;

  // Visualizer (created during initialize)
  private ShotVisualizer visualizer = null;
  private Supplier<Pose2d> robotPoseSupplier = null;
  private Supplier<ChassisSpeeds> fieldSpeedsSupplier = null;

  // Optional feeding suppression check — when true, launchFuel() is a no-op
  private BooleanSupplier feedingSuppressedSupplier = () -> false;

  // Current shot data
  private ShotCalculator.ShotResult currentShot = null;

  // Pass target offset tunables — separate for left and right trench
  private final LoggedTunableNumber passLeftAdjustX =
      new LoggedTunableNumber("Shots/Pass/Left/AdjustX", 0.0);
  private final LoggedTunableNumber passLeftAdjustY =
      new LoggedTunableNumber("Shots/Pass/Left/AdjustY", 0.0);
  private final LoggedTunableNumber passRightAdjustX =
      new LoggedTunableNumber("Shots/Pass/Right/AdjustX", 0.0);
  private final LoggedTunableNumber passRightAdjustY =
      new LoggedTunableNumber("Shots/Pass/Right/AdjustY", 0.0);

  // Pass shot launch angle (tunable for adjusting pass arc)
  private final LoggedTunableNumber passLaunchAngleDeg =
      new LoggedTunableNumber("Shots/Pass/LaunchAngleDeg", 44.0);

  // Cached pass targets — only recomputed when tunables change
  private Translation3d cachedLeftTarget = null;
  private Translation3d cachedRightTarget = null;

  // Comparison trajectories: show all strategies simultaneously in AdvantageScope
  private final LoggedTunableNumber showComparisonTrajectories =
      new LoggedTunableNumber("Shots/Visualization/ShowComparison", 0.0);

  // Auto-shoot: fires automatically when conditions are met (for autonomous)
  private boolean autoShootEnabled = false;
  private Runnable onShotFiredCallback = null;
  private double lastShotTimestamp = 0.0;

  // Match shot tracking (counts persist across auto→teleop transition)
  private int totalShots = 0;
  private int autoShots = 0;
  private int teleopShots = 0;
  private final LoggedTunableNumber autoShootMinInterval =
      new LoggedTunableNumber("BenchTest/AutoShoot/MinInterval", 0.15);
  private final LoggedTunableNumber maxFeedSpeedMps =
      new LoggedTunableNumber("Shots/SmartLaunch/MaxFeedSpeedMps", 1.75);

  /**
   * Creates a new ShootingCoordinator.
   *
   * @param turret The turret subsystem (rotation control only)
   * @param hood The hood subsystem (launch angle control), may be null
   * @param launcher The launcher subsystem (RPM control), may be null
   * @param motivator The motivator subsystem (ball feeder), may be null
   */
  public ShootingCoordinator(Turret turret, Hood hood, Launcher launcher, Motivator motivator) {
    this.turret = turret;
    this.hood = hood;
    this.launcher = launcher;
    this.motivator = motivator;

    var config = Constants.getRobotConfig();
    this.turretConfig =
        new ShotCalculator.TurretConfig(
            config.getTurretHeightMeters(), config.getTurretOffsetX(), config.getTurretOffsetY());

    // Initialize RPM-dependent efficiency model and register with ShotCalculator
    this.efficiencyModel = new LaunchEfficiencyModel(turretConfig.heightMeters());
    ShotCalculator.setEfficiencyModel(efficiencyModel);

    // Initialize shot strategies
    this.lutStrategy = new LUTShotStrategy(lookupTable);
    this.hybridStrategy = new HybridShotStrategy(lookupTable);
    this.activeStrategy = lutStrategy;

    // Strategy dropdown on dashboard
    strategyChooser.setDefaultOption("LUT (Lookup Table)", "LUT");
    strategyChooser.addOption("Parametric (Calibrated)", "Parametric");
    strategyChooser.addOption("Parametric (Raw)", "RawParametric");
    strategyChooser.addOption("Hybrid (LUT TOF + Physics RPM)", "Hybrid");
    SmartDashboard.putData("Shots/Strategy/Mode", strategyChooser);

    // Dashboard button: refit efficiency model from current LUT data
    SmartDashboard.putData(
        "Shots/Efficiency/RefitFromLUT",
        runOnce(this::refitEfficiencyFromLUT).withName("Refit Efficiency"));

    // Load any previously recorded LUT data from disk (also auto-fits efficiency)
    reloadLUTData();
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
          return new Pose3d(pose2d);
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
    DriverStation.Alliance alliance = RobotStatus.getAlliance();
    boolean isBlueAlliance = RobotStatus.isBlueAlliance();

    // Never command actuators while disabled — only run visualization
    if (DriverStation.isDisabled()) {
      if (visualizer != null && robotPoseSupplier != null) {
        visualizer.update(createVisualizerSnapshot(isBlueAlliance));
      }
      return;
    }

    // Always log robot speed for shoot-on-the-move tuning
    if (fieldSpeedsSupplier != null) {
      ChassisSpeeds speeds = fieldSpeedsSupplier.get();
      Logger.recordOutput(
          "SmartLaunch/RobotSpeedMps",
          Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    }

    updateShotCalculation(alliance, isBlueAlliance);
    runAutoShoot(alliance);
    logShotState();

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

      // Recompute pass targets only when tunables change
      if (cachedLeftTarget == null
          || LoggedTunableNumber.hasChanged(
              passLeftAdjustX, passLeftAdjustY, passRightAdjustX, passRightAdjustY)) {
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
        cachedLeftTarget =
            new Translation3d(
                Math.max(minX, Math.min(maxX, leftRawX)),
                Math.max(minY, Math.min(maxY, leftRawY)),
                0.0);
        Logger.recordOutput(
            "Turret/Pass/Left/Target", new Pose3d(cachedLeftTarget, Rotation3d.kZero));

        // Right trench target (with offsets, clamped to alliance zone)
        double rightRawX = baseX + passRightAdjustX.get() * (maxX - minX) / 2.0;
        double rightRawY =
            Constants.StrategyConstants.LEFT_PASS_TARGET_Y
                + passRightAdjustY.get() * (maxY - minY) / 2.0;
        cachedRightTarget =
            new Translation3d(
                Math.max(minX, Math.min(maxX, rightRawX)),
                Math.max(minY, Math.min(maxY, rightRawY)),
                0.0);
        Logger.recordOutput(
            "Turret/Pass/Right/Target", new Pose3d(cachedRightTarget, Rotation3d.kZero));
      }
      Translation3d leftTarget = cachedLeftTarget;
      Translation3d rightTarget = cachedRightTarget;

      if (aimResult.mode() == TurretAimingHelper.AimMode.SHOOT) {
        calculateShotToHub(robotPose, fieldSpeeds, isBlueAlliance);
      } else {
        boolean isLeftTrench = robotPose.getY() < FieldConstants.fieldWidth / 2.0;
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

  /** Calculate shot to a specific target and log results. */
  private void calculateShotToTarget(
      Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d target) {

    // Update active strategy based on tunable selector
    updateActiveStrategy();

    ShotCalculator.ShotResult result =
        activeStrategy.calculateShot(
            robotPose,
            fieldSpeeds,
            target,
            turretConfig,
            turret.getOutsideCurrentAngle(),
            turret.getMinAngle(),
            turret.getMaxAngle(),
            hood != null ? hood.getMinAngle() : 16.0,
            hood != null ? hood.getMaxAngle() : 46.0);

    Logger.recordOutput("Shots/Strategy/Active", activeStrategy.getName());
    Logger.recordOutput("Shots/Strategy/LUTDataPoints", lookupTable.size());

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
    Logger.recordOutput("Turret/Shot/IdealRPM", result.launcherRPM());
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
    Logger.recordOutput("Turret/Shot/Hub", new Pose3d(target, Rotation3d.kZero));

    // Log distance to target
    double distanceToTarget =
        Math.sqrt(Math.pow(target.getX() - turretX, 2) + Math.pow(target.getY() - turretY, 2));
    Logger.recordOutput("Turret/Shot/DistanceToTargetM", distanceToTarget);

    // Compute comparison trajectories from all strategies (when enabled)
    updateComparisonTrajectories(robotPose, fieldSpeeds, target);
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
            turret.getMinAngle(),
            turret.getMaxAngle());

    currentShot = result;

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
    Logger.recordOutput("Turret/Shot/Hub", new Pose3d(target, Rotation3d.kZero));

    // Log velocity compensation
    double aimOffsetM =
        Math.sqrt(
            Math.pow(result.aimTarget().getX() - target.getX(), 2)
                + Math.pow(result.aimTarget().getY() - target.getY(), 2));
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    Logger.recordOutput("Turret/Shot/VelocityCompensation/AimOffsetM", aimOffsetM);
    Logger.recordOutput("Turret/Shot/VelocityCompensation/RobotSpeedMps", robotSpeed);
  }

  // ========== Shot State Logging ==========

  /**
   * Log current shot state to AdvantageKit every cycle. Runs in periodic() so readiness, targets,
   * and tracking errors are always visible — regardless of which command (smart launch, fixed shot,
   * auto-track, or none) is active.
   */
  private void logShotState() {
    // --- Readiness flags ---
    boolean launcherReady = launcher != null && launcher.atSetpoint();
    boolean motivatorReady = motivator == null || motivator.isMotivatorAtSetpoint();
    boolean turretReady = turret.atTarget();
    boolean hoodReady = hood == null || hood.atTarget();
    boolean achievable = currentShot != null && currentShot.achievable();

    Logger.recordOutput("SmartLaunch/Ready/Launcher", launcherReady);
    Logger.recordOutput("SmartLaunch/Ready/Motivator", motivatorReady);
    Logger.recordOutput("SmartLaunch/Ready/Turret", turretReady);
    Logger.recordOutput("SmartLaunch/Ready/Hood", hoodReady);
    Logger.recordOutput("SmartLaunch/Ready/Achievable", achievable);
    Logger.recordOutput(
        "SmartLaunch/Ready/All",
        launcherReady && motivatorReady && turretReady && hoodReady && achievable);

    // --- Efficiency model state ---
    if (efficiencyModel != null && currentShot != null) {
      Logger.recordOutput(
          "SmartLaunch/Efficiency/AtTargetRPM",
          efficiencyModel.getEfficiency(currentShot.launcherRPM()));
      Logger.recordOutput("SmartLaunch/Efficiency/Fitted", efficiencyModel.isFitted());
      Logger.recordOutput("SmartLaunch/Efficiency/DataPoints", efficiencyModel.getDataPointCount());
    }

    if (currentShot != null) {
      double targetRPM = currentShot.launcherRPM();

      // --- Commanded targets ---
      Logger.recordOutput("SmartLaunch/Target/RPM", targetRPM);
      Logger.recordOutput("SmartLaunch/Target/TurretDeg", currentShot.turretAngleDeg());
      Logger.recordOutput("SmartLaunch/Target/HoodDeg", currentShot.hoodAngleDeg());
      Logger.recordOutput("SmartLaunch/Target/ExitVelocityMps", currentShot.exitVelocityMps());

      // --- Tracking errors ---
      double turretError = turret.getOutsideCurrentAngle() - currentShot.turretAngleDeg();
      Logger.recordOutput("SmartLaunch/Error/TurretDeg", turretError);

      if (launcher != null) {
        Logger.recordOutput("SmartLaunch/Error/LauncherRPM", launcher.getVelocity() - targetRPM);
      }

      if (hood != null) {
        double hoodError = hood.getCurrentAngle() - currentShot.hoodAngleDeg();
        Logger.recordOutput("SmartLaunch/Error/HoodDeg", hoodError);
      }
    }
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

    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
    double robotSpeedMps = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    boolean robotSlow = !inAllianceZone || robotSpeedMps <= maxFeedSpeedMps.get();

    Logger.recordOutput("Turret/AutoShoot/LauncherReady", launcherReady);
    Logger.recordOutput("Turret/AutoShoot/Aimed", aimed);
    Logger.recordOutput("Turret/AutoShoot/HasFuel", hasFuel);
    Logger.recordOutput("Turret/AutoShoot/ZoneOk", zoneOk);
    Logger.recordOutput("Turret/AutoShoot/AimMode", aimResult.mode().name());
    Logger.recordOutput("Turret/AutoShoot/RobotSpeedMps", robotSpeedMps);
    Logger.recordOutput("Turret/AutoShoot/RobotSlow", robotSlow);
    Logger.recordOutput(
        "Turret/AutoShoot/FuelRemaining", visualizer != null ? visualizer.getFuelCount() : 0);

    if (launcherReady && hasShot && aimed && hasFuel && intervalElapsed && robotSlow && zoneOk) {
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

  /** Set a supplier that, when true, prevents launchFuel() from firing. */
  public void setFeedingSuppressedSupplier(BooleanSupplier supplier) {
    this.feedingSuppressedSupplier = supplier;
  }

  /** Launch a fuel ball using the current shot parameters. */
  public void launchFuel() {
    if (feedingSuppressedSupplier.getAsBoolean()) return;
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
  }

  /** Disable auto-shoot mode. */
  public void disableAutoShoot() {
    autoShootEnabled = false;
  }

  /**
   * Check if auto-shoot is currently enabled.
   *
   * @return True if auto-shoot is active
   */
  public boolean isAutoShootEnabled() {
    return autoShootEnabled;
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

  // ========== Shot Strategy Management ==========

  /** Update the active strategy based on the dashboard dropdown. */
  private void updateActiveStrategy() {
    String selected = strategyChooser.getSelected();
    if (selected == null) selected = "LUT";

    ShotStrategy newStrategy =
        switch (selected) {
          case "LUT" -> lutStrategy;
          case "Hybrid" -> hybridStrategy;
          case "RawParametric" -> rawParametricStrategy;
          default -> parametricStrategy;
        };

    if (newStrategy != activeStrategy) {
      System.out.println("[ShootingCoordinator] Strategy changed to: " + newStrategy.getName());
      activeStrategy = newStrategy;
    }
  }

  /**
   * Compute comparison trajectories from all strategies and pass to visualizer. Only runs when the
   * comparison toggle is enabled (Shots/Visualization/ShowComparison = 1). Each non-active strategy
   * gets its own trajectory log key for independent coloring in AdvantageScope.
   */
  private void updateComparisonTrajectories(
      Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d target) {
    if (visualizer == null) return;

    boolean enabled = showComparisonTrajectories.get() > 0.5;
    if (!enabled) {
      visualizer.clearComparisonTrajectories();
      return;
    }

    double currentTurretAngle = turret.getOutsideCurrentAngle();
    double minAngle = turret.getMinAngle();
    double maxAngle = turret.getMaxAngle();
    double hoodMin = hood != null ? hood.getMinAngle() : 16.0;
    double hoodMax = hood != null ? hood.getMaxAngle() : 46.0;

    // Compute each strategy's result (skip the active one — already shown as main trajectory)
    ShotCalculator.ShotResult lutShot = null;
    ShotCalculator.ShotResult parametricShot = null;
    ShotCalculator.ShotResult rawParametricShot = null;

    if (activeStrategy != lutStrategy) {
      try {
        lutShot =
            lutStrategy.calculateShot(
                robotPose, fieldSpeeds, target, turretConfig, currentTurretAngle, minAngle,
                maxAngle, hoodMin, hoodMax);
      } catch (Exception e) {
        // LUT may not have data — that's fine
      }
    }

    if (activeStrategy != parametricStrategy) {
      parametricShot =
          parametricStrategy.calculateShot(
              robotPose, fieldSpeeds, target, turretConfig, currentTurretAngle, minAngle, maxAngle,
              hoodMin, hoodMax);
    }

    if (activeStrategy != rawParametricStrategy) {
      rawParametricShot =
          rawParametricStrategy.calculateShot(
              robotPose, fieldSpeeds, target, turretConfig, currentTurretAngle, minAngle, maxAngle,
              hoodMin, hoodMax);
    }

    // Log comparison RPM/hood for easy numeric comparison on dashboard
    if (lutShot != null) {
      Logger.recordOutput("Shots/Compare/LUT/RPM", lutShot.launcherRPM());
      Logger.recordOutput("Shots/Compare/LUT/HoodDeg", lutShot.hoodAngleDeg());
    }
    if (parametricShot != null) {
      Logger.recordOutput("Shots/Compare/Parametric/RPM", parametricShot.launcherRPM());
      Logger.recordOutput("Shots/Compare/Parametric/HoodDeg", parametricShot.hoodAngleDeg());
    }
    if (rawParametricShot != null) {
      Logger.recordOutput("Shots/Compare/RawParametric/RPM", rawParametricShot.launcherRPM());
      Logger.recordOutput("Shots/Compare/RawParametric/HoodDeg", rawParametricShot.hoodAngleDeg());
    }

    visualizer.visualizeComparisonTrajectories(
        lutShot, parametricShot, rawParametricShot, activeStrategy.getName());
  }

  /**
   * Reload LUT data from the recorder (disk). Call after recording new shots or at startup.
   *
   * <p>Loads only empirical data — no parametric seeding. The LUT strategy falls back to parametric
   * (with calibrated efficiency) for distances outside the empirical range.
   */
  public void reloadLUTData() {
    // Refit efficiency model from empirical data
    var empiricalEntries = batchRecorder.getLUTEntries();
    if (empiricalEntries.size() >= 2) {
      efficiencyModel.fitFromLUTData(empiricalEntries);
    }

    // Pure empirical LUT — no parametric seeding
    lookupTable.clear();
    lookupTable.addFromLUTEntries(empiricalEntries);

    System.out.println(
        "[ShootingCoordinator] LUT reloaded: "
            + empiricalEntries.size()
            + " empirical entries"
            + (efficiencyModel.isFitted()
                ? " (efficiency fitted from " + efficiencyModel.getDataPointCount() + " points)"
                : " (using default efficiency)"));
  }

  /**
   * Refit the efficiency model from the current LUT data. Called from the dashboard button or
   * programmatically after recording new shots.
   */
  public void refitEfficiencyFromLUT() {
    var entries = batchRecorder.getLUTEntries();
    int pointCount = efficiencyModel.fitFromLUTData(entries);
    if (pointCount > 0) {
      // Re-seed parametric entries with updated efficiency
      reloadLUTData();
    }
  }

  /** Get the batch recorder for recording new data collection sessions. */
  public StationaryShotBatchRecorder getBatchRecorder() {
    return batchRecorder;
  }

  /** Get the lookup table (for dashboard display of entry count, etc.). */
  public ShotLookupTable getLookupTable() {
    return lookupTable;
  }

  /** Get the turret config (for external calculations like distance-to-target). */
  public ShotCalculator.TurretConfig getTurretConfig() {
    return turretConfig;
  }

  /** Get the robot pose supplier (for recording shot data). */
  public Supplier<Pose2d> getRobotPoseSupplier() {
    return robotPoseSupplier;
  }

  /** Get the field speeds supplier (for recording shot data). */
  public Supplier<ChassisSpeeds> getFieldSpeedsSupplier() {
    return fieldSpeedsSupplier;
  }

  /** Get the max robot speed for smart launch and auto-shoot feeding. */
  public double getMaxFeedSpeedMps() {
    return maxFeedSpeedMps.get();
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
        turret.getMinAngle(),
        turret.getMaxAngle(),
        turret.getOutsideCenterDeg(),
        turret.getWarningZoneDeg(),
        currentShot,
        turretConfig.heightMeters(),
        turretConfig.xOffset(),
        turretConfig.yOffset());
  }
}
