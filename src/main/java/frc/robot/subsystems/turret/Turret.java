package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig;
import frc.robot.subsystems.hood.TrajectoryOptimizer;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TurretAimingHelper;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final RobotConfig robotConfig;

  private final double turretHeightMeters;

  // Initialize tunable limits (adjustable via NetworkTables at runtime)
  private final double configAngleMin;
  private final double configAngleMax;
  private final double centerOffsetDeg;
  private final double flipAngleDeg;

  // Visualizer for trajectory display (initialized when suppliers are set)
  private TurretVisualizer visualizer = null;
  private Supplier<Pose2d> robotPoseSupplier = null;
  private Supplier<ChassisSpeeds> fieldSpeedsSupplier = null;

  // Current shot data
  private TurretCalculator.ShotData currentShot = null;

  // Pass target offset tunables — separate for left and right trench
  private final LoggedTunableNumber passLeftAdjustX =
      new LoggedTunableNumber("Tuning/Turret/Pass/Left/AdjustX", 0.0);
  private final LoggedTunableNumber passLeftAdjustY =
      new LoggedTunableNumber("Tuning/Turret/Pass/Left/AdjustY", 0.0);
  private final LoggedTunableNumber passRightAdjustX =
      new LoggedTunableNumber("Tuning/Turret/Pass/Right/AdjustX", 0.0);
  private final LoggedTunableNumber passRightAdjustY =
      new LoggedTunableNumber("Tuning/Turret/Pass/Right/AdjustY", 0.0);

  private final LoggedTunableNumber warningZoneDeg;

  // Dual-mode range: narrower range when tracking (idle) to reduce cable wear,
  // full range only when actively launching. Inspired by 6328's tracking/launch mode.
  private final LoggedTunableNumber trackingFlipAngleDeg;
  private boolean launchModeActive = false;

  // Cached effective limits (updated when tunables change or mode switches)
  private double effectiveMinAngleDeg;
  private double effectiveMaxAngleDeg;

  // Auto-shoot: fires automatically when conditions are met (for autonomous)
  private boolean autoShootEnabled = false;
  private Supplier<Boolean> launcherReadySupplier = null;
  private Runnable onShotFiredCallback = null;
  private double lastShotTimestamp = 0.0;

  // Match shot tracking (counts persist across auto→teleop transition)
  private int totalShots = 0;
  private int autoShots = 0;
  private int teleopShots = 0;
  private final LoggedTunableNumber autoShootMinInterval =
      new LoggedTunableNumber("Tuning/Turret/AutoShootMinInterval", 0.15);

  // Pass shot launch angle (tunable for adjusting pass arc)
  private final LoggedTunableNumber passLaunchAngleDeg =
      new LoggedTunableNumber("Tuning/Turret/Pass/LaunchAngleDeg", 65.0);

  // Minimum fuel % before auto-shoot fires in PASS mode (0.0-1.0, default 0.8 = 80%)
  private final LoggedTunableNumber passFuelThreshold =
      new LoggedTunableNumber("Tuning/Turret/Pass/FuelThresholdPct", 0.8);

  // Startup safety - turret locked until operator confirms position
  private boolean operatorConfirmed = false;

  /**
   * Creates a new Turret subsystem.
   *
   * @param io The IO implementation to use (real hardware or simulation)
   */
  public Turret(TurretIO io) {
    this.io = io;

    // Log turret configuration (logged once at startup)
    robotConfig = Constants.getRobotConfig();

    this.turretHeightMeters = robotConfig.getTurretHeightMeters();

    configAngleMin = robotConfig.getTurretMinAngleDegrees();
    configAngleMax = robotConfig.getTurretMaxAngleDegrees();
    centerOffsetDeg = (configAngleMax + configAngleMin) / 2.0;
    flipAngleDeg = (configAngleMax - configAngleMin) / 2.0;

    warningZoneDeg = new LoggedTunableNumber("Tuning/Turret/WarningZoneDeg", 20.0);
    // Tracking mode uses a narrower range (10° less each side) to reduce cable stress
    trackingFlipAngleDeg =
        new LoggedTunableNumber("Tuning/Turret/TrackingFlipAngleDeg", flipAngleDeg - 10.0);

    // Calculate initial effective limits
    updateEffectiveLimits();
  }

  /** Update effective min/max limits from tunable flip angle, center offset, and active mode. */
  private void updateEffectiveLimits() {
    double center = centerOffsetDeg;
    double flipAngle = launchModeActive ? flipAngleDeg : trackingFlipAngleDeg.get();
    // Clamp to hard limits (mechanical stops)
    effectiveMinAngleDeg = Math.max(configAngleMin, center - flipAngle);
    effectiveMaxAngleDeg = Math.min(configAngleMax, center + flipAngle);
  }

  /**
   * Generate a line of 3D poses for field overlay visualization. The line extends from a start
   * point in the given direction at a specified height.
   *
   * @param startX Start X position (meters)
   * @param startY Start Y position (meters)
   * @param height Height above ground (meters)
   * @param angleDeg Direction angle in degrees (field-relative, 0 = +X, 90 = +Y)
   * @param length Length of line in meters
   * @param numPoints Number of poses along the line
   * @return Array of Pose3d representing the line
   */
  private Pose3d[] generateFieldLine3d(
      double startX, double startY, double height, double angleDeg, double length, int numPoints) {
    Pose3d[] poses = new Pose3d[numPoints];
    double angleRad = Math.toRadians(angleDeg);
    Rotation3d rotation = new Rotation3d(0, 0, angleRad);

    for (int i = 0; i < numPoints; i++) {
      double t = (double) i / (numPoints - 1); // 0 to 1
      double x = startX + t * length * Math.cos(angleRad);
      double y = startY + t * length * Math.sin(angleRad);
      poses[i] = new Pose3d(x, y, height, rotation);
    }
    return poses;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // Check if tunable limits have changed
    if (LoggedTunableNumber.hasChanged(trackingFlipAngleDeg)) {
      updateEffectiveLimits();
    }

    // Log dual-mode state
    Logger.recordOutput("Turret/LaunchModeActive", launchModeActive);

    // Log additional useful debugging values
    double angleError = inputs.targetAngleDegrees - inputs.currentAngleDegrees;
    Logger.recordOutput("Turret/AngleError", angleError);
    Logger.recordOutput("Turret/AtTarget", atTarget());

    // ========== Safety Logging ==========
    double currentAngle = inputs.currentAngleDegrees;
    double roomCW = effectiveMaxAngleDeg - currentAngle;
    double roomCCW = currentAngle - effectiveMinAngleDeg;
    double warningZone = warningZoneDeg.get();
    double dangerZone = warningZone / 2.0; // Half of warning zone is danger zone

    // Calculate position as percentage of travel (0% = CCW limit, 100% = CW limit)
    double totalRange = effectiveMaxAngleDeg - effectiveMinAngleDeg;
    double positionPercent =
        totalRange > 0 ? ((currentAngle - effectiveMinAngleDeg) / totalRange) * 100.0 : 50.0;

    // Log safety metrics (current angle already in inputs, no need to duplicate)
    boolean nearLimit = roomCW <= warningZone || roomCCW <= warningZone;
    boolean inDangerZone = roomCW <= dangerZone || roomCCW <= dangerZone;
    Logger.recordOutput("Turret/Safety/RoomCW_Deg", roomCW);
    Logger.recordOutput("Turret/Safety/RoomCCW_Deg", roomCCW);
    Logger.recordOutput("Turret/Safety/PositionPercent", positionPercent);
    Logger.recordOutput("Turret/Safety/NearLimit", nearLimit);
    Logger.recordOutput("Turret/Safety/InDangerZone", inDangerZone);

    // ========== Turret Overlay (3D field visualization) ==========
    if (robotPoseSupplier != null) {
      Pose2d robotPose = robotPoseSupplier.get();
      double robotX = robotPose.getX();
      double robotY = robotPose.getY();
      double robotHeadingDeg = robotPose.getRotation().getDegrees();
      double robotHeadingRad = robotPose.getRotation().getRadians();

      // Calculate turret's actual position on the field (offset from robot center)
      double turretX =
          robotX
              + (robotConfig.getTurretOffsetX() * Math.cos(robotHeadingRad)
                  - robotConfig.getTurretOffsetY() * Math.sin(robotHeadingRad));
      double turretY =
          robotY
              + (robotConfig.getTurretOffsetX() * Math.sin(robotHeadingRad)
                  + robotConfig.getTurretOffsetY() * Math.cos(robotHeadingRad));

      // Calculate field-relative angles
      // Turret angle is relative to robot, so add robot heading to get field-relative
      double currentFieldAngle = robotHeadingDeg + currentAngle;
      double cwLimitFieldAngle = robotHeadingDeg + effectiveMaxAngleDeg;
      double ccwLimitFieldAngle = robotHeadingDeg + effectiveMinAngleDeg;

      // Heights for 3D visualization - all at same height for consistency
      double turretHeight = 0.36; // All lines at turret height

      // Generate and log field overlay lines (emanating from turret position)
      // Current turret direction - length is distance to hub so line points at target
      boolean isBlueAlliance =
          edu.wpi.first.wpilibj.DriverStation.getAlliance()
              .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue)
              .equals(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);
      double hubX =
          isBlueAlliance
              ? FieldConstants.Hub.innerCenterPoint.getX()
              : FieldConstants.Hub.oppInnerCenterPoint.getX();
      double hubY =
          isBlueAlliance
              ? FieldConstants.Hub.innerCenterPoint.getY()
              : FieldConstants.Hub.oppInnerCenterPoint.getY();
      double distanceToHub = Math.sqrt(Math.pow(hubX - turretX, 2) + Math.pow(hubY - turretY, 2));
      Logger.recordOutput(
          "Turret/Overlay/Aim",
          generateFieldLine3d(turretX, turretY, turretHeight, currentFieldAngle, distanceToHub, 5));

      // Limit visualization logic:
      // - When centered (within 10° of center): show both limits at fixed 0.4m length
      // - When approaching one limit: show only that limit, scaled 0.4m to 0.6m
      double maxRoom = effectiveMaxAngleDeg - effectiveMinAngleDeg; // Total travel range
      double center = centerOffsetDeg;
      double centeredZoneDeg = 10.0; // Show both limits when within this many degrees of center

      // Calculate proximity to each limit (0 = far, 1 = at limit)
      double cwProximity = 1.0 - (roomCW / maxRoom);
      cwProximity = Math.max(0, Math.min(1, cwProximity)); // Clamp to 0-1
      double ccwProximity = 1.0 - (roomCCW / maxRoom);
      ccwProximity = Math.max(0, Math.min(1, ccwProximity)); // Clamp to 0-1

      boolean isCentered = Math.abs(currentAngle - center) <= centeredZoneDeg;
      boolean approachingCW = !isCentered && currentAngle > center;
      boolean approachingCCW = !isCentered && currentAngle < center;

      if (approachingCW) {
        // Only show CW limit, scaled from 0.4m to 0.6m based on proximity
        double cwLineLength = 0.4 + (cwProximity - 0.5) * 0.4; // 0.4m at 0.5 proximity, 0.6m at 1.0
        Logger.recordOutput(
            "Turret/Overlay/CWLimit",
            generateFieldLine3d(
                turretX, turretY, turretHeight, cwLimitFieldAngle, cwLineLength, 2));
        Logger.recordOutput("Turret/Overlay/CCWLimit", new Pose3d[] {});
      } else if (approachingCCW) {
        // Only show CCW limit, scaled from 0.4m to 0.6m based on proximity
        double ccwLineLength =
            0.4 + (ccwProximity - 0.5) * 0.4; // 0.4m at 0.5 proximity, 0.6m at 1.0
        Logger.recordOutput("Turret/Overlay/CWLimit", new Pose3d[] {});
        Logger.recordOutput(
            "Turret/Overlay/CCWLimit",
            generateFieldLine3d(
                turretX, turretY, turretHeight, ccwLimitFieldAngle, ccwLineLength, 2));
      } else {
        // Centered - show both limits at fixed modest length
        double fixedLength = 0.4;
        Logger.recordOutput(
            "Turret/Overlay/CWLimit",
            generateFieldLine3d(turretX, turretY, turretHeight, cwLimitFieldAngle, fixedLength, 2));
        Logger.recordOutput(
            "Turret/Overlay/CCWLimit",
            generateFieldLine3d(
                turretX, turretY, turretHeight, ccwLimitFieldAngle, fixedLength, 2));
      }

      // Center line - fixed length nub showing where center is
      double centerFieldAngle = robotHeadingDeg + centerOffsetDeg;
      Logger.recordOutput(
          "Turret/Overlay/Center",
          generateFieldLine3d(turretX, turretY, turretHeight, centerFieldAngle, 0.5, 2));

      // Log the target direction if we have a current shot
      if (currentShot != null) {
        double targetX = currentShot.getTarget().getX();
        double targetY = currentShot.getTarget().getY();
        double targetAngle = Math.toDegrees(Math.atan2(targetY - turretY, targetX - turretX));
        double distanceToTarget =
            Math.sqrt(Math.pow(targetX - turretX, 2) + Math.pow(targetY - turretY, 2));
        Logger.recordOutput(
            "Turret/Overlay/Target",
            generateFieldLine3d(
                turretX, turretY, turretHeight, targetAngle, Math.min(distanceToTarget, 3.0), 4));
      }
    }

    // Update visualizer if initialized
    if (visualizer != null) {
      visualizer.logFuelStatus();

      // Only auto-calculate shot in COMPETITION mode
      // In TEST mode, manual parameters are set by testLaunchCommand
      if (robotPoseSupplier != null
          && fieldSpeedsSupplier != null
          && !frc.robot.commands.ShootingCommands.isTestMode()) {
        // Auto-calculate shot every periodic cycle for live trajectory display
        boolean isBlue =
            edu.wpi.first.wpilibj.DriverStation.getAlliance()
                .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue)
                .equals(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);
        Pose2d robotPose = robotPoseSupplier.get();
        DriverStation.Alliance alliance =
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
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
            isBlue
                ? Constants.StrategyConstants.BLUE_PASS_TARGET_X
                : Constants.StrategyConstants.RED_PASS_TARGET_X;

        // Left trench target (with offsets, clamped to alliance zone)
        double leftRawX = baseX + passLeftAdjustX.get() * (maxX - minX) / 2.0;
        double leftRawY =
            Constants.StrategyConstants.LEFT_TRENCH_Y + passLeftAdjustY.get() * (maxY - minY) / 2.0;
        Translation3d leftTarget =
            new Translation3d(
                Math.max(minX, Math.min(maxX, leftRawX)),
                Math.max(minY, Math.min(maxY, leftRawY)),
                0.0);
        Logger.recordOutput("Turret/Pass/Left/Target", new Pose3d(leftTarget, new Rotation3d()));

        // Right trench target (with offsets, clamped to alliance zone)
        double rightRawX = baseX + passRightAdjustX.get() * (maxX - minX) / 2.0;
        double rightRawY =
            Constants.StrategyConstants.RIGHT_TRENCH_Y
                + passRightAdjustY.get() * (maxY - minY) / 2.0;
        Translation3d rightTarget =
            new Translation3d(
                Math.max(minX, Math.min(maxX, rightRawX)),
                Math.max(minY, Math.min(maxY, rightRawY)),
                0.0);
        Logger.recordOutput("Turret/Pass/Right/Target", new Pose3d(rightTarget, new Rotation3d()));

        if (aimResult.mode() == TurretAimingHelper.AimMode.SHOOT) {
          calculateShotToHub(isBlue);
        } else {
          // Pick the active pass target based on which trench side the robot is on
          boolean isLeftTrench = robotPose.getY() < fieldW / 2.0;
          Translation3d activeTarget = isLeftTrench ? leftTarget : rightTarget;
          Logger.recordOutput("Turret/Pass/Active", isLeftTrench ? "LEFT" : "RIGHT");
          calculatePassToTarget(activeTarget);
        }
      }

      // Update trajectory visualization if we have a current shot
      if (currentShot != null && robotPoseSupplier != null) {
        double currentTurretAngleRad = Math.toRadians(inputs.currentAngleDegrees);
        double robotHeadingRad = robotPoseSupplier.get().getRotation().getRadians();
        visualizer.visualizeShot(currentShot, currentTurretAngleRad, robotHeadingRad);
      }
    }

    // ========== Auto-Shoot Logic ==========
    Logger.recordOutput("Turret/AutoShootEnabled", autoShootEnabled);
    if (autoShootEnabled && launcherReadySupplier != null && robotPoseSupplier != null) {
      boolean launcherReady = launcherReadySupplier.get();
      boolean hasShot = currentShot != null;
      boolean aimed = atTarget();
      boolean hasFuel = visualizer != null && visualizer.getFuelCount() > 0;
      double now = Timer.getFPGATimestamp();
      boolean intervalElapsed = (now - lastShotTimestamp) >= autoShootMinInterval.get();

      // Zone check: only fire when in alliance zone (uses hysteresis to prevent flickering)
      Pose2d robotPose = robotPoseSupplier.get();
      DriverStation.Alliance alliance =
          DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
      TurretAimingHelper.AimResult aimResult =
          TurretAimingHelper.getAimTarget(robotPose.getX(), robotPose.getY(), alliance);
      boolean inAllianceZone = aimResult.mode() == TurretAimingHelper.AimMode.SHOOT;

      // In PASS mode, wait until fuel exceeds threshold before firing
      boolean fuelThresholdMet = true;
      if (!inAllianceZone && visualizer != null) {
        int capacity = 40; // matches TurretVisualizer.FUEL_CAPACITY
        double threshold = passFuelThreshold.get();
        fuelThresholdMet = visualizer.getFuelCount() >= (int) (capacity * threshold);
      }

      Logger.recordOutput("Turret/AutoShoot/LauncherReady", launcherReady);
      Logger.recordOutput("Turret/AutoShoot/Aimed", aimed);
      Logger.recordOutput("Turret/AutoShoot/HasFuel", hasFuel);
      Logger.recordOutput("Turret/AutoShoot/ZoneOk", inAllianceZone);
      Logger.recordOutput("Turret/AutoShoot/FuelThresholdMet", fuelThresholdMet);
      Logger.recordOutput(
          "Turret/AutoShoot/FuelRemaining", visualizer != null ? visualizer.getFuelCount() : 0);

      if (launcherReady && hasShot && aimed && hasFuel && intervalElapsed && fuelThresholdMet) {
        // Snapshot key calibration data at the instant of firing (for future LUT tuning)
        double distAtFire =
            Math.sqrt(
                Math.pow(currentShot.getTarget().getX() - robotPose.getX(), 2)
                    + Math.pow(currentShot.getTarget().getY() - robotPose.getY(), 2));
        Logger.recordOutput("Match/ShotLog/Timestamp", now);
        Logger.recordOutput("Match/ShotLog/DistanceM", distAtFire);
        Logger.recordOutput(
            "Match/ShotLog/RobotSpeedMps",
            Math.hypot(
                fieldSpeedsSupplier.get().vxMetersPerSecond,
                fieldSpeedsSupplier.get().vyMetersPerSecond));
        Logger.recordOutput("Match/ShotLog/ExitVelocityMps", currentShot.getExitVelocity());
        Logger.recordOutput("Match/ShotLog/LaunchAngleDeg", currentShot.getLaunchAngleDegrees());
        Logger.recordOutput("Match/ShotLog/TurretAngleDeg", inputs.currentAngleDegrees);

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
  }

  /**
   * Initialize the turret visualizer with robot pose and speed suppliers.
   *
   * @param poseSupplier Supplier for robot's 2D pose
   * @param speedsSupplier Supplier for field-relative chassis speeds
   */
  public void initializeVisualizer(
      Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
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
        new TurretVisualizer(
            pose3dSupplier,
            speedsSupplier,
            turretHeightMeters,
            robotConfig.getTurretOffsetX(),
            robotConfig.getTurretOffsetY());
  }

  /**
   * Calculate shot parameters for the given target and update the trajectory visualization. Also
   * aims the turret at the target.
   *
   * @param target Target position (3D)
   */
  public void calculateShotToTarget(Translation3d target) {
    if (robotPoseSupplier == null || fieldSpeedsSupplier == null) return;

    Pose2d robotPose = robotPoseSupplier.get();
    double robotHeadingRad = robotPose.getRotation().getRadians();

    // Calculate turret's actual position on the field (offset from robot center)
    double turretX =
        robotPose.getX()
            + (robotConfig.getTurretOffsetX() * Math.cos(robotHeadingRad)
                - robotConfig.getTurretOffsetY() * Math.sin(robotHeadingRad));
    double turretY =
        robotPose.getY()
            + (robotConfig.getTurretOffsetX() * Math.sin(robotHeadingRad)
                + robotConfig.getTurretOffsetY() * Math.cos(robotHeadingRad));
    Translation3d turretPos = new Translation3d(turretX, turretY, turretHeightMeters);

    // Velocity compensation: adjust aim point to counteract robot movement during flight.
    // The ball inherits robot velocity, so we "lead" the target in the opposite direction.
    // Uses iterative refinement (3 iterations) since time-of-flight depends on distance
    // which changes as we adjust the aim point.
    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
    Translation3d aimTarget = target;

    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    if (robotSpeed > 0.1) { // Only compensate when moving meaningfully
      // Initial estimate: calculate time-of-flight to the real target
      TrajectoryOptimizer.OptimalShot initialShot =
          TrajectoryOptimizer.calculateOptimalShot(turretPos, target);
      double distanceToTarget =
          Math.sqrt(Math.pow(target.getX() - turretX, 2) + Math.pow(target.getY() - turretY, 2));
      double tof =
          TurretCalculator.calculateTimeOfFlight(
              initialShot.exitVelocityMps,
              Math.toRadians(initialShot.launchAngleDeg),
              distanceToTarget);

      // Iteratively refine: adjust aim point, recalculate shot, update time-of-flight
      for (int i = 0; i < 3; i++) {
        aimTarget = TurretCalculator.predictTargetPos(target, fieldSpeeds, tof);
        double aimDistance =
            Math.sqrt(
                Math.pow(aimTarget.getX() - turretX, 2) + Math.pow(aimTarget.getY() - turretY, 2));
        TrajectoryOptimizer.OptimalShot refinedShot =
            TrajectoryOptimizer.calculateOptimalShot(turretPos, aimTarget);
        tof =
            TurretCalculator.calculateTimeOfFlight(
                refinedShot.exitVelocityMps,
                Math.toRadians(refinedShot.launchAngleDeg),
                aimDistance);
      }
    }

    // Calculate optimal shot to the compensated aim target
    TrajectoryOptimizer.OptimalShot optimalShot =
        TrajectoryOptimizer.calculateOptimalShot(turretPos, aimTarget);

    // Create shot data pointing at the compensated aim target so trajectory and ball match
    currentShot =
        new TurretCalculator.ShotData(
            optimalShot.exitVelocityMps, Math.toRadians(optimalShot.launchAngleDeg), aimTarget);

    // Aim turret at the compensated target (from turret position, not robot center)
    double azimuthRad = Math.atan2(aimTarget.getY() - turretY, aimTarget.getX() - turretX);
    double azimuthDeg = Math.toDegrees(azimuthRad);
    // Convert to robot-relative angle using closest-offset strategy
    double robotHeadingDeg = robotPose.getRotation().getDegrees();
    double relativeAngleDeg =
        calculateTurretAngle(
            robotPose.getX(),
            robotPose.getY(),
            robotHeadingDeg,
            aimTarget.getX(),
            aimTarget.getY());
    setTurretAngle(relativeAngleDeg);

    // Log shot data
    Logger.recordOutput("Turret/Shot/ExitVelocityMps", currentShot.getExitVelocity());
    Logger.recordOutput("Turret/Shot/LaunchAngleDeg", currentShot.getLaunchAngleDegrees());
    Logger.recordOutput("Turret/Shot/IdealRPM", optimalShot.rpm);
    Logger.recordOutput("Turret/Shot/Achievable", optimalShot.achievable);
    Logger.recordOutput("Turret/Shot/AzimuthDeg", azimuthDeg);
    Logger.recordOutput("Turret/Shot/RelativeAngleDeg", relativeAngleDeg);

    // Log velocity compensation
    double aimOffsetM =
        Math.sqrt(
            Math.pow(aimTarget.getX() - target.getX(), 2)
                + Math.pow(aimTarget.getY() - target.getY(), 2));
    Logger.recordOutput("Turret/Shot/VelocityCompensation/AimOffsetM", aimOffsetM);
    Logger.recordOutput("Turret/Shot/VelocityCompensation/RobotSpeedMps", robotSpeed);

    // Log real hub position (before compensation) — visible as field marker in AdvantageScope
    Logger.recordOutput("Turret/Shot/Hub", new Pose3d(target, new Rotation3d()));

    // Log distance to target
    double distanceToTarget =
        Math.sqrt(Math.pow(target.getX() - turretX, 2) + Math.pow(target.getY() - turretY, 2));
    Logger.recordOutput("Turret/Shot/DistanceToTargetM", distanceToTarget);
  }

  /**
   * Calculate shot to the alliance hub.
   *
   * @param isBlueAlliance True if targeting blue hub, false for red
   */
  public void calculateShotToHub(boolean isBlueAlliance) {
    Translation3d hubTarget =
        isBlueAlliance
            ? FieldConstants.Hub.innerCenterPoint
            : FieldConstants.Hub.oppInnerCenterPoint;
    calculateShotToTarget(hubTarget);
  }

  /**
   * Calculate shot parameters for a pass to a ground-level target. Uses simple projectile physics
   * with a tunable launch angle instead of the hub-specific TrajectoryOptimizer.
   *
   * @param target Target position (3D, Z=0 for ground level)
   */
  public void calculatePassToTarget(Translation3d target) {
    if (robotPoseSupplier == null || fieldSpeedsSupplier == null) return;

    Pose2d robotPose = robotPoseSupplier.get();
    double robotHeadingRad = robotPose.getRotation().getRadians();

    // Calculate turret's actual position on the field (offset from robot center)
    double turretX =
        robotPose.getX()
            + (robotConfig.getTurretOffsetX() * Math.cos(robotHeadingRad)
                - robotConfig.getTurretOffsetY() * Math.sin(robotHeadingRad));
    double turretY =
        robotPose.getY()
            + (robotConfig.getTurretOffsetX() * Math.sin(robotHeadingRad)
                + robotConfig.getTurretOffsetY() * Math.cos(robotHeadingRad));

    double launchAngleRad = Math.toRadians(passLaunchAngleDeg.get());
    double h = turretHeightMeters - target.getZ(); // height difference (turret above target)
    double horizontalDist =
        Math.sqrt(Math.pow(target.getX() - turretX, 2) + Math.pow(target.getY() - turretY, 2));

    // Solve for velocity: v² = g*D² / (2*cos²(θ) * (D*tan(θ) + h))
    double cosTheta = Math.cos(launchAngleRad);
    double tanTheta = Math.tan(launchAngleRad);
    double denominator = 2.0 * cosTheta * cosTheta * (horizontalDist * tanTheta + h);

    double exitVelocity;
    if (denominator > 0) {
      exitVelocity = Math.sqrt(9.81 * horizontalDist * horizontalDist / denominator);
    } else {
      exitVelocity = 10.0; // fallback
    }

    // Velocity compensation for robot movement (same pattern as calculateShotToTarget)
    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
    Translation3d aimTarget = target;

    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    if (robotSpeed > 0.1) {
      double tof =
          TurretCalculator.calculateTimeOfFlight(exitVelocity, launchAngleRad, horizontalDist);
      for (int i = 0; i < 3; i++) {
        aimTarget = TurretCalculator.predictTargetPos(target, fieldSpeeds, tof);
        double aimDist =
            Math.sqrt(
                Math.pow(aimTarget.getX() - turretX, 2) + Math.pow(aimTarget.getY() - turretY, 2));
        double aimH = turretHeightMeters - aimTarget.getZ();
        double aimDenom = 2.0 * cosTheta * cosTheta * (aimDist * tanTheta + aimH);
        if (aimDenom > 0) {
          exitVelocity = Math.sqrt(9.81 * aimDist * aimDist / aimDenom);
        }
        tof = TurretCalculator.calculateTimeOfFlight(exitVelocity, launchAngleRad, aimDist);
      }
    }

    // Create shot data
    currentShot = new TurretCalculator.ShotData(exitVelocity, launchAngleRad, aimTarget);

    // Aim turret at the compensated target
    double robotHeadingDeg = robotPose.getRotation().getDegrees();
    double relativeAngleDeg =
        calculateTurretAngle(
            robotPose.getX(),
            robotPose.getY(),
            robotHeadingDeg,
            aimTarget.getX(),
            aimTarget.getY());
    setTurretAngle(relativeAngleDeg);

    // Log pass shot data
    Logger.recordOutput("Turret/Shot/ExitVelocityMps", currentShot.getExitVelocity());
    Logger.recordOutput("Turret/Shot/LaunchAngleDeg", currentShot.getLaunchAngleDegrees());
    Logger.recordOutput("Turret/Shot/Achievable", true);
    double azimuthDeg =
        Math.toDegrees(Math.atan2(aimTarget.getY() - turretY, aimTarget.getX() - turretX));
    Logger.recordOutput("Turret/Shot/AzimuthDeg", azimuthDeg);
    Logger.recordOutput("Turret/Shot/RelativeAngleDeg", relativeAngleDeg);
    Logger.recordOutput("Turret/Shot/DistanceToTargetM", horizontalDist);

    // Log pass target marker
    Logger.recordOutput("Turret/Shot/Hub", new Pose3d(target, new Rotation3d()));

    // Log velocity compensation
    double aimOffsetM =
        Math.sqrt(
            Math.pow(aimTarget.getX() - target.getX(), 2)
                + Math.pow(aimTarget.getY() - target.getY(), 2));
    Logger.recordOutput("Turret/Shot/VelocityCompensation/AimOffsetM", aimOffsetM);
    Logger.recordOutput("Turret/Shot/VelocityCompensation/RobotSpeedMps", robotSpeed);
  }

  /** Launch a fuel ball using the current shot parameters. */
  public void launchFuel() {
    if (visualizer != null && currentShot != null && robotPoseSupplier != null) {
      // Calculate azimuth angle (field-relative direction from turret to target)
      Pose2d robotPose = robotPoseSupplier.get();
      double robotHeadingRad = robotPose.getRotation().getRadians();

      // Calculate turret's actual position on the field (offset from robot center)
      double turretX =
          robotPose.getX()
              + (robotConfig.getTurretOffsetX() * Math.cos(robotHeadingRad)
                  - robotConfig.getTurretOffsetY() * Math.sin(robotHeadingRad));
      double turretY =
          robotPose.getY()
              + (robotConfig.getTurretOffsetX() * Math.sin(robotHeadingRad)
                  + robotConfig.getTurretOffsetY() * Math.cos(robotHeadingRad));

      Translation3d target = currentShot.getTarget();
      double azimuthAngle = Math.atan2(target.getY() - turretY, target.getX() - turretX);

      // Use ideal trajectory parameters from currentShot (the calculated shot that hits the target)
      // This avoids RPM timing issues and shows where the shot SHOULD go
      visualizer.launchFuel(
          currentShot.getExitVelocity(), currentShot.getLaunchAngle(), azimuthAngle);

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

  /** Reset match shot counters. Call this when resetting the simulation/field. */
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
    // Use suppliers so currentShot is checked at runtime, not binding time
    return visualizer.repeatedlyLaunchFuel(
        () -> currentShot != null ? currentShot.getExitVelocity() : 0.0,
        () -> currentShot != null ? currentShot.getLaunchAngle() : 0.0,
        this);
  }

  /**
   * Get the current shot data.
   *
   * @return Current shot parameters, or null if not calculated
   */
  public TurretCalculator.ShotData getCurrentShot() {
    return currentShot;
  }

  /**
   * Set manual shot parameters for test mode. This overrides the auto-calculated shot and uses the
   * provided RPM and hood angle for trajectory visualization and fuel launching.
   *
   * @param launcherRPM Launcher wheel RPM (converted to exit velocity internally)
   * @param hoodAngleDeg Hood/launch angle in degrees
   * @param targetTurretAngleDeg Target turret angle in degrees (for trajectory direction)
   */
  public void setManualShotParameters(
      double launcherRPM, double hoodAngleDeg, double targetTurretAngleDeg) {
    // Convert RPM to exit velocity
    double exitVelocity = TurretCalculator.calculateExitVelocityFromRPM(launcherRPM);

    // Convert hood angle to radians
    double launchAngleRad = Math.toRadians(hoodAngleDeg);

    // Calculate target position based on TARGET turret angle (not current)
    // This shows where the shot WILL go once turret reaches position
    if (robotPoseSupplier != null) {
      Pose2d robotPose = robotPoseSupplier.get();
      double robotHeadingRad = robotPose.getRotation().getRadians();

      // Calculate turret's actual position on the field
      double turretX =
          robotPose.getX()
              + (robotConfig.getTurretOffsetX() * Math.cos(robotHeadingRad)
                  - robotConfig.getTurretOffsetY() * Math.sin(robotHeadingRad));
      double turretY =
          robotPose.getY()
              + (robotConfig.getTurretOffsetX() * Math.sin(robotHeadingRad)
                  + robotConfig.getTurretOffsetY() * Math.cos(robotHeadingRad));

      // Calculate field-relative turret aim direction using TARGET angle
      double turretAimRad = robotHeadingRad + Math.toRadians(targetTurretAngleDeg);

      // Project target 5 meters in aim direction at appropriate height
      double targetDistance = 5.0;
      double targetX = turretX + targetDistance * Math.cos(turretAimRad);
      double targetY = turretY + targetDistance * Math.sin(turretAimRad);

      // Calculate target height based on trajectory (where ball would be at that distance)
      double timeOfFlight =
          TurretCalculator.calculateTimeOfFlight(exitVelocity, launchAngleRad, targetDistance);
      double verticalVelocity = exitVelocity * Math.sin(launchAngleRad);
      double targetZ =
          turretHeightMeters
              + verticalVelocity * timeOfFlight
              - 0.5 * 9.81 * timeOfFlight * timeOfFlight;
      targetZ = Math.max(0, targetZ); // Don't go below ground

      Translation3d target = new Translation3d(targetX, targetY, targetZ);
      currentShot = new TurretCalculator.ShotData(exitVelocity, launchAngleRad, target);

      // Also update the TurretCalculator's target RPM for consistency
      TurretCalculator.setTargetLauncherRPM(launcherRPM);

      // Log manual shot parameters
      Logger.recordOutput("Turret/ManualShot/LauncherRPM", launcherRPM);
      Logger.recordOutput("Turret/ManualShot/HoodAngleDeg", hoodAngleDeg);
      Logger.recordOutput("Turret/ManualShot/ExitVelocityMps", exitVelocity);
    }
  }

  /** Clear manual shot parameters and return to auto-calculated shots. */
  public void clearManualShotParameters() {
    // The next periodic() call will recalculate the shot automatically
    currentShot = null;
    Logger.recordOutput("Turret/ManualShot/LauncherRPM", 0.0);
    Logger.recordOutput("Turret/ManualShot/HoodAngleDeg", 0.0);
  }

  /**
   * Get the visualizer instance.
   *
   * @return TurretVisualizer or null if not initialized
   */
  public TurretVisualizer getVisualizer() {
    return visualizer;
  }

  /**
   * Set the turret to point at a specific angle relative to the robot's front. The angle will be
   * clamped to the effective limits (travelCenter ± travelRange).
   *
   * @param angleDegrees Angle in degrees (positive = counter-clockwise when viewed from above)
   */
  public void setTurretAngle(double angleDegrees) {
    // Clamp to effective limits (these may be different from config limits if tuned)
    double clampedAngle =
        Math.max(effectiveMinAngleDeg, Math.min(effectiveMaxAngleDeg, angleDegrees));

    // Log when clamping occurs (only logs on clamp events, not every call)
    if (clampedAngle != angleDegrees) {
      Logger.recordOutput("Turret/Safety/ClampedRequestDeg", angleDegrees);
    }

    io.setTurretAngle(new Rotation2d(Rotation2d.fromDegrees(clampedAngle).getRadians()));
  }

  // ========== Dual-Mode Range Methods ==========

  /**
   * Enable launch mode (full turret range). Call when actively shooting to allow the turret to use
   * its full mechanical range.
   */
  public void enableLaunchMode() {
    if (!launchModeActive) {
      launchModeActive = true;
      updateEffectiveLimits();
    }
  }

  /**
   * Disable launch mode (narrower tracking range). Call when done shooting to constrain the turret
   * to a narrower range that reduces cable wear and avoids hard-stop collisions.
   */
  public void disableLaunchMode() {
    if (launchModeActive) {
      launchModeActive = false;
      updateEffectiveLimits();
    }
  }

  /**
   * Check if launch mode is active.
   *
   * @return True if turret is using full launch range
   */
  public boolean isLaunchModeActive() {
    return launchModeActive;
  }

  // ========== Auto-Shoot Methods ==========

  /**
   * Configure auto-shoot with external dependencies. Must be called before enableAutoShoot().
   * Follows the same Supplier pattern used by robotPoseSupplier/fieldSpeedsSupplier.
   *
   * @param launcherReady Supplier that returns true when the launcher is at setpoint
   * @param onShotFired Callback invoked after each auto-shot (e.g., to notify launcher for recovery
   *     sim)
   */
  public void configureAutoShoot(Supplier<Boolean> launcherReady, Runnable onShotFired) {
    this.launcherReadySupplier = launcherReady;
    this.onShotFiredCallback = onShotFired;
  }

  /** Enable auto-shoot mode. Turret will fire automatically when all conditions are met. */
  public void enableAutoShoot() {
    autoShootEnabled = true;
    enableLaunchMode();
  }

  /** Disable auto-shoot mode. Returns turret to normal tracking range. */
  public void disableAutoShoot() {
    autoShootEnabled = false;
    disableLaunchMode();
  }

  /**
   * Check if auto-shoot is currently enabled.
   *
   * @return True if auto-shoot is active
   */
  public boolean isAutoShootEnabled() {
    return autoShootEnabled;
  }

  // ========== Safety Control Methods ==========

  /**
   * Confirm that the turret position is correct. Call this after visually verifying the turret is
   * where the code thinks it is.
   */
  public void confirmPosition() {
    operatorConfirmed = true;
    System.out.println(
        "[Turret] Operator confirmed position at " + inputs.currentAngleDegrees + "°");
  }

  /**
   * Check if the operator has confirmed the turret position.
   *
   * @return True if position has been confirmed
   */
  public boolean isPositionConfirmed() {
    return operatorConfirmed;
  }

  /**
   * Get the room available to rotate clockwise (positive direction).
   *
   * @return Degrees of room until CW limit
   */
  public double getRoomCW() {
    return effectiveMaxAngleDeg - inputs.currentAngleDegrees;
  }

  /**
   * Get the room available to rotate counter-clockwise (negative direction).
   *
   * @return Degrees of room until CCW limit
   */
  public double getRoomCCW() {
    return inputs.currentAngleDegrees - effectiveMinAngleDeg;
  }

  /**
   * Check if the turret is near either limit.
   *
   * @return True if within warning zone of either limit
   */
  public boolean isNearLimit() {
    double warningZone = warningZoneDeg.get();
    return getRoomCW() <= warningZone || getRoomCCW() <= warningZone;
  }

  /**
   * Get the effective minimum angle (CCW limit).
   *
   * @return Minimum angle in degrees
   */
  public double getEffectiveMinAngle() {
    return effectiveMinAngleDeg;
  }

  /**
   * Get the effective maximum angle (CW limit).
   *
   * @return Maximum angle in degrees
   */
  public double getEffectiveMaxAngle() {
    return effectiveMaxAngleDeg;
  }

  /**
   * Calculate and set the turret angle to point at a field position.
   *
   * @param robotX Robot's X position on field (meters)
   * @param robotY Robot's Y position on field (meters)
   * @param robotOmega Robot's heading in degrees (0-360)
   * @param targetX Target X position on field (meters)
   * @param targetY Target Y position on field (meters)
   */
  public void aimAtFieldPosition(
      double robotX, double robotY, double robotOmega, double targetX, double targetY) {
    double turretAngle = calculateTurretAngle(robotX, robotY, robotOmega, targetX, targetY);
    setTurretAngle(turretAngle);
  }

  /**
   * Calculate the turret angle needed to point at a target location on the field, accounting for
   * the turret's offset from the robot's center. Uses closest-offset strategy: instead of
   * normalizing to [-180, +180], tries the target angle ±360° and picks whichever is closest to the
   * current turret position AND within the legal range. This avoids unnecessary 340°+ sweeps when a
   * short rotation in the opposite direction is legal.
   *
   * @param robotX Robot center's X position on field (meters)
   * @param robotY Robot center's Y position on field (meters)
   * @param robotOmega Robot's heading in degrees (0-360, counter-clockwise from +X axis)
   * @param targetX Target X position on field (meters)
   * @param targetY Target Y position on field (meters)
   * @return Angle in degrees the turret needs to rotate relative to robot heading, within legal
   *     range
   */
  private double calculateTurretAngle(
      double robotX, double robotY, double robotOmega, double targetX, double targetY) {
    // Convert robot heading to radians for rotation calculations
    double robotOmegaRad = Math.toRadians(robotOmega);

    // Calculate turret's actual position on the field
    // The turret offset is in robot-relative coordinates, so we need to rotate it
    // to field coordinates based on the robot's heading
    double turretFieldX =
        robotX
            + (robotConfig.getTurretOffsetX() * Math.cos(robotOmegaRad)
                - robotConfig.getTurretOffsetY() * Math.sin(robotOmegaRad));

    double turretFieldY =
        robotY
            + (robotConfig.getTurretOffsetX() * Math.sin(robotOmegaRad)
                + robotConfig.getTurretOffsetY() * Math.cos(robotOmegaRad));

    // Calculate vector from turret position to target
    double deltaX = targetX - turretFieldX;
    double deltaY = targetY - turretFieldY;

    // Calculate field-relative angle to target
    double absoluteAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

    // Calculate robot-relative angle (raw, not yet wrapped)
    double baseAngle = absoluteAngle - robotOmega;

    // Closest-offset strategy: try baseAngle at multiple 360° offsets and pick
    // the one closest to our current position that is within the legal range.
    double currentAngle = inputs.currentAngleDegrees;
    double bestAngle = Double.NaN;
    double bestDistance = Double.MAX_VALUE;

    for (int offset = -2; offset <= 2; offset++) {
      double candidate = baseAngle + offset * 360.0;
      // Check if candidate is within legal turret range
      if (candidate >= effectiveMinAngleDeg && candidate <= effectiveMaxAngleDeg) {
        double distance = Math.abs(candidate - currentAngle);
        if (distance < bestDistance) {
          bestDistance = distance;
          bestAngle = candidate;
        }
      }
    }

    // If no offset was in range, fall back to clamping the closest candidate
    if (Double.isNaN(bestAngle)) {
      // Find the candidate closest to center of legal range and clamp
      double rangeCenter = (effectiveMinAngleDeg + effectiveMaxAngleDeg) / 2.0;
      bestAngle = baseAngle;
      bestDistance = Double.MAX_VALUE;
      for (int offset = -2; offset <= 2; offset++) {
        double candidate = baseAngle + offset * 360.0;
        double distance = Math.abs(candidate - rangeCenter);
        if (distance < bestDistance) {
          bestDistance = distance;
          bestAngle = candidate;
        }
      }
      bestAngle = Math.max(effectiveMinAngleDeg, Math.min(effectiveMaxAngleDeg, bestAngle));
    }

    return bestAngle;
  }

  /**
   * Get the current turret angle.
   *
   * @return Current angle in degrees (-180 to 180)
   */
  public double getCurrentAngle() {
    return inputs.currentAngleDegrees;
  }

  /**
   * Get the target turret angle.
   *
   * @return Target angle in degrees
   */
  public double getTargetAngle() {
    return inputs.targetAngleDegrees;
  }

  /**
   * Check if the turret is at the target angle within tolerance.
   *
   * @return True if at target
   */
  public boolean atTarget() {
    return Math.abs(inputs.currentAngleDegrees - inputs.targetAngleDegrees)
        <= TurretConstants.ANGLE_TOLERANCE_DEGREES;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Turret")
  public Pose3d getPose() {
    return new Pose3d(
        robotConfig.getTurretOffsetX(),
        robotConfig.getTurretOffsetY(),
        robotConfig.getTurretHeightMeters(),
        new Rotation3d(0.0, 0.0, Rotation2d.fromDegrees(getCurrentAngle()).getRadians()));
  }

  /**
   * Get the turret height from configuration.
   *
   * @return Turret height in meters
   */
  public double getTurretHeightMeters() {
    return turretHeightMeters;
  }
}
