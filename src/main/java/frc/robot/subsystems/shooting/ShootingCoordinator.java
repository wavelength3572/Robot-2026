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

  // Shot strategy system — two completely independent modes:
  //   Parametric: physics-based, tuned via single efficiency constant
  //   LUT: pure empirical data, no physics involved
  private final SendableChooser<String> strategyChooser = new SendableChooser<>();

  /** Strategy for selecting which pass target (left vs right trench) to use. */
  public enum PassingStrategy {
    /** Pick target based on robot's Y position relative to field center. */
    SYMMETRIC,
    /** Pick target based on driver station number from FMS. */
    DRIVER_STATION
  }

  private final SendableChooser<PassingStrategy> passingStrategyChooser = new SendableChooser<>();
  private final ShotLookupTable lookupTable = new ShotLookupTable();
  private final StationaryShotBatchRecorder batchRecorder = new StationaryShotBatchRecorder();
  private final ParametricShotStrategy parametricStrategy = new ParametricShotStrategy();
  private final LUTShotStrategy lutStrategy;
  private ShotStrategy activeStrategy;

  // Visualizer (created during initialize)
  private ShotVisualizer visualizer = null;

  // Throttle counter for non-critical logging (reduces loop time)
  private int periodicCounter = 0;
  private Supplier<Pose2d> robotPoseSupplier = null;
  private Supplier<ChassisSpeeds> fieldSpeedsSupplier = null;

  // Optional feeding suppression check — when true, launchFuel() is a no-op
  private BooleanSupplier feedingSuppressedSupplier = () -> false;

  // Trench avoidance — clamps hood angle when robot is under a trench
  private final LoggedTunableNumber trenchHoodMaxDeg =
      new LoggedTunableNumber("Shots/TrenchMode/HoodMaxDeg", 18.0);
  private final LoggedTunableNumber trenchMarginMeters =
      new LoggedTunableNumber(
          "Shots/TrenchMode/MarginMeters", FieldConstants.TrenchZones.DEFAULT_MARGIN_METERS);
  private boolean trenchModeEnabled = false;
  private boolean trenchModeActive = false; // true when robot is currently in a trench zone

  // Visualizer throttle — run at 10Hz instead of 50Hz (pure display, not control)
  private int visualizerCounter = 0;
  private static final int VISUALIZER_DIVISOR = 5; // 50Hz / 5 = 10Hz

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

  // Two-point trajectory tunables for pass shots
  private final LoggedTunableNumber symmetricArcPeakHeightM =
      new LoggedTunableNumber("Shots/Pass/Symmetric/ArcPeakHeightM", 3.0);
  private final LoggedTunableNumber lobNetClearanceMarginM =
      new LoggedTunableNumber("Shots/Pass/Lob/NetClearanceMarginM", 0.3);
  private final LoggedTunableNumber lobMaxPeakHeightM =
      new LoggedTunableNumber("Shots/Pass/Lob/MaxPeakHeightM", 5.0);
  private final LoggedTunableNumber lobStation12AdjustY =
      new LoggedTunableNumber("Shots/Pass/Lob/Station12/AdjustY", 0.0);
  private final LoggedTunableNumber lobStation3AdjustY =
      new LoggedTunableNumber("Shots/Pass/Lob/Station3/AdjustY", 0.0);

  // Cached pass targets — only recomputed when tunables change
  private Translation3d cachedLeftTarget = null;
  private Translation3d cachedRightTarget = null;
  private Translation3d cachedLobStation12Target = null;
  private Translation3d cachedLobStation3Target = null;

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

    // Initialize shot strategies — completely independent, no shared efficiency model
    this.lutStrategy = new LUTShotStrategy(lookupTable);
    this.activeStrategy = parametricStrategy;

    // Strategy dropdown on dashboard — just two clean options
    strategyChooser.setDefaultOption("Parametric", "Parametric");
    strategyChooser.addOption("LUT (Lookup Table)", "LUT");
    SmartDashboard.putData("Shots/Strategy/Mode", strategyChooser);

    // Passing strategy chooser — how to pick left vs right pass target
    passingStrategyChooser.setDefaultOption("Symmetric (Y-based)", PassingStrategy.SYMMETRIC);
    passingStrategyChooser.addOption("Driver Station", PassingStrategy.DRIVER_STATION);
    SmartDashboard.putData("Shots/Pass/Strategy", passingStrategyChooser);

    // Trench mode toggle — off by default, can be enabled via dashboard or code
    SmartDashboard.putBoolean("Shots/TrenchMode/Enabled", trenchModeEnabled);

    // Load any previously recorded LUT data from disk
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

    // Advance visualizer throttle counter
    visualizerCounter++;

    // Never command actuators while disabled — only run visualization
    if (DriverStation.isDisabled()) {
      if (visualizerCounter % VISUALIZER_DIVISOR == 0
          && visualizer != null
          && robotPoseSupplier != null) {
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

    // Throttle non-critical logging to every other cycle to reduce loop time
    periodicCounter++;
    if (periodicCounter % 2 == 0) {
      logShotState();
    }

    // Visualization runs AFTER all control logic (passive observer, throttled to 10Hz)
    if (visualizerCounter % VISUALIZER_DIVISOR == 0
        && visualizer != null
        && robotPoseSupplier != null) {
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

      // Sync trench mode enabled state from dashboard toggle
      trenchModeEnabled = SmartDashboard.getBoolean("Shots/TrenchMode/Enabled", trenchModeEnabled);

      // Check if robot is in any trench zone (all 4, alliance-independent)
      trenchModeActive =
          trenchModeEnabled
              && FieldConstants.TrenchZones.isInAnyTrenchZone(
                  robotPose.getX(), robotPose.getY(), trenchMarginMeters.get());
      Logger.recordOutput("Shots/TrenchMode/Enabled", trenchModeEnabled);
      Logger.recordOutput("Shots/TrenchMode/Active", trenchModeActive);
      if (trenchModeActive) {
        Logger.recordOutput(
            "Shots/TrenchMode/Zone",
            FieldConstants.TrenchZones.getActiveTrenchZone(
                robotPose.getX(), robotPose.getY(), trenchMarginMeters.get()));
      }

      TurretAimingHelper.AimResult aimResult =
          TurretAimingHelper.getAimTarget(robotPose.getX(), robotPose.getY(), alliance);
      Logger.recordOutput("Turret/Aim/Mode", aimResult.mode().name());

      // Shared alliance-zone X bounds (used by both strategies)
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

      // Recompute symmetric pass targets only when tunables change
      if (cachedLeftTarget == null
          || LoggedTunableNumber.hasChanged(
              passLeftAdjustX, passLeftAdjustY, passRightAdjustX, passRightAdjustY)) {
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

      // Recompute lob pass targets only when lob tunables change
      if (cachedLobStation12Target == null
          || LoggedTunableNumber.hasChanged(lobStation12AdjustY, lobStation3AdjustY)) {
        // Station 1/2 target (blue = low Y, red = high Y)
        double st12BaseY =
            isBlueAlliance
                ? Constants.StrategyConstants.LOB_STATION_1_2_TARGET_Y
                : fieldW - Constants.StrategyConstants.LOB_STATION_1_2_TARGET_Y;
        double st12RawY = st12BaseY + lobStation12AdjustY.get() * (maxY - minY) / 2.0;
        cachedLobStation12Target =
            new Translation3d(
                Math.max(minX, Math.min(maxX, baseX)),
                Math.max(minY, Math.min(maxY, st12RawY)),
                0.0);
        Logger.recordOutput(
            "Turret/Pass/Lob/Station12/Target",
            new Pose3d(cachedLobStation12Target, Rotation3d.kZero));

        // Station 3 target (blue = high Y near outpost, red = low Y near outpost)
        double st3BaseY =
            isBlueAlliance
                ? Constants.StrategyConstants.LOB_STATION_3_TARGET_Y
                : fieldW - Constants.StrategyConstants.LOB_STATION_3_TARGET_Y;
        double st3RawY = st3BaseY + lobStation3AdjustY.get() * (maxY - minY) / 2.0;
        cachedLobStation3Target =
            new Translation3d(
                Math.max(minX, Math.min(maxX, baseX)),
                Math.max(minY, Math.min(maxY, st3RawY)),
                0.0);
        Logger.recordOutput(
            "Turret/Pass/Lob/Station3/Target",
            new Pose3d(cachedLobStation3Target, Rotation3d.kZero));
      }

      if (aimResult.mode() == TurretAimingHelper.AimMode.SHOOT) {
        calculateShotToHub(robotPose, fieldSpeeds, isBlueAlliance);
      } else {
        PassingStrategy strategy = passingStrategyChooser.getSelected();
        Logger.recordOutput("Turret/Pass/StrategyUsed", strategy.name());

        if (strategy == PassingStrategy.DRIVER_STATION) {
          // Lob pass: pick target based on driver station number, use steep launch angle
          var location = DriverStation.getLocation();
          int station = location.isPresent() ? location.getAsInt() : 2;
          Translation3d activeTarget =
              (station <= 2) ? cachedLobStation12Target : cachedLobStation3Target;
          Logger.recordOutput(
              "Turret/Pass/Active", (station <= 2) ? "LOB_STATION_1_2" : "LOB_STATION_3");
          calculatePassToTarget(
              robotPose, fieldSpeeds, activeTarget, PassingStrategy.DRIVER_STATION);
        } else {
          // Symmetric pass: pick target based on robot Y position
          boolean isLeftTrench = selectIsLeftTrench(robotPose);
          Translation3d activeTarget = isLeftTrench ? cachedLeftTarget : cachedRightTarget;
          Logger.recordOutput("Turret/Pass/Active", isLeftTrench ? "LEFT" : "RIGHT");
          calculatePassToTarget(robotPose, fieldSpeeds, activeTarget, PassingStrategy.SYMMETRIC);
        }
      }
    }
  }

  /**
   * Select whether to pass to the left trench based on the active passing strategy.
   *
   * <p>SYMMETRIC: picks based on robot Y position relative to field center. DRIVER_STATION: picks
   * based on FMS driver station number (1=left, 3=right, 2=fallback to symmetric).
   */
  private boolean selectIsLeftTrench(Pose2d robotPose) {
    PassingStrategy strategy = passingStrategyChooser.getSelected();
    if (strategy == PassingStrategy.DRIVER_STATION) {
      var location = DriverStation.getLocation();
      if (location.isPresent()) {
        int station = location.getAsInt();
        // Station numbering mirrors between alliances because drivers face
        // opposite directions. Station 1 is high Y for blue, low Y for red.
        // Station 2 is grouped with station 1. Tower is between 2 and 3.
        boolean isBlue = RobotStatus.isBlueAlliance();
        if (station <= 2) return isBlue;
        if (station == 3) return !isBlue;
      }
    }
    // Default: symmetric Y-based selection
    return robotPose.getY() < FieldConstants.fieldWidth / 2.0;
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

    double hoodMin = hood != null ? hood.getMinAngle() : 16.0;
    double hoodMax = hood != null ? hood.getMaxAngle() : 46.0;
    if (trenchModeActive) {
      hoodMax = Math.min(hoodMax, trenchHoodMaxDeg.get());
    }

    ShotCalculator.ShotResult result =
        activeStrategy.calculateShot(
            robotPose,
            fieldSpeeds,
            target,
            turretConfig,
            turret.getOutsideCurrentAngle(),
            turret.getMinAngle(),
            turret.getMaxAngle(),
            hoodMin,
            hoodMax);

    currentShot = result;

    // Shot parameters are calculated only — commands use getCurrentShot() to drive actuators

    // Throttle shot diagnostics logging to every other cycle
    if (periodicCounter % 2 == 0) {
      Logger.recordOutput("Shots/Strategy/Active", activeStrategy.getName());
      Logger.recordOutput("Shots/Strategy/LUTDataPoints", lookupTable.size());

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
    }

    // === Side-by-side comparison: run BOTH strategies and log under Shots/Compare/ ===
    // Throttled to every 10th cycle (~200ms) to minimize CPU cost.
    if (periodicCounter % 10 == 0 && lookupTable.hasEnoughData()) {
      double currentAngle = turret.getOutsideCurrentAngle();
      double turretMin = turret.getMinAngle();
      double turretMax = turret.getMaxAngle();
      double compareHoodMin = hood != null ? hood.getMinAngle() : 16.0;
      double compareHoodMax = hood != null ? hood.getMaxAngle() : 46.0;
      if (trenchModeActive) {
        compareHoodMax = Math.min(compareHoodMax, trenchHoodMaxDeg.get());
      }

      ShotCalculator.ShotResult parametricResult =
          parametricStrategy.calculateShot(
              robotPose,
              fieldSpeeds,
              target,
              turretConfig,
              currentAngle,
              turretMin,
              turretMax,
              compareHoodMin,
              compareHoodMax);
      ShotCalculator.ShotResult lutResult =
          lutStrategy.calculateShot(
              robotPose,
              fieldSpeeds,
              target,
              turretConfig,
              currentAngle,
              turretMin,
              turretMax,
              compareHoodMin,
              compareHoodMax);

      // Distance for context
      double robotHeadingRad = robotPose.getRotation().getRadians();
      double[] tPos =
          ShotCalculator.getTurretFieldPosition(
              robotPose.getX(), robotPose.getY(), robotHeadingRad, turretConfig);
      double dist =
          Math.sqrt(Math.pow(target.getX() - tPos[0], 2) + Math.pow(target.getY() - tPos[1], 2));
      Logger.recordOutput("Shots/Compare/DistanceM", dist);
      Logger.recordOutput("Shots/Compare/InLUTRange", lookupTable.isInRange(dist));

      // Parametric side
      Logger.recordOutput("Shots/Compare/Parametric/RPM", parametricResult.launcherRPM());
      Logger.recordOutput("Shots/Compare/Parametric/HoodAngleDeg", parametricResult.hoodAngleDeg());
      Logger.recordOutput(
          "Shots/Compare/Parametric/ExitVelocityMps", parametricResult.exitVelocityMps());
      Logger.recordOutput("Shots/Compare/Parametric/Achievable", parametricResult.achievable());

      // LUT side
      Logger.recordOutput("Shots/Compare/LUT/RPM", lutResult.launcherRPM());
      Logger.recordOutput("Shots/Compare/LUT/HoodAngleDeg", lutResult.hoodAngleDeg());
      Logger.recordOutput("Shots/Compare/LUT/ExitVelocityMps", lutResult.exitVelocityMps());
      Logger.recordOutput("Shots/Compare/LUT/Achievable", lutResult.achievable());

      // Deltas — how much the two models disagree
      Logger.recordOutput(
          "Shots/Compare/Delta/RPM", parametricResult.launcherRPM() - lutResult.launcherRPM());
      Logger.recordOutput(
          "Shots/Compare/Delta/HoodAngleDeg",
          parametricResult.hoodAngleDeg() - lutResult.hoodAngleDeg());
      Logger.recordOutput(
          "Shots/Compare/Delta/ExitVelocityMps",
          parametricResult.exitVelocityMps() - lutResult.exitVelocityMps());
    }
  }

  /** Hub net height in meters (from TrajectoryOptimizer). */
  private static final double HUB_NET_HEIGHT = 1.83;

  /** Calculate and apply pass shot using two-point trajectory solver. */
  private void calculatePassToTarget(
      Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d target, PassingStrategy strategy) {

    double robotHeadingRad = robotPose.getRotation().getRadians();
    double[] turretFieldPos =
        ShotCalculator.getTurretFieldPosition(
            robotPose.getX(), robotPose.getY(), robotHeadingRad, turretConfig);
    double turretX = turretFieldPos[0];
    double turretY = turretFieldPos[1];

    double horizontalDist =
        Math.sqrt(Math.pow(target.getX() - turretX, 2) + Math.pow(target.getY() - turretY, 2));

    double hoodMin = hood != null ? hood.getMinAngle() : 16.0;
    double hoodMax = hood != null ? hood.getMaxAngle() : 46.0;
    if (trenchModeActive) {
      hoodMax = Math.min(hoodMax, trenchHoodMaxDeg.get());
    }

    double constraintX;
    double constraintH;
    double maxPeakHeight;

    if (strategy == PassingStrategy.DRIVER_STATION) {
      // LOB: clearance point is at the hub net
      boolean isBlue = RobotStatus.isBlueAlliance();
      double hubCenterX =
          isBlue
              ? FieldConstants.LinesVertical.hubCenter
              : FieldConstants.LinesVertical.oppHubCenter;

      // Project hub center onto the shot line to get distance along shot direction
      // Shot direction vector from turret to target
      double dx = target.getX() - turretX;
      double dy = target.getY() - turretY;
      double shotLen = Math.sqrt(dx * dx + dy * dy);
      if (shotLen < 0.01) shotLen = 0.01;
      double shotDirX = dx / shotLen;
      double shotDirY = dy / shotLen;

      // Project hub center onto shot line: distance = dot(hubCenter - turret, shotDir)
      double hubDistAlongShot =
          (hubCenterX - turretX) * shotDirX
              + (FieldConstants.fieldWidth / 2.0 - turretY) * shotDirY;

      if (hubDistAlongShot <= 0.5 || hubDistAlongShot >= horizontalDist - 0.5) {
        // Shot doesn't meaningfully cross the hub — fall back to symmetric arc
        constraintX = horizontalDist / 2.0;
        constraintH = symmetricArcPeakHeightM.get();
        maxPeakHeight = lobMaxPeakHeightM.get();
        Logger.recordOutput("Turret/Pass/TwoPoint/LobFallback", true);
      } else {
        constraintX = hubDistAlongShot;
        constraintH = HUB_NET_HEIGHT + lobNetClearanceMarginM.get();
        maxPeakHeight = lobMaxPeakHeightM.get();
        Logger.recordOutput("Turret/Pass/TwoPoint/LobFallback", false);
      }
    } else {
      // SYMMETRIC: clearance point is the midpoint, height is the desired arc peak
      constraintX = horizontalDist / 2.0;
      constraintH = symmetricArcPeakHeightM.get();
      maxPeakHeight = symmetricArcPeakHeightM.get() + 1.0; // allow small margin above desired peak
    }

    ShotCalculator.ShotResult result =
        ShotCalculator.calculatePassShotTwoPoint(
            robotPose,
            fieldSpeeds,
            target,
            turretConfig,
            constraintX,
            constraintH,
            maxPeakHeight,
            turret.getOutsideCurrentAngle(),
            turret.getMinAngle(),
            turret.getMaxAngle(),
            hoodMin,
            hoodMax);

    currentShot = result;

    // Log pass shot data
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

    // --- Efficiency constant ---
    Logger.recordOutput("SmartLaunch/Efficiency/Value", ShotCalculator.getEfficiency());

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

    // Zone check for speed gating
    Pose2d robotPose = robotPoseSupplier.get();
    TurretAimingHelper.AimResult aimResult =
        TurretAimingHelper.getAimTarget(robotPose.getX(), robotPose.getY(), alliance);
    boolean inAllianceZone = aimResult.mode() == TurretAimingHelper.AimMode.SHOOT;

    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
    double robotSpeedMps = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    boolean robotSlow = !inAllianceZone || robotSpeedMps <= maxFeedSpeedMps.get();

    Logger.recordOutput("Turret/AutoShoot/LauncherReady", launcherReady);
    Logger.recordOutput("Turret/AutoShoot/Aimed", aimed);
    Logger.recordOutput("Turret/AutoShoot/HasFuel", hasFuel);
    Logger.recordOutput("Turret/AutoShoot/AimMode", aimResult.mode().name());
    Logger.recordOutput("Turret/AutoShoot/RobotSpeedMps", robotSpeedMps);
    Logger.recordOutput("Turret/AutoShoot/RobotSlow", robotSlow);
    Logger.recordOutput(
        "Turret/AutoShoot/FuelRemaining", visualizer != null ? visualizer.getFuelCount() : 0);

    if (launcherReady && hasShot && aimed && hasFuel && intervalElapsed && robotSlow) {
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

      // Use the pre-computed exit velocity from the shot result — it was calculated with
      // the correct distance at strategy time, avoiding stale/manual distance mismatches.
      double actualExitVelocity = currentShot.exitVelocityMps();
      visualizer.launchFuel(actualExitVelocity, currentShot.launchAngleRad(), azimuthAngle);

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

  // ========== Trench Avoidance Mode ==========

  /** Enable trench avoidance mode. Hood angle will be clamped when robot is under a trench. */
  public void enableTrenchMode() {
    trenchModeEnabled = true;
    SmartDashboard.putBoolean("Shots/TrenchMode/Enabled", true);
  }

  /** Disable trench avoidance mode. */
  public void disableTrenchMode() {
    trenchModeEnabled = false;
    trenchModeActive = false;
    SmartDashboard.putBoolean("Shots/TrenchMode/Enabled", false);
  }

  /** Check if trench avoidance mode is enabled. */
  public boolean isTrenchModeEnabled() {
    return trenchModeEnabled;
  }

  /** Check if trench avoidance is currently active (enabled AND robot is in a trench zone). */
  public boolean isTrenchModeActive() {
    return trenchModeActive;
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
    if (selected == null) selected = "Parametric";

    ShotStrategy newStrategy = "LUT".equals(selected) ? lutStrategy : parametricStrategy;

    if (newStrategy != activeStrategy) {
      System.out.println("[ShootingCoordinator] Strategy changed to: " + newStrategy.getName());
      activeStrategy = newStrategy;
    }
  }

  /**
   * Reload LUT data. Loads the hardcoded baseline table first, then overlays any field-recorded
   * entries on top (field data overrides baseline at matching distances).
   *
   * <p>To update the baseline: edit {@link ShotTableConstants#BASELINE_TABLE} and push code. To add
   * data on the fly: use the batch recorder during practice.
   */
  public void reloadLUTData() {
    lookupTable.clear();

    // Load hardcoded baseline only — field-recorded data is for offline review,
    // not runtime use. To update shots, edit ShotTableConstants and redeploy.
    int baselineCount = ShotTableConstants.loadBaseline(lookupTable);

    // Log full table to AdvantageKit for live dashboard viewing
    lookupTable.logTable("LUTDev/Table");

    frc.robot.util.StartupLogger.log(
        "[ShootingCoordinator] LUT loaded: " + baselineCount + " baseline entries");
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
