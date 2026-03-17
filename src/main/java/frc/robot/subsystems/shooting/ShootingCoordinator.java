package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.motivator.Motivator;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RobotStatus;
import frc.robot.util.TurretAimingHelper;
import frc.robot.util.ZoneDetector;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
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
  private final Spindexer spindexer;

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
  private final ShotLookupTable alternateLookupTable = new ShotLookupTable();
  private final StationaryShotBatchRecorder batchRecorder = new StationaryShotBatchRecorder();
  private final ParametricShotStrategy parametricStrategy = new ParametricShotStrategy();
  private final LUTShotStrategy lutStrategy;
  private final LUTShotStrategy alternateLutStrategy;
  private ShotStrategy activeStrategy;

  // Visualizer (created during initialize)
  private ShotVisualizer visualizer = null;

  // Throttle counter for non-critical logging (reduces loop time)
  private int periodicCounter = 0;
  private Supplier<Pose2d> robotPoseSupplier = null;
  private Supplier<ChassisSpeeds> fieldSpeedsSupplier = null;
  private DoubleSupplier pitchDegSupplier = () -> 0.0;

  // Optional feeding suppression check — when true, launchFuel() is a no-op
  private BooleanSupplier feedingSuppressedSupplier = () -> false;

  // Trench avoidance — always active; clamps hood angle when robot is under a trench or bump.
  // Zone detection with robot-size margins is handled by ZoneDetector.
  private final LoggedTunableNumber trenchHoodMaxDeg =
      new LoggedTunableNumber("Shots/TrenchMode/HoodMaxDeg", 18.0);
  private boolean trenchModeActive = false; // true when robot is in a trench or bump zone

  // Visualizer throttle — run at 10Hz instead of 50Hz (pure display, not control)
  private int visualizerCounter = 0;
  private static final int VISUALIZER_DIVISOR = 10; // 50Hz / 10 = 5Hz

  // Current shot data
  private ShotCalculator.ShotResult currentShot = null;
  private double currentDistanceM = -1;

  // Pass target offset tunables — separate for left and right trench
  private final LoggedTunableNumber passLeftAdjustX =
      new LoggedTunableNumber("SmartLaunch/Pass/Left/AdjustX", 0.0);
  private final LoggedTunableNumber passLeftAdjustY =
      new LoggedTunableNumber("SmartLaunch/Pass/Left/AdjustY", 0.0);
  private final LoggedTunableNumber passRightAdjustX =
      new LoggedTunableNumber("SmartLaunch/Pass/Right/AdjustX", 0.0);
  private final LoggedTunableNumber passRightAdjustY =
      new LoggedTunableNumber("SmartLaunch/Pass/Right/AdjustY", 0.0);

  // Two-point trajectory tunables for pass shots
  private final LoggedTunableNumber symmetricArcPeakHeightM =
      new LoggedTunableNumber("SmartLaunch/Pass/Symmetric/ArcPeakHeightM", 1.5);
  private final LoggedTunableNumber lobNetClearanceMarginM =
      new LoggedTunableNumber("SmartLaunch/Pass/Lob/NetClearanceMarginM", 0.3);
  private final LoggedTunableNumber lobMaxPeakHeightM =
      new LoggedTunableNumber("SmartLaunch/Pass/Lob/MaxPeakHeightM", 5.0);
  private final LoggedTunableNumber lobStation1AdjustY =
      new LoggedTunableNumber("SmartLaunch/Pass/DriverStation/Station1/AdjustY", 0.0);
  private final LoggedTunableNumber lobStation3AdjustY =
      new LoggedTunableNumber("SmartLaunch/Pass/DriverStation/Station3/AdjustY", 0.0);

  // Cached pass targets — only recomputed when tunables change
  private Translation3d cachedLeftTarget = null;
  private Translation3d cachedRightTarget = null;
  private Translation3d cachedLobStation1Target = null;
  private Translation3d cachedLobStation3Target = null;

  // Match shot tracking (counts persist across auto→teleop transition)
  private int totalShots = 0;
  private int autoShots = 0;
  private int teleopShots = 0;
  // Zone-aware speed thresholds for feeding and drive limiting.
  // SHOOT_ON_THE_MOVE: max speed for hub shots in open alliance zone.
  // Used for both spindexer gating and active drive speed limiting.
  private final LoggedTunableNumber shootOnTheMoveSpeedMps =
      new LoggedTunableNumber("Shots/SpeedLimits/ShootOnTheMoveSpeedMps", 1.25);
  // SHOOT_STATIONARY: max speed for hub shots under trench (tight clearance, low hood).
  private final LoggedTunableNumber stationarySpeedMps =
      new LoggedTunableNumber("Shots/SpeedLimits/StationarySpeedMps", 0.3);
  // PASS / LONG_PASS: max speed for pass shots in neutral/opponent zones.
  private final LoggedTunableNumber passSpeedMps =
      new LoggedTunableNumber("Shots/SpeedLimits/PassSpeedMps", 3.0);

  /**
   * Creates a new ShootingCoordinator.
   *
   * @param turret The turret subsystem (rotation control only)
   * @param hood The hood subsystem (launch angle control), may be null
   * @param launcher The launcher subsystem (RPM control), may be null
   * @param motivator The motivator subsystem (ball feeder), may be null
   * @param spindexer The spindexer subsystem (fuel indexer), may be null
   */
  public ShootingCoordinator(
      Turret turret, Hood hood, Launcher launcher, Motivator motivator, Spindexer spindexer) {
    this.turret = turret;
    this.hood = hood;
    this.launcher = launcher;
    this.motivator = motivator;
    this.spindexer = spindexer;

    var config = Constants.getRobotConfig();
    this.turretConfig =
        new ShotCalculator.TurretConfig(
            config.getTurretHeightMeters(), config.getTurretOffsetX(), config.getTurretOffsetY());

    // Initialize shot strategies — completely independent, no shared efficiency model
    this.lutStrategy = new LUTShotStrategy(lookupTable);
    this.alternateLutStrategy = new LUTShotStrategy(alternateLookupTable);
    this.activeStrategy = lutStrategy;

    // Strategy dropdown on dashboard — three options
    strategyChooser.addOption("Parametric", "Parametric");
    strategyChooser.setDefaultOption("LUT (Lookup Table)", "LUT");
    strategyChooser.addOption("LUT Alternate", "LUT_ALTERNATE");
    SmartDashboard.putData("Shots/Strategy/Mode", strategyChooser);

    // Passing strategy chooser — how to pick left vs right pass target
    passingStrategyChooser.setDefaultOption("Symmetric (Y-based)", PassingStrategy.SYMMETRIC);
    passingStrategyChooser.addOption("Driver Station", PassingStrategy.DRIVER_STATION);
    SmartDashboard.putData("SmartLaunch/Pass/Strategy", passingStrategyChooser);

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
    initialize(poseSupplier, speedsSupplier, () -> 0.0);
  }

  /**
   * Initialize the coordinator with robot pose, speed, and pitch suppliers.
   *
   * @param poseSupplier Supplier for robot's 2D pose
   * @param speedsSupplier Supplier for field-relative chassis speeds
   * @param pitchDegSupplier Supplier for gyro pitch in degrees (for bump detection)
   */
  public void initialize(
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      DoubleSupplier pitchDegSupplier) {
    this.robotPoseSupplier = poseSupplier;
    this.fieldSpeedsSupplier = speedsSupplier;
    this.pitchDegSupplier = pitchDegSupplier;

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
  }

  @Override
  public void periodic() {
    DriverStation.Alliance alliance = RobotStatus.getAlliance();
    boolean isBlueAlliance = RobotStatus.isBlueAlliance();

    // Advance visualizer throttle counter
    visualizerCounter++;

    // Shot calculation and logging run even while disabled so you can
    // see strategy comparisons and shot parameters before enabling.
    periodicCounter++;
    updateShotCalculation(alliance, isBlueAlliance);
    if (periodicCounter % 5 == 0) {
      logShotState();
    }

    // Never command actuators while disabled — only run visualization
    if (DriverStation.isDisabled()) {
      if (visualizerCounter % VISUALIZER_DIVISOR == 0
          && visualizer != null
          && robotPoseSupplier != null) {
        visualizer.update(createVisualizerSnapshot(isBlueAlliance));
      }
      return;
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

      // Zone detection — uses turret field position for trench detection (hood clearance)
      // and robot center for bump detection (chassis on ramp).
      double robotHeadingRad = robotPose.getRotation().getRadians();
      double[] turretFieldPos =
          ShotCalculator.getTurretFieldPosition(
              robotPose.getX(), robotPose.getY(), robotHeadingRad, turretConfig);
      TurretAimingHelper.AimResult aimResult =
          TurretAimingHelper.getAimTarget(
              robotPose.getX(),
              robotPose.getY(),
              alliance,
              pitchDegSupplier.getAsDouble(),
              turretFieldPos[0],
              turretFieldPos[1]);
      trenchModeActive =
          aimResult.zone() == ZoneDetector.Zone.TRENCH_NEAR
              || aimResult.zone() == ZoneDetector.Zone.TRENCH_FAR
              || aimResult.zone() == ZoneDetector.Zone.BUMP;

      // Log zone and aim mode (throttled to 10Hz)
      if (periodicCounter % 5 == 0) {
        Logger.recordOutput("SmartLaunch/Zone", aimResult.zone().name());
        Logger.recordOutput("SmartLaunch/AllowedAction", aimResult.mode().name());
      }

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
        // Left trench target (high Y, with offsets, clamped to alliance zone)
        double leftRawX = baseX + passLeftAdjustX.get() * (maxX - minX) / 2.0;
        double leftRawY =
            Constants.StrategyConstants.LEFT_PASS_TARGET_Y
                + passLeftAdjustY.get() * (maxY - minY) / 2.0;
        cachedLeftTarget =
            new Translation3d(
                Math.max(minX, Math.min(maxX, leftRawX)),
                Math.max(minY, Math.min(maxY, leftRawY)),
                0.0);
        Logger.recordOutput(
            "SmartLaunch/Pass/Symmetric/Left", new Pose3d(cachedLeftTarget, Rotation3d.kZero));

        // Right trench target (low Y, with offsets, clamped to alliance zone)
        double rightRawX = baseX + passRightAdjustX.get() * (maxX - minX) / 2.0;
        double rightRawY =
            Constants.StrategyConstants.RIGHT_PASS_TARGET_Y
                + passRightAdjustY.get() * (maxY - minY) / 2.0;
        cachedRightTarget =
            new Translation3d(
                Math.max(minX, Math.min(maxX, rightRawX)),
                Math.max(minY, Math.min(maxY, rightRawY)),
                0.0);
        Logger.recordOutput(
            "SmartLaunch/Pass/Symmetric/Right", new Pose3d(cachedRightTarget, Rotation3d.kZero));
      }

      // Recompute lob pass targets only when lob tunables change
      if (cachedLobStation1Target == null
          || LoggedTunableNumber.hasChanged(lobStation1AdjustY, lobStation3AdjustY)) {
        // Station 1/2 target (blue = high Y, red = low Y)
        double st12BaseY =
            isBlueAlliance
                ? fieldW - Constants.StrategyConstants.LOB_STATION_1_TARGET_Y
                : Constants.StrategyConstants.LOB_STATION_1_TARGET_Y;
        double st12RawY = st12BaseY + lobStation1AdjustY.get() * (maxY - minY) / 2.0;
        cachedLobStation1Target =
            new Translation3d(
                Math.max(minX, Math.min(maxX, baseX)),
                Math.max(minY, Math.min(maxY, st12RawY)),
                0.0);
        Logger.recordOutput(
            "SmartLaunch/Pass/DriverStation/Station1",
            new Pose3d(cachedLobStation1Target, Rotation3d.kZero));

        // Station 3 target (blue = low Y near outpost, red = high Y near outpost)
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
            "SmartLaunch/Pass/DriverStation/Station3",
            new Pose3d(cachedLobStation3Target, Rotation3d.kZero));
      }

      switch (aimResult.mode()) {
        case SHOOT_ON_THE_MOVE, SHOOT_STATIONARY -> {
          Logger.recordOutput("SmartLaunch/Strategy", "Hub " + activeStrategy.getName());
          calculateShotToHub(robotPose, fieldSpeeds, isBlueAlliance);
        }
        case PASS -> {
          PassingStrategy strategy = passingStrategyChooser.getSelected();

          if (strategy == PassingStrategy.DRIVER_STATION) {
            Logger.recordOutput("SmartLaunch/Strategy", "Pass Lob");
            var location = DriverStation.getLocation();
            int station = location.isPresent() ? location.getAsInt() : 2;
            Translation3d activeTarget =
                (station <= 2) ? cachedLobStation1Target : cachedLobStation3Target;
            Logger.recordOutput(
                "SmartLaunch/Pass/Target", (station <= 2) ? "STATION_1" : "STATION_3");
            calculatePassToTarget(
                robotPose, fieldSpeeds, activeTarget, PassingStrategy.DRIVER_STATION);
          } else {
            Logger.recordOutput("SmartLaunch/Strategy", "Pass Low");
            boolean isLeftTrench = selectIsLeftTrench(robotPose);
            Translation3d activeTarget = isLeftTrench ? cachedLeftTarget : cachedRightTarget;
            Logger.recordOutput("SmartLaunch/Pass/Target", isLeftTrench ? "LEFT" : "RIGHT");
            calculatePassToTarget(
                robotPose, fieldSpeeds, activeTarget, PassingStrategy.SYMMETRIC);
          }
        }
        case LONG_PASS -> {
          Logger.recordOutput("SmartLaunch/Strategy", "Pass LongLob");
          boolean isLeftTrench = selectIsLeftTrench(robotPose);
          Translation3d activeTarget =
              isLeftTrench ? cachedLobStation1Target : cachedLobStation3Target;
          Logger.recordOutput("SmartLaunch/Pass/Target", isLeftTrench ? "LOB_LEFT" : "LOB_RIGHT");
          calculatePassToTarget(
              robotPose, fieldSpeeds, activeTarget, PassingStrategy.DRIVER_STATION);
        }
        case NONE -> {
          Logger.recordOutput("SmartLaunch/Strategy", "Hub " + activeStrategy.getName());
          calculateShotToHub(robotPose, fieldSpeeds, isBlueAlliance);
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
    // Default: symmetric Y-based selection (left trench = high Y)
    return robotPose.getY() > FieldConstants.fieldWidth / 2.0;
  }

  // Tunable pass shot parameters
  private final LoggedTunableNumber passMaxRPM =
      new LoggedTunableNumber("SmartLaunch/Pass/MaxRPM", 4000.0);

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

    double currentTurretAngle = turret.getOutsideCurrentAngle();
    double turretMin = turret.getMinAngle();
    double turretMax = turret.getMaxAngle();

    // Active strategy runs every cycle for control
    ShotCalculator.ShotResult activeResult =
        activeStrategy.calculateShot(
            robotPose,
            fieldSpeeds,
            target,
            turretConfig,
            currentTurretAngle,
            turretMin,
            turretMax,
            hoodMin,
            hoodMax);
    currentShot = activeResult;

    // Throttle logging to ~10Hz
    if (periodicCounter % 5 == 0) {
      // Log distance to target
      double robotHeadingRad = robotPose.getRotation().getRadians();
      double[] turretFieldPos =
          ShotCalculator.getTurretFieldPosition(
              robotPose.getX(), robotPose.getY(), robotHeadingRad, turretConfig);
      double turretX = turretFieldPos[0];
      double turretY = turretFieldPos[1];
      double distanceToTarget =
          Math.sqrt(Math.pow(target.getX() - turretX, 2) + Math.pow(target.getY() - turretY, 2));
      currentDistanceM = distanceToTarget;
      Logger.recordOutput("SmartLaunch/DistanceM", distanceToTarget);
    }
  }

  /** Hub net top height in meters (120.36 inches — top of the net, not the lip). */
  private static final double HUB_NET_HEIGHT = 3.057;

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
        Logger.recordOutput("SmartLaunch/Pass/TwoPoint/LobFallback", true);
      } else {
        constraintX = hubDistAlongShot;
        constraintH = HUB_NET_HEIGHT + lobNetClearanceMarginM.get();
        maxPeakHeight = lobMaxPeakHeightM.get();
        Logger.recordOutput("SmartLaunch/Pass/TwoPoint/LobFallback", false);
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
    currentDistanceM = horizontalDist;

    // Pass shot distance logged at 10Hz (RPM/hood already covered by Shots/Status/)
    if (periodicCounter % 5 == 0) {
      Logger.recordOutput("SmartLaunch/DistanceM", horizontalDist);
    }
  }

  // ========== Shot State Logging ==========

  /**
   * Log current shot state to AdvantageKit every cycle. Runs in periodic() so readiness, targets,
   * and tracking errors are always visible — regardless of which command (smart launch, fixed shot,
   * auto-track, or none) is active.
   */
  private void logShotState() {
    // --- Readiness flags (use subsystem state machines, not raw at-setpoint checks) ---
    boolean launcherReady = launcher != null && launcher.isReady();
    boolean motivatorReady =
        motivator == null || motivator.getState() == Motivator.MotivatorState.READY;
    boolean turretReady = turret.getState() == Turret.TurretState.READY;
    boolean hoodReady = hood == null || hood.getState() == Hood.HoodState.READY;
    boolean achievable = currentShot != null && currentShot.achievable();

    Logger.recordOutput("SmartLaunch/Ready/Launcher", launcherReady);
    Logger.recordOutput("SmartLaunch/Ready/Motivator", motivatorReady);
    Logger.recordOutput("SmartLaunch/Ready/Turret", turretReady);
    Logger.recordOutput("SmartLaunch/Ready/Hood", hoodReady);
    Logger.recordOutput("SmartLaunch/Ready/Achievable", achievable);
    Logger.recordOutput(
        "SmartLaunch/Ready/All",
        launcherReady && motivatorReady && turretReady && hoodReady && achievable);

    // --- Targets (what we're commanding) ---
    boolean overridesActive = SmartDashboard.getBoolean("LUTDev/UseOverrides", false);
    Logger.recordOutput("SmartLaunch/OverridesActive", overridesActive);

    double targetRPM;
    double targetHoodDeg;
    if (overridesActive) {
      targetRPM = SmartDashboard.getNumber("LUTDev/OverrideRPM", 0);
      targetHoodDeg = SmartDashboard.getNumber("LUTDev/OverrideHoodDeg", 0);
    } else if (currentShot != null) {
      targetRPM = currentShot.launcherRPM();
      targetHoodDeg = currentShot.hoodAngleDeg();
    } else {
      targetRPM = 0;
      targetHoodDeg = 0;
    }

    Logger.recordOutput("SmartLaunch/Target/LauncherRPM", targetRPM);
    Logger.recordOutput("SmartLaunch/Target/HoodDeg", targetHoodDeg);
    Logger.recordOutput(
        "SmartLaunch/Target/TurretDeg", currentShot != null ? currentShot.turretAngleDeg() : 0.0);
    Logger.recordOutput(
        "SmartLaunch/Target/LaunchAngleDeg",
        currentShot != null ? currentShot.getLaunchAngleDegrees() : 0.0);
    Logger.recordOutput(
        "SmartLaunch/Target/ExitVelocityMps",
        currentShot != null ? currentShot.exitVelocityMps() : 0.0);
    Logger.recordOutput(
        "SmartLaunch/Target/Achievable",
        currentShot != null && currentShot.achievable());
    Logger.recordOutput(
        "SmartLaunch/Target/MotivatorRPM",
        frc.robot.commands.ShootingCommands.getMotivatorRPM(targetRPM));
    Logger.recordOutput(
        "SmartLaunch/Target/SpindexerRPM",
        frc.robot.commands.ShootingCommands.getSpindexerRPM(currentDistanceM));

    // --- Actuals (what hardware is doing) ---
    Logger.recordOutput(
        "SmartLaunch/Actual/LauncherRPM", launcher != null ? launcher.getVelocity() : 0.0);
    Logger.recordOutput("SmartLaunch/Actual/HoodDeg", hood != null ? hood.getCurrentAngle() : 0.0);
    Logger.recordOutput("SmartLaunch/Actual/TurretDeg", turret.getOutsideCurrentAngle());
    Logger.recordOutput(
        "SmartLaunch/Actual/MotivatorRPM",
        motivator != null ? motivator.getMotivatorWheelVelocity() : 0.0);
    Logger.recordOutput(
        "SmartLaunch/Actual/SpindexerRPM",
        spindexer != null ? spindexer.getSpindexerWheelVelocity() : 0.0);

    // Distance and TOF
    if (currentShot != null && robotPoseSupplier != null) {
      Pose2d robotPose = robotPoseSupplier.get();
      double robotHeadingRad = robotPose.getRotation().getRadians();
      double[] turretPos =
          ShotCalculator.getTurretFieldPosition(
              robotPose.getX(), robotPose.getY(), robotHeadingRad, turretConfig);
      Translation3d aim = currentShot.aimTarget();
      double distance =
          Math.sqrt(
              Math.pow(aim.getX() - turretPos[0], 2) + Math.pow(aim.getY() - turretPos[1], 2));
      Logger.recordOutput("SmartLaunch/DistanceM", distance);

      // Parametric TOF from physics (exit velocity + launch angle)
      double parametricTOF =
          ShotCalculator.calculateTimeOfFlight(
              currentShot.exitVelocityMps(), currentShot.launchAngleRad(), distance);
      Logger.recordOutput("SmartLaunch/Parametric/TOF", parametricTOF);

      // LUT TOF from empirical data
      Logger.recordOutput("SmartLaunch/LUT/TOF", lookupTable.lookupTOF(distance));
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

      // Compute exit velocity from ballistic physics to reach the aim target.
      // The RPM-based exitVelocityMps underestimates because it doesn't account for the
      // motivator's contribution to ball speed. Solving for the velocity that hits the target
      // at the given launch angle matches real-world LUT shot behavior.
      double distanceToTarget =
          Math.sqrt(Math.pow(target.getX() - turretX, 2) + Math.pow(target.getY() - turretY, 2));
      double heightDelta = target.getZ() - turretConfig.heightMeters();
      double launchAngle = currentShot.launchAngleRad();
      double cosTheta = Math.cos(launchAngle);
      double tanTheta = Math.tan(launchAngle);
      double denom = 2.0 * cosTheta * cosTheta * (distanceToTarget * tanTheta - heightDelta);
      double actualExitVelocity =
          (denom > 0 && distanceToTarget >= 0.1)
              ? Math.sqrt(9.81 * distanceToTarget * distanceToTarget / denom)
              : currentShot.exitVelocityMps();
      visualizer.launchFuel(actualExitVelocity, currentShot.launchAngleRad(), azimuthAngle);

      // Track shot counts (auto vs teleop)
      totalShots++;
      if (DriverStation.isAutonomous()) {
        autoShots++;
      } else {
        teleopShots++;
      }
      Logger.recordOutput("ShotLog/TotalShots", totalShots);
      Logger.recordOutput("ShotLog/AutoShots", autoShots);
      Logger.recordOutput("ShotLog/TeleopShots", teleopShots);
    }
  }

  /** Reset match shot counters. */
  public void resetShotCounts() {
    totalShots = 0;
    autoShots = 0;
    teleopShots = 0;
    Logger.recordOutput("ShotLog/TotalShots", 0);
    Logger.recordOutput("ShotLog/AutoShots", 0);
    Logger.recordOutput("ShotLog/TeleopShots", 0);
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
    }
  }

  /** Clear manual shot parameters and return to auto-calculated shots. */
  public void clearManualShotParameters() {
    currentShot = null;
  }

  // ========== Zone-Based Speed Check ==========

  /**
   * Check if the robot is slow enough to fire in the current zone. Uses zone-aware speed
   * thresholds: alliance zone allows shoot-on-the-move, trench requires stationary, pass zones
   * allow high speed. Used by continuousSmartLaunch.
   *
   * @return true if the robot speed is within the zone's firing threshold
   */
  public boolean isRobotSlowEnoughForCurrentZone() {
    if (fieldSpeedsSupplier == null || robotPoseSupplier == null) return true;

    ChassisSpeeds speeds = fieldSpeedsSupplier.get();
    double robotSpeedMps = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    Pose2d robotPose = robotPoseSupplier.get();
    DriverStation.Alliance alliance = RobotStatus.getAlliance();
    TurretAimingHelper.AimResult aimResult =
        TurretAimingHelper.getAimTarget(
            robotPose.getX(), robotPose.getY(), alliance, pitchDegSupplier.getAsDouble());

    return switch (aimResult.mode()) {
      case SHOOT_ON_THE_MOVE -> robotSpeedMps <= shootOnTheMoveSpeedMps.get();
      case SHOOT_STATIONARY -> robotSpeedMps <= stationarySpeedMps.get();
      case PASS, LONG_PASS -> robotSpeedMps <= passSpeedMps.get();
      case NONE -> false;
    };
  }

  /**
   * Check if the robot is currently in a pass zone (PASS or LONG_PASS aim mode). Used by
   * continuousSmartLaunch to arm feeding only after reaching the pass zone during sprint autos.
   *
   * @return true if the robot is in a zone where passing is the aim mode
   */
  public boolean isInPassZone() {
    if (robotPoseSupplier == null) return false;
    Pose2d robotPose = robotPoseSupplier.get();
    DriverStation.Alliance alliance = RobotStatus.getAlliance();
    TurretAimingHelper.AimResult aimResult =
        TurretAimingHelper.getAimTarget(
            robotPose.getX(), robotPose.getY(), alliance, pitchDegSupplier.getAsDouble());
    return aimResult.mode() == TurretAimingHelper.AimMode.PASS
        || aimResult.mode() == TurretAimingHelper.AimMode.LONG_PASS;
  }

  /**
   * Get the current zone the robot is in.
   *
   * @return Current zone, or ALLIANCE as fallback if pose is unavailable
   */
  public ZoneDetector.Zone getCurrentZone() {
    if (robotPoseSupplier == null) return ZoneDetector.Zone.ALLIANCE;
    Pose2d robotPose = robotPoseSupplier.get();
    DriverStation.Alliance alliance = RobotStatus.getAlliance();
    return ZoneDetector.getCurrentZone(
        robotPose.getX(), robotPose.getY(), alliance, pitchDegSupplier.getAsDouble());
  }

  /**
   * Check if the robot is stationary (speed <= stationarySpeedMps tunable). Used by auto-tracking
   * stationary shooting strategy.
   *
   * @return true if the robot speed is within the stationary threshold
   */
  public boolean isRobotStationary() {
    if (fieldSpeedsSupplier == null) return true;
    ChassisSpeeds speeds = fieldSpeedsSupplier.get();
    double robotSpeedMps = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    return robotSpeedMps <= stationarySpeedMps.get();
  }

  // ========== Trench Avoidance Mode ==========

  /**
   * Check if trench avoidance is currently active (robot is in a trench zone). Trench clamping is
   * always enabled — there is no toggle. The hood is clamped whenever the robot is detected inside
   * any trench zone.
   */
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

    ShotStrategy newStrategy;
    switch (selected) {
      case "LUT" -> newStrategy = lutStrategy;
      case "LUT_ALTERNATE" -> newStrategy = alternateLutStrategy;
      default -> newStrategy = parametricStrategy;
    }

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
    alternateLookupTable.clear();

    // Load hardcoded baseline only — field-recorded data is for offline review,
    // not runtime use. To update shots, edit ShotTableConstants and redeploy.
    int baselineCount = ShotTableConstants.loadBaseline(lookupTable);
    int alternateCount = ShotTableConstants.loadAlternate(alternateLookupTable);

    // Log full table to AdvantageKit for live dashboard viewing
    lookupTable.logTable("LUTDev/Table");
    alternateLookupTable.logTable("LUTDev/AlternateTable");

    frc.robot.util.StartupLogger.log(
        "[ShootingCoordinator] LUT loaded: "
            + baselineCount
            + " baseline, "
            + alternateCount
            + " alternate entries");
  }

  /** Get the batch recorder for recording new data collection sessions. */
  public StationaryShotBatchRecorder getBatchRecorder() {
    return batchRecorder;
  }

  /** Get the lookup table (for dashboard display of entry count, etc.). */
  public ShotLookupTable getLookupTable() {
    return lookupTable;
  }

  /**
   * Get the horizontal distance from the turret to the current aim target. Updated every cycle by
   * the shot calculation — no redundant math, just reads the cached value.
   *
   * @return Distance in meters, or -1 if no shot has been calculated yet
   */
  public double getDistanceToTarget() {
    return currentDistanceM;
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

  /** Get the shoot-on-the-move speed limit (for active drive speed capping). */
  public double getShootOnTheMoveSpeedMps() {
    return shootOnTheMoveSpeedMps.get();
  }

  // ========== Snapshot Creation ==========

  /**
   * Build a read-only snapshot of current shooting state for the visualizer.
   *
   * @param isBlueAlliance True if on blue alliance
   * @return Snapshot containing all state the visualizer needs
   */
  private ShotSnapshot createVisualizerSnapshot(boolean isBlueAlliance) {
    // Compute trajectory readiness from actual gating logic
    ShotSnapshot.TrajectoryReadiness readiness;
    if (currentShot == null) {
      readiness = ShotSnapshot.TrajectoryReadiness.NOT_ACTIVE;
    } else {
      boolean launcherReady = launcher != null && launcher.isReady();
      boolean motivatorReady =
          motivator == null || motivator.getState() == Motivator.MotivatorState.READY;
      boolean turretReady = turret.getState() == Turret.TurretState.READY;
      boolean hoodReady = hood == null || hood.getState() == Hood.HoodState.READY;
      boolean achievable = currentShot.achievable();
      readiness =
          (launcherReady && motivatorReady && turretReady && hoodReady && achievable)
              ? ShotSnapshot.TrajectoryReadiness.READY
              : ShotSnapshot.TrajectoryReadiness.NOT_READY;
    }

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
        turretConfig.yOffset(),
        readiness);
  }
}
