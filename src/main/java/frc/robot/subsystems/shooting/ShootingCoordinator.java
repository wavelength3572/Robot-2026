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
import frc.robot.commands.DriveCommands;
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

  // Shot strategy system: parametric (physics-based) with procedural fallback chain
  // (NORMAL → FIXED_HOOD → RELAXED → LUT). Alliance sub-zones (CLOSE/MID/FAR) use
  // continuous distance-based efficiency interpolation — no per-zone strategy needed.
  // Trench shots bypass the strategy pipeline entirely via empirical RPM lerp.
  private final SendableChooser<String> strategyChooser = new SendableChooser<>();

  /** State machine for coordinated shooting. Drives feeding decisions in SmartLaunch. */
  public enum CoordinatorState {
    /** SmartLaunch not active. */
    INACTIVE,
    /** Active but not yet armed — outbound in auto, subsystems idle. */
    UNARMED,
    /** Armed, subsystems moving to targets, not ready to fire yet. */
    AIMING,
    /** All subsystems at target — feeding allowed. */
    FIRING,
    /** Big setpoint change in progress (turret flip, zone transition) — waiting to settle. */
    SETTLING,
    /** Driver holding fire — subsystems track but feeding suppressed. */
    HELD,
    /** In a no-fire zone (BUMP) — shooting blocked by position. */
    NO_FIRE_ZONE
  }

  /**
   * Arming trigger for sprint-start autos. Controls when the state machine is allowed to transition
   * from UNARMED to AIMING. Prevents shooting preloads at the start of sprint autos.
   */
  public enum ArmTrigger {
    /** Teleop / shoot-preloads — armed immediately, no gating. */
    IMMEDIATE,
    /** Sprint + AUTO_SHOOT — arm when first entering a pass zone (NEUTRAL/OPPONENT). */
    ON_PASS_ZONE,
    /** Sprint + STATIONARY — arm when entering alliance trench after visiting neutral zone. */
    ON_TRENCH_RETURN
  }

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
  private final ParametricWithLUTFallbackStrategy parametricWithLutFallback;
  private final ParametricWithProceduralFallbackStrategy parametricWithProceduralFallback;
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

  // Master toggle for trench mode. When false, the normal smartShot strategy runs
  // even under the trench (hood is not clamped, no empirical RPM lerp).
  private boolean trenchModeEnabled = false;

  // Trench mode: hood clamped to trenchHoodMaxDeg, RPM from empirical lerp between
  // two tested distance/RPM endpoints per trench side (Left/Right tunables below).
  // Completely independent of the global strategy pipeline and launch efficiency.
  private final LoggedTunableNumber trenchHoodMaxDeg =
      new LoggedTunableNumber("Shots/TrenchMode/HoodMaxDeg", 18.0);
  // Trench RPM lerp: empirically tested endpoints per trench side. RPM interpolates
  // linearly between close and far distances, clamped outside. Hood stays fixed at
  // trenchHoodMaxDeg. Independent of global launch efficiency — trench tuning won't
  // affect alliance zone shots. Left = high-Y trench, Right = low-Y trench.
  private final LoggedTunableNumber trenchLeftCloseDistM =
      new LoggedTunableNumber("Shots/TrenchMode/Left/CloseDistM", 3.094);
  private final LoggedTunableNumber trenchLeftCloseRPM =
      new LoggedTunableNumber("Shots/TrenchMode/Left/CloseRPM", 2650.0);
  private final LoggedTunableNumber trenchLeftFarDistM =
      new LoggedTunableNumber("Shots/TrenchMode/Left/FarDistM", 3.732);
  private final LoggedTunableNumber trenchLeftFarRPM =
      new LoggedTunableNumber("Shots/TrenchMode/Left/FarRPM", 3151.0);
  private final LoggedTunableNumber trenchRightCloseDistM =
      new LoggedTunableNumber("Shots/TrenchMode/Right/CloseDistM", 3.103);
  private final LoggedTunableNumber trenchRightCloseRPM =
      new LoggedTunableNumber("Shots/TrenchMode/Right/CloseRPM", 2650.0);
  private final LoggedTunableNumber trenchRightFarDistM =
      new LoggedTunableNumber("Shots/TrenchMode/Right/FarDistM", 3.743);
  private final LoggedTunableNumber trenchRightFarRPM =
      new LoggedTunableNumber("Shots/TrenchMode/Right/FarRPM", 3156.0);
  // Trench motivator RPM lerp: same distance endpoints as launcher, independent RPM values.
  // Both default to 1800 — adjust far value to tune motivator speed at longer trench distances.
  private final LoggedTunableNumber trenchLeftMotivatorCloseRPM =
      new LoggedTunableNumber("Shots/TrenchMode/Left/MotivatorCloseRPM", 1800.0);
  private final LoggedTunableNumber trenchLeftMotivatorFarRPM =
      new LoggedTunableNumber("Shots/TrenchMode/Left/MotivatorFarRPM", 1800.0);
  private final LoggedTunableNumber trenchRightMotivatorCloseRPM =
      new LoggedTunableNumber("Shots/TrenchMode/Right/MotivatorCloseRPM", 1800.0);
  private final LoggedTunableNumber trenchRightMotivatorFarRPM =
      new LoggedTunableNumber("Shots/TrenchMode/Right/MotivatorFarRPM", 1800.0);
  private boolean trenchModeActive = false; // true when robot is in a trench or bump zone
  private double trenchMotivatorRPM = 1800.0; // lerped motivator RPM for current trench position

  // Trench hood safety: limits drive speed when hood is above safe angle while moving in trench.
  // Protects the hood from hitting the trench structure during transit.
  private final LoggedTunableNumber trenchSafetySpeedLimitMps =
      new LoggedTunableNumber("Shots/TrenchMode/SafetySpeedLimitMps", 2.0);
  private final LoggedTunableNumber trenchMovingThresholdMps =
      new LoggedTunableNumber("Shots/TrenchMode/MovingThresholdMps", 0.6);
  // Hood clamp/unclamp thresholds with hysteresis. Clamp is low (fast response when
  // accelerating into trench), unclamp is higher (hood starts rising earlier when decelerating).
  private final LoggedTunableNumber trenchHoodClampSpeedMps =
      new LoggedTunableNumber("Shots/TrenchMode/HoodClampSpeedMps", 0.3);
  private final LoggedTunableNumber trenchHoodUnclampSpeedMps =
      new LoggedTunableNumber("Shots/TrenchMode/HoodUnclampSpeedMps", 0.5);
  private boolean trenchHoodSafetyActive = false;
  private boolean movingInTrench = false; // true when robot is moving in a trench zone

  // Auto passing: when false, PASS/LONG_PASS zones are treated as no-fire zones in auto.
  // Set by AutoWrapperFactory based on path strategy (AUTO_SHOOT enables, others disable).
  // Teleop always allows passing regardless of this flag.
  private boolean autoPassingEnabled = false;

  /** Enable or disable passing during auto. Called by AutoWrapperFactory. */
  public void setAutoPassingEnabled(boolean enabled) {
    this.autoPassingEnabled = enabled;
  }

  // ========== CoordinatorState Machine ==========
  // Centralized state that drives feeding decisions in SmartLaunch commands.
  // Updated every cycle in updateCoordinatorState() after shot calculation.
  private CoordinatorState coordinatorState = CoordinatorState.INACTIVE;
  private boolean smartLaunchActive = false;
  private boolean previousTrenchMode = false;
  private TurretAimingHelper.AimMode previousAimMode = null;
  private ArmTrigger armTrigger = ArmTrigger.IMMEDIATE;
  private boolean armed = true; // true = feeding allowed once subsystems ready
  private boolean hasVisitedNeutral = false; // tracks whether robot has been to neutral zone
  private final Timer readyTimeoutTimer = new Timer();
  private boolean readyTimeoutRunning = false;
  private static final LoggedTunableNumber readyTimeoutSec =
      new LoggedTunableNumber("Shots/SmartLaunch/ReadyTimeoutSec", 3.0);

  // Cached aim result — computed once per cycle in updateShotCalculation(), used by all zone
  // queries (isRobotSlowEnoughForCurrentZone, isInPassZone, getCurrentZone, getZoneSpeedLimitMps)
  private TurretAimingHelper.AimResult cachedAimResult = null;

  // Visualizer throttle — run at 10Hz instead of 50Hz (pure display, not control)
  private int visualizerCounter = 0;
  private static final int VISUALIZER_DIVISOR = 10; // 50Hz / 10 = 5Hz

  // ===== Operator Turret Angle Trim =====
  // Rotates the aim target around the turret position so the entire shot pipeline
  // (turret angle, velocity compensation, achievability) adjusts naturally.
  // Positive = counter-clockwise / left, Negative = clockwise / right.
  private static final double TURRET_TRIM_STEP_DEG = 1.0;
  private static final double TURRET_TRIM_MAX_DEG = 3.0;
  private static double turretTrimDeg = 0.0;

  /** Nudge the turret trim left (CCW) by 0.5 deg. Clamped to +/- 3.0 deg. */
  public static void trimLeft() {
    setTurretTrimDeg(turretTrimDeg + TURRET_TRIM_STEP_DEG);
  }

  /** Nudge the turret trim right (CW) by 0.5 deg. Clamped to +/- 3.0 deg. */
  public static void trimRight() {
    setTurretTrimDeg(turretTrimDeg - TURRET_TRIM_STEP_DEG);
  }

  /** Reset the turret trim to zero. */
  public static void resetTurretTrim() {
    setTurretTrimDeg(0.0);
  }

  private static void setTurretTrimDeg(double trimDeg) {
    turretTrimDeg = Math.max(-TURRET_TRIM_MAX_DEG, Math.min(TURRET_TRIM_MAX_DEG, trimDeg));
    SmartDashboard.putNumber("Trim/TurretDeg", turretTrimDeg);
    Logger.recordOutput("Trim/TurretDeg", turretTrimDeg);
  }

  /** Get the current turret trim offset in degrees. */
  public static double getTurretTrimDeg() {
    return turretTrimDeg;
  }

  /**
   * Rotate a 3D aim target around the turret's field position by the current trim angle. Returns
   * the original target when trim is zero.
   */
  private Translation3d applyTurretTrim(Translation3d target, double turretX, double turretY) {
    if (turretTrimDeg == 0.0) return target;
    double dx = target.getX() - turretX;
    double dy = target.getY() - turretY;
    double trimRad = Math.toRadians(turretTrimDeg);
    double cos = Math.cos(trimRad);
    double sin = Math.sin(trimRad);
    return new Translation3d(
        turretX + dx * cos - dy * sin, turretY + dx * sin + dy * cos, target.getZ());
  }

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

  // Two-point trajectory tunables for pass shots (dashboard value in inches,
  // converted to meters)
  private final LoggedTunableNumber symmetricArcPeakHeightIn =
      new LoggedTunableNumber("SmartLaunch/Pass/Symmetric/ArcPeakHeightIn", 62.0);
  private final LoggedTunableNumber lobNetClearanceMarginM =
      new LoggedTunableNumber("SmartLaunch/Pass/Lob/NetClearanceMarginM", 0.3);
  private final LoggedTunableNumber lobMaxPeakHeightM =
      new LoggedTunableNumber("SmartLaunch/Pass/Lob/MaxPeakHeightM", 5.0);
  private final LoggedTunableNumber lobMinHubDistM =
      new LoggedTunableNumber("SmartLaunch/Pass/Lob/MinHubDistM", 4.0);
  private final LoggedTunableNumber lobStation1AdjustY =
      new LoggedTunableNumber("SmartLaunch/Pass/DriverStation/Station1/AdjustY", 0.0);
  private final LoggedTunableNumber lobStation3AdjustY =
      new LoggedTunableNumber("SmartLaunch/Pass/DriverStation/Station3/AdjustY", 0.0);

  // Cached pass targets — recomputed when tunables or alliance change
  private Translation3d cachedLeftTarget = null;
  private Translation3d cachedRightTarget = null;
  private Translation3d cachedLobStation1Target = null;
  private Translation3d cachedLobStation3Target = null;
  private DriverStation.Alliance cachedPassAlliance = null;

  // Match shot tracking (counts persist across auto→teleop transition)
  private int totalShots = 0;
  private int autoShots = 0;
  private int teleopShots = 0;
  // Zone-aware speed thresholds for feeding and drive limiting.
  // SHOOT_ON_THE_MOVE: max speed for hub shots in open alliance zone.
  // Used for both spindexer gating and active drive speed limiting.
  private final LoggedTunableNumber shootOnTheMoveSpeedMps =
      new LoggedTunableNumber("Shots/SpeedLimits/ShootOnTheMoveSpeedMps", 1.25);
  // PASS / LONG_PASS: max speed for pass shots in neutral/opponent zones.
  private final LoggedTunableNumber passSpeedMps =
      new LoggedTunableNumber("Shots/SpeedLimits/PassSpeedMps", 3.0);
  // Auto-specific pass speed — lower than teleop to work with PathPlanner speed zones.
  // Set a PathPlanner velocity constraint (e.g. 1.0 m/s) in the pass zone, and this
  // threshold just above it (1.1 m/s) so passing only happens during the slow segment.
  private final LoggedTunableNumber autoPassSpeedMps =
      new LoggedTunableNumber("Shots/SpeedLimits/AutoPassSpeedMps", 1.1);

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

    // Initialize shot strategies — completely independent, no shared efficiency
    // model
    this.lutStrategy = new LUTShotStrategy(lookupTable);
    this.alternateLutStrategy = new LUTShotStrategy(alternateLookupTable);
    this.parametricWithLutFallback =
        new ParametricWithLUTFallbackStrategy(parametricStrategy, lutStrategy);
    this.parametricWithProceduralFallback =
        new ParametricWithProceduralFallbackStrategy(lutStrategy);
    this.activeStrategy = parametricWithProceduralFallback;

    // Strategy dropdown on dashboard — four options
    strategyChooser.addOption("Parametric", "Parametric");
    strategyChooser.addOption("LUT (Lookup Table)", "LUT");
    strategyChooser.addOption("LUT Alternate", "LUT_ALTERNATE");
    strategyChooser.addOption("Parametric + LUT Fallback", "PARAMETRIC_LUT_FALLBACK");
    strategyChooser.setDefaultOption(
        "Parametric + Procedural Fallback", "PARAMETRIC_PROCEDURAL_FALLBACK");
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

    if (Constants.currentMode == Constants.Mode.SIM) {
      this.visualizer =
          new ShotVisualizer(
              pose3dSupplier,
              speedsSupplier,
              turretConfig.heightMeters(),
              turretConfig.xOffset(),
              turretConfig.yOffset());
    }
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
    updateCoordinatorState();
    updateTrenchHoodSafety();
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

    // Visualization runs AFTER all control logic (passive observer, throttled to
    // 10Hz)
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

      // Zone detection — uses turret field position for trench detection (hood
      // clearance)
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

      cachedAimResult = aimResult;

      // Update distance to aim target every cycle (for dashboard and shot calculations)
      currentDistanceM =
          Math.hypot(
              aimResult.target().getX() - turretFieldPos[0],
              aimResult.target().getY() - turretFieldPos[1]);

      // Zone is fully determined by position — ZoneDetector computes distance to hub
      // internally for alliance sub-zone classification (CLOSE/MID/FAR).
      trenchModeActive =
          trenchModeEnabled
              && (aimResult.zone() == ZoneDetector.Zone.ALLIANCE_TRENCH
                  || aimResult.zone() == ZoneDetector.Zone.NEUTRAL_TRENCH
                  || aimResult.zone() == ZoneDetector.Zone.BUMP);

      // Log zone and aim mode (throttled to 10Hz)
      if (periodicCounter % 5 == 0) {
        Logger.recordOutput("SmartLaunch/Status/Zone", aimResult.zone().name());
        Logger.recordOutput("SmartLaunch/Status/AllowedAction", aimResult.mode().name());
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

      // Invalidate all cached targets when alliance changes
      boolean allianceChanged = cachedPassAlliance != alliance;
      if (allianceChanged) {
        cachedPassAlliance = alliance;
        cachedLeftTarget = null;
        cachedLobStation1Target = null;
      }

      // Recompute symmetric pass targets when tunables or alliance change
      if (cachedLeftTarget == null
          || LoggedTunableNumber.hasChanged(
              passLeftAdjustX, passLeftAdjustY, passRightAdjustX, passRightAdjustY)) {
        // Left trench target (from driver perspective: high Y for blue, low Y for red)
        double leftBaseY =
            isBlueAlliance
                ? Constants.StrategyConstants.LEFT_PASS_TARGET_Y
                : Constants.StrategyConstants.RIGHT_PASS_TARGET_Y;
        double leftRawX = baseX + passLeftAdjustX.get() * (maxX - minX) / 2.0;
        double leftRawY = leftBaseY + passLeftAdjustY.get() * (maxY - minY) / 2.0;
        cachedLeftTarget =
            new Translation3d(
                Math.max(minX, Math.min(maxX, leftRawX)),
                Math.max(minY, Math.min(maxY, leftRawY)),
                0.0);
        Logger.recordOutput(
            "SmartLaunch/Pass/Symmetric/Left", new Pose3d(cachedLeftTarget, Rotation3d.kZero));

        // Right trench target (from driver perspective: low Y for blue, high Y for red)
        double rightBaseY =
            isBlueAlliance
                ? Constants.StrategyConstants.RIGHT_PASS_TARGET_Y
                : Constants.StrategyConstants.LEFT_PASS_TARGET_Y;
        double rightRawX = baseX + passRightAdjustX.get() * (maxX - minX) / 2.0;
        double rightRawY = rightBaseY + passRightAdjustY.get() * (maxY - minY) / 2.0;
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

      // Reset movingInTrench for non-hub-shot modes. The flag is only updated inside
      // calculateShotToTarget (called for SHOOT_ON_THE_MOVE/NONE), but PASS/LONG_PASS
      // call calculatePassToTarget instead, leaving the flag stale from the last trench visit.
      if (aimResult.mode() == TurretAimingHelper.AimMode.PASS
          || aimResult.mode() == TurretAimingHelper.AimMode.LONG_PASS) {
        movingInTrench = false;
      }

      switch (aimResult.mode()) {
        case SHOOT_ON_THE_MOVE -> {
          Logger.recordOutput("SmartLaunch/Status/Strategy", "Hub " + activeStrategy.getName());
          calculateShotToHub(robotPose, fieldSpeeds, isBlueAlliance);
        }
        case PASS -> {
          PassingStrategy strategy = passingStrategyChooser.getSelected();

          if (strategy == PassingStrategy.DRIVER_STATION && !isTooCloseToHub(robotPose)) {
            Logger.recordOutput("SmartLaunch/Status/Strategy", "Pass Lob");
            var location = DriverStation.getLocation();
            int station = location.isPresent() ? location.getAsInt() : 2;
            Translation3d activeTarget =
                (station <= 2) ? cachedLobStation1Target : cachedLobStation3Target;
            Logger.recordOutput(
                "SmartLaunch/Pass/Target", (station <= 2) ? "STATION_1" : "STATION_3");
            calculatePassToTarget(
                robotPose, fieldSpeeds, activeTarget, PassingStrategy.DRIVER_STATION);
          } else {
            // Symmetric low pass (also used as fallback when too close to hub for lob)
            if (strategy == PassingStrategy.DRIVER_STATION) {
              Logger.recordOutput("SmartLaunch/Status/Strategy", "Pass Low (hub fallback)");
            } else {
              Logger.recordOutput("SmartLaunch/Status/Strategy", "Pass Low");
            }
            boolean isLeftTrench = selectIsLeftTrench(robotPose);
            Translation3d activeTarget = isLeftTrench ? cachedLeftTarget : cachedRightTarget;
            Logger.recordOutput("SmartLaunch/Pass/Target", isLeftTrench ? "LEFT" : "RIGHT");
            calculatePassToTarget(robotPose, fieldSpeeds, activeTarget, PassingStrategy.SYMMETRIC);
          }
        }
        case LONG_PASS -> {
          Logger.recordOutput("SmartLaunch/Status/Strategy", "Pass LongLob");
          boolean isLeftTrench = selectIsLeftTrench(robotPose);
          Translation3d activeTarget =
              isLeftTrench ? cachedLobStation1Target : cachedLobStation3Target;
          Logger.recordOutput("SmartLaunch/Pass/Target", isLeftTrench ? "LOB_LEFT" : "LOB_RIGHT");
          calculatePassToTarget(
              robotPose, fieldSpeeds, activeTarget, PassingStrategy.DRIVER_STATION);
        }
        case NONE -> {
          Logger.recordOutput("SmartLaunch/Status/Strategy", "Hub " + activeStrategy.getName());
          calculateShotToHub(robotPose, fieldSpeeds, isBlueAlliance);
        }
      }
    }
  }

  /** Check if turret is too close to hub center for a reliable lob pass (< 4m). */
  private boolean isTooCloseToHub(Pose2d robotPose) {
    double robotHeadingRad = robotPose.getRotation().getRadians();
    double[] turretFieldPos =
        ShotCalculator.getTurretFieldPosition(
            robotPose.getX(), robotPose.getY(), robotHeadingRad, turretConfig);
    boolean isBlue = RobotStatus.isBlueAlliance();
    double hubCenterX =
        isBlue ? FieldConstants.LinesVertical.hubCenter : FieldConstants.LinesVertical.oppHubCenter;
    double hubCenterY = FieldConstants.fieldWidth / 2.0;
    double dist =
        Math.sqrt(
            Math.pow(hubCenterX - turretFieldPos[0], 2)
                + Math.pow(hubCenterY - turretFieldPos[1], 2));
    return dist < 4.0;
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
    // Default: symmetric Y-based selection. Left trench = high Y for blue,
    // low Y for red (targets are already alliance-swapped, so flip selection).
    boolean highY = robotPose.getY() > FieldConstants.fieldWidth / 2.0;
    return RobotStatus.isBlueAlliance() ? highY : !highY;
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

    // Apply operator turret trim — rotate aim target around turret field position
    if (turretTrimDeg != 0.0) {
      double headingRad = robotPose.getRotation().getRadians();
      double[] tfp =
          ShotCalculator.getTurretFieldPosition(
              robotPose.getX(), robotPose.getY(), headingRad, turretConfig);
      target = applyTurretTrim(target, tfp[0], tfp[1]);
    }

    // Update active strategy based on tunable selector
    updateActiveStrategy();

    double hoodMin = hood != null ? hood.getMinAngle() : 16.0;
    double hoodMax = hood != null ? hood.getMaxAngle() : 46.0;

    // Safety: always clamp hood to trench-safe angle when moving in a trench zone,
    // regardless of trenchModeEnabled. Prevents the hood from hitting the trench structure.
    // When stationary in trench, full hood range is allowed for better shot accuracy.
    boolean inTrenchZone =
        cachedAimResult != null
            && (cachedAimResult.zone() == ZoneDetector.Zone.ALLIANCE_TRENCH
                || cachedAimResult.zone() == ZoneDetector.Zone.NEUTRAL_TRENCH);
    boolean inNeutralTrench =
        cachedAimResult != null && cachedAimResult.zone() == ZoneDetector.Zone.NEUTRAL_TRENCH;
    if (inTrenchZone) {
      double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
      // Hysteresis: clamp hood almost instantly when accelerating (0.1 m/s), but don't
      // unclamp until well into deceleration (0.5 m/s). Wide band prevents toggling.
      double threshold =
          movingInTrench
              ? trenchHoodUnclampSpeedMps.get() // already moving: drop below 0.5 to unclamp
              : trenchHoodClampSpeedMps.get(); // stopped: clamp as soon as above 0.1
      boolean isMoving = robotSpeed > threshold;
      // Hood stays clamped to min while moving in any trench, and always in neutral trench.
      // Only pops up when stationary in the alliance trench.
      if (isMoving || inNeutralTrench) {
        hoodMax = hoodMin;
      }
      movingInTrench = isMoving;
    } else {
      movingInTrench = false;
    }

    // Trench RPM lerp (only when trenchModeEnabled and moving in trench)
    boolean useTrenchLerp = false;
    if (trenchModeActive) {
      double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
      if (robotSpeed > trenchMovingThresholdMps.get()) {
        useTrenchLerp = true;
      }
    }

    double currentTurretAngle = turret.getOutsideCurrentAngle();
    double turretMin = turret.getMinAngle();
    double turretMax = turret.getMaxAngle();

    // In trench mode while moving, use empirical RPM lerp: interpolate between two tested
    // endpoints (close and far distance) with fixed hood angle. No physics solver — pure
    // empirical data, independent of the global launch efficiency.
    // When stationary in trench, the normal strategy runs with full hood range.
    ShotCalculator.ShotResult activeResult;
    if (useTrenchLerp) {
      double robotHeadingRad = robotPose.getRotation().getRadians();
      double[] turretFieldPos =
          ShotCalculator.getTurretFieldPosition(
              robotPose.getX(), robotPose.getY(), robotHeadingRad, turretConfig);
      double distToHub =
          Math.hypot(target.getX() - turretFieldPos[0], target.getY() - turretFieldPos[1]);

      // Pick left or right trench lerp parameters based on robot Y position
      boolean isLeftTrench = robotPose.getY() > FieldConstants.fieldWidth / 2.0;
      double dClose = isLeftTrench ? trenchLeftCloseDistM.get() : trenchRightCloseDistM.get();
      double dFar = isLeftTrench ? trenchLeftFarDistM.get() : trenchRightFarDistM.get();
      double rpmClose = isLeftTrench ? trenchLeftCloseRPM.get() : trenchRightCloseRPM.get();
      double rpmFar = isLeftTrench ? trenchLeftFarRPM.get() : trenchRightFarRPM.get();
      // Lerp RPM between close and far, clamp outside
      double t = Math.max(0, Math.min(1, (distToHub - dClose) / (dFar - dClose)));
      double trenchRPM = rpmClose + t * (rpmFar - rpmClose);

      // Lerp motivator RPM using the same t (same distance endpoints)
      double motClose =
          isLeftTrench ? trenchLeftMotivatorCloseRPM.get() : trenchRightMotivatorCloseRPM.get();
      double motFar =
          isLeftTrench ? trenchLeftMotivatorFarRPM.get() : trenchRightMotivatorFarRPM.get();
      trenchMotivatorRPM = motClose + t * (motFar - motClose);

      double turretAngleDeg =
          ShotCalculator.calculateOutsideTurretAngle(
              robotPose.getX(),
              robotPose.getY(),
              robotPose.getRotation().getDegrees(),
              target.getX(),
              target.getY(),
              currentTurretAngle,
              turretMin,
              turretMax,
              turretConfig);

      double trenchExitVelocity = ShotCalculator.calculateExitVelocityFromRPM(trenchRPM, distToHub);
      double trenchLaunchAngleRad = Math.toRadians(90.0 - hoodMax);

      // Velocity compensation: shift aim target to counteract robot movement during flight
      Translation3d aimTarget =
          new edu.wpi.first.math.geometry.Translation3d(
              target.getX(), target.getY(), target.getZ());
      double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
      if (robotSpeed > 0.1 && trenchExitVelocity > 0.1) {
        double vx = trenchExitVelocity * Math.cos(trenchLaunchAngleRad);
        double horizontalTof = (vx > 0.1) ? distToHub / vx : 0.0;
        aimTarget = ShotCalculator.predictTargetPos(aimTarget, fieldSpeeds, horizontalTof);

        // Recalculate turret angle to the velocity-compensated target
        turretAngleDeg =
            ShotCalculator.calculateOutsideTurretAngle(
                robotPose.getX(),
                robotPose.getY(),
                robotPose.getRotation().getDegrees(),
                aimTarget.getX(),
                aimTarget.getY(),
                currentTurretAngle,
                turretMin,
                turretMax,
                turretConfig);
      }

      activeResult =
          new ShotCalculator.ShotResult(
              trenchExitVelocity,
              trenchRPM,
              trenchLaunchAngleRad,
              hoodMax, // fixed hood angle
              turretAngleDeg,
              aimTarget,
              true);

      if (periodicCounter % 5 == 0) {
        Logger.recordOutput("SmartLaunch/TrenchLerp/Side", isLeftTrench ? "LEFT" : "RIGHT");
        Logger.recordOutput("SmartLaunch/TrenchLerp/DistToHub", distToHub);
        Logger.recordOutput("SmartLaunch/TrenchLerp/RPM", trenchRPM);
        Logger.recordOutput("SmartLaunch/TrenchLerp/MotivatorRPM", trenchMotivatorRPM);
        Logger.recordOutput("SmartLaunch/TrenchLerp/T", t);
        Logger.recordOutput("SmartLaunch/TrenchLerp/VelocityCompActive", robotSpeed > 0.1);
      }
      Logger.recordOutput("SmartLaunch/Status/UsingTrenchParametric", true);
    } else {
      // Normal strategy runs in open field
      activeResult =
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
      Logger.recordOutput("SmartLaunch/Status/UsingTrenchParametric", false);
    }
    currentShot = activeResult;

    // Throttle target/distance logging to ~10Hz (distance itself is updated every
    // cycle in updateShotCalculation for zone refinement)
    if (periodicCounter % 5 == 0) {
      Logger.recordOutput("SmartLaunch/Status/TargetX", target.getX());
      Logger.recordOutput("SmartLaunch/Status/TargetY", target.getY());
      Logger.recordOutput("SmartLaunch/Status/DistanceM", currentDistanceM);
      Logger.recordOutput(
          "SmartLaunch/Status/Efficiency", ShotCalculator.getEfficiency(currentDistanceM));
      if (currentShot != null && currentShot.aimTarget() != null) {
        Logger.recordOutput("SmartLaunch/Status/AimTargetX", currentShot.aimTarget().getX());
        Logger.recordOutput("SmartLaunch/Status/AimTargetY", currentShot.aimTarget().getY());
      }
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

    // Apply operator turret trim — rotate aim target around turret field position
    target = applyTurretTrim(target, turretX, turretY);

    double horizontalDist =
        Math.sqrt(Math.pow(target.getX() - turretX, 2) + Math.pow(target.getY() - turretY, 2));

    // For passes, allow the full theoretical hood range — the solver will compute
    // the
    // ideal angle and the result gets clamped to mechanical limits at command time.
    // Hub shots enforce strict hood limits, but passes need steep angles (50-60°)
    // that
    // would be rejected by the normal [13-46] range.
    double hoodMin = 0.0;
    double hoodMax = 90.0;

    double constraintX;
    double constraintH;
    double maxPeakHeight;

    if (strategy == PassingStrategy.DRIVER_STATION) {
      // LOB: clearance point is at the hub net (proximity check already done at
      // dispatch)
      boolean isBlue = RobotStatus.isBlueAlliance();
      double hubCenterX =
          isBlue
              ? FieldConstants.LinesVertical.hubCenter
              : FieldConstants.LinesVertical.oppHubCenter;
      double hubCenterY = FieldConstants.fieldWidth / 2.0;

      // Project hub center onto the shot line to get distance along shot direction
      double dx = target.getX() - turretX;
      double dy = target.getY() - turretY;
      double shotLen = Math.sqrt(dx * dx + dy * dy);
      if (shotLen < 0.01) shotLen = 0.01;
      double shotDirX = dx / shotLen;
      double shotDirY = dy / shotLen;

      double hubDistAlongShot =
          (hubCenterX - turretX) * shotDirX + (hubCenterY - turretY) * shotDirY;

      if (hubDistAlongShot <= 0.5 || hubDistAlongShot >= horizontalDist - 0.5) {
        // Shot doesn't cross the hub — use low arc
        constraintX = horizontalDist / 2.0;
        constraintH = symmetricArcPeakHeightIn.get() * 0.0254;
        maxPeakHeight = symmetricArcPeakHeightIn.get() * 0.0254 + 1.0;
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
      constraintH = symmetricArcPeakHeightIn.get() * 0.0254;
      maxPeakHeight =
          symmetricArcPeakHeightIn.get() * 0.0254 + 1.0; // allow small margin above desired peak
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
      Logger.recordOutput("SmartLaunch/Status/DistanceMCPT", horizontalDist);
    }
  }

  // ========== Shot State Logging ==========

  /**
   * Log current shot state to AdvantageKit every cycle. Runs in periodic() so readiness, targets,
   * and tracking errors are always visible — regardless of which command (smart launch, fixed shot,
   * auto-track, or none) is active.
   */
  private void logShotState() {
    // --- Readiness flags (use subsystem state machines, not raw at-setpoint
    // checks) ---
    boolean launcherReady = launcher != null && launcher.isReady();
    boolean motivatorReady =
        motivator == null || motivator.getState() == Motivator.MotivatorState.READY;
    boolean turretReady = turret.atTarget();
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
    Logger.recordOutput("LUTDev/OverridesActive", overridesActive);

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
        "SmartLaunch/Target/Achievable", currentShot != null && currentShot.achievable());
    Logger.recordOutput(
        "SmartLaunch/Target/MotivatorRPM",
        frc.robot.commands.ShootingCommands.getMotivatorRPM(targetRPM, this));
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
      Logger.recordOutput("SmartLaunch/Status/DistanceMLSS", distance);

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
      // The RPM-based exitVelocityMps underestimates because it doesn't account for
      // the
      // motivator's contribution to ball speed. Solving for the velocity that hits
      // the target
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
    if (fieldSpeedsSupplier == null || cachedAimResult == null) return true;

    ChassisSpeeds speeds = fieldSpeedsSupplier.get();
    double robotSpeedMps = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    TurretAimingHelper.AimMode mode = cachedAimResult.mode();
    // In auto, if passing is disabled, treat PASS/LONG_PASS as no-fire zones.
    boolean passingBlocked =
        (mode == TurretAimingHelper.AimMode.PASS || mode == TurretAimingHelper.AimMode.LONG_PASS)
            && DriverStation.isAutonomous()
            && !autoPassingEnabled;

    double thresholdMps =
        switch (mode) {
          case SHOOT_ON_THE_MOVE -> shootOnTheMoveSpeedMps.get();
          case PASS, LONG_PASS -> DriverStation.isAutonomous()
              ? autoPassSpeedMps.get()
              : passSpeedMps.get();
          case NONE -> 0.0;
        };
    boolean slowEnough =
        mode != TurretAimingHelper.AimMode.NONE && !passingBlocked && robotSpeedMps <= thresholdMps;

    Logger.recordOutput("SmartLaunch/SpeedCheck/RobotMps", robotSpeedMps);
    Logger.recordOutput("SmartLaunch/SpeedCheck/ThresholdMps", thresholdMps);
    Logger.recordOutput("SmartLaunch/SpeedCheck/SlowEnough", slowEnough);

    return slowEnough;
  }

  /**
   * Check if the robot is currently in a pass zone (PASS or LONG_PASS aim mode). Used by
   * continuousSmartLaunch to arm feeding only after reaching the pass zone during sprint autos.
   *
   * @return true if the robot is in a zone where passing is the aim mode
   */
  public boolean isInPassZone() {
    if (cachedAimResult == null) return false;
    return cachedAimResult.mode() == TurretAimingHelper.AimMode.PASS
        || cachedAimResult.mode() == TurretAimingHelper.AimMode.LONG_PASS;
  }

  /**
   * Check if the launcher should idle at 0 RPM. True when the operator is suppressing feeding and
   * the robot is outside alliance zones (neutral/opponent), where there is no reason to keep the
   * flywheel spinning.
   */
  public boolean shouldIdleLauncher() {
    return feedingSuppressedSupplier.getAsBoolean() && !isInAllianceZone();
  }

  /**
   * Check if shooting subsystems should idle to conserve power. True in auto when:
   * - In open neutral/opponent zones (collecting balls), OR
   * - Not yet armed (heading outbound before visiting neutral — no reason to spin up)
   * NOT true in trenches after armed — when returning through neutral trench,
   * launcher/turret/motivator should track so they're ready at the alliance trench boundary.
   * In teleop this always returns false.
   */
  public boolean isAutoCollecting() {
    if (!DriverStation.isAutonomous()) return false;
    if (cachedAimResult == null) return false;
    if (armTrigger == ArmTrigger.IMMEDIATE) return false;
    // Outbound (haven't visited neutral yet) — idle everywhere
    if (!hasVisitedNeutral) return true;
    // Returning: in neutral/opponent zones, idle only if passing is disabled.
    // AUTO_SHOOT (passing enabled) should spin up for pass shots in neutral.
    boolean inNeutralOrOpponent =
        cachedAimResult.zone() == ZoneDetector.Zone.NEUTRAL
            || cachedAimResult.zone() == ZoneDetector.Zone.OPPONENT;
    if (inNeutralOrOpponent && !autoPassingEnabled) return true;
    return false;
  }

  /**
   * Get the effective launcher RPM, accounting for auto collecting idle and operator suppression.
   * Returns 0 when idling, or the full shot RPM otherwise.
   */
  public double getEffectiveLauncherRPM(double shotRPM) {
    if (shouldIdleLauncher()) return 0;
    if (isAutoCollecting()) return 0;
    return shotRPM;
  }

  /**
   * Check if the robot is in an alliance zone where hub shooting (and agitation) is appropriate.
   *
   * @return true if in ALLIANCE_CLOSE, ALLIANCE_MID, ALLIANCE_FAR, or ALLIANCE_TRENCH
   */
  public boolean isInAllianceZone() {
    if (cachedAimResult == null) return true; // Default to alliance behavior
    ZoneDetector.Zone zone = cachedAimResult.zone();
    return zone == ZoneDetector.Zone.ALLIANCE_CLOSE
        || zone == ZoneDetector.Zone.ALLIANCE_MID
        || zone == ZoneDetector.Zone.ALLIANCE_FAR
        || zone == ZoneDetector.Zone.ALLIANCE_TRENCH;
  }

  /**
   * Get the current zone the robot is in, refined to ALLIANCE_CLOSE/MID/FAR when distance is
   * available.
   *
   * @return Current zone, or ALLIANCE_MID as fallback if pose is unavailable
   */
  public ZoneDetector.Zone getCurrentZone() {
    if (cachedAimResult == null) return ZoneDetector.Zone.ALLIANCE_MID;
    return cachedAimResult.zone();
  }

  // ========== Trench Avoidance Mode ==========

  /**
   * Check if trench avoidance is currently active (robot is in a trench zone and trench mode is
   * enabled). When trench mode is disabled, this always returns false.
   */
  public boolean isTrenchModeActive() {
    return trenchModeActive;
  }

  /** Toggle trench mode on/off. */
  public void toggleTrenchMode() {
    trenchModeEnabled = !trenchModeEnabled;
  }

  /** Enable or disable trench mode. When disabled, smartShot runs normally under the trench. */
  public void setTrenchModeEnabled(boolean enabled) {
    this.trenchModeEnabled = enabled;
  }

  /** Check whether trench mode is enabled. */
  public boolean isTrenchModeEnabled() {
    return trenchModeEnabled;
  }

  /**
   * Get the distance-lerped motivator RPM for the current trench position. Only meaningful when
   * trench mode is active.
   */
  public double getTrenchMotivatorRPM() {
    return trenchMotivatorRPM;
  }

  /**
   * Check if trench hood safety is active (hood above safe angle while moving in trench). When
   * active, drive speed is limited and shooting is suppressed until the hood lowers.
   */
  public boolean isTrenchHoodSafetyActive() {
    return trenchHoodSafetyActive;
  }

  /**
   * Update trench hood safety state. When moving in a trench zone with the hood physically above
   * the safe angle (trenchHoodMaxDeg), limits drive speed to trenchSafetySpeedLimitMps and lets the
   * hood lower. Shooting is naturally suppressed because the hood won't be at its target yet. Once
   * the hood reaches the safe angle, the speed limit is cleared.
   */
  private void updateTrenchHoodSafety() {
    boolean wasActive = trenchHoodSafetyActive;

    boolean inTrenchZone =
        cachedAimResult != null
            && (cachedAimResult.zone() == ZoneDetector.Zone.ALLIANCE_TRENCH
                || cachedAimResult.zone() == ZoneDetector.Zone.NEUTRAL_TRENCH);

    if (inTrenchZone && hood != null && fieldSpeedsSupplier != null) {
      ChassisSpeeds speeds = fieldSpeedsSupplier.get();
      double robotSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      boolean isMoving = robotSpeed > trenchMovingThresholdMps.get();
      boolean hoodAboveSafe = hood.getCurrentAngle() > trenchHoodMaxDeg.get();

      trenchHoodSafetyActive = isMoving && hoodAboveSafe;
    } else {
      trenchHoodSafetyActive = false;
    }

    // Apply or release drive speed limit on state transitions
    if (trenchHoodSafetyActive) {
      DriveCommands.setSpeedLimit(trenchSafetySpeedLimitMps.get());
    } else if (wasActive) {
      DriveCommands.clearSpeedLimit();
    }

    if (periodicCounter % 5 == 0) {
      Logger.recordOutput("SmartLaunch/TrenchHoodSafety/Active", trenchHoodSafetyActive);
    }
  }

  // ========== CoordinatorState Machine API ==========

  /**
   * Notify the coordinator that SmartLaunch has started or stopped. Called by the SmartLaunch
   * command on start/end to drive INACTIVE transitions. Uses IMMEDIATE arm trigger (teleop default).
   */
  public void setSmartLaunchActive(boolean active) {
    setSmartLaunchActive(active, ArmTrigger.IMMEDIATE);
  }

  /**
   * Notify the coordinator that SmartLaunch has started or stopped, with a specific arm trigger.
   * For sprint autos, use ON_PASS_ZONE or ON_TRENCH_RETURN to prevent shooting preloads at start.
   */
  public void setSmartLaunchActive(boolean active, ArmTrigger trigger) {
    this.smartLaunchActive = active;
    if (!active) {
      coordinatorState = CoordinatorState.INACTIVE;
      armed = true;
      hasVisitedNeutral = false;
      readyTimeoutRunning = false;
    } else if (coordinatorState == CoordinatorState.INACTIVE) {
      armTrigger = trigger;
      armed = (trigger == ArmTrigger.IMMEDIATE);
      coordinatorState = armed ? CoordinatorState.AIMING : CoordinatorState.UNARMED;
      hasVisitedNeutral = false;
      readyTimeoutTimer.restart();
      readyTimeoutRunning = true;
      // Initialize previous-state tracking so the first cycle doesn't false-trigger transitions
      previousTrenchMode = trenchModeActive;
      previousAimMode = cachedAimResult != null ? cachedAimResult.mode() : null;
    }
  }

  /** Get the current coordinator state. */
  public CoordinatorState getCoordinatorState() {
    return coordinatorState;
  }

  /**
   * Check if feeding is allowed by the coordinator state machine. Only true in FIRING state — all
   * subsystems at target, no transitions in progress, no suppression active.
   */
  public boolean isFeedingAllowed() {
    return coordinatorState == CoordinatorState.FIRING;
  }

  private boolean firingEntryPending = false;

  /**
   * Returns true exactly once each time the coordinator enters FIRING state. Used by
   * continuousSmartLaunch to trigger a brief reverse pulse before feeding forward.
   */
  public boolean consumeFiringEntry() {
    if (firingEntryPending) {
      firingEntryPending = false;
      return true;
    }
    return false;
  }

  /**
   * Update the coordinator state machine. Called every cycle from periodic() after shot
   * calculation. Uses event-driven entry (specific triggers) and condition-driven exit (subsystem
   * readiness) to avoid false positives during normal shoot-on-the-move tracking.
   */
  private void updateCoordinatorState() {
    // Not active — stay idle
    if (!smartLaunchActive) {
      coordinatorState = CoordinatorState.INACTIVE;
      return;
    }

    ZoneDetector.Zone currentZone =
        cachedAimResult != null ? cachedAimResult.zone() : ZoneDetector.Zone.ALLIANCE_MID;
    TurretAimingHelper.AimMode currentAimMode =
        cachedAimResult != null ? cachedAimResult.mode() : null;

    // --- Arming logic (sprint-start protection) ---
    // Track whether the robot has visited neutral/opponent zone
    if (currentZone == ZoneDetector.Zone.NEUTRAL || currentZone == ZoneDetector.Zone.OPPONENT) {
      hasVisitedNeutral = true;
    }
    // Reset cycle: when we re-enter neutral trench after shooting, we're heading back
    // out. Disarm and clear hasVisitedNeutral so the outbound trip idles subsystems.
    if (armed && hasVisitedNeutral
        && currentZone == ZoneDetector.Zone.NEUTRAL_TRENCH
        && armTrigger != ArmTrigger.IMMEDIATE) {
      armed = false;
      hasVisitedNeutral = false;
      // Force to UNARMED so feeding stops immediately
      coordinatorState = CoordinatorState.UNARMED;
      readyTimeoutTimer.restart();
      readyTimeoutRunning = true;
    }
    // Check arm conditions based on trigger type
    if (!armed) {
      switch (armTrigger) {
        case IMMEDIATE -> armed = true;
        case ON_PASS_ZONE -> {
          // Arm when entering pass zone (neutral/opponent)
          if (hasVisitedNeutral) {
            armed = true;
            Logger.recordOutput("SmartLaunch/Armed", true);
          }
        }
        case ON_TRENCH_RETURN -> {
          // Arm when entering alliance trench after having visited neutral
          if (hasVisitedNeutral && currentZone == ZoneDetector.Zone.ALLIANCE_TRENCH) {
            armed = true;
            Logger.recordOutput("SmartLaunch/Armed", true);
          }
        }
      }
    }
    // Transition UNARMED → AIMING once armed
    if (armed && coordinatorState == CoordinatorState.UNARMED) {
      coordinatorState = CoordinatorState.AIMING;
      readyTimeoutTimer.restart();
      readyTimeoutRunning = true;
    }

    // Readiness variables — declared here so they're available for logging in all paths
    boolean launcherReady = false;
    boolean turretReady = false;
    boolean hoodReady = false;
    boolean shotAchievable = false;
    boolean speedOk = false;
    boolean allReady = false;

    CoordinatorState prevState = coordinatorState;

    // --- UNARMED trumps everything — skip all state logic until armed ---
    if (coordinatorState == CoordinatorState.UNARMED) {
      previousTrenchMode = trenchModeActive;
      previousAimMode = currentAimMode;
    }
    // --- Zone suppression check ---
    else if (currentZone == ZoneDetector.Zone.NEUTRAL_TRENCH
        || currentZone == ZoneDetector.Zone.BUMP) {
      coordinatorState = CoordinatorState.NO_FIRE_ZONE;
      readyTimeoutRunning = false;
      // Still update previous values so we detect the transition OUT correctly
      previousTrenchMode = trenchModeActive;
      previousAimMode = currentAimMode;
    } else if (coordinatorState == CoordinatorState.NO_FIRE_ZONE) {
      // --- Leaving zone suppression → transition (subsystems need to settle) ---
      coordinatorState = CoordinatorState.SETTLING;
      readyTimeoutTimer.restart();
      readyTimeoutRunning = true;
      previousTrenchMode = trenchModeActive;
      previousAimMode = currentAimMode;
    } else if (feedingSuppressedSupplier.getAsBoolean()) {
      // --- Operator suppression check ---
      if (coordinatorState == CoordinatorState.FIRING
          || coordinatorState == CoordinatorState.HELD) {
        coordinatorState = CoordinatorState.HELD;
      }
      readyTimeoutRunning = false;
      previousTrenchMode = trenchModeActive;
      previousAimMode = currentAimMode;
    } else {
      // --- Detect transition triggers (only from FIRING state) ---
      if (coordinatorState == CoordinatorState.FIRING) {
        boolean trenchChanged = (trenchModeActive != previousTrenchMode);
        boolean aimModeChanged = (currentAimMode != previousAimMode);
        boolean turretFlipping = (turret.getState() == Turret.TurretState.FLIPPING);

        if (trenchChanged || aimModeChanged || turretFlipping) {
          coordinatorState = CoordinatorState.SETTLING;
          readyTimeoutTimer.restart();
          readyTimeoutRunning = true;
        }
      }

      // --- Update previous values for next cycle's change detection ---
      previousTrenchMode = trenchModeActive;
      previousAimMode = currentAimMode;

      // --- Readiness check for state advancement ---
      launcherReady = launcher != null && launcher.isReady();
      turretReady = turret.getState() == Turret.TurretState.READY;
      hoodReady = hood == null || hood.getState() == Hood.HoodState.READY;
      shotAchievable = currentShot != null && currentShot.achievable();
      boolean turretNotFlipping = turret.getState() != Turret.TurretState.FLIPPING;
      speedOk = isRobotSlowEnoughForCurrentZone();
      allReady = armed && launcherReady && turretReady && hoodReady && shotAchievable && speedOk;

      // --- Timeout: force FIRING if stuck in AIMING too long ---
      boolean timeoutForced = false;
      if (readyTimeoutRunning
          && readyTimeoutTimer.hasElapsed(readyTimeoutSec.get())
          && coordinatorState == CoordinatorState.AIMING) {
        coordinatorState = CoordinatorState.FIRING;
        readyTimeoutRunning = false;
        timeoutForced = true;
        Logger.recordOutput("SmartLaunch/TimeoutForced", true);
      } else {
        Logger.recordOutput("SmartLaunch/TimeoutForced", false);
      }

      if (!timeoutForced) {
        switch (coordinatorState) {
          case AIMING -> {
            if (allReady) {
              coordinatorState = CoordinatorState.FIRING;
            }
          }
          case SETTLING -> {
            if (allReady && turretNotFlipping) {
              coordinatorState = CoordinatorState.FIRING;
            }
          }
          case HELD -> {
            // feedingSuppressed was already checked above and returned false to reach here
            if (allReady) {
              coordinatorState = CoordinatorState.FIRING;
            }
          }
          case FIRING -> {
            // Drop back to AIMING if robot exceeds zone speed threshold
            if (!speedOk) {
              coordinatorState = CoordinatorState.AIMING;
              readyTimeoutTimer.restart();
              readyTimeoutRunning = true;
            }
            // Other transitions out (trench, turret flip, etc.) handled by trigger checks above
          }
          default -> {
            // INACTIVE/UNARMED handled at top
          }
        }
      }

      // Stop timeout timer when we reach FIRING normally
      if (coordinatorState == CoordinatorState.FIRING) {
        readyTimeoutRunning = false;
      }
    }

    // Flag every entry into FIRING so consumeFiringEntry() can trigger a reverse pulse.
    if (coordinatorState == CoordinatorState.FIRING && prevState != CoordinatorState.FIRING) {
      firingEntryPending = true;
      Logger.recordOutput("SmartLaunch/ReversePulse/FiringEntrySet", true);
    } else {
      Logger.recordOutput("SmartLaunch/ReversePulse/FiringEntrySet", false);
    }

    // Log state every cycle (50Hz) — always reached regardless of which branch executed
    Logger.recordOutput("SmartLaunch/CoordinatorState", coordinatorState.name());
    // Readiness details throttled to 10Hz (less critical)
    if (periodicCounter % 5 == 0) {
      Logger.recordOutput("SmartLaunch/Armed", armed);
      Logger.recordOutput("SmartLaunch/HasVisitedNeutral", hasVisitedNeutral);
      Logger.recordOutput("SmartLaunch/SM/LauncherReady", launcherReady);
      Logger.recordOutput("SmartLaunch/SM/TurretReady", turretReady);
      Logger.recordOutput("SmartLaunch/SM/HoodReady", hoodReady);
      Logger.recordOutput("SmartLaunch/SM/ShotAchievable", shotAchievable);
      Logger.recordOutput("SmartLaunch/SM/SpeedOk", speedOk);
      Logger.recordOutput("SmartLaunch/SM/AllReady", allReady);
    }
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
    if (selected == null) selected = "PARAMETRIC_PROCEDURAL_FALLBACK";

    ShotStrategy newStrategy;
    switch (selected) {
      case "Parametric" -> newStrategy = parametricStrategy;
      case "LUT" -> newStrategy = lutStrategy;
      case "LUT_ALTERNATE" -> newStrategy = alternateLutStrategy;
      case "PARAMETRIC_LUT_FALLBACK" -> newStrategy = parametricWithLutFallback;
      case "PARAMETRIC_PROCEDURAL_FALLBACK" -> newStrategy = parametricWithProceduralFallback;
      default -> newStrategy = parametricWithProceduralFallback;
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

  /**
   * Get the zone-appropriate drive speed limit. In ALLIANCE zone returns just under the
   * shoot-on-the-move threshold; in PASS/LONG_PASS zones returns the pass threshold. Returns max
   * speed for zones where shooting is suppressed (NONE).
   */
  public double getZoneSpeedLimitMps() {
    double maxSpeed = Constants.getRobotConfig().getMaxSpeedMetersPerSec();
    if (cachedAimResult == null) return maxSpeed;
    return switch (cachedAimResult.mode()) {
      case SHOOT_ON_THE_MOVE -> shootOnTheMoveSpeedMps.get() - 0.05;
      case PASS, LONG_PASS -> passSpeedMps.get() - 0.05;
      case NONE -> maxSpeed;
    };
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
      boolean turretReady = turret.atTarget();
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
