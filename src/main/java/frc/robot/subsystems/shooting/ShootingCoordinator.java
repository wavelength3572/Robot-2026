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

  // Shot strategies: fixed-height parabola for hub shots and passes.
  // Both use the same vertex-form solver with different peak heights.

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
  private final FixedHeightShotStrategy fixedHeightStrategy = new FixedHeightShotStrategy();
  private final FixedHeightPassStrategy fixedHeightPassStrategy = new FixedHeightPassStrategy();

  // Visualizer (created during initialize)
  private ShotVisualizer visualizer = null;

  // Throttle counter for non-critical logging (reduces loop time)
  private int periodicCounter = 0;
  private Supplier<Pose2d> robotPoseSupplier = null;
  private Supplier<ChassisSpeeds> fieldSpeedsSupplier = null;
  private DoubleSupplier pitchDegSupplier = () -> 0.0;

  // Optional feeding suppression check — when true, launchFuel() is a no-op
  private BooleanSupplier feedingSuppressedSupplier = () -> false;

  // Trench hood safety: the max hood angle considered safe under the trench structure.
  // If hood is above this while moving in trench, drive speed is limited until it lowers.
  private final LoggedTunableNumber trenchHoodMaxDeg =
      new LoggedTunableNumber(
          "Shots/TrenchMode/HoodMaxDeg", Constants.getRobotConfig().getTrenchHoodMaxDeg());

  // Trench hood safety: limits drive speed when hood is above safe angle while moving in trench.
  // Protects the hood from hitting the trench structure during transit.
  private final LoggedTunableNumber trenchSafetySpeedLimitMps =
      new LoggedTunableNumber(
          "Shots/TrenchMode/SafetySpeedLimitMps",
          Constants.getRobotConfig().getTrenchSafetySpeedLimitMps());
  private final LoggedTunableNumber trenchMovingThresholdMps =
      new LoggedTunableNumber(
          "Shots/TrenchMode/MovingThresholdMps",
          Constants.getRobotConfig().getTrenchMovingThresholdMps());
  // Hood clamp/unclamp thresholds with hysteresis. Clamp is low (fast response when
  // accelerating into trench), unclamp is higher (hood starts rising earlier when decelerating).
  private final LoggedTunableNumber trenchHoodClampSpeedMps =
      new LoggedTunableNumber(
          "Shots/TrenchMode/HoodClampSpeedMps",
          Constants.getRobotConfig().getTrenchHoodClampSpeedMps());
  private final LoggedTunableNumber trenchHoodUnclampSpeedMps =
      new LoggedTunableNumber(
          "Shots/TrenchMode/HoodUnclampSpeedMps",
          Constants.getRobotConfig().getTrenchHoodUnclampSpeedMps());
  private boolean trenchHoodSafetyActive = false;
  private boolean movingInTrench = false; // true when robot is moving in a trench zone

  // Danger trench safety: near-stops the robot when hood is above safe angle in the danger zone
  // at the alliance/neutral trench boundary. Timeout relaxes limit for broken-hood escape.
  private final LoggedTunableNumber dangerTrenchSpeedLimitMps =
      new LoggedTunableNumber("Shots/TrenchMode/DangerSpeedLimitMps", 0.3);
  private final LoggedTunableNumber dangerTrenchTimeoutSec =
      new LoggedTunableNumber("Shots/TrenchMode/DangerTimeoutSec", 2.0);
  private final LoggedTunableNumber dangerTrenchFallbackSpeedMps =
      new LoggedTunableNumber("Shots/TrenchMode/DangerFallbackSpeedMps", 2.0);
  private boolean dangerTrenchActive = false;
  private double dangerTrenchEntryTimestamp = 0.0;

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
  private TurretAimingHelper.AimMode previousAimMode = null;
  private ArmTrigger armTrigger = ArmTrigger.IMMEDIATE;
  private boolean armed = true; // true = feeding allowed once subsystems ready
  private boolean hasVisitedNeutral = false; // tracks whether robot has been to neutral zone
  private final Timer readyTimeoutTimer = new Timer();
  private boolean readyTimeoutRunning = false;
  private static final LoggedTunableNumber readyTimeoutSec =
      new LoggedTunableNumber(
          "Shots/SmartLaunch/ReadyTimeoutSec",
          Constants.getRobotConfig().getSmartLaunchReadyTimeoutSec());

  // Delayed-log state — snapshotted each cycle, logged next cycle to align with subsystem logs
  private CoordinatorState loggedCoordinatorState = CoordinatorState.INACTIVE;
  private CoordinatorState loggedPrevState = CoordinatorState.INACTIVE;
  private boolean loggedArmed = true;
  private boolean loggedHasVisitedNeutral = false;
  private String loggedStateReason = "";
  private String loggedBlocking = "none";

  // Cached aim result — computed once per cycle in updateShotCalculation(), used by all zone
  // queries (isRobotSlowEnoughForCurrentZone, isInPassZone, getCurrentZone, getZoneSpeedLimitMps)
  private TurretAimingHelper.AimResult cachedAimResult = null;
  private TurretAimingHelper.AimResult previousAimResult = null;

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
      new LoggedTunableNumber(
          "SmartLaunch/Pass/Left/AdjustX", Constants.getRobotConfig().getPassLeftAdjustX());
  private final LoggedTunableNumber passLeftAdjustY =
      new LoggedTunableNumber(
          "SmartLaunch/Pass/Left/AdjustY", Constants.getRobotConfig().getPassLeftAdjustY());
  private final LoggedTunableNumber passRightAdjustX =
      new LoggedTunableNumber(
          "SmartLaunch/Pass/Right/AdjustX", Constants.getRobotConfig().getPassRightAdjustX());
  private final LoggedTunableNumber passRightAdjustY =
      new LoggedTunableNumber(
          "SmartLaunch/Pass/Right/AdjustY", Constants.getRobotConfig().getPassRightAdjustY());

  // Two-point trajectory tunables for pass shots (dashboard value in inches,
  // converted to meters)
  private final LoggedTunableNumber symmetricArcPeakHeightMinIn =
      new LoggedTunableNumber(
          "SmartLaunch/Pass/Symmetric/ArcPeakHeightMinIn",
          Constants.getRobotConfig().getSymmetricArcPeakHeightMinIn());
  private final LoggedTunableNumber symmetricArcPeakHeightMaxIn =
      new LoggedTunableNumber(
          "SmartLaunch/Pass/Symmetric/ArcPeakHeightMaxIn",
          Constants.getRobotConfig().getSymmetricArcPeakHeightMaxIn());
  private final LoggedTunableNumber symmetricArcDistMinM =
      new LoggedTunableNumber(
          "SmartLaunch/Pass/Symmetric/ArcDistMinM",
          Constants.getRobotConfig().getSymmetricArcDistMinM());
  private final LoggedTunableNumber symmetricArcDistMaxM =
      new LoggedTunableNumber(
          "SmartLaunch/Pass/Symmetric/ArcDistMaxM",
          Constants.getRobotConfig().getSymmetricArcDistMaxM());
  private final LoggedTunableNumber passRpmPerDegCompensation =
      new LoggedTunableNumber(
          "SmartLaunch/Pass/RPMPerDegCompensation",
          Constants.getRobotConfig().getPassRpmPerDegCompensation());
  private final LoggedTunableNumber passMaxRpmCompensation =
      new LoggedTunableNumber(
          "SmartLaunch/Pass/MaxRPMCompensation",
          Constants.getRobotConfig().getPassMaxRpmCompensation());
  private final LoggedTunableNumber lobNetClearanceMarginM =
      new LoggedTunableNumber(
          "SmartLaunch/Pass/Lob/NetClearanceMarginM",
          Constants.getRobotConfig().getLobNetClearanceMarginM());
  private final LoggedTunableNumber lobMaxPeakHeightM =
      new LoggedTunableNumber(
          "SmartLaunch/Pass/Lob/MaxPeakHeightM", Constants.getRobotConfig().getLobMaxPeakHeightM());
  private final LoggedTunableNumber lobMinHubDistM =
      new LoggedTunableNumber(
          "SmartLaunch/Pass/Lob/MinHubDistM", Constants.getRobotConfig().getLobMinHubDistM());
  private final LoggedTunableNumber lobStation1AdjustY =
      new LoggedTunableNumber(
          "SmartLaunch/Pass/DriverStation/Station1/AdjustY",
          Constants.getRobotConfig().getLobStation1AdjustY());
  private final LoggedTunableNumber lobStation3AdjustY =
      new LoggedTunableNumber(
          "SmartLaunch/Pass/DriverStation/Station3/AdjustY",
          Constants.getRobotConfig().getLobStation3AdjustY());

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
      new LoggedTunableNumber(
          "Shots/SpeedLimits/ShootOnTheMoveSpeedMps",
          Constants.getRobotConfig().getShootOnTheMoveSpeedMps());
  // PASS / LONG_PASS: max speed for pass shots in neutral/opponent zones.
  private final LoggedTunableNumber passSpeedMps =
      new LoggedTunableNumber(
          "Shots/SpeedLimits/PassSpeedMps", Constants.getRobotConfig().getPassSpeedMps());
  // Auto-specific pass speed — lower than teleop to work with PathPlanner speed zones.
  // Set a PathPlanner velocity constraint (e.g. 1.0 m/s) in the pass zone, and this
  // threshold just above it (1.1 m/s) so passing only happens during the slow segment.
  private final LoggedTunableNumber autoPassSpeedMps =
      new LoggedTunableNumber(
          "Shots/SpeedLimits/AutoPassSpeedMps", Constants.getRobotConfig().getAutoPassSpeedMps());

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

    // Passing strategy chooser — how to pick left vs right pass target
    passingStrategyChooser.setDefaultOption("Symmetric (Y-based)", PassingStrategy.SYMMETRIC);
    passingStrategyChooser.addOption("Driver Station", PassingStrategy.DRIVER_STATION);
    SmartDashboard.putData("SmartLaunch/Pass/Strategy", passingStrategyChooser);
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
    updateCoordinatorState();
    updateTrenchHoodSafety();
    updateDangerTrenchSafety();
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

      // Hood trench clamping — runs for ALL aim modes so setClamped(false) is always
      // called when leaving a trench, regardless of whether we're shooting or passing.
      boolean inTrenchZone =
          aimResult.zone() == ZoneDetector.Zone.ALLIANCE_TRENCH
              || aimResult.zone() == ZoneDetector.Zone.NEUTRAL_TRENCH
              || aimResult.zone() == ZoneDetector.Zone.DANGER_TRENCH;
      boolean inNeutralTrench = aimResult.zone() == ZoneDetector.Zone.NEUTRAL_TRENCH;
      boolean inDangerTrench = aimResult.zone() == ZoneDetector.Zone.DANGER_TRENCH;
      if (inTrenchZone) {
        double robotSpeed =
            Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
        double threshold =
            movingInTrench ? trenchHoodUnclampSpeedMps.get() : trenchHoodClampSpeedMps.get();
        boolean isMoving = robotSpeed > threshold;
        // Neutral and danger zones always clamp; alliance trench does not clamp
        boolean hoodClamped = inNeutralTrench || inDangerTrench;
        if (hood != null) {
          hood.setClamped(hoodClamped);
        }
        movingInTrench = isMoving;
      } else {
        movingInTrench = false;
        if (hood != null) {
          hood.setClamped(false);
        }
      }

      // Update distance to aim target every cycle (for dashboard and shot calculations)
      currentDistanceM =
          Math.hypot(
              aimResult.target().getX() - turretFieldPos[0],
              aimResult.target().getY() - turretFieldPos[1]);

      // Log zone and aim mode delayed by one cycle so transitions line up with subsystem
      // state logs. Subsystems react to the zone in command execute() but don't log their
      // new state until next cycle's periodic(), so logging the previous zone here aligns them.
      if (previousAimResult != null) {
        Logger.recordOutput("SmartLaunch/Status/Zone", previousAimResult.zone().name());
        Logger.recordOutput("SmartLaunch/Status/AllowedAction", previousAimResult.mode().name());
      }
      previousAimResult = aimResult;

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

      switch (aimResult.mode()) {
        case SHOOT_ON_THE_MOVE -> {
          Logger.recordOutput("SmartLaunch/Status/Strategy", "Hub FixedHeight");
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
          Logger.recordOutput("SmartLaunch/Status/Strategy", "Hub FixedHeight");
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
      new LoggedTunableNumber(
          "SmartLaunch/Pass/MaxRPM", Constants.getRobotConfig().getPassMaxRPM());

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

    double hoodMin = hood != null ? hood.getMinAngle() : 16.0;
    double hoodMax = hood != null ? hood.getMaxAngle() : 46.0;

    // Force hood to minimum in neutral trench and danger zone (setClamped already handled above)
    boolean inNeutralTrench =
        cachedAimResult != null && cachedAimResult.zone() == ZoneDetector.Zone.NEUTRAL_TRENCH;
    boolean inDangerTrench =
        cachedAimResult != null && cachedAimResult.zone() == ZoneDetector.Zone.DANGER_TRENCH;
    if (inNeutralTrench || inDangerTrench) {
      hoodMax = hoodMin;
    }

    double currentTurretAngle = turret.getOutsideCurrentAngle();
    double turretMin = turret.getMinAngle();
    double turretMax = turret.getMaxAngle();

    currentShot =
        fixedHeightStrategy.calculateShot(
            robotPose,
            fieldSpeeds,
            target,
            turretConfig,
            currentTurretAngle,
            turretMin,
            turretMax,
            hoodMin,
            hoodMax);

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

  /** Interpolate symmetric arc peak height (inches) based on horizontal distance (meters). */
  private double interpolatePeakHeight(double horizontalDistM) {
    double distMin = symmetricArcDistMinM.get();
    double distMax = symmetricArcDistMaxM.get();
    double peakMin = symmetricArcPeakHeightMinIn.get();
    double peakMax = symmetricArcPeakHeightMaxIn.get();
    double t = Math.max(0.0, Math.min(1.0, (horizontalDistM - distMin) / (distMax - distMin)));
    double peakIn = peakMin + t * (peakMax - peakMin);
    Logger.recordOutput("SmartLaunch/Pass/Symmetric/InterpolatedPeakIn", peakIn);
    return peakIn;
  }

  /** Calculate and apply pass shot using fixed-height parabola strategy. */
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

    // Pass the landing target directly to the strategy. The solver treats it as the
    // pass-through point at ground level — peak height is the only arc control,
    // same concept as the hub strategy's fixed peak.
    Logger.recordOutput("SmartLaunch/Pass/TargetDistM", horizontalDist);

    double hoodMax = hood != null ? hood.getMaxAngle() : 46.0;

    ShotCalculator.ShotResult result =
        fixedHeightPassStrategy.calculateShot(
            robotPose,
            fieldSpeeds,
            target,
            turretConfig,
            turret.getOutsideCurrentAngle(),
            turret.getMinAngle(),
            turret.getMaxAngle(),
            18.0, // hood min floor handled inside strategy, but pass as baseline
            hoodMax);

    currentShot = result;
    currentDistanceM = horizontalDist;
  }

  // ========== Shot State Logging ==========

  /**
   * Log current shot state to AdvantageKit every cycle. Runs in periodic() so readiness, targets,
   * and tracking errors are always visible — regardless of which command (smart launch, fixed shot,
   * auto-track, or none) is active.
   */
  private void logShotState() {
    // --- Targets (what we're commanding, respecting per-actuator overrides) ---
    double targetRPM = frc.robot.commands.ShootingCommands.getEffectiveRPM(currentShot);
    double targetHoodDeg = frc.robot.commands.ShootingCommands.getEffectiveHoodDeg(currentShot);

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
        frc.robot.commands.ShootingCommands.getEffectiveMotivatorRPM(targetRPM, this));
    Logger.recordOutput(
        "SmartLaunch/Target/SpindexerRPM",
        frc.robot.commands.ShootingCommands.getEffectiveSpindexerRPM(this));

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
      Logger.recordOutput("SmartLaunch/Status/DistanceToAimPoint", distance);

      // TOF from physics (exit velocity + launch angle)
      double tof =
          ShotCalculator.calculateTimeOfFlight(
              currentShot.exitVelocityMps(), currentShot.launchAngleRad(), distance);
      Logger.recordOutput("SmartLaunch/TOF", tof);
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
      // at the given launch angle matches real-world shot behavior.
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
   * Check if shooting subsystems should idle to conserve power. True in auto when: - In open
   * neutral/opponent zones (collecting balls), OR - Not yet armed (heading outbound before visiting
   * neutral — no reason to spin up) NOT true in trenches after armed — when returning through
   * neutral trench, launcher/turret/motivator should track so they're ready at the alliance trench
   * boundary. In teleop this always returns false.
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
                || cachedAimResult.zone() == ZoneDetector.Zone.NEUTRAL_TRENCH
                || cachedAimResult.zone() == ZoneDetector.Zone.DANGER_TRENCH);

    if (inTrenchZone && hood != null && fieldSpeedsSupplier != null) {
      ChassisSpeeds speeds = fieldSpeedsSupplier.get();
      double robotSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      boolean isMoving = robotSpeed > trenchMovingThresholdMps.get();
      boolean hoodAboveSafe = hood.getCurrentAngle() > trenchHoodMaxDeg.get();

      trenchHoodSafetyActive = isMoving && hoodAboveSafe;
    } else {
      trenchHoodSafetyActive = false;
    }

    // Apply or release drive speed limit on state transitions.
    // Defer to danger trench safety when it's active — it uses a more aggressive limit.
    if (trenchHoodSafetyActive && !dangerTrenchActive) {
      DriveCommands.setSpeedLimit(trenchSafetySpeedLimitMps.get());
    } else if (wasActive && !dangerTrenchActive) {
      DriveCommands.clearSpeedLimit();
    }

    if (periodicCounter % 5 == 0) {
      Logger.recordOutput("SmartLaunch/TrenchHoodSafety/Active", trenchHoodSafetyActive);
    }
  }

  /**
   * Update danger trench safety state. When in the DANGER_TRENCH zone with the hood physically
   * above the safe angle, near-stops the robot (0.3 m/s) and forces the hood to lower. After a
   * tunable timeout (default 2s), relaxes to a fallback speed so the driver can escape if the hood
   * is broken/jammed. Once the hood reaches the safe angle, the limit is cleared immediately.
   *
   * <p>Speed limiting uses DriveCommands (teleop joystick only). In auto, path speeds should be
   * designed to stay within safe limits in trench zones. The hood clamping and force-to-min angle
   * work in both teleop and auto regardless.
   */
  private void updateDangerTrenchSafety() {
    boolean wasActive = dangerTrenchActive;

    boolean inDangerZone =
        cachedAimResult != null && cachedAimResult.zone() == ZoneDetector.Zone.DANGER_TRENCH;

    if (inDangerZone && hood != null) {
      boolean hoodAboveSafe = hood.getCurrentAngle() > trenchHoodMaxDeg.get();

      if (hoodAboveSafe) {
        if (!wasActive) {
          // Just entered danger zone with hood up — record entry time for timeout
          dangerTrenchEntryTimestamp = Timer.getFPGATimestamp();
        }
        dangerTrenchActive = true;

        // After timeout, relax to fallback speed so driver can escape a broken hood
        double elapsed = Timer.getFPGATimestamp() - dangerTrenchEntryTimestamp;
        if (elapsed > dangerTrenchTimeoutSec.get()) {
          DriveCommands.setSpeedLimit(dangerTrenchFallbackSpeedMps.get());
        } else {
          DriveCommands.setSpeedLimit(dangerTrenchSpeedLimitMps.get());
        }
      } else {
        // Hood is at or below safe angle — allow passage
        dangerTrenchActive = false;
      }
    } else {
      dangerTrenchActive = false;
    }

    // Clear drive speed limit when danger trench safety deactivates
    if (wasActive && !dangerTrenchActive) {
      DriveCommands.clearSpeedLimit();
    }

    if (periodicCounter % 5 == 0) {
      Logger.recordOutput("SmartLaunch/DangerTrenchSafety/Active", dangerTrenchActive);
    }
  }

  // ========== CoordinatorState Machine API ==========

  /**
   * Notify the coordinator that SmartLaunch has started or stopped. Called by the SmartLaunch
   * command on start/end to drive INACTIVE transitions. Uses IMMEDIATE arm trigger (teleop
   * default).
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
      wasFiringBeforeZoneSuppression = false;
      firingEntryPending = false;
    } else if (coordinatorState == CoordinatorState.INACTIVE) {
      armTrigger = trigger;
      armed = (trigger == ArmTrigger.IMMEDIATE);
      coordinatorState = armed ? CoordinatorState.AIMING : CoordinatorState.UNARMED;
      hasVisitedNeutral = false;
      readyTimeoutTimer.restart();
      readyTimeoutRunning = true;
      // Initialize previous-state tracking so the first cycle doesn't false-trigger transitions
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
  private boolean wasFiringBeforeZoneSuppression = false;

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
    // Arm trigger only gates the very first outbound trip. Once armed, stay armed —
    // zone suppression (NO_FIRE_ZONE) and hood clamping handle safety on subsequent trips.
    // Check arm conditions based on trigger type
    if (!armed) {
      switch (armTrigger) {
        case IMMEDIATE -> armed = true;
        case ON_PASS_ZONE -> {
          // Arm when entering pass zone (neutral/opponent)
          if (hasVisitedNeutral) {
            armed = true;
          }
        }
        case ON_TRENCH_RETURN -> {
          // Arm when entering alliance trench after having visited neutral
          if (hasVisitedNeutral && currentZone == ZoneDetector.Zone.ALLIANCE_TRENCH) {
            armed = true;
          }
        }
      }
    }
    // Transition UNARMED → AIMING when subsystems start spinning (no longer collecting),
    // BUT only if we're not in a no-fire zone — otherwise go straight to NO_FIRE_ZONE
    // to avoid a single-cycle AIMING blip that's invisible in logs.
    boolean inNoFireZone =
        currentZone == ZoneDetector.Zone.NEUTRAL_TRENCH
            || currentZone == ZoneDetector.Zone.DANGER_TRENCH
            || currentZone == ZoneDetector.Zone.BUMP;
    if (!isAutoCollecting() && coordinatorState == CoordinatorState.UNARMED) {
      if (inNoFireZone) {
        coordinatorState = CoordinatorState.NO_FIRE_ZONE;
        readyTimeoutRunning = false;
      } else {
        coordinatorState = CoordinatorState.AIMING;
        readyTimeoutTimer.restart();
        readyTimeoutRunning = true;
      }
    }

    // Readiness variables — declared here so they're available for logging in all paths
    boolean launcherReady = false;
    boolean motivatorReady = false;
    boolean turretReady = false;
    boolean hoodReady = false;
    boolean shotAchievable = false;
    boolean speedOk = false;
    boolean allReady = false;

    CoordinatorState prevState = coordinatorState;
    String stateReason = "";

    // --- UNARMED trumps everything — skip all state logic until armed ---
    if (coordinatorState == CoordinatorState.UNARMED) {
      previousAimMode = currentAimMode;
    }
    // --- Zone suppression check ---
    else if (inNoFireZone) {
      if (coordinatorState != CoordinatorState.NO_FIRE_ZONE) {
        stateReason = "entered " + currentZone.name();
      }
      coordinatorState = CoordinatorState.NO_FIRE_ZONE;
      readyTimeoutRunning = false;
      // Still update previous values so we detect the transition OUT correctly
      previousAimMode = currentAimMode;
    } else if (coordinatorState == CoordinatorState.NO_FIRE_ZONE) {
      // --- Leaving zone suppression ---
      // If we were FIRING before entering the zone, settle. Otherwise go straight to AIMING
      // since there's nothing to settle — subsystems were never locked on.
      if (prevState == CoordinatorState.NO_FIRE_ZONE && wasFiringBeforeZoneSuppression) {
        coordinatorState = CoordinatorState.SETTLING;
        stateReason = "left no-fire zone (was firing)";
      } else {
        coordinatorState = CoordinatorState.AIMING;
        stateReason = "left no-fire zone";
      }
      readyTimeoutTimer.restart();
      readyTimeoutRunning = true;
      previousAimMode = currentAimMode;
    } else if (feedingSuppressedSupplier.getAsBoolean()) {
      // --- Operator suppression check ---
      if (coordinatorState == CoordinatorState.FIRING
          || coordinatorState == CoordinatorState.HELD) {
        if (coordinatorState != CoordinatorState.HELD) {
          stateReason = "operator suppressed";
        }
        coordinatorState = CoordinatorState.HELD;
      }
      readyTimeoutRunning = false;
      previousAimMode = currentAimMode;
    } else {
      // --- Detect transition triggers (only from FIRING state) ---
      if (coordinatorState == CoordinatorState.FIRING) {
        boolean aimModeChanged = (currentAimMode != previousAimMode);
        boolean turretFlipping = (turret.getState() == Turret.TurretState.FLIPPING);

        if (aimModeChanged || turretFlipping) {
          coordinatorState = CoordinatorState.SETTLING;
          stateReason =
              "settling:"
                  + (aimModeChanged ? " aim_changed" : "")
                  + (turretFlipping ? " turret_flip" : "");
          readyTimeoutTimer.restart();
          readyTimeoutRunning = true;
        }
      }

      // --- Update previous values for next cycle's change detection ---
      previousAimMode = currentAimMode;

      // --- Readiness check for state advancement ---
      launcherReady = launcher != null && launcher.isReady();
      motivatorReady = motivator == null || motivator.getState() == Motivator.MotivatorState.READY;
      turretReady = turret.getState() == Turret.TurretState.READY;
      hoodReady = hood == null || hood.getState() == Hood.HoodState.READY;
      shotAchievable = currentShot != null && currentShot.achievable();
      boolean turretNotFlipping = turret.getState() != Turret.TurretState.FLIPPING;
      speedOk = isRobotSlowEnoughForCurrentZone();
      allReady =
          armed
              && launcherReady
              && motivatorReady
              && turretReady
              && hoodReady
              && shotAchievable
              && speedOk;

      // --- Timeout: force FIRING if stuck in AIMING too long ---
      boolean timeoutForced = false;
      if (readyTimeoutRunning
          && readyTimeoutTimer.hasElapsed(readyTimeoutSec.get())
          && coordinatorState == CoordinatorState.AIMING) {
        coordinatorState = CoordinatorState.FIRING;
        readyTimeoutRunning = false;
        timeoutForced = true;
        stateReason = "timeout forced";
      }

      if (!timeoutForced) {
        switch (coordinatorState) {
          case AIMING -> {
            if (allReady) {
              coordinatorState = CoordinatorState.FIRING;
              stateReason = "all ready";
            }
          }
          case SETTLING -> {
            if (allReady && turretNotFlipping) {
              coordinatorState = CoordinatorState.FIRING;
              stateReason = "settled, all ready";
            }
          }
          case HELD -> {
            // feedingSuppressed was already checked above and returned false to reach here
            if (allReady) {
              coordinatorState = CoordinatorState.FIRING;
              stateReason = "released, all ready";
            }
          }
          case FIRING -> {
            // Drop back to AIMING if robot exceeds zone speed threshold
            if (!speedOk) {
              coordinatorState = CoordinatorState.AIMING;
              stateReason = "too fast";
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

    // Track whether we were firing before entering a no-fire zone, so we know
    // whether to SETTLE or go straight to AIMING when leaving.
    if (coordinatorState == CoordinatorState.NO_FIRE_ZONE
        && prevState != CoordinatorState.NO_FIRE_ZONE) {
      wasFiringBeforeZoneSuppression = (prevState == CoordinatorState.FIRING);
    }

    // Flag every entry into FIRING so consumeFiringEntry() can trigger a reverse pulse.
    if (coordinatorState == CoordinatorState.FIRING && prevState != CoordinatorState.FIRING) {
      firingEntryPending = true;
    }

    // --- Logging (delayed one cycle to align with subsystem state logs) ---
    // Subsystems react to coordinator decisions in command execute() but log their new
    // state in next cycle's periodic(). Delaying these logs keeps everything in sync.
    Logger.recordOutput("SmartLaunch/CoordinatorState", loggedCoordinatorState.name());
    Logger.recordOutput("SmartLaunch/Armed", loggedArmed);
    Logger.recordOutput("SmartLaunch/HasVisitedNeutral", loggedHasVisitedNeutral);

    if (loggedCoordinatorState != loggedPrevState) {
      Logger.recordOutput(
          "SmartLaunch/StateTransition",
          loggedPrevState.name()
              + " -> "
              + loggedCoordinatorState.name()
              + " ("
              + loggedStateReason
              + ")");
    }

    if (loggedCoordinatorState == CoordinatorState.AIMING
        || loggedCoordinatorState == CoordinatorState.SETTLING) {
      Logger.recordOutput("SmartLaunch/Blocking", loggedBlocking);
    } else {
      Logger.recordOutput("SmartLaunch/Blocking", "");
    }

    // Snapshot current values for next cycle's delayed log
    loggedPrevState = loggedCoordinatorState;
    loggedCoordinatorState = coordinatorState;
    loggedArmed = armed;
    loggedHasVisitedNeutral = hasVisitedNeutral;
    loggedStateReason = stateReason;
    {
      StringBuilder blocking = new StringBuilder();
      if (!armed) blocking.append("armed ");
      if (!launcherReady) blocking.append("launcher ");
      if (!motivatorReady) blocking.append("motivator ");
      if (!turretReady) blocking.append("turret ");
      if (!hoodReady) blocking.append("hood ");
      if (!shotAchievable) blocking.append("shot ");
      if (!speedOk) blocking.append("speed ");
      loggedBlocking = blocking.length() > 0 ? blocking.toString().trim() : "none";
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
        readiness,
        cachedAimResult != null ? cachedAimResult.zone() : null);
  }
}
