package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
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
    /** Sprint — arm when entering any alliance zone after visiting neutral. */
    ON_ALLIANCE_RETURN
  }

  /** Strategy for selecting which pass target (left vs right trench) to use. */
  public enum PassingStrategy {
    /** Pick target based on robot's Y position relative to field center. */
    SYMMETRIC,
    /** Pick target based on driver station number from FMS. */
    DRIVER_STATION
  }

  /**
   * Active shooting goal. Commands set a goal; {@code periodic()} applies it to hardware every
   * cycle. Commands become trivial one-liners; all behavior lives in the coordinator.
   */
  public enum Goal {
    /** All subsystems idle — no output applied. */
    IDLE,
    /** Smart launch armed immediately (teleop default). */
    SMART_SHOOT,
    /** Smart launch armed when entering a pass zone (sprint auto). */
    SMART_SHOOT_ON_PASS_ZONE,
    /** Smart launch armed on alliance return after visiting neutral (sprint auto). */
    SMART_SHOOT_ON_ALLIANCE_RETURN,
    /** Fixed hub shot at tunable setpoints. */
    FIXED_HUB,
    /** Fixed left-trench shot at tunable setpoints. */
    FIXED_LEFT_TRENCH,
    /** Fixed right-trench shot at tunable setpoints. */
    FIXED_RIGHT_TRENCH
  }

  /** Tunable shot parameters for a fixed-position goal. Suppliers read live dashboard values. */
  private record FixedShotParams(
      DoubleSupplier launcherRPM,
      DoubleSupplier hoodAngleDeg,
      DoubleSupplier turretAngleDeg,
      DoubleSupplier motivatorRPM,
      DoubleSupplier spindexerRPM) {}

  private final SendableChooser<PassingStrategy> passingStrategyChooser = new SendableChooser<>();
  private final FixedHeightShotStrategy fixedHeightStrategy = new FixedHeightShotStrategy();
  private final FixedHeightPassStrategy fixedHeightPassStrategy = new FixedHeightPassStrategy();
  private final WaypointPassStrategy waypointPassStrategy = new WaypointPassStrategy();

  // Pass strategy chooser — selectable via dashboard dropdown
  private static final String PASS_STRAT_FIXED = "FixedHeight";
  private static final String PASS_STRAT_WAYPOINT = "Waypoint";
  private final SendableChooser<String> passStrategyChooser = new SendableChooser<>();

  // Hub strategy chooser — selectable via dashboard dropdown
  private final SendableChooser<ShotStrategy> hubStrategyChooser = new SendableChooser<>();
  private final TwoStageShotStrategy twoStageStrategy = new TwoStageShotStrategy();

  // Velocity compensation tunables (moved from ShotCalculator)
  private final LoggedTunableNumber velocityCompX =
      new LoggedTunableNumber(
          "Shots/VelocityComp/X", Constants.getRobotConfig().getShotVelocityCompX());
  private final LoggedTunableNumber velocityCompY =
      new LoggedTunableNumber(
          "Shots/VelocityComp/Y", Constants.getRobotConfig().getShotVelocityCompY());
  // Max angular divergence (deg) between static and velocity-compensated aim points.
  // If exceeded, the shot is flagged as not achievable (too fast to compensate).
  private static final LoggedTunableNumber velocityCompMaxDivergenceDeg =
      new LoggedTunableNumber("Shots/VelocityComp/MaxDivergenceDeg", 20.0);

  // Geo-fenced passing: replaces iterative velcomp + divergence gate for PASS/LONG_PASS.
  // When enabled, uses single-pass lead correction and a predicted-landing-point safety check
  // instead of the 20-degree divergence gate that blocks 50% of passes.
  private static final LoggedTunableNumber geoFencePassEnabled =
      new LoggedTunableNumber("Shots/GeoFencePass/Enabled", 1.0); // 1.0 = enabled
  private static final LoggedTunableNumber geoFenceMarginM =
      new LoggedTunableNumber("Shots/GeoFencePass/MarginM", 0.5);

  // ========== Coordinator-computed fields (derived from strategy + geometry) ==========
  // These are computed after the strategy returns and exposed via getters.
  private double currentTurretAngleDeg = 0.0;
  private Translation3d compensatedAimTarget = null;
  private double currentExitVelocityMps = 0.0;
  private boolean currentShotAchievable = true;
  private double currentDivergenceDeg = 0.0;
  // Geo-fence pass state (logged each cycle when geo-fence mode is active)
  private boolean geoFenceActive = false; // true when geo-fence mode was used this cycle
  private boolean geoFenceResult = false;
  private String geoFenceReason = "";
  private double predictedLandingX = 0.0;
  private double predictedLandingY = 0.0;

  // Visualizer (created during initialize)
  private ShotVisualizer visualizer = null;

  // Throttle counter for non-critical logging (reduces loop time)
  private int periodicCounter = 0;
  private Supplier<Pose2d> robotPoseSupplier = null;
  private Supplier<ChassisSpeeds> fieldSpeedsSupplier = null;
  private DoubleSupplier pitchDegSupplier = () -> 0.0;

  // Optional feeding suppression check — when true, launchFuel() is a no-op
  private BooleanSupplier feedingSuppressedSupplier = () -> false;

  // ===== Fixed Shot Tunables (hub, left trench, right trench) =====
  private final LoggedTunableNumber hubShotLauncherRPM =
      new LoggedTunableNumber(
          "Shots/HubShot/LauncherRPM", Constants.getRobotConfig().getHubShotLauncherRPM());
  private final LoggedTunableNumber hubShotHoodAngleDeg =
      new LoggedTunableNumber(
          "Shots/HubShot/HoodAngleDeg", Constants.getRobotConfig().getHubShotHoodAngleDeg());
  private final LoggedTunableNumber hubShotTurretAngleDeg =
      new LoggedTunableNumber(
          "Shots/HubShot/TurretAngleDeg", Constants.getRobotConfig().getHubShotTurretAngleDeg());
  private final LoggedTunableNumber hubShotMotivatorRPM =
      new LoggedTunableNumber(
          "Shots/HubShot/MotivatorRPM", Constants.getRobotConfig().getHubShotMotivatorRPM());
  private final LoggedTunableNumber hubShotSpindexerRPM =
      new LoggedTunableNumber(
          "Shots/HubShot/SpindexerRPM", Constants.getRobotConfig().getHubShotSpindexerRPM());

  private final LoggedTunableNumber leftTrenchLauncherRPM =
      new LoggedTunableNumber(
          "Shots/LeftTrench/LauncherRPM", Constants.getRobotConfig().getLeftTrenchLauncherRPM());
  private final LoggedTunableNumber leftTrenchHoodAngleDeg =
      new LoggedTunableNumber(
          "Shots/LeftTrench/HoodAngleDeg",
          Constants.getRobotConfig().getLeftTrenchHoodAngleDeg());
  private final LoggedTunableNumber leftTrenchTurretAngleDeg =
      new LoggedTunableNumber(
          "Shots/LeftTrench/TurretAngleDeg",
          Constants.getRobotConfig().getLeftTrenchTurretAngleDeg());
  private final LoggedTunableNumber leftTrenchMotivatorRPM =
      new LoggedTunableNumber(
          "Shots/LeftTrench/MotivatorRPM",
          Constants.getRobotConfig().getLeftTrenchMotivatorRPM());
  private final LoggedTunableNumber leftTrenchSpindexerRPM =
      new LoggedTunableNumber(
          "Shots/LeftTrench/SpindexerRPM",
          Constants.getRobotConfig().getLeftTrenchSpindexerRPM());

  private final LoggedTunableNumber rightTrenchLauncherRPM =
      new LoggedTunableNumber(
          "Shots/RightTrench/LauncherRPM", Constants.getRobotConfig().getRightTrenchLauncherRPM());
  private final LoggedTunableNumber rightTrenchHoodAngleDeg =
      new LoggedTunableNumber(
          "Shots/RightTrench/HoodAngleDeg",
          Constants.getRobotConfig().getRightTrenchHoodAngleDeg());
  private final LoggedTunableNumber rightTrenchTurretAngleDeg =
      new LoggedTunableNumber(
          "Shots/RightTrench/TurretAngleDeg",
          Constants.getRobotConfig().getRightTrenchTurretAngleDeg());
  private final LoggedTunableNumber rightTrenchMotivatorRPM =
      new LoggedTunableNumber(
          "Shots/RightTrench/MotivatorRPM",
          Constants.getRobotConfig().getRightTrenchMotivatorRPM());
  private final LoggedTunableNumber rightTrenchSpindexerRPM =
      new LoggedTunableNumber(
          "Shots/RightTrench/SpindexerRPM",
          Constants.getRobotConfig().getRightTrenchSpindexerRPM());

  // Trench hood safety and danger zone speed limiting — see TrenchSafetyManager for details.
  private final TrenchSafetyManager trenchSafety;

  // ===== Goal-based output =====
  private Goal goal = Goal.IDLE;

  // Motivator pre-spinner state — persists across periodic() cycles so the reverse pulse
  // fires exactly once per idle→spinning transition, not on every 20ms tick.
  private static final double MOTIVATOR_REVERSE_PULSE_SEC = 0.2;
  private final Timer motivatorReversePulseTimer = new Timer();
  private boolean motivatorReversing = false;
  private boolean motivatorWasIdle = true;

  // Sim ball fire throttle — prevents multiple fires per frame in SIM/REPLAY modes.
  private static final double MIN_SHOT_INTERVAL_SECONDS = 0.14;
  private double lastSimFireTimestamp = 0;

  // Auto passing: when false, PASS/LONG_PASS zones are treated as no-fire zones in auto.
  // Set by AutoWrapperFactory based on path strategy (AUTO_SHOOT enables, others disable).
  // Teleop always allows passing regardless of this flag.
  private boolean autoPassingEnabled = false;

  // Cease-fire: when true, all shooting subsystems immediately idle and hood stows.
  // Set by the "CeaseFire" PathPlanner event marker during auto routines.
  // Cleared when SmartLaunch is next activated via setSmartLaunchActive(true, ...).
  private boolean ceaseFireRequested = false;

  /** Enable or disable passing during auto. Called by AutoWrapperFactory. */
  public void setAutoPassingEnabled(boolean enabled) {
    this.autoPassingEnabled = enabled;
  }

  /**
   * Request an immediate cease-fire: all shooting subsystems idle, hood stows, feeding stops. Used
   * by the "CeaseFire" PathPlanner event marker to kill shooting mid-auto. The flag is
   * automatically cleared when SmartLaunch is next activated.
   */
  public void requestCeaseFire() {
    ceaseFireRequested = true;
    coordinatorState = CoordinatorState.UNARMED;
    armed = false;
    Logger.recordOutput("SmartLaunch/Phase", "CEASE_FIRE");
  }

  /** Check if a cease-fire has been requested. */
  public boolean isCeaseFireRequested() {
    return ceaseFireRequested;
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
  private double currentDistanceM = -1; // velocity-compensated distance fed to strategy
  private double rawDistanceToTargetM = -1; // straight-line turret-to-target, no velocity comp
  private String currentDistanceMode = ""; // what target we're shooting at (for logging)
  private Translation3d rawTarget = null; // target before velocity compensation

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
  // HUB: max speed for hub shots in open alliance zone.
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

  // Long pass peak height — higher arc for opponent zone passes to keep hood in mechanical range.
  private final LoggedTunableNumber longPassPeakHeightIn =
      new LoggedTunableNumber(
          "Shots/FixedHeightPass/LongPassPeakHeightIn",
          Constants.getRobotConfig().getFixedHeightLongPassPeakHeightIn());

  // ========== Waypoint Pass Tunables ==========
  // 3D waypoint above the bump/trench that the ball must pass through.
  // X is blue-alliance-relative (mirrored for red). Y/Z are absolute field coordinates.
  // Left waypoint Y = fieldWidth - rightWaypointY (symmetric across centerline).
  private final LoggedTunableNumber waypointPassBlueXM =
      new LoggedTunableNumber(
          "Shots/WaypointPass/BlueXM", Constants.getRobotConfig().getWaypointPassBlueXM());
  private final LoggedTunableNumber waypointPassRightYM =
      new LoggedTunableNumber(
          "Shots/WaypointPass/RightYM", Constants.getRobotConfig().getWaypointPassRightYM());
  private final LoggedTunableNumber waypointPassHeightM =
      new LoggedTunableNumber(
          "Shots/WaypointPass/HeightM", Constants.getRobotConfig().getWaypointPassHeightM());
  private final LoggedTunableNumber waypointPassPeakHeightIn =
      new LoggedTunableNumber(
          "Shots/WaypointPass/PeakHeightIn",
          Constants.getRobotConfig().getWaypointPassPeakHeightIn());

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
    this.trenchSafety = new TrenchSafetyManager(hood, turretConfig);

    // Passing strategy chooser — how to pick left vs right pass target
    passingStrategyChooser.setDefaultOption("Symmetric (Y-based)", PassingStrategy.SYMMETRIC);
    passingStrategyChooser.addOption("Driver Station", PassingStrategy.DRIVER_STATION);
    SmartDashboard.putData("SmartLaunch/Pass/Strategy", passingStrategyChooser);

    // Pass strategy chooser — which pass strategy to use (FixedHeight vs Waypoint)
    passStrategyChooser.setDefaultOption("FixedHeight (arc to peak)", PASS_STRAT_FIXED);
    passStrategyChooser.addOption("Waypoint (arc through point)", PASS_STRAT_WAYPOINT);
    SmartDashboard.putData("SmartLaunch/PassStrategy", passStrategyChooser);

    // Hub strategy chooser — which shot strategy to use for hub shots
    hubStrategyChooser.setDefaultOption("FixedHeight (baseline)", fixedHeightStrategy);
    hubStrategyChooser.addOption("TwoStage (motivator model)", twoStageStrategy);
    SmartDashboard.putData("SmartLaunch/HubStrategy", hubStrategyChooser);
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
    trenchSafety.init(poseSupplier, speedsSupplier);

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
    boolean shouldLog = periodicCounter % 5 == 0;
    updateShotCalculation(alliance, isBlueAlliance);
    trenchSafety.updateDangerTrenchImminent(cachedAimResult, alliance, shouldLog);
    updateCoordinatorState();
    trenchSafety.updateTrenchHoodSafety(cachedAimResult, shouldLog);
    trenchSafety.updateDangerTrenchSafety(cachedAimResult, shouldLog);
    if (shouldLog) {
      logShotState();
    }

    // 3D hood component pose for AdvantageScope — mounted on turret, articulates by hood angle
    logHoodPose();

    // Never command actuators while disabled — only run visualization
    if (DriverStation.isDisabled()) {
      if (visualizerCounter % VISUALIZER_DIVISOR == 0
          && visualizer != null
          && robotPoseSupplier != null) {
        visualizer.update(createVisualizerSnapshot(isBlueAlliance));
      }
      return;
    }

    // Apply outputs to hardware based on current goal (set by active shoot command).
    if (goal != Goal.IDLE) {
      applyOutputs();
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
    if (robotPoseSupplier != null && fieldSpeedsSupplier != null) {
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
      double robotSpeed =
          Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
      trenchSafety.updateHoodClamping(aimResult.zone(), robotSpeed);

      // Raw distance to aim target (no velocity compensation) — for logging only
      rawDistanceToTargetM =
          Math.hypot(
              aimResult.target().getX() - turretFieldPos[0],
              aimResult.target().getY() - turretFieldPos[1]);

      // Log zone and aim mode delayed by one cycle so transitions line up with subsystem
      // state logs. Subsystems react to the zone in command execute() but don't log their
      // new state until next cycle's periodic(), so logging the previous zone here aligns them.
      if (previousAimResult != null) {
        Logger.recordOutput("SmartLaunch/Status/Zone", previousAimResult.zone().name());
        Logger.recordOutput("SmartLaunch/Status/AimMode", previousAimResult.mode().name());
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

      // Fixed goals set their own shot parameters; skip auto-calculation for those.
      if (!isFixedGoal(goal)) {
        switch (aimResult.mode()) {
          case HUB -> {
            currentDistanceMode = "Hub FixedHeight";
            Logger.recordOutput("SmartLaunch/Status/Strategy", currentDistanceMode);
            calculateShotToHub(robotPose, fieldSpeeds, isBlueAlliance);
          }
          case PASS -> {
            boolean useWaypoint = PASS_STRAT_WAYPOINT.equals(passStrategyChooser.getSelected());
            PassingStrategy strategy = passingStrategyChooser.getSelected();

            if (useWaypoint) {
              boolean isLeftTrench = selectIsLeftTrench(robotPose);
              currentDistanceMode = "Pass Waypoint";
              Logger.recordOutput("SmartLaunch/Status/Strategy", currentDistanceMode);
              Translation3d activeTarget = isLeftTrench ? cachedLeftTarget : cachedRightTarget;
              Logger.recordOutput("SmartLaunch/Pass/Target", isLeftTrench ? "LEFT" : "RIGHT");
              Translation3d waypoint = getWaypoint(isLeftTrench, isBlueAlliance);
              calculateWaypointPassToTarget(
                  robotPose, fieldSpeeds, activeTarget, waypoint, waypointPassPeakHeightIn.get());
            } else if (strategy == PassingStrategy.DRIVER_STATION
                && !isTooCloseToHub(robotPose)) {
              currentDistanceMode = "Pass Lob";
              Logger.recordOutput("SmartLaunch/Status/Strategy", currentDistanceMode);
              var location = DriverStation.getLocation();
              int station = location.isPresent() ? location.getAsInt() : 2;
              Translation3d activeTarget =
                  (station <= 2) ? cachedLobStation1Target : cachedLobStation3Target;
              Logger.recordOutput(
                  "SmartLaunch/Pass/Target", (station <= 2) ? "STATION_1" : "STATION_3");
              calculatePassToTarget(
                  robotPose, fieldSpeeds, activeTarget, PassingStrategy.DRIVER_STATION);
            } else {
              if (strategy == PassingStrategy.DRIVER_STATION) {
                currentDistanceMode = "Pass Symmetric (hub fallback)";
              } else {
                currentDistanceMode = "Pass Symmetric";
              }
              Logger.recordOutput("SmartLaunch/Status/Strategy", currentDistanceMode);
              boolean isLeftTrench = selectIsLeftTrench(robotPose);
              Translation3d activeTarget = isLeftTrench ? cachedLeftTarget : cachedRightTarget;
              Logger.recordOutput("SmartLaunch/Pass/Target", isLeftTrench ? "LEFT" : "RIGHT");
              calculatePassToTarget(
                  robotPose, fieldSpeeds, activeTarget, PassingStrategy.SYMMETRIC);
            }
          }
          case LONG_PASS -> {
            boolean useWaypoint = PASS_STRAT_WAYPOINT.equals(passStrategyChooser.getSelected());
            boolean isLeftTrench = selectIsLeftTrench(robotPose);
            Translation3d activeTarget = isLeftTrench ? cachedLeftTarget : cachedRightTarget;
            Logger.recordOutput("SmartLaunch/Pass/Target", isLeftTrench ? "LEFT" : "RIGHT");

            if (useWaypoint) {
              currentDistanceMode = "Pass Waypoint Long";
              Logger.recordOutput("SmartLaunch/Status/Strategy", currentDistanceMode);
              Translation3d waypoint = getWaypoint(isLeftTrench, isBlueAlliance);
              calculateWaypointPassToTarget(
                  robotPose, fieldSpeeds, activeTarget, waypoint, waypointPassPeakHeightIn.get());
            } else {
              currentDistanceMode = "Pass Symmetric Long";
              Logger.recordOutput("SmartLaunch/Status/Strategy", currentDistanceMode);
              calculatePassToTarget(
                  robotPose,
                  fieldSpeeds,
                  activeTarget,
                  PassingStrategy.SYMMETRIC,
                  longPassPeakHeightIn.get());
            }
          }
          case NONE -> {
            currentDistanceMode = "Hub FixedHeight";
            Logger.recordOutput("SmartLaunch/Status/Strategy", currentDistanceMode);
            calculateShotToHub(robotPose, fieldSpeeds, isBlueAlliance);
          }
        }
      } else {
        // Fixed goal: set manual parameters from tunables (refreshed every cycle).
        FixedShotParams p = fixedShotParamsFor(goal);
        double rpm =
            Math.min(
                p.launcherRPM().getAsDouble() + ShotOverrides.getLauncherTrimRPM(), 5000.0);
        setManualShotParameters(
            rpm, p.hoodAngleDeg().getAsDouble(), p.turretAngleDeg().getAsDouble());
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

    double robotHeadingRad = robotPose.getRotation().getRadians();
    double[] turretFieldPos =
        ShotCalculator.getTurretFieldPosition(
            robotPose.getX(), robotPose.getY(), robotHeadingRad, turretConfig);
    double turretX = turretFieldPos[0];
    double turretY = turretFieldPos[1];

    // Apply operator turret trim — rotate aim target around turret field position
    if (turretTrimDeg != 0.0) {
      target = applyTurretTrim(target, turretX, turretY);
    }
    rawTarget = target;

    // Select active hub strategy from chooser
    ShotStrategy activeStrategy = hubStrategyChooser.getSelected();
    if (activeStrategy == null) activeStrategy = fixedHeightStrategy;

    // Velocity compensation: iterative refinement loop (3 passes).
    // Each pass: compute shot at current aim distance → get TOF → shift target by velocity × TOF.
    // Re-solving at the shifted distance refines TOF, which converges in 2-3 iterations.
    // Uses horizontal TOF (stable, no divergence at steep angles).
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    compensatedAimTarget = target;
    double hubContractionRate = -1.0;

    if (robotSpeed > 0.1) {
      double prevCorrectionDist = 0.0;
      for (int i = 0; i < 3; i++) {
        double dist =
            Math.hypot(
                compensatedAimTarget.getX() - turretX, compensatedAimTarget.getY() - turretY);
        ShotCalculator.ShotResult iterShot = activeStrategy.calculateShot(dist);
        double exitVel = activeStrategy.estimateExitVelocity(iterShot.launcherRPM(), dist);
        double launchAngle = iterShot.launchAngleRad();
        double tof = ShotCalculator.calculateTimeOfFlight(exitVel, launchAngle, dist);
        if (tof <= 0 || tof >= Double.MAX_VALUE) break;
        Translation2d prevAim =
            new Translation2d(compensatedAimTarget.getX(), compensatedAimTarget.getY());
        compensatedAimTarget = predictTargetPos(target, fieldSpeeds, tof);
        double correctionDist =
            prevAim.getDistance(
                new Translation2d(compensatedAimTarget.getX(), compensatedAimTarget.getY()));
        if (i > 0 && prevCorrectionDist > 0.001) {
          hubContractionRate = correctionDist / prevCorrectionDist;
        }
        prevCorrectionDist = correctionDist;
      }
    }

    // Compute final distance to the compensated target and run the strategy
    double D =
        Math.hypot(compensatedAimTarget.getX() - turretX, compensatedAimTarget.getY() - turretY);
    currentShot = activeStrategy.calculateShot(D);
    currentDistanceM = D;

    // Compute exit velocity for logging/sim
    currentExitVelocityMps = activeStrategy.estimateExitVelocity(currentShot.launcherRPM(), D);

    // Compute turret angle (coordinator's job now)
    currentTurretAngleDeg =
        ShotCalculator.calculateOutsideTurretAngle(
            robotPose.getX(),
            robotPose.getY(),
            robotPose.getRotation().getDegrees(),
            compensatedAimTarget.getX(),
            compensatedAimTarget.getY(),
            turret.getOutsideCurrentAngle(),
            turret.getMinAngle(),
            turret.getMaxAngle(),
            turretConfig);

    // Achievability — flag if velocity compensation diverged too far (moving too fast to lead)
    double staticAngle = Math.atan2(target.getY() - turretY, target.getX() - turretX);
    double compAngle =
        Math.atan2(compensatedAimTarget.getY() - turretY, compensatedAimTarget.getX() - turretX);
    double divergenceDeg = Math.abs(Math.toDegrees(staticAngle - compAngle));
    if (divergenceDeg > 180) divergenceDeg = 360 - divergenceDeg;
    currentDivergenceDeg = divergenceDeg;
    currentShotAchievable = divergenceDeg <= velocityCompMaxDivergenceDeg.get();

    Logger.recordOutput("SmartLaunch/Status/ActiveStrategy", activeStrategy.getName());
    Logger.recordOutput("SmartLaunch/VelocityComp/DivergenceDeg", divergenceDeg);
    Logger.recordOutput("SmartLaunch/VelocityComp/ContractionRate", hubContractionRate);

    // Target positions now logged centrally in logShotState (SmartLaunch/Distance/)
  }

  /**
   * Predict where the target will be after a given time, accounting for robot motion. Used for
   * velocity compensation (leading the shot).
   */
  private Translation3d predictTargetPos(
      Translation3d target, ChassisSpeeds fieldSpeeds, double timeOfFlight) {
    double predictedX =
        target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight * velocityCompX.get();
    double predictedY =
        target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight * velocityCompY.get();
    return new Translation3d(predictedX, predictedY, target.getZ());
  }

  /** Calculate and apply pass shot using fixed-height parabola strategy. */
  private void calculatePassToTarget(
      Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d target, PassingStrategy strategy) {
    calculatePassToTarget(robotPose, fieldSpeeds, target, strategy, -1);
  }

  /**
   * Calculate and apply pass shot with an optional peak height override.
   *
   * <p>When geo-fence pass mode is enabled, replaces the iterative velocity compensation and 20-deg
   * divergence gate with a single-pass open-loop lead correction and a predicted-landing-point
   * safety check. This dramatically improves pass reliability (the divergence gate blocked ~50% of
   * passes due to long TOF amplifying solver error).
   *
   * @param peakHeightIn Peak height in inches, or -1 to use the strategy's default
   */
  private void calculatePassToTarget(
      Pose2d robotPose,
      ChassisSpeeds fieldSpeeds,
      Translation3d target,
      PassingStrategy strategy,
      double peakHeightIn) {

    double robotHeadingRad = robotPose.getRotation().getRadians();
    double[] turretFieldPos =
        ShotCalculator.getTurretFieldPosition(
            robotPose.getX(), robotPose.getY(), robotHeadingRad, turretConfig);
    double turretX = turretFieldPos[0];
    double turretY = turretFieldPos[1];

    // Apply operator turret trim — rotate aim target around turret field position
    target = applyTurretTrim(target, turretX, turretY);
    rawTarget = target;

    boolean useGeoFence = geoFencePassEnabled.get() >= 1.0;
    geoFenceActive = useGeoFence;

    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    compensatedAimTarget = target;
    double passContractionRate = -1.0;

    if (useGeoFence) {
      // --- Geo-fence mode: single-pass open-loop lead ---
      // Step 1: compute shot at static target to get TOF estimate
      double staticDist = Math.hypot(target.getX() - turretX, target.getY() - turretY);
      ShotCalculator.ShotResult staticShot =
          peakHeightIn > 0
              ? fixedHeightPassStrategy.calculateShot(staticDist, peakHeightIn)
              : fixedHeightPassStrategy.calculateShot(staticDist);
      double exitVel =
          fixedHeightPassStrategy.estimateExitVelocity(staticShot.launcherRPM(), staticDist);
      double launchAngle = staticShot.launchAngleRad();
      double tof = ShotCalculator.calculateTimeOfFlight(exitVel, launchAngle, staticDist);

      // Step 2: single-pass lead correction — shift aim by robot_velocity * TOF
      if (robotSpeed > 0.1 && tof > 0 && tof < Double.MAX_VALUE) {
        compensatedAimTarget = predictTargetPos(target, fieldSpeeds, tof);
      }
    } else {
      // --- Legacy mode: iterative 3-pass velocity compensation ---
      if (robotSpeed > 0.1) {
        double prevCorrectionDist = 0.0;
        for (int i = 0; i < 3; i++) {
          double dist =
              Math.hypot(
                  compensatedAimTarget.getX() - turretX, compensatedAimTarget.getY() - turretY);
          ShotCalculator.ShotResult iterShot =
              peakHeightIn > 0
                  ? fixedHeightPassStrategy.calculateShot(dist, peakHeightIn)
                  : fixedHeightPassStrategy.calculateShot(dist);
          double exitVel =
              fixedHeightPassStrategy.estimateExitVelocity(iterShot.launcherRPM(), dist);
          double launchAngle = iterShot.launchAngleRad();
          double tof = ShotCalculator.calculateTimeOfFlight(exitVel, launchAngle, dist);
          if (tof <= 0 || tof >= Double.MAX_VALUE) break;
          Translation2d prevAim =
              new Translation2d(compensatedAimTarget.getX(), compensatedAimTarget.getY());
          compensatedAimTarget = predictTargetPos(target, fieldSpeeds, tof);
          double correctionDist =
              prevAim.getDistance(
                  new Translation2d(compensatedAimTarget.getX(), compensatedAimTarget.getY()));
          if (i > 0 && prevCorrectionDist > 0.001) {
            passContractionRate = correctionDist / prevCorrectionDist;
          }
          prevCorrectionDist = correctionDist;
        }
      }
    }

    // Compute final shot at the (possibly lead-corrected) aim target
    double horizontalDist =
        Math.hypot(compensatedAimTarget.getX() - turretX, compensatedAimTarget.getY() - turretY);

    currentShot =
        peakHeightIn > 0
            ? fixedHeightPassStrategy.calculateShot(horizontalDist, peakHeightIn)
            : fixedHeightPassStrategy.calculateShot(horizontalDist);
    currentDistanceM = horizontalDist;

    // Compute exit velocity for logging/sim
    currentExitVelocityMps =
        fixedHeightPassStrategy.estimateExitVelocity(currentShot.launcherRPM(), horizontalDist);

    // Compute turret angle
    currentTurretAngleDeg =
        ShotCalculator.calculateOutsideTurretAngle(
            robotPose.getX(),
            robotPose.getY(),
            robotPose.getRotation().getDegrees(),
            compensatedAimTarget.getX(),
            compensatedAimTarget.getY(),
            turret.getOutsideCurrentAngle(),
            turret.getMinAngle(),
            turret.getMaxAngle(),
            turretConfig);

    if (useGeoFence) {
      // --- Geo-fence achievability: predict landing point, check safe zone ---
      Translation2d landing =
          predictBallLanding(turretX, turretY, fieldSpeeds, currentShot, currentExitVelocityMps);
      predictedLandingX = landing.getX();
      predictedLandingY = landing.getY();

      boolean isBlue =
          DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
              == DriverStation.Alliance.Blue;
      String[] reasonOut = new String[1];
      geoFenceResult = isInsideGeoFence(landing, isBlue, geoFenceMarginM.get(), reasonOut);
      geoFenceReason = reasonOut[0];
      currentShotAchievable = geoFenceResult;
      currentDivergenceDeg = 0.0; // not applicable in geo-fence mode

      Logger.recordOutput("SmartLaunch/Pass/PredictedLandingX", predictedLandingX);
      Logger.recordOutput("SmartLaunch/Pass/PredictedLandingY", predictedLandingY);
      Logger.recordOutput("SmartLaunch/Pass/GeoFencePass", geoFenceResult);
      Logger.recordOutput("SmartLaunch/Pass/FenceReason", geoFenceReason);
    } else {
      // --- Legacy achievability: divergence gate ---
      geoFenceActive = false;
      geoFenceResult = false;
      geoFenceReason = "";
      double staticAngle = Math.atan2(target.getY() - turretY, target.getX() - turretX);
      double compAngle =
          Math.atan2(
              compensatedAimTarget.getY() - turretY, compensatedAimTarget.getX() - turretX);
      double divergenceDeg = Math.abs(Math.toDegrees(staticAngle - compAngle));
      if (divergenceDeg > 180) divergenceDeg = 360 - divergenceDeg;
      currentDivergenceDeg = divergenceDeg;
      currentShotAchievable = divergenceDeg <= velocityCompMaxDivergenceDeg.get();
    }
    Logger.recordOutput("SmartLaunch/VelocityComp/DivergenceDeg", currentDivergenceDeg);
    Logger.recordOutput("SmartLaunch/VelocityComp/PassContractionRate", passContractionRate);
  }

  /**
   * Predict where a launched ball will land (z=0) using vacuum ballistics. Combines the ball's
   * launch velocity (from turret aim direction + shot parameters) with the robot's field velocity.
   *
   * @param turretX Turret field X position
   * @param turretY Turret field Y position
   * @param fieldSpeeds Robot's field-relative chassis speeds
   * @param shot Current shot parameters (for hood angle → launch angle)
   * @param exitVelocityMps Ball exit velocity in m/s
   * @return Predicted landing point in field coordinates
   */
  private Translation2d predictBallLanding(
      double turretX,
      double turretY,
      ChassisSpeeds fieldSpeeds,
      ShotCalculator.ShotResult shot,
      double exitVelocityMps) {

    double launchAngle = shot.launchAngleRad();
    double vBallHorizontal = exitVelocityMps * Math.cos(launchAngle);
    double vBallVertical = exitVelocityMps * Math.sin(launchAngle);

    // Aim direction: turret → compensated aim target (2D, normalized)
    double dx = compensatedAimTarget.getX() - turretX;
    double dy = compensatedAimTarget.getY() - turretY;
    double aimDist = Math.hypot(dx, dy);
    if (aimDist < 0.001) {
      return new Translation2d(turretX, turretY); // degenerate — turret on target
    }
    double aimDirX = dx / aimDist;
    double aimDirY = dy / aimDist;

    // Total horizontal velocity = ball horizontal (in aim direction) + robot velocity
    double vTotalX = vBallHorizontal * aimDirX + fieldSpeeds.vxMetersPerSecond;
    double vTotalY = vBallHorizontal * aimDirY + fieldSpeeds.vyMetersPerSecond;

    // Time to land (z=0), launching from turret height h0
    // h0 + vz*t - 0.5*g*t^2 = 0  →  t = (vz + sqrt(vz^2 + 2*g*h0)) / g
    double h0 = turretConfig.heightMeters();
    double discriminant = vBallVertical * vBallVertical + 2.0 * ShotCalculator.GRAVITY * h0;
    if (discriminant < 0) {
      return new Translation2d(turretX, turretY); // can't reach ground (shouldn't happen)
    }
    double tLand = (vBallVertical + Math.sqrt(discriminant)) / ShotCalculator.GRAVITY;

    return new Translation2d(turretX + vTotalX * tLand, turretY + vTotalY * tLand);
  }

  /**
   * Check if a predicted landing point is inside the geo-fence safe zone. The safe zone is the
   * alliance zone rectangle shrunk inward by the specified margin.
   *
   * @param landing Predicted landing point in field coordinates
   * @param isBlueAlliance True if blue alliance (affects X bounds)
   * @param margin Safety margin in meters (shrinks the zone inward)
   * @param reasonOut Single-element array; receives the reason string on failure, "ok" on success
   * @return True if landing is inside the safe zone
   */
  private static boolean isInsideGeoFence(
      Translation2d landing, boolean isBlueAlliance, double margin, String[] reasonOut) {

    double x = landing.getX();
    double y = landing.getY();

    // X bounds (alliance-dependent)
    double minX, maxX;
    if (isBlueAlliance) {
      minX = FieldConstants.PassSafeZone.blueMinX + margin;
      maxX = FieldConstants.PassSafeZone.blueMaxX - margin;
    } else {
      // Red: mirror across field center
      minX = FieldConstants.fieldLength - FieldConstants.PassSafeZone.blueMaxX + margin;
      maxX = FieldConstants.fieldLength - FieldConstants.PassSafeZone.blueMinX - margin;
    }

    // Y bounds (same for both alliances)
    double minY = FieldConstants.PassSafeZone.minY + margin;
    double maxY = FieldConstants.PassSafeZone.maxY - margin;

    if (x < minX) {
      reasonOut[0] = String.format("x=%.1f < minX=%.1f (behind wall)", x, minX);
      return false;
    }
    if (x > maxX) {
      reasonOut[0] = String.format("x=%.1f > maxX=%.1f (past zone)", x, maxX);
      return false;
    }
    if (y < minY) {
      reasonOut[0] = String.format("y=%.1f < minY=%.1f (off right side)", y, minY);
      return false;
    }
    if (y > maxY) {
      reasonOut[0] = String.format("y=%.1f > maxY=%.1f (off left side)", y, maxY);
      return false;
    }
    reasonOut[0] = "ok";
    return true;
  }

  /**
   * Get the 3D waypoint for a pass, based on which trench and alliance. Blue alliance uses the
   * configured X directly; red mirrors it across the field center. Left waypoint Y = fieldWidth -
   * rightY (symmetric across centerline).
   */
  private Translation3d getWaypoint(boolean isLeftTrench, boolean isBlueAlliance) {
    double blueX = waypointPassBlueXM.get();
    double rightY = waypointPassRightYM.get();
    double z = waypointPassHeightM.get();

    double x = isBlueAlliance ? blueX : FieldConstants.fieldLength - blueX;
    // "Left trench" is high Y on blue, low Y on red (driver perspective flips).
    // Match the same convention used by the cached pass targets.
    boolean highY = isBlueAlliance ? isLeftTrench : !isLeftTrench;
    double y = highY ? FieldConstants.fieldWidth - rightY : rightY;

    Translation3d waypoint = new Translation3d(x, y, z);
    Logger.recordOutput("SmartLaunch/Pass/Waypoint", new Pose3d(waypoint, Rotation3d.kZero));
    return waypoint;
  }

  /**
   * Calculate and apply a waypoint-constrained pass. The turret aims at the pass target (for
   * direction), but the shot parameters (RPM, hood angle) are computed so the ball arcs through the
   * 3D waypoint above the bump.
   *
   * @param robotPose Current robot pose
   * @param fieldSpeeds Field-relative chassis speeds
   * @param target Pass landing target (used for turret aim direction)
   * @param waypoint 3D waypoint the ball must pass through
   * @param peakHeightIn Peak height in inches
   */
  private void calculateWaypointPassToTarget(
      Pose2d robotPose,
      ChassisSpeeds fieldSpeeds,
      Translation3d target,
      Translation3d waypoint,
      double peakHeightIn) {

    double robotHeadingRad = robotPose.getRotation().getRadians();
    double[] turretFieldPos =
        ShotCalculator.getTurretFieldPosition(
            robotPose.getX(), robotPose.getY(), robotHeadingRad, turretConfig);
    double turretX = turretFieldPos[0];
    double turretY = turretFieldPos[1];

    // Apply operator turret trim to the aim target (for turret direction)
    target = applyTurretTrim(target, turretX, turretY);
    rawTarget = target;

    // Compute horizontal distance from turret to waypoint
    double waypointDistM = Math.hypot(waypoint.getX() - turretX, waypoint.getY() - turretY);
    double waypointHeightM = waypoint.getZ();

    // Velocity compensation — use waypoint distance for initial shot calc
    compensatedAimTarget = target;
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    if (robotSpeed > 0.1) {
      ShotCalculator.ShotResult staticShot =
          waypointPassStrategy.calculateShot(waypointDistM, waypointHeightM, peakHeightIn);
      double exitVel =
          waypointPassStrategy.estimateExitVelocity(staticShot.launcherRPM(), waypointDistM);
      double launchAngle = staticShot.launchAngleRad();
      // Use distance to target (not waypoint) for time-of-flight lead
      double targetDist = Math.hypot(target.getX() - turretX, target.getY() - turretY);
      double tof = ShotCalculator.calculateTimeOfFlight(exitVel, launchAngle, targetDist);
      if (tof > 0 && tof < Double.MAX_VALUE) {
        compensatedAimTarget = predictTargetPos(target, fieldSpeeds, tof);
      }
    }

    // Recompute waypoint distance along the compensated aim line
    // (waypoint is fixed on the field, but turret position may shift with vel comp)
    double compensatedWaypointDist =
        Math.hypot(waypoint.getX() - turretX, waypoint.getY() - turretY);

    currentShot =
        waypointPassStrategy.calculateShot(compensatedWaypointDist, waypointHeightM, peakHeightIn);
    currentDistanceM = compensatedWaypointDist;

    // Compute exit velocity for logging/sim
    currentExitVelocityMps =
        waypointPassStrategy.estimateExitVelocity(
            currentShot.launcherRPM(), compensatedWaypointDist);

    // Compute turret angle — aim at the pass target, not the waypoint
    currentTurretAngleDeg =
        ShotCalculator.calculateOutsideTurretAngle(
            robotPose.getX(),
            robotPose.getY(),
            robotPose.getRotation().getDegrees(),
            compensatedAimTarget.getX(),
            compensatedAimTarget.getY(),
            turret.getOutsideCurrentAngle(),
            turret.getMinAngle(),
            turret.getMaxAngle(),
            turretConfig);

    // Waypoint pass already uses single-pass velcomp (no iteration, no divergence gate).
    // Log geo-fence signals for consistency — predict landing and check safe zone.
    geoFenceActive = geoFencePassEnabled.get() >= 1.0;
    if (geoFenceActive) {
      Translation2d landing =
          predictBallLanding(turretX, turretY, fieldSpeeds, currentShot, currentExitVelocityMps);
      predictedLandingX = landing.getX();
      predictedLandingY = landing.getY();
      boolean isBlue =
          DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
              == DriverStation.Alliance.Blue;
      String[] reasonOut = new String[1];
      geoFenceResult = isInsideGeoFence(landing, isBlue, geoFenceMarginM.get(), reasonOut);
      geoFenceReason = reasonOut[0];
      currentShotAchievable = geoFenceResult;

      Logger.recordOutput("SmartLaunch/Pass/PredictedLandingX", predictedLandingX);
      Logger.recordOutput("SmartLaunch/Pass/PredictedLandingY", predictedLandingY);
      Logger.recordOutput("SmartLaunch/Pass/GeoFencePass", geoFenceResult);
      Logger.recordOutput("SmartLaunch/Pass/FenceReason", geoFenceReason);
    } else {
      currentShotAchievable = true;
    }
  }

  // ========== Shot State Logging ==========

  // Hood shaft offset from turret center in WPILib space (meters)
  // Derived from calibrated zeroedPosition: negated values = shaft position relative to turret
  private static final double HOOD_SHAFT_DX = 0.10668;
  private static final double HOOD_SHAFT_DY = -0.00762;
  private static final double HOOD_SHAFT_DZ = 0.1016;
  // The hood model was exported at its min angle — subtract it so pitch=0 at the baked-in pose
  private static final double HOOD_MODEL_BASE_DEG = 13.0;

  /** Log the hood's 3D pose for AdvantageScope component visualization (model_4). */
  private void logHoodPose() {
    double hoodAngleDeg = hood != null ? hood.getCurrentAngle() : 0.0;
    double turretYawRad =
        edu.wpi.first.math.geometry.Rotation2d.fromDegrees(turret.getOutsideCurrentAngle())
            .getRadians();

    // Forward kinematics: rotate shaft offset by turret yaw
    double shaftX =
        turretConfig.xOffset()
            + HOOD_SHAFT_DX * Math.cos(turretYawRad)
            - HOOD_SHAFT_DY * Math.sin(turretYawRad);
    double shaftY =
        turretConfig.yOffset()
            + HOOD_SHAFT_DX * Math.sin(turretYawRad)
            + HOOD_SHAFT_DY * Math.cos(turretYawRad);
    double shaftZ = turretConfig.heightMeters() - 0.04445 + HOOD_SHAFT_DZ;

    Logger.recordOutput(
        "Visualizations/Robot/4_Hood",
        new Pose3d(
            shaftX,
            shaftY,
            shaftZ,
            new Rotation3d(0.0, Math.toRadians(hoodAngleDeg - HOOD_MODEL_BASE_DEG), turretYawRad)));
  }

  /**
   * Log current shot state to AdvantageKit every cycle. Runs in periodic() so readiness, targets,
   * and tracking errors are always visible — regardless of which command (smart launch, fixed shot,
   * auto-track, or none) is active.
   */
  private void logShotState() {
    // --- Targets (what we're commanding, respecting per-actuator overrides) ---
    Logger.recordOutput("SmartLaunch/Target/LauncherRPM", ShotOverrides.getLauncherRPM(currentShot));
    Logger.recordOutput("SmartLaunch/Target/HoodDeg", ShotOverrides.getHoodDeg(currentShot));
    Logger.recordOutput("SmartLaunch/Target/TurretDeg", currentTurretAngleDeg);
    Logger.recordOutput(
        "SmartLaunch/Target/LaunchAngleDeg",
        currentShot != null ? currentShot.getLaunchAngleDegrees() : 0.0);
    Logger.recordOutput("SmartLaunch/Target/ExitVelocityMps", currentExitVelocityMps);
    Logger.recordOutput("SmartLaunch/Target/Achievable", currentShotAchievable);
    Logger.recordOutput("SmartLaunch/Target/MotivatorRPM", ShotOverrides.getMotivatorRPM(currentShot));
    Logger.recordOutput("SmartLaunch/Target/SpindexerRPM", ShotOverrides.getSpindexerRPM(currentShot));

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

    // --- Distance group (all in one place, always fresh, never stale) ---
    Logger.recordOutput("SmartLaunch/Distance/Mode", currentDistanceMode);
    Logger.recordOutput("SmartLaunch/Distance/RawDistanceM", rawDistanceToTargetM);
    Logger.recordOutput("SmartLaunch/Distance/VelocityCompensatedDistanceM", currentDistanceM);
    if (rawTarget != null) {
      Logger.recordOutput("SmartLaunch/Distance/RawTargetX", rawTarget.getX());
      Logger.recordOutput("SmartLaunch/Distance/RawTargetY", rawTarget.getY());
    }
    if (compensatedAimTarget != null) {
      Logger.recordOutput(
          "SmartLaunch/Distance/VelocityCompensatedTargetX", compensatedAimTarget.getX());
      Logger.recordOutput(
          "SmartLaunch/Distance/VelocityCompensatedTargetY", compensatedAimTarget.getY());
    }
    if (currentShot != null) {
      double tof =
          ShotCalculator.calculateTimeOfFlight(
              currentExitVelocityMps, currentShot.launchAngleRad(), currentDistanceM);
      Logger.recordOutput("SmartLaunch/Distance/TOF", tof);
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
    if (visualizer != null
        && currentShot != null
        && robotPoseSupplier != null
        && compensatedAimTarget != null) {
      Pose2d robotPose = robotPoseSupplier.get();
      double robotHeadingRad = robotPose.getRotation().getRadians();

      double[] turretFieldPos =
          ShotCalculator.getTurretFieldPosition(
              robotPose.getX(), robotPose.getY(), robotHeadingRad, turretConfig);
      double turretX = turretFieldPos[0];
      double turretY = turretFieldPos[1];

      double azimuthAngle =
          Math.atan2(compensatedAimTarget.getY() - turretY, compensatedAimTarget.getX() - turretX);

      // Use the strategy's exit velocity estimate for simulation
      double launchAngle = currentShot.launchAngleRad();
      visualizer.launchFuel(currentExitVelocityMps, launchAngle, azimuthAngle);

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
        () -> currentExitVelocityMps,
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

  /** Get the coordinator-computed turret angle (from velocity-compensated aim target). */
  public double getCurrentTurretAngleDeg() {
    return currentTurretAngleDeg;
  }

  /** Get the velocity-compensated aim target (for visualizer). May be null if no shot active. */
  public Translation3d getCompensatedAimTarget() {
    return compensatedAimTarget;
  }

  /** Get the coordinator-computed exit velocity (from active strategy). */
  public double getCurrentExitVelocityMps() {
    return currentExitVelocityMps;
  }

  /** Get whether the current shot is achievable. */
  public boolean isCurrentShotAchievable() {
    return currentShotAchievable;
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
    currentShot = ShotCalculator.calculateManualShot(launcherRPM, hoodAngleDeg);
    currentTurretAngleDeg = targetTurretAngleDeg;
    currentExitVelocityMps = 0.0;
    currentShotAchievable = true;
    currentDivergenceDeg = 0.0;
    compensatedAimTarget = null;

    // Update the ShotCalculator's target RPM for consistency
    ShotCalculator.setTargetLauncherRPM(launcherRPM);
  }

  /** Clear manual shot parameters and return to auto-calculated shots. */
  public void clearManualShotParameters() {
    currentShot = null;
  }

  // ========== Zone-Based Speed Check ==========

  /**
   * Check if firing is allowed in the current zone. Returns false when the aim mode is NONE, or
   * when passing is blocked during auto (autoPassingEnabled is false in PASS/LONG_PASS zones). This
   * is a policy check, not a speed check.
   */
  public boolean isFiringAllowedInCurrentZone() {
    if (cachedAimResult == null) return true;
    TurretAimingHelper.AimMode mode = cachedAimResult.mode();
    if (mode == TurretAimingHelper.AimMode.NONE) return false;
    boolean passingBlocked =
        (mode == TurretAimingHelper.AimMode.PASS || mode == TurretAimingHelper.AimMode.LONG_PASS)
            && DriverStation.isAutonomous()
            && !autoPassingEnabled;
    return !passingBlocked;
  }

  /**
   * Check if the robot is slow enough to fire in the current zone. Uses zone-aware speed
   * thresholds: alliance zone allows shoot-on-the-move, pass zones use pass speed limits. Only
   * checks speed — use {@link #isFiringAllowedInCurrentZone()} for zone permission.
   *
   * @return true if firing is allowed AND the robot speed is within the zone's threshold
   */
  public boolean isRobotSlowEnoughForCurrentZone() {
    if (!isFiringAllowedInCurrentZone()) return false;
    if (fieldSpeedsSupplier == null || cachedAimResult == null) return true;

    ChassisSpeeds speeds = fieldSpeedsSupplier.get();
    double robotSpeedMps = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    TurretAimingHelper.AimMode mode = cachedAimResult.mode();
    double thresholdMps =
        switch (mode) {
          case HUB -> shootOnTheMoveSpeedMps.get();
          case PASS, LONG_PASS -> DriverStation.isAutonomous()
              ? autoPassSpeedMps.get()
              : passSpeedMps.get();
          case NONE -> 0.0;
        };
    return robotSpeedMps <= thresholdMps;
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
    // Only idle when suppressed, outside alliance zone, AND the coordinator isn't actively
    // tracking a shot. Without the state check, holding fire in a pass zone idles the launcher
    // so it never reaches ready and the HELD state (cyan LEDs) is unreachable.
    boolean activelyTracking =
        coordinatorState == CoordinatorState.AIMING
            || coordinatorState == CoordinatorState.SETTLING
            || coordinatorState == CoordinatorState.FIRING
            || coordinatorState == CoordinatorState.HELD;
    return feedingSuppressedSupplier.getAsBoolean() && !isInAllianceZone() && !activelyTracking;
  }

  /**
   * Check if shooting subsystems should idle to conserve power. True in auto when:
   *
   * <ul>
   *   <li>Cease-fire is active
   *   <li>Outbound trip before visiting neutral (zone-gated triggers only, not IMMEDIATE)
   *   <li>In open neutral/opponent zones when passing is disabled
   * </ul>
   *
   * NOT true in trenches — when returning through neutral trench, subsystems should track so
   * they're ready at the alliance trench boundary. In teleop this always returns false.
   */
  public boolean isAutoCollecting() {
    if (ceaseFireRequested) return true; // cease-fire forces idle
    if (!DriverStation.isAutonomous()) return false;
    if (cachedAimResult == null) return false;
    // Outbound (haven't visited neutral yet) — idle everywhere.
    // Only applies to zone-gated triggers; IMMEDIATE is always ready in alliance.
    if (!hasVisitedNeutral && armTrigger != ArmTrigger.IMMEDIATE) return true;
    // In neutral/opponent zones, idle if passing is disabled (collecting only).
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
    return trenchSafety.isTrenchHoodSafetyActive();
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
   * For sprint autos, use ON_PASS_ZONE or ON_ALLIANCE_RETURN to prevent shooting preloads at start.
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
      ceaseFireRequested = false; // clear any previous cease-fire
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

  /** Check if SmartLaunch is currently active. */
  public boolean isSmartLaunchActive() {
    return smartLaunchActive;
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

    // Cease-fire locks the state machine in UNARMED until the robot reaches the
    // neutral zone. When clearing, respect the arm trigger instead of forcing armed:
    //   - IMMEDIATE / ON_PASS_ZONE: arm now (we're in neutral, condition is met)
    //   - ON_ALLIANCE_RETURN: stay unarmed until we reach alliance zone
    if (ceaseFireRequested) {
      if (currentZone == ZoneDetector.Zone.NEUTRAL || currentZone == ZoneDetector.Zone.OPPONENT) {
        ceaseFireRequested = false;
        boolean shouldArm = (armTrigger != ArmTrigger.ON_ALLIANCE_RETURN);
        armed = shouldArm;
        coordinatorState = shouldArm ? CoordinatorState.AIMING : CoordinatorState.UNARMED;
        if (shouldArm) {
          readyTimeoutTimer.restart();
          readyTimeoutRunning = true;
        }
        Logger.recordOutput("SmartLaunch/Phase", "CEASE_FIRE_CLEARED");
      } else {
        coordinatorState = CoordinatorState.UNARMED;
        return;
      }
    }
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
        case ON_ALLIANCE_RETURN -> {
          // Arm when entering any alliance zone after having visited neutral
          if (hasVisitedNeutral && isInAllianceZone()) {
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
            || currentZone == ZoneDetector.Zone.BUMP
            || trenchSafety.isDangerTrenchImminent();
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
    boolean shotExists = false;
    boolean velCompOk = false;
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
      if (coordinatorState == CoordinatorState.FIRING) {
        coordinatorState = CoordinatorState.HELD;
        stateReason = "operator suppressed";
      } else if (coordinatorState == CoordinatorState.AIMING
          || coordinatorState == CoordinatorState.SETTLING) {
        // Hold pressed before ready — check readiness so we transition to HELD once ready
        launcherReady = launcher != null && launcher.isReady();
        motivatorReady =
            motivator == null || motivator.getState() == Motivator.MotivatorState.READY;
        turretReady =
            turret.getState() == Turret.TurretState.READY
                || turret.getState() == Turret.TurretState.LOCKED;
        hoodReady = hood == null || hood.getState() == Hood.HoodState.READY;
        shotExists = currentShot != null;
        velCompOk = currentShotAchievable;
        speedOk = isRobotSlowEnoughForCurrentZone();
        boolean turretNotFlipping = turret.getState() != Turret.TurretState.FLIPPING;
        allReady =
            armed
                && launcherReady
                && motivatorReady
                && turretReady
                && hoodReady
                && shotExists
                && velCompOk
                && speedOk;
        if (allReady && (coordinatorState == CoordinatorState.AIMING || turretNotFlipping)) {
          coordinatorState = CoordinatorState.HELD;
          stateReason = "all ready, held";
        }
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
      turretReady =
          turret.getState() == Turret.TurretState.READY
              || turret.getState() == Turret.TurretState.LOCKED;
      hoodReady = hood == null || hood.getState() == Hood.HoodState.READY;
      shotExists = currentShot != null;
      velCompOk = currentShotAchievable;
      boolean turretNotFlipping = turret.getState() != Turret.TurretState.FLIPPING;
      speedOk = isRobotSlowEnoughForCurrentZone();
      allReady =
          armed
              && launcherReady
              && motivatorReady
              && turretReady
              && hoodReady
              && shotExists
              && velCompOk
              && speedOk;

      // --- Timeout: force FIRING if stuck in AIMING too long ---
      // Only timeout-force when firing is allowed in the current zone. Without this, the
      // timeout bypasses zone policy and fires in zones where shooting is not permitted
      // (e.g. passing in neutral when autoPassingEnabled is false).
      boolean timeoutForced = false;
      if (readyTimeoutRunning
          && readyTimeoutTimer.hasElapsed(readyTimeoutSec.get())
          && coordinatorState == CoordinatorState.AIMING
          && isFiringAllowedInCurrentZone()) {
        if (feedingSuppressedSupplier.getAsBoolean()) {
          coordinatorState = CoordinatorState.HELD;
          stateReason = "timeout, held";
        } else {
          coordinatorState = CoordinatorState.FIRING;
          stateReason = "timeout forced";
        }
        readyTimeoutRunning = false;
        timeoutForced = true;
      }

      if (!timeoutForced) {
        switch (coordinatorState) {
          case AIMING -> {
            if (allReady) {
              if (feedingSuppressedSupplier.getAsBoolean()) {
                coordinatorState = CoordinatorState.HELD;
                stateReason = "all ready, held";
              } else {
                coordinatorState = CoordinatorState.FIRING;
                stateReason = "all ready";
              }
            }
          }
          case SETTLING -> {
            if (allReady && turretNotFlipping) {
              if (feedingSuppressedSupplier.getAsBoolean()) {
                coordinatorState = CoordinatorState.HELD;
                stateReason = "settled, held";
              } else {
                coordinatorState = CoordinatorState.FIRING;
                stateReason = "settled, all ready";
              }
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
      if (!shotExists) blocking.append("no_shot ");
      if (!velCompOk) {
        if (geoFenceActive) {
          blocking.append(String.format("geofence(%s) ", geoFenceReason));
        } else {
          blocking.append(
              String.format(
                  "velcomp(%.1f/%.0fdeg) ",
                  currentDivergenceDeg, velocityCompMaxDivergenceDeg.get()));
        }
      }
      if (!speedOk) blocking.append("speed ");
      loggedBlocking = blocking.length() > 0 ? blocking.toString().trim() : "none";
    }
  }

  // ========== Goal-based Output API ==========

  private static boolean isFixedGoal(Goal g) {
    return g == Goal.FIXED_HUB || g == Goal.FIXED_LEFT_TRENCH || g == Goal.FIXED_RIGHT_TRENCH;
  }

  private FixedShotParams fixedShotParamsFor(Goal g) {
    return switch (g) {
      case FIXED_HUB ->
          new FixedShotParams(
              hubShotLauncherRPM::get,
              hubShotHoodAngleDeg::get,
              hubShotTurretAngleDeg::get,
              hubShotMotivatorRPM::get,
              hubShotSpindexerRPM::get);
      case FIXED_LEFT_TRENCH ->
          new FixedShotParams(
              leftTrenchLauncherRPM::get,
              leftTrenchHoodAngleDeg::get,
              leftTrenchTurretAngleDeg::get,
              leftTrenchMotivatorRPM::get,
              leftTrenchSpindexerRPM::get);
      case FIXED_RIGHT_TRENCH ->
          new FixedShotParams(
              rightTrenchLauncherRPM::get,
              rightTrenchHoodAngleDeg::get,
              rightTrenchTurretAngleDeg::get,
              rightTrenchMotivatorRPM::get,
              rightTrenchSpindexerRPM::get);
      default -> throw new IllegalArgumentException("Not a fixed goal: " + g);
    };
  }

  /** Set the active goal. Commands call this in startEnd(on, off). */
  public void setGoal(Goal newGoal) {
    setGoal(newGoal, ArmTrigger.IMMEDIATE);
  }

  /**
   * Set the active goal with an explicit arm trigger for smart-shoot goals. Smart-shoot goals
   * activate the state machine; fixed goals also use it (IMMEDIATE trigger). IDLE stops hardware.
   */
  public void setGoal(Goal newGoal, ArmTrigger trigger) {
    if (newGoal == this.goal) return;
    Goal prevGoal = this.goal;
    this.goal = newGoal;
    Logger.recordOutput("SmartLaunch/Goal", newGoal.name());
    SmartDashboard.putString("Match/Status/Goal", newGoal.name());

    boolean wasActive = prevGoal != Goal.IDLE;
    boolean nowActive = newGoal != Goal.IDLE;

    if (!wasActive && nowActive) {
      ArmTrigger at =
          switch (newGoal) {
            case SMART_SHOOT_ON_PASS_ZONE -> ArmTrigger.ON_PASS_ZONE;
            case SMART_SHOOT_ON_ALLIANCE_RETURN -> ArmTrigger.ON_ALLIANCE_RETURN;
            default -> trigger;
          };
      setSmartLaunchActive(true, at);
      motivatorWasIdle = true;
    } else if (wasActive && !nowActive) {
      setSmartLaunchActive(false);
      applyIdle();
    } else {
      // Active goal changed (e.g. fixed hub → fixed left trench) — restart state machine.
      setSmartLaunchActive(false);
      ArmTrigger at =
          switch (newGoal) {
            case SMART_SHOOT_ON_PASS_ZONE -> ArmTrigger.ON_PASS_ZONE;
            case SMART_SHOOT_ON_ALLIANCE_RETURN -> ArmTrigger.ON_ALLIANCE_RETURN;
            default -> trigger;
          };
      setSmartLaunchActive(true, at);
      motivatorWasIdle = true;
    }
  }

  public Goal getGoal() {
    return goal;
  }

  /**
   * Returns all subsystems managed by this coordinator (itself + launcher, hood, motivator,
   * spindexer, turret). Pass to {@code Commands.startEnd()} so the command scheduler knows which
   * subsystems the shoot command owns.
   */
  public Subsystem[] allRequirements() {
    List<Subsystem> reqs = new ArrayList<>();
    reqs.add(this);
    if (launcher != null) reqs.add(launcher);
    if (hood != null) reqs.add(hood);
    if (motivator != null) reqs.add(motivator);
    if (spindexer != null) reqs.add(spindexer);
    reqs.add(turret);
    return reqs.toArray(new Subsystem[0]);
  }

  // ========== Output Application (periodic → hardware) ==========

  private void applyOutputs() {
    switch (goal) {
      case SMART_SHOOT,
              SMART_SHOOT_ON_PASS_ZONE,
              SMART_SHOOT_ON_ALLIANCE_RETURN ->
          applySmartShoot();
      case FIXED_HUB -> applyFixed(fixedShotParamsFor(Goal.FIXED_HUB));
      case FIXED_LEFT_TRENCH -> applyFixed(fixedShotParamsFor(Goal.FIXED_LEFT_TRENCH));
      case FIXED_RIGHT_TRENCH -> applyFixed(fixedShotParamsFor(Goal.FIXED_RIGHT_TRENCH));
      case IDLE -> {} // applyIdle() is called once on the IDLE transition in setGoal()
    }
  }

  private void applySmartShoot() {
    ShotCalculator.ShotResult shot = currentShot;

    // Launcher — track shot RPM; zero when no shot or idling
    if (launcher != null) {
      double rpm =
          shot != null ? getEffectiveLauncherRPM(ShotOverrides.getLauncherRPM(shot)) : 0;
      launcher.setVelocity(rpm);
    }

    // Turret — track shot angle; hold position during auto collecting.
    // Not marked actively commanded while UNARMED so turret stays IDLE in logs.
    boolean unarmed = coordinatorState == CoordinatorState.UNARMED;
    if (!unarmed) {
      if (isAutoCollecting()) {
        turret.setActivelyCommanded(true);
      } else if (shot != null) {
        turret.setActivelyCommanded(true);
        turret.setOutsideTurretAngle(currentTurretAngleDeg);
      }
    }

    // Hood — follow shot angle; force to min in no-fire zones, while collecting, or when held.
    if (hood != null) {
      boolean forceMin =
          coordinatorState == CoordinatorState.NO_FIRE_ZONE || unarmed;
      if (forceMin) {
        hood.setActivelyCommanded(true);
        hood.setHoodAngle(hood.getMinAngle());
      } else if (spindexer != null && spindexer.isFeedingSuppressed()) {
        // Hold Fire button → keep hood down; pops up when released
        hood.setActivelyCommanded(true);
        hood.setHoodAngle(hood.getMinAngle());
      } else if (isAutoCollecting()) {
        hood.setActivelyCommanded(true);
        hood.setHoodAngle(hood.getMinAngle());
      } else if (shot != null) {
        hood.setActivelyCommanded(true);
        hood.setHoodAngle(ShotOverrides.getHoodDeg(shot));
      }
    }

    // Motivator — pre-spins with a brief reverse pulse on each idle→spinning transition
    if (motivator != null) {
      applyMotivatorPreSpinner(shot);
    }

    // Spindexer — feeds only when coordinator reaches FIRING; stops during auto collecting
    if (spindexer != null) {
      if (isFeedingAllowed()) {
        if (launcher != null) launcher.setFeedingActive(true);
        double spnRPM = ShotOverrides.getSpindexerRPM(shot);
        if (turret.getState() == Turret.TurretState.FLIPPING) {
          spindexer.stopSpindexer();
        } else {
          spindexer.setSpindexerVelocity(spnRPM);
          fireSimBallIfReady();
        }
      } else if (isAutoCollecting()) {
        if (launcher != null) launcher.setFeedingActive(false);
        spindexer.stopSpindexer();
      } else {
        spindexer.stopSpindexer();
      }
    }
  }

  private void applyMotivatorPreSpinner(ShotCalculator.ShotResult shot) {
    if (isAutoCollecting()) {
      motivator.stopMotivator();
      motivatorWasIdle = true;
      motivatorReversing = false;
      return;
    }
    // Trigger reverse pulse each time motivator transitions from idle to spinning
    if (motivatorWasIdle) {
      motivatorWasIdle = false;
      motivatorReversing = true;
      motivatorReversePulseTimer.restart();
    }
    if (motivatorReversing) {
      if (motivatorReversePulseTimer.hasElapsed(MOTIVATOR_REVERSE_PULSE_SEC)) {
        motivatorReversing = false;
      } else {
        motivator.setMotivatorVoltage(-1.0);
        return;
      }
    }
    if (shot != null) {
      motivator.setMotivatorVelocity(ShotOverrides.getMotivatorRPM(shot));
    } else {
      motivator.stopMotivator();
    }
  }

  private void applyFixed(FixedShotParams params) {
    // currentShot and currentTurretAngleDeg were already set in updateShotCalculation()
    // from the same params, so the state machine evaluated readiness correctly this cycle.
    double motivatorRPM = params.motivatorRPM().getAsDouble();
    double spindexerRPM = params.spindexerRPM().getAsDouble();

    if (launcher != null) {
      launcher.setVelocity(currentShot != null ? ShotOverrides.getLauncherRPM(currentShot) : 0);
    }
    turret.setActivelyCommanded(true);
    turret.setOutsideTurretAngle(currentTurretAngleDeg);
    if (hood != null) {
      hood.setActivelyCommanded(true);
      hood.setHoodAngle(
          currentShot != null ? ShotOverrides.getHoodDeg(currentShot) : hood.getMinAngle());
    }
    if (motivator != null) {
      // Spin motivator only once launcher + turret + hood have all reached their targets
      boolean allPositioned =
          (launcher == null || launcher.isReady())
              && turret.atTarget()
              && (hood == null || hood.atTarget())
              && turret.getState() != Turret.TurretState.FLIPPING
              && turret.getState() != Turret.TurretState.STALLED;
      if (allPositioned) {
        motivator.setMotivatorVelocity(motivatorRPM);
      } else {
        motivator.stopMotivator();
      }
    }
    if (spindexer != null) {
      if (isFeedingAllowed()) {
        if (launcher != null) launcher.setFeedingActive(true);
        if (turret.getState() != Turret.TurretState.FLIPPING) {
          spindexer.setSpindexerVelocity(spindexerRPM);
          fireSimBallIfReady();
        } else {
          spindexer.stopSpindexer();
        }
      } else {
        spindexer.stopSpindexer();
      }
    }
  }

  private void applyIdle() {
    if (launcher != null) {
      launcher.setFeedingActive(false);
      launcher.stop();
    }
    if (motivator != null) motivator.stopMotivator();
    if (spindexer != null) spindexer.stopSpindexer();
    if (hood != null) {
      hood.setActivelyCommanded(false);
      hood.setHoodAngle(hood.getMinAngle());
    }
    turret.setActivelyCommanded(false);
    if (turret.getState() == Turret.TurretState.STALLED) {
      turret.forceTurretOutOfStallState();
    }
    clearManualShotParameters();
    motivatorWasIdle = true;
    motivatorReversing = false;
  }

  private void fireSimBallIfReady() {
    if (Constants.currentMode == Constants.Mode.REAL) return;
    double now = Timer.getFPGATimestamp();
    if (now - lastSimFireTimestamp < MIN_SHOT_INTERVAL_SECONDS) return;
    if (visualizer != null && visualizer.getFuelCount() <= 0) return;
    launchFuel();
    if (launcher != null) launcher.notifyBallFired();
    lastSimFireTimestamp = now;
    Logger.recordOutput("ShotLog/LastShotTime", now);
    Logger.recordOutput("ShotLog/FuelRemaining", visualizer != null ? visualizer.getFuelCount() : 0);
  }

  // ========== Command Factories ==========

  /** Smart launch command — arms immediately (teleop default). */
  public Command shootCommand() {
    return shootCommand(ArmTrigger.IMMEDIATE);
  }

  /** Smart launch command with configurable arm trigger (for sprint autos). */
  public Command shootCommand(ArmTrigger trigger) {
    Goal targetGoal =
        switch (trigger) {
          case IMMEDIATE -> Goal.SMART_SHOOT;
          case ON_PASS_ZONE -> Goal.SMART_SHOOT_ON_PASS_ZONE;
          case ON_ALLIANCE_RETURN -> Goal.SMART_SHOOT_ON_ALLIANCE_RETURN;
        };
    return Commands.startEnd(
            () -> setGoal(targetGoal), () -> setGoal(Goal.IDLE), allRequirements())
        .withName("SmartLaunch");
  }

  /** Fixed hub shot at tunable setpoints. */
  public Command hubShotCommand() {
    return Commands.startEnd(
            () -> setGoal(Goal.FIXED_HUB), () -> setGoal(Goal.IDLE), allRequirements())
        .withName("HubShot");
  }

  /** Fixed left-trench shot at tunable setpoints. */
  public Command leftTrenchShotCommand() {
    return Commands.startEnd(
            () -> setGoal(Goal.FIXED_LEFT_TRENCH), () -> setGoal(Goal.IDLE), allRequirements())
        .withName("LeftTrenchShot");
  }

  /** Fixed right-trench shot at tunable setpoints. */
  public Command rightTrenchShotCommand() {
    return Commands.startEnd(
            () -> setGoal(Goal.FIXED_RIGHT_TRENCH), () -> setGoal(Goal.IDLE), allRequirements())
        .withName("RightTrenchShot");
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
      case HUB -> shootOnTheMoveSpeedMps.get() - 0.05;
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
      boolean achievable = currentShotAchievable;
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
        compensatedAimTarget,
        currentExitVelocityMps,
        turretConfig.heightMeters(),
        turretConfig.xOffset(),
        turretConfig.yOffset(),
        readiness,
        cachedAimResult != null ? cachedAimResult.zone() : null);
  }
}
