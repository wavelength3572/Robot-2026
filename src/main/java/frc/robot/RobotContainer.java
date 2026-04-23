// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoWrapperFactory;
import frc.robot.commands.ShootingCommands;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSpark;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherIO;
import frc.robot.subsystems.launcher.LauncherIOSim;
import frc.robot.subsystems.launcher.LauncherIOSparkFlex;
import frc.robot.subsystems.led.IndicatorLight;
import frc.robot.subsystems.motivator.Motivator;
import frc.robot.subsystems.motivator.MotivatorIO;
import frc.robot.subsystems.motivator.MotivatorIOSim;
import frc.robot.subsystems.motivator.MotivatorIOSparkFlex;
import frc.robot.subsystems.shooting.ShootingCoordinator;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOSim;
import frc.robot.subsystems.spindexer.SpindexerIOSparkMax;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.FuelSim;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.RobotStatus;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private final Drive drive;
  private final Turret turret;
  private final Vision vision;
  private final Intake intake;
  private final Launcher launcher;
  private final Hood hood;
  private final Motivator motivator;
  private final Spindexer spindexer;
  private final Climber climber;
  private final ShootingCoordinator shootingCoordinator;
  private final IndicatorLight leds;
  private OperatorInterface oi = new OperatorInterface() {};

  private LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<String> allianceWinChooser;
  private SendableChooser<AutoWrapperFactory.StartStrategy> startStrategyChooser;
  private SendableChooser<AutoWrapperFactory.PathShootingStrategy> pathShootingChooser;
  private boolean lastCompetitionMode = true;

  // Maps display name (e.g. "[Shot] 3 Piece Source") → raw auto name ("3 Piece Source").
  // Rebuilt alongside autoChooser so display prefixes are a pure presentation concern.
  private Map<String, String> displayToAutoName = new HashMap<>();

  // Simulation: track last selected auto for pose updates
  private String lastSelectedAutoName = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    RobotConfig config = Constants.getRobotConfig();

    // Instantiate all subsystems
    switch (Constants.currentMode) {
      case REAL:
        turret = config.hasTurret() ? new Turret(new TurretIOSparkMax()) : null;
        intake = config.hasIntake() ? new Intake(new IntakeIOSparkMax()) : null;
        launcher = config.hasLauncher() ? new Launcher(new LauncherIOSparkFlex()) : null;
        hood = config.hasHood() ? new Hood(new HoodIOSparkMax()) : null;
        motivator = config.hasMotivator() ? new Motivator(new MotivatorIOSparkFlex()) : null;
        spindexer = config.hasSpindexer() ? new Spindexer(new SpindexerIOSparkMax()) : null;
        climber = config.hasClimber() ? new Climber(new ClimberIOSpark()) : null;

        drive =
            config.hasDrive()
                ? new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOSpark(0),
                    new ModuleIOSpark(1),
                    new ModuleIOSpark(2),
                    new ModuleIOSpark(3),
                    turret)
                : new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    turret);

        vision =
            config.hasVision()
                ? new Vision(
                    drive::addVisionMeasurement,
                    new String[] {"CenterRear", "RightFront", "LeftRear", "RightRear"},
                    new VisionIOPhotonVision(
                        VisionConstants.centerRearCam, VisionConstants.mainBotToCenterRearCam),
                    new VisionIOPhotonVision(
                        VisionConstants.rightFrontCam, VisionConstants.mainBotToRightFrontCam),
                    new VisionIOPhotonVision(
                        VisionConstants.leftRearCam, VisionConstants.mainBotToLeftRearCam),
                    new VisionIOPhotonVision(
                        VisionConstants.rightRearCam, VisionConstants.mainBotToRightRearCam))
                : null;
        break;

      case PIT:
        // Pit mode: simulated drive, real everything else.
        // Lets the team "drive" around the field virtually while observing real
        // turret, launcher, hood, vision, and shot calculations.
        turret = config.hasTurret() ? new Turret(new TurretIOSparkMax()) : null;
        intake = config.hasIntake() ? new Intake(new IntakeIOSparkMax()) : null;
        launcher = config.hasLauncher() ? new Launcher(new LauncherIOSparkFlex()) : null;
        hood = config.hasHood() ? new Hood(new HoodIOSparkMax()) : null;
        motivator = config.hasMotivator() ? new Motivator(new MotivatorIOSparkFlex()) : null;
        spindexer = config.hasSpindexer() ? new Spindexer(new SpindexerIOSparkMax()) : null;
        climber = config.hasClimber() ? new Climber(new ClimberIOSpark()) : null;

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                turret);

        vision =
            config.hasVision()
                ? new Vision(
                    drive::addVisionMeasurement,
                    new String[] {"CenterRear", "RightFront", "LeftRear", "RightRear"},
                    new VisionIOPhotonVision(
                        VisionConstants.centerRearCam, VisionConstants.mainBotToCenterRearCam),
                    new VisionIOPhotonVision(
                        VisionConstants.rightFrontCam, VisionConstants.mainBotToRightFrontCam),
                    new VisionIOPhotonVision(
                        VisionConstants.leftRearCam, VisionConstants.mainBotToLeftRearCam),
                    new VisionIOPhotonVision(
                        VisionConstants.rightRearCam, VisionConstants.mainBotToRightRearCam))
                : null;
        break;

      case SIM:
        turret = config.hasTurret() ? new Turret(new TurretIOSim()) : null;
        intake = config.hasIntake() ? new Intake(new IntakeIOSim()) : null;
        launcher = config.hasLauncher() ? new Launcher(new LauncherIOSim()) : null;
        hood = config.hasHood() ? new Hood(new HoodIOSim()) : null;
        motivator = config.hasMotivator() ? new Motivator(new MotivatorIOSim()) : null;
        spindexer = config.hasSpindexer() ? new Spindexer(new SpindexerIOSim()) : null;
        climber = config.hasClimber() ? new Climber(new ClimberIOSim()) : null;

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                turret);

        vision =
            config.hasVision()
                ? new Vision(
                    drive::addVisionMeasurement,
                    new String[] {"CenterRear", "RightFront", "LeftRear", "RightRear"},
                    new VisionIOPhotonVisionSim(
                        VisionConstants.centerRearCam,
                        VisionConstants.mainBotToCenterRearCam,
                        RobotStatus::getRobotPose),
                    new VisionIOPhotonVisionSim(
                        VisionConstants.rightFrontCam,
                        VisionConstants.mainBotToRightFrontCam,
                        RobotStatus::getRobotPose),
                    new VisionIOPhotonVisionSim(
                        VisionConstants.leftRearCam,
                        VisionConstants.mainBotToLeftRearCam,
                        RobotStatus::getRobotPose),
                    new VisionIOPhotonVisionSim(
                        VisionConstants.rightRearCam,
                        VisionConstants.mainBotToRightRearCam,
                        RobotStatus::getRobotPose))
                : null;
        break;

      default:
        // Replay mode — no-op IO for all subsystems
        turret = config.hasTurret() ? new Turret(new TurretIO() {}) : null;
        intake = config.hasIntake() ? new Intake(new IntakeIO() {}) : null;
        launcher = config.hasLauncher() ? new Launcher(new LauncherIO() {}) : null;
        hood = config.hasHood() ? new Hood(new HoodIO() {}) : null;
        motivator = config.hasMotivator() ? new Motivator(new MotivatorIO() {}) : null;
        spindexer = config.hasSpindexer() ? new Spindexer(new SpindexerIO() {}) : null;
        climber = config.hasClimber() ? new Climber(new ClimberIO() {}) : null;

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                turret);

        vision =
            config.hasVision()
                ? new Vision(
                    (pose, time, stdDevs) -> {},
                    new String[] {"CenterRear", "RightFront", "LeftRear", "RightRear"},
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {})
                : null;
        break;
    }

    // Hood default command: hold at minimum angle (safe for trench clearance).
    // Fire commands (smartLaunch, fixedPositionLaunch) interrupt this via subsystem
    // requirements; when they end, hood automatically returns to min angle.
    if (hood != null) {
      hood.setDefaultCommand(
          Commands.run(
                  () -> {
                    hood.setActivelyCommanded(false);
                    hood.setHoodAngle(hood.getMinAngle());
                  },
                  hood)
              .withName("HoodStow"));
    }

    // Initialize RobotStatus with subsystem references (vision may be null for
    // RectangleBot)
    RobotStatus.initialize(drive, vision);

    // Connect vision to drive for adaptive std dev scaling based on robot speed
    if (vision != null) {
      vision.setRobotSpeedSupplier(
          () -> {
            var speeds = drive.getChassisSpeeds();
            return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
          });
    }

    // Connect intake to drive for velocity-based roller speed
    if (intake != null) {
      intake.setRobotVelocitySupplier(
          () -> {
            var speeds = drive.getChassisSpeeds();
            return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
          });
    }

    // Create ShootingCoordinator (orchestrates turret + hood + launcher)
    // Must be created BEFORE FuelSim so intake registration can reference the
    // coordinator
    if (turret != null) {
      shootingCoordinator = new ShootingCoordinator(turret, hood, launcher, motivator, spindexer);
      shootingCoordinator.initialize(
          drive::getPose, drive::getFieldRelativeSpeeds, drive::getPitchDeg);
      // Gate launching: zone-aware hub shift gating with pre-active cutoff
      // (unless "Ignore Hub State" dashboard toggle is on)
      shootingCoordinator.setFeedingSuppressedSupplier(
          () -> {
            // Auto hold-fire (autonomous named commands) — unconditional
            if (autoFeedingSuppressed) return true;

            // Compute hub-shift suppression (only when Ignore Hub State is OFF)
            boolean hubSuppressed = false;
            if (!SmartDashboard.getBoolean("Match/Ignore Hub State", false)) {
              HubShiftUtil.ShiftInfo shifted = HubShiftUtil.getShiftedShiftInfo();

              if (!shifted.active()) {
                double cutoff = HubShiftUtil.preActiveCutoffSeconds.get();
                if (shifted.remainingTime() <= cutoff) {
                  // Pre-active cutoff: suppress ALL (stop passing, reposition)
                  hubSuppressed = true;
                } else if (shootingCoordinator.isInAllianceZone()) {
                  // Alliance zone + inactive: can't score
                  hubSuppressed = true;
                }
                // else: neutral/opponent zone + inactive + outside cutoff: passing OK
              }
            }
            Logger.recordOutput("HubShift/AutoSuppressed", hubSuppressed);

            // Operator hold-fire — checked AFTER so we can detect conflict
            boolean operatorSuppressed = spindexer != null && spindexer.isFeedingSuppressed();
            // Log: operator blocking firing that hub state allows
            Logger.recordOutput(
                "HubShift/OperatorOverridingAllowed", operatorSuppressed && !hubSuppressed);

            return hubSuppressed || operatorSuppressed;
          });
    } else {
      shootingCoordinator = null;
    }

    // LED subsystem: 42 LEDs on PWM 0, wired in parallel to two physical strips
    leds = new IndicatorLight();

    // Wire turret encoder validation to LEDs for startup error/warning patterns
    if (turret != null) {
      leds.setTurretEncoderStatusSupplier(turret::getEncoderValidationStatus);
    }

    // Wire shooting coordinator state to LEDs for SmartLaunch status overlay
    if (shootingCoordinator != null) {
      leds.setShootingCoordinatorSuppliers(
          shootingCoordinator::getCoordinatorState, shootingCoordinator::isSmartLaunchActive);
    }

    // Climber (pre-feed wheel): runs at spindexerTargetRPM / Tuning/Climber/SpindexerFollowRatio
    // whenever the spindexer is commanded to feed. Stops otherwise.
    if (climber != null && spindexer != null) {
      climber.setDefaultCommand(
          Commands.run(
                  () -> {
                    double spinTarget = spindexer.getSpindexerTargetRPM();
                    if (spinTarget != 0.0) {
                      double ratio = Climber.getSpindexerFollowRatio();
                      if (ratio != 0.0) {
                        climber.setClimberVelocity(spinTarget / ratio);
                      } else {
                        climber.stopClimber();
                      }
                    } else {
                      climber.stopClimber();
                    }
                  },
                  climber)
              .withName("ClimberFollowSpindexer"));
    }

    // Initialize FuelSim for simulation mode (after coordinator so intake can be
    // registered)
    if (Constants.currentMode == Constants.Mode.SIM
        || Constants.currentMode == Constants.Mode.PIT) {
      initializeFuelSim();
    }

    // Register NamedCommands for PathPlanner autos (must be BEFORE
    // buildAutoChooser)
    registerNamedCommands();

    // Dashboard toggle: ignore hub shift state (bypass launch gating)
    SmartDashboard.putBoolean("Match/Ignore Hub State", true);

    // Alliance win override chooser (for HubShiftUtil shift schedule)
    allianceWinChooser = new LoggedDashboardChooser<>("Alliance Win Override");
    allianceWinChooser.addDefaultOption("Auto (FMS)", "auto");
    allianceWinChooser.addOption("Won Auto", "won");
    allianceWinChooser.addOption("Lost Auto", "lost");
    HubShiftUtil.setAllianceWinOverride(
        () -> {
          String value = allianceWinChooser.get();
          if ("won".equals(value)) return Optional.of(true);
          if ("lost".equals(value)) return Optional.of(false);
          return Optional.empty();
        });

    // Launcher default command: pre-spin flywheel before active hub shifts so
    // the robot is ready to fire instantly.  Shooting commands (smartLaunch,
    // hubShot, etc.) override this via subsystem requirements; when they end the
    // default resumes.
    if (launcher != null) {
      launcher.setDefaultCommand(
          Commands.run(
                  () -> {
                    // When ignoring hub state (practice mode), skip pre-spin entirely
                    if (SmartDashboard.getBoolean("Match/Ignore Hub State", true)) {
                      launcher.stop();
                      return;
                    }
                    // Safety: without FMS, require an explicit alliance win override
                    // so the flywheel doesn't surprise-spin at the shop.
                    if (!edu.wpi.first.wpilibj.DriverStation.isFMSAttached()
                        && "auto".equals(allianceWinChooser.get())) {
                      launcher.stop();
                      return;
                    }
                    HubShiftUtil.ShiftInfo shift = HubShiftUtil.getShiftedShiftInfo();
                    boolean shouldSpin =
                        shift.active() || (!shift.active() && shift.remainingTime() <= 3.0);
                    if (shouldSpin
                        && shootingCoordinator != null
                        && shootingCoordinator.getCurrentShot() != null) {
                      launcher.setVelocity(shootingCoordinator.getCurrentShot().launcherRPM());
                    } else if (shouldSpin) {
                      launcher.setVelocity(1500.0);
                    } else {
                      launcher.stop();
                    }
                  },
                  launcher)
              .withName("LauncherShiftIdle"));
    }

    // Dashboard toggle: defaults to competition mode (hides test autos)
    SmartDashboard.putBoolean("Competition Mode", true);
    autoChooser = buildAutoChooserForMode(true);

    // Auton strategy choosers — created once; selection resets when auto changes.
    initStrategyChoosers();

    updateOI();
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      return;
    }
    normalModeOI();
  }

  public void normalModeOI() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    oi = OISelector.findOperatorInterface();
    ButtonsAndDashboardBindings.configureBindings(
        oi,
        drive,
        vision,
        intake,
        turret,
        launcher,
        motivator,
        spindexer,
        hood,
        shootingCoordinator,
        climber);
  }

  // Comp autos get the full shooting wrap (fuel, auto-shoot, intake, launcher
  // spin-up).
  // Maps auto name → folder name (e.g. "TrenchLeft", "Depot", "Retired").
  // Built dynamically at startup by scanning PathPlanner .auto files.
  private final Map<String, String> autoFolderMap = loadAutoFolderMap();

  // Toggled by holdFire/releaseFire named commands during auto paths.
  // Wired into feedingSuppressedSupplier so ShootingCoordinator respects it.
  private boolean autoFeedingSuppressed = false;

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class. Dispatches to the
   * appropriate wrapper based on the auto's folder. The folder provides default strategies which
   * the dashboard choosers can override. Test autos and unknown folders run bare.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command selectedAuto = autoChooser.get();
    if (selectedAuto == null) return null;

    String displayName = autoChooser.getSendableChooser().getSelected();
    if (displayName == null) return selectedAuto;

    // Resolve display name → raw auto name for PathPlanner lookup
    String rawName = displayToAutoName.getOrDefault(displayName, displayName);
    edu.wpi.first.math.geometry.Pose2d startingPose = resolveStartingPose(rawName);

    // Read strategies directly — choosers always have concrete values
    AutoWrapperFactory.StartStrategy startStrategy = startStrategyChooser.getSelected();
    AutoWrapperFactory.PathShootingStrategy pathStrategy = pathShootingChooser.getSelected();
    if (startStrategy == null) startStrategy = defaultStartStrategy(rawName);
    if (pathStrategy == null) pathStrategy = defaultPathShootingStrategy(rawName);

    return AutoWrapperFactory.compWrapped(
        selectedAuto,
        startingPose,
        startStrategy,
        pathStrategy,
        drive,
        intake,
        launcher,
        shootingCoordinator,
        motivator,
        turret,
        hood,
        spindexer);
  }

  /**
   * Per-auto strategy defaults. Each auto maps to its (StartStrategy, PathShootingStrategy) pair.
   * Autos not in this map run bare (no comp wrapper). Folders are now purely organizational (by
   * start location), so defaults live here instead.
   */
  private static final Map<String, AutoWrapperFactory.StartStrategy> AUTO_START_DEFAULTS =
      Map.ofEntries(
          // TrenchLeft autos
          Map.entry("TrenchLeftFollow", AutoWrapperFactory.StartStrategy.SHOOT_PRELOADS),
          Map.entry("TrenchLeft1CycleDepot", AutoWrapperFactory.StartStrategy.SPRINT),
          Map.entry("TrenchLeft2.5Loops", AutoWrapperFactory.StartStrategy.SPRINT),
          Map.entry("TrenchRight2.5Loops", AutoWrapperFactory.StartStrategy.SPRINT),
          Map.entry("TrenchLeft2CyclesAggressive", AutoWrapperFactory.StartStrategy.SPRINT),
          Map.entry("TrenchLeft2CyclesSafe", AutoWrapperFactory.StartStrategy.SPRINT),
          Map.entry("TrenchLeft2LoopsClimb", AutoWrapperFactory.StartStrategy.SPRINT),
          Map.entry("TrenchRight2LoopsClimb", AutoWrapperFactory.StartStrategy.SPRINT),
          Map.entry("TrenchLeftSnowblowFrontDepotClimb", AutoWrapperFactory.StartStrategy.SPRINT),
          Map.entry("TrenchLeftSnowblowSideDepotClimb", AutoWrapperFactory.StartStrategy.SPRINT),
          Map.entry("TrenchLeftBumpDepot", AutoWrapperFactory.StartStrategy.SPRINT),
          Map.entry("TrenchLeftBumpDepotClimb", AutoWrapperFactory.StartStrategy.SPRINT),
          // TrenchRight autos
          Map.entry("TrenchRightFollow", AutoWrapperFactory.StartStrategy.SHOOT_PRELOADS),
          Map.entry("TrenchRight2CyclesAggressive", AutoWrapperFactory.StartStrategy.SPRINT),
          Map.entry("TrenchRight2CyclesBulldogs", AutoWrapperFactory.StartStrategy.SPRINT),
          Map.entry("TrenchRight2CyclesSafe", AutoWrapperFactory.StartStrategy.SPRINT),
          Map.entry("TrenchRightSnowblowClimb", AutoWrapperFactory.StartStrategy.SPRINT),
          // Depot autos
          Map.entry("DepotClimb", AutoWrapperFactory.StartStrategy.SHOOT_PRELOADS),
          Map.entry("DepotLeftTrench", AutoWrapperFactory.StartStrategy.SHOOT_PRELOADS),
          Map.entry("Depot-Outpost", AutoWrapperFactory.StartStrategy.SHOOT_PRELOADS),
          // Outpost autos
          Map.entry("Outpost-Depot", AutoWrapperFactory.StartStrategy.SHOOT_PRELOADS),
          Map.entry("OutpostRightTrench", AutoWrapperFactory.StartStrategy.SHOOT_PRELOADS),
          // Retired autos (wrapped so they work if switched to practice mode at the field)
          Map.entry("TrenchLeftAggressive", AutoWrapperFactory.StartStrategy.SPRINT),
          Map.entry("TrenchLeftSafe", AutoWrapperFactory.StartStrategy.SPRINT),
          Map.entry("TrenchRightAggressive", AutoWrapperFactory.StartStrategy.SPRINT),
          Map.entry("TrenchRightSafe", AutoWrapperFactory.StartStrategy.SPRINT));

  private static final Map<String, AutoWrapperFactory.PathShootingStrategy>
      AUTO_PATH_SHOOTING_DEFAULTS =
          Map.ofEntries(
              // TrenchLeft autos
              Map.entry("TrenchLeftFollow", AutoWrapperFactory.PathShootingStrategy.PASS_AND_SHOOT),
              Map.entry("TrenchLeft1CycleDepot", AutoWrapperFactory.PathShootingStrategy.NO_PASS),
              Map.entry("TrenchLeft2.5Loops", AutoWrapperFactory.PathShootingStrategy.NO_PASS),
              Map.entry("TrenchRight2.5Loops", AutoWrapperFactory.PathShootingStrategy.NO_PASS),
              Map.entry(
                  "TrenchLeft2CyclesAggressive", AutoWrapperFactory.PathShootingStrategy.NO_PASS),
              Map.entry("TrenchLeft2CyclesSafe", AutoWrapperFactory.PathShootingStrategy.NO_PASS),
              Map.entry("TrenchLeft2LoopsClimb", AutoWrapperFactory.PathShootingStrategy.NO_PASS),
              Map.entry("TrenchRight2LoopsClimb", AutoWrapperFactory.PathShootingStrategy.NO_PASS),
              Map.entry(
                  "TrenchLeftSnowblowFrontDepotClimb",
                  AutoWrapperFactory.PathShootingStrategy.PASS_AND_SHOOT),
              Map.entry(
                  "TrenchLeftSnowblowSideDepotClimb",
                  AutoWrapperFactory.PathShootingStrategy.PASS_AND_SHOOT),
              Map.entry("TrenchLeftBumpDepot", AutoWrapperFactory.PathShootingStrategy.NO_PASS),
              Map.entry(
                  "TrenchLeftBumpDepotClimb", AutoWrapperFactory.PathShootingStrategy.NO_PASS),
              // TrenchRight autos
              Map.entry(
                  "TrenchRightFollow", AutoWrapperFactory.PathShootingStrategy.PASS_AND_SHOOT),
              Map.entry(
                  "TrenchRight2CyclesAggressive", AutoWrapperFactory.PathShootingStrategy.NO_PASS),
              Map.entry(
                  "TrenchRight2CyclesBulldogs", AutoWrapperFactory.PathShootingStrategy.NO_PASS),
              Map.entry("TrenchRight2CyclesSafe", AutoWrapperFactory.PathShootingStrategy.NO_PASS),
              Map.entry(
                  "TrenchRightSnowblowClimb",
                  AutoWrapperFactory.PathShootingStrategy.PASS_AND_SHOOT),
              // Depot autos
              Map.entry("DepotClimb", AutoWrapperFactory.PathShootingStrategy.IMMEDIATE_ARM),
              Map.entry("DepotLeftTrench", AutoWrapperFactory.PathShootingStrategy.IMMEDIATE_ARM),
              Map.entry("Depot-Outpost", AutoWrapperFactory.PathShootingStrategy.IMMEDIATE_ARM),
              // Outpost autos
              Map.entry("Outpost-Depot", AutoWrapperFactory.PathShootingStrategy.IMMEDIATE_ARM),
              Map.entry(
                  "OutpostRightTrench", AutoWrapperFactory.PathShootingStrategy.IMMEDIATE_ARM),
              // Retired autos
              Map.entry("TrenchLeftAggressive", AutoWrapperFactory.PathShootingStrategy.NO_PASS),
              Map.entry("TrenchLeftSafe", AutoWrapperFactory.PathShootingStrategy.NO_PASS),
              Map.entry("TrenchRightAggressive", AutoWrapperFactory.PathShootingStrategy.NO_PASS),
              Map.entry("TrenchRightSafe", AutoWrapperFactory.PathShootingStrategy.NO_PASS));

  /** Returns true if the auto has known strategy defaults and should receive the comp wrapper. */
  private static boolean hasStrategyDefaults(String autoName) {
    return AUTO_START_DEFAULTS.containsKey(autoName);
  }

  /** Name-based default for start strategy. */
  private static AutoWrapperFactory.StartStrategy defaultStartStrategy(String autoName) {
    return AUTO_START_DEFAULTS.getOrDefault(autoName, AutoWrapperFactory.StartStrategy.SPRINT);
  }

  /** Name-based default for path shooting strategy. */
  private static AutoWrapperFactory.PathShootingStrategy defaultPathShootingStrategy(
      String autoName) {
    return AUTO_PATH_SHOOTING_DEFAULTS.getOrDefault(
        autoName, AutoWrapperFactory.PathShootingStrategy.NO_PASS);
  }

  /**
   * Rebuild and publish the strategy choosers with the given folder's defaults pre-selected. Called
   * on startup (with null folder) and whenever the auto selection changes. The operator can then
   * override from the dashboard.
   */
  /** Create the strategy choosers once at startup. Called from the constructor. */
  private void initStrategyChoosers() {
    startStrategyChooser = new SendableChooser<>();
    startStrategyChooser.setDefaultOption("—", null);
    startStrategyChooser.addOption(
        "Shoot Preloads", AutoWrapperFactory.StartStrategy.SHOOT_PRELOADS);
    startStrategyChooser.addOption("Sprint", AutoWrapperFactory.StartStrategy.SPRINT);
    SmartDashboard.putData("Auton Start Strategy", startStrategyChooser);

    pathShootingChooser = new SendableChooser<>();
    pathShootingChooser.setDefaultOption("—", null);
    pathShootingChooser.addOption(
        "End of Path", AutoWrapperFactory.PathShootingStrategy.END_OF_PATH);
    pathShootingChooser.addOption(
        "Pass and Shoot", AutoWrapperFactory.PathShootingStrategy.PASS_AND_SHOOT);
    pathShootingChooser.addOption("No Pass", AutoWrapperFactory.PathShootingStrategy.NO_PASS);
    pathShootingChooser.addOption(
        "Immediate Arm", AutoWrapperFactory.PathShootingStrategy.IMMEDIATE_ARM);
    SmartDashboard.putData("Auton Path Shooting Strategy", pathShootingChooser);
  }

  /**
   * Reset the strategy chooser selections to the auto-appropriate defaults. The chooser widgets
   * stay alive on the dashboard — only the selected value changes.
   */
  private void resetStrategySelections(String autoName) {
    AutoWrapperFactory.StartStrategy defaultStart =
        (autoName != null && AUTO_START_DEFAULTS.containsKey(autoName))
            ? defaultStartStrategy(autoName)
            : null;
    AutoWrapperFactory.PathShootingStrategy defaultPath =
        (autoName != null && AUTO_PATH_SHOOTING_DEFAULTS.containsKey(autoName))
            ? defaultPathShootingStrategy(autoName)
            : null;

    // Determine the option name strings to select
    String startName = "—";
    if (defaultStart == AutoWrapperFactory.StartStrategy.SHOOT_PRELOADS)
      startName = "Shoot Preloads";
    else if (defaultStart == AutoWrapperFactory.StartStrategy.SPRINT) startName = "Sprint";

    String pathName = "—";
    if (defaultPath == AutoWrapperFactory.PathShootingStrategy.END_OF_PATH)
      pathName = "End of Path";
    else if (defaultPath == AutoWrapperFactory.PathShootingStrategy.PASS_AND_SHOOT)
      pathName = "Pass and Shoot";
    else if (defaultPath == AutoWrapperFactory.PathShootingStrategy.NO_PASS) pathName = "No Pass";
    else if (defaultPath == AutoWrapperFactory.PathShootingStrategy.IMMEDIATE_ARM)
      pathName = "Immediate Arm";

    // Write the desired default into the NT "selected" key so the dashboard + getSelected() update
    var nt = NetworkTableInstance.getDefault();
    nt.getTable("SmartDashboard/Auton Start Strategy").getEntry("selected").setString(startName);
    nt.getTable("SmartDashboard/Auton Path Shooting Strategy")
        .getEntry("selected")
        .setString(pathName);
  }

  /**
   * Resolve the starting pose for a PathPlanner auto, flipped for red alliance. Returns null if the
   * auto has no valid starting pose.
   */
  private edu.wpi.first.math.geometry.Pose2d resolveStartingPose(String autoName) {
    try {
      PathPlannerAuto ppAuto = new PathPlannerAuto(autoName);
      edu.wpi.first.math.geometry.Pose2d pose = ppAuto.getStartingPose();
      if (pose != null) {
        edu.wpi.first.wpilibj.DriverStation.Alliance alliance =
            edu.wpi.first.wpilibj.DriverStation.getAlliance()
                .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);
        if (alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
          pose = com.pathplanner.lib.util.FlippingUtil.flipFieldPose(pose);
        }
      }
      return pose;
    } catch (Exception e) {
      return null;
    }
  }

  /**
   * Get the turret subsystem if it exists (SquareBot only).
   *
   * @return Turret subsystem or null if not present
   */
  public Turret getTurret() {
    return turret;
  }

  /**
   * Check if the turret subsystem is present on this robot.
   *
   * @return true if turret exists
   */
  public boolean hasTurret() {
    return turret != null;
  }

  /**
   * Get the shooting coordinator if it exists.
   *
   * @return ShootingCoordinator or null if not present
   */
  public ShootingCoordinator getShootingCoordinator() {
    return shootingCoordinator;
  }

  public Spindexer getSpindexer() {
    return spindexer;
  }

  /**
   * Get the vision subsystem if it exists (SquareBot only).
   *
   * @return Vision subsystem or null if not present
   */
  public Vision getVision() {
    return vision;
  }

  /**
   * Check if the vision subsystem is present on this robot.
   *
   * @return true if vision exists
   */
  public boolean hasVision() {
    return vision != null;
  }

  /**
   * Get the intake subsystem if it exists (SquareBot only).
   *
   * @return Intake subsystem or null if not present
   */
  public Intake getIntake() {
    return intake;
  }

  /**
   * Check if the intake subsystem is present on this robot.
   *
   * @return true if intake exists
   */
  public boolean hasIntake() {
    return intake != null;
  }

  /**
   * Get the launcher subsystem if it exists
   *
   * @return Launcher subsystem or null if not present
   */
  public Launcher getLauncher() {
    return launcher;
  }

  /**
   * Check if the launcher subsystem is present on this robot.
   *
   * @return true if launcher exists
   */
  public boolean hasLauncher() {
    return launcher != null;
  }

  /**
   * Check if the hood subsystem is present on this robot.
   *
   * @return true if hood exists
   */
  public boolean hasHood() {
    return hood != null;
  }

  /**
   * Get the hood subsystem.
   *
   * @return Hood subsystem or null if not present
   */
  public Hood getHood() {
    return hood;
  }

  /**
   * Get the drive subsystem.
   *
   * @return drive subsystem or null if not present
   */
  public Drive getDrive() {
    return drive;
  }

  /**
   * Check if the motivator subsystem is present on this robot.
   *
   * @return true if motivator exists
   */
  public boolean hasMotivator() {
    return motivator != null;
  }

  /**
   * Get the motivator subsystem.
   *
   * @return Motivator subsystem or null if not present
   */
  public Motivator getMotivator() {
    return motivator;
  }

  /** Get the climber subsystem (may be null). */
  public Climber getClimber() {
    return climber;
  }

  /** Register NamedCommands for PathPlanner autos. Must be called before buildAutoChooser. */
  private void registerNamedCommands() {
    // Set fuel count commands for testing
    NamedCommands.registerCommand(
        "setFuel40",
        Commands.runOnce(
            () -> {
              if (shootingCoordinator != null && shootingCoordinator.getVisualizer() != null) {
                shootingCoordinator.getVisualizer().setFuelCount(40);
              }
            }));

    NamedCommands.registerCommand(
        "setFuel25",
        Commands.runOnce(
            () -> {
              if (shootingCoordinator != null && shootingCoordinator.getVisualizer() != null) {
                shootingCoordinator.getVisualizer().setFuelCount(25);
              }
            }));

    // Wait until all fuel has been fired (with timeout for jams)
    NamedCommands.registerCommand(
        "waitUntilFuelEmpty",
        Commands.waitUntil(
                () -> {
                  if (shootingCoordinator == null || shootingCoordinator.getVisualizer() == null)
                    return true;
                  return shootingCoordinator.getVisualizer().getFuelCount() <= 0;
                })
            .withTimeout(5.0)
            .withName("WaitUntilFuelEmpty"));

    // Reset simulation commands
    if (shootingCoordinator != null) {
      NamedCommands.registerCommand(
          "resetSim", ShootingCommands.resetSimulationCommand(shootingCoordinator));
      NamedCommands.registerCommand(
          "resetStartingField", ShootingCommands.resetStartingFieldCommand(shootingCoordinator));
    }

    // Agitate: repeating kick agitation for zone event markers in auto paths.
    if (intake != null) {
      NamedCommands.registerCommand("Agitate", intake.agitateCommand(() -> false));
    }

    // PreClimbFlush: jostle + reverse rollers + stow (used before climb in auto)
    if (intake != null) {
      NamedCommands.registerCommand("PreClimbFlush", intake.preClimbFlushCommand());
    }

    // RunIntake: deploy intake and start rollers (used by Depot auto)
    if (intake != null) {
      NamedCommands.registerCommand(
          "RunIntake",
          Commands.runOnce(
              () -> {
                intake.deploy();
                intake.runIntake();
              },
              intake));
    }

    // SmartLaunch: state-machine-driven version (zone-aware, transition-safe).
    // Automatically stows hood when the button is released (teleop safety).
    {
      Command smartLaunch =
          ShootingCommands.smartLaunchDangerousCommand(
              launcher, shootingCoordinator, motivator, turret, hood, spindexer);
      if (hood != null) {
        Command autoStowHood =
            Commands.run(() -> hood.setHoodAngle(hood.getMinAngle()), hood)
                .until(() -> hood.getCurrentAngle() <= 18.0)
                .withTimeout(1.5);
        smartLaunch = smartLaunch.andThen(autoStowHood);
      }
      NamedCommands.registerCommand("SmartLaunch", smartLaunch);
    }

    // holdFire / releaseFire: suppress/allow feeding during auto paths.
    // Useful for accumulating balls before dumping at hub.
    NamedCommands.registerCommand("holdFire", Commands.runOnce(() -> autoFeedingSuppressed = true));
    NamedCommands.registerCommand(
        "releaseFire", Commands.runOnce(() -> autoFeedingSuppressed = false));

    // CeaseFire: immediately kill all shooting — launcher idles, hood stows, feeding stops.
    // Use this event marker when the robot needs to stop shooting and duck under the trench
    // mid-auto (e.g., loop autos that cross the bump after firing).
    // Automatically re-arms when the robot reaches the neutral zone.
    if (shootingCoordinator != null && hood != null) {
      NamedCommands.registerCommand(
          "CeaseFireAndStowHood",
          Commands.runOnce(() -> shootingCoordinator.requestCeaseFire())
              .andThen(Commands.waitUntil(hood::atTarget)));
    }

    // RetractIntake: retract intake and stop rollers
    if (intake != null) {
      NamedCommands.registerCommand(
          "RetractIntake",
          Commands.runOnce(
              () -> {
                intake.retract();
                intake.stopRollers();
              },
              intake));
    }

    // Climber is no longer a climbing mechanism — stub old auto climb markers as no-ops so any
    // existing .auto files that still reference them don't fail to parse.
    NamedCommands.registerCommand("ClimberExtend", Commands.none());
    NamedCommands.registerCommand("ClimberClimb", Commands.none());
    NamedCommands.registerCommand("AutoClimb", Commands.none());
  }

  // Track last alliance to detect changes
  private edu.wpi.first.wpilibj.DriverStation.Alliance lastAlliance = null;

  /**
   * Updates the robot's simulation pose based on the currently selected auto. Call this from
   * disabledPeriodic() to automatically position the robot at the auto's starting location when the
   * auto selection changes or when the alliance changes.
   */
  public void updateSimulationPoseFromAuto() {
    // Get the currently selected auto name and alliance
    String selectedDisplayName = autoChooser.getSendableChooser().getSelected();
    edu.wpi.first.wpilibj.DriverStation.Alliance currentAlliance =
        edu.wpi.first.wpilibj.DriverStation.getAlliance()
            .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);

    // Update if either the auto selection or alliance changed
    boolean autoChanged =
        selectedDisplayName != null && !selectedDisplayName.equals(lastSelectedAutoName);
    boolean allianceChanged = currentAlliance != lastAlliance;

    if (autoChanged || allianceChanged) {
      lastSelectedAutoName = selectedDisplayName;
      lastAlliance = currentAlliance;

      // Skip if no auto selected or "None" is selected
      if (selectedDisplayName == null
          || selectedDisplayName.isEmpty()
          || selectedDisplayName.equals("None")) {
        if (autoChanged) resetStrategySelections(null);
        return;
      }

      // Resolve display name → raw auto name for PathPlanner
      String rawAutoName = displayToAutoName.getOrDefault(selectedDisplayName, selectedDisplayName);

      // Rebuild strategy choosers with this auto's defaults
      if (autoChanged) {
        resetStrategySelections(rawAutoName);
      }

      try {
        // Get the starting pose from the PathPlanner auto
        // PathPlannerAuto.getStartingPose() returns the pose relative to blue alliance
        // origin
        com.pathplanner.lib.commands.PathPlannerAuto auto =
            new com.pathplanner.lib.commands.PathPlannerAuto(rawAutoName);
        edu.wpi.first.math.geometry.Pose2d startingPose = auto.getStartingPose();

        if (startingPose != null) {
          // Flip pose if on red alliance (same logic as AutoBuilder uses)
          if (currentAlliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
            startingPose = com.pathplanner.lib.util.FlippingUtil.flipFieldPose(startingPose);
          }
          drive.setPose(startingPose);
        }
      } catch (Exception e) {
        // Auto doesn't have a valid starting pose - silently ignore
        // This can happen for non-PathPlanner autos or invalid selections
      }
    }
  }

  /**
   * Rebuild the auto chooser when the "Competition Mode" toggle changes. Call this from
   * disabledPeriodic() so the dropdown updates while the robot is disabled before a match.
   */
  public void updateAutoChooserForMode() {
    boolean compMode = SmartDashboard.getBoolean("Competition Mode", true);
    if (compMode != lastCompetitionMode) {
      lastCompetitionMode = compMode;
      autoChooser = buildAutoChooserForMode(compMode);
    }
  }

  /**
   * Build the auto chooser for the given mode. In competition mode, all autos except those in the
   * Retired folder are included. In practice mode, all PathPlanner autos are included.
   */
  private LoggedDashboardChooser<Command> buildAutoChooserForMode(boolean competitionMode) {
    displayToAutoName = new HashMap<>();

    if (competitionMode) {
      SendableChooser<Command> sendable = new SendableChooser<>();
      sendable.setDefaultOption("None", Commands.none());
      autoFolderMap.entrySet().stream()
          .filter(e -> !"Retired".equals(e.getValue()))
          // .filter(e -> !e.getKey().toLowerCase().contains("climb"))
          .sorted(Map.Entry.comparingByKey())
          .forEachOrdered(
              entry -> {
                String rawName = entry.getKey();
                displayToAutoName.put(rawName, rawName);
                sendable.addOption(rawName, new PathPlannerAuto(rawName));
              });
      return new LoggedDashboardChooser<>("Auto Choices", sendable);
    } else {
      // Practice mode: include all autos (retired included and properly wrapped)
      SendableChooser<Command> sendable = new SendableChooser<>();
      sendable.setDefaultOption("None", Commands.none());
      autoFolderMap.entrySet().stream()
          // .filter(e -> !e.getKey().toLowerCase().contains("climb"))
          .sorted(Map.Entry.comparingByKey())
          .forEachOrdered(
              entry -> {
                String rawName = entry.getKey();
                displayToAutoName.put(rawName, rawName);
                sendable.addOption(rawName, new PathPlannerAuto(rawName));
              });

      LoggedDashboardChooser<Command> chooser =
          new LoggedDashboardChooser<>("Auto Choices", sendable);

      // Add SysId routines only on real robot (not in simulation)
      // if (Constants.currentMode != Constants.Mode.SIM) {
      // chooser.addOption(
      //     "[Util] Drive Wheel Radius Characterization",
      //     DriveCommands.wheelRadiusCharacterization(drive));
      // chooser.addOption(
      //     "[Util] Drive Simple FF Characterization",
      //     DriveCommands.feedforwardCharacterizationDrive(drive));
      // //   chooser.addOption(
      //       "[Util] Drive SysId (Quasistatic Forward)",
      //       drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      //   chooser.addOption(
      //       "[Util] Drive SysId (Quasistatic Reverse)",
      //       drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      //   chooser.addOption(
      //       "[Util] Drive SysId (Dynamic Forward)",
      //       drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      //   chooser.addOption(
      //       "[Util] Drive SysId (Dynamic Reverse)",
      //       drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

      //   if (launcher != null) {
      //     chooser.addOption(
      //         "[Util] Launcher Simple FF Characterization",
      //         LauncherCommands.feedforwardCharacterization(launcher));
      //   }
      //   if (motivator != null) {
      //     chooser.addOption(
      //         "[Util] Motivator Simple FF Characterization",
      //         MotivatorCommands.feedforwardCharacterization(motivator));
      //   }

      //   if (spindexer != null) {
      //     chooser.addOption(
      //         "[Util] Spindexer Simple FF Characterization",
      //         SpindexerCommands.spindexerCharacterization(spindexer));
      //   }
      // }
      return chooser;
    }
  }

  /**
   * Scan PathPlanner auto files and return a map of auto name → folder name. Folders are
   * organizational (by start location: TrenchLeft, TrenchRight, Depot, Outpost, Retired). Strategy
   * defaults are determined per-auto, not per-folder. Autos without a folder map to "".
   */
  private static Map<String, String> loadAutoFolderMap() {
    Map<String, String> map = new HashMap<>();
    File autosDir =
        new File(edu.wpi.first.wpilibj.Filesystem.getDeployDirectory(), "pathplanner/autos");
    File[] autoFiles = autosDir.listFiles((dir, name) -> name.endsWith(".auto"));
    if (autoFiles == null) return map;

    for (File file : autoFiles) {
      try {
        String content = Files.readString(file.toPath());
        String autoName = file.getName().replace(".auto", "");
        // Extract folder field value from JSON content
        String folder = extractJsonStringField(content, "folder");
        map.put(autoName, folder);
      } catch (IOException e) {
        // Skip unreadable files
      }
    }
    return map;
  }

  /** Extract a string field value from simple JSON content. Returns "" if not found. */
  private static String extractJsonStringField(String json, String field) {
    // Match "field": "value" or "field":"value"
    int idx = -1;
    // Simple search — avoid regex dependency for a trivial parse
    for (String variant :
        new String[] {"\"" + field + "\": \"", "\"" + field + "\":\"", "\"" + field + "\" : \""}) {
      idx = json.indexOf(variant);
      if (idx >= 0) {
        idx += variant.length();
        break;
      }
    }
    if (idx < 0) return "";
    int end = json.indexOf('"', idx);
    if (end < 0) return "";
    return json.substring(idx, end);
  }

  /**
   * Initialize the FuelSim physics simulation for simulation mode. Registers the robot, spawns
   * starting fuel, and initializes the turret visualizer.
   */
  private void initializeFuelSim() {
    FuelSim fuelSim = FuelSim.getInstance();

    // Robot dimensions from config (bumper-to-bumper)
    RobotConfig robotConfig = Constants.getRobotConfig();
    double robotWidth = robotConfig.getBumperWidth();
    double robotLength = robotConfig.getBumperLength();
    double bumperHeight = 0.2; // meters

    // Register the robot with the simulation
    fuelSim.registerRobot(
        robotWidth, robotLength, bumperHeight, drive::getPose, drive::getFieldRelativeSpeeds);
    // fuelSim.enableAirResistance();
    if (shootingCoordinator != null && shootingCoordinator.getVisualizer() != null) {
      fuelSim.setRobotFuelStoredSupplier(() -> shootingCoordinator.getVisualizer().getFuelCount());
    }

    // Register intake with fuel simulation for pickup collision detection
    // Intake zone: 10 inches (0.254m) from front frame, 30 inches (0.762m) wide
    // centered
    if (intake != null && shootingCoordinator != null) {
      fuelSim.registerIntake(
          0.35,
          0.604, // xMin, xMax (front bumper edge to 10" past it)
          -0.381,
          0.381, // yMin, yMax (30" wide, centered)
          () ->
              intake.isDeployed()
                  && shootingCoordinator.getVisualizer() != null
                  && shootingCoordinator.getVisualizer().canIntake(),
          () -> {
            if (shootingCoordinator.getVisualizer() != null) {
              shootingCoordinator.getVisualizer().queueFuel();
            }
          });
    }

    // Spawn the full match fuel layout
    fuelSim.spawnStartingFuel();

    // Default sim pose: origin, facing forward (+X) for easy visual debugging
    drive.setPose(
        new edu.wpi.first.math.geometry.Pose2d(
            0.0, 0.0, new edu.wpi.first.math.geometry.Rotation2d()));

    // Start the simulation
    fuelSim.start();
  }

  /** Update the fuel simulation. Call this from robotPeriodic(). */
  public void updateFuelSim() {
    if (Constants.currentMode == Constants.Mode.SIM
        || Constants.currentMode == Constants.Mode.PIT) {
      FuelSim.getInstance().updateSim();
    }
  }
}
