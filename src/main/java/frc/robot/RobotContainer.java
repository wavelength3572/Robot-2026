// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.LauncherCommands;
import frc.robot.commands.MotivatorCommands;
import frc.robot.commands.ShootingCommands;
import frc.robot.commands.SpindexerCommands;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
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
import frc.robot.subsystems.vision.VisionIODoubleVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.FuelSim;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.RobotStatus;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
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
  private final ShootingCoordinator shootingCoordinator;
  private final IndicatorLight leds;
  private OperatorInterface oi = new OperatorInterface() {};

  private LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<String> allianceWinChooser;
  private boolean lastCompetitionMode = true;

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
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIODoubleVision(
                        VisionConstants.leftRearCam, VisionConstants.mainBotToLeftRearCam),
                    new VisionIODoubleVision(
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
          Commands.run(() -> hood.setHoodAngle(hood.getMinAngle()), hood).withName("HoodStow"));
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
      shootingCoordinator = new ShootingCoordinator(turret, hood, launcher, motivator);
      shootingCoordinator.initialize(drive::getPose, drive::getFieldRelativeSpeeds);
      // Gate launching: suppress if spindexer is suppressed OR hub shift is inactive
      // (unless "Ignore Hub State" dashboard toggle is on)
      shootingCoordinator.setFeedingSuppressedSupplier(
          () -> {
            // Spindexer operator suppression (button box)
            if (spindexer != null && spindexer.isFeedingSuppressed()) return true;
            // Hub shift gating (skip if override is on)
            if (!SmartDashboard.getBoolean("Match/Ignore Hub State", false)) {
              return !HubShiftUtil.getShiftedShiftInfo().active();
            }
            return false;
          });
    } else {
      shootingCoordinator = null;
    }

    // LED subsystem: 42 LEDs on PWM 0, wired in parallel to two physical strips
    leds = new IndicatorLight();

    // Initialize FuelSim for simulation mode (after coordinator so intake can be
    // registered)
    if (Constants.currentMode == Constants.Mode.SIM) {
      initializeFuelSim();
    }

    // Register NamedCommands for PathPlanner autos (must be BEFORE
    // buildAutoChooser)
    registerNamedCommands();

    // Dashboard toggle: ignore hub shift state (bypass launch gating)
    SmartDashboard.putBoolean("Match/Ignore Hub State", false);

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

    // Dashboard toggle: defaults to competition mode (safe for matches)
    SmartDashboard.putBoolean("Competition Mode", false);
    autoChooser = buildAutoChooserForMode(true);

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
        shootingCoordinator);
  }

  // Starting fuel count for autonomous (always 8)
  private static final int AUTO_START_FUEL_COUNT = 8;

  // Comp autos get the full shooting wrap (fuel, auto-shoot, intake, launcher
  // spin-up).
  // Test autos run bare — no setup/teardown, just the path.
  // Built dynamically at startup by scanning PathPlanner auto files for "folder":
  // "Comp".
  private final Set<String> compAutos = loadCompAutoNames();

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class. Wraps the selected
   * auto with setup (fuel, launcher spin-up, auto-shoot enable) and teardown (disable auto-shoot,
   * stop motors).
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command selectedAuto = autoChooser.get();
    if (selectedAuto == null) return null;

    // Only wrap comp autos with shooting setup/teardown; test autos run bare
    String selectedName = autoChooser.getSendableChooser().getSelected();
    if (selectedName == null || !compAutos.contains(selectedName)) {
      return selectedAuto;
    }

    // Reset odometry to the auto's starting pose BEFORE any shooting commands run.
    // Without this, the initial SmartLaunch uses a stale pose (e.g. heading=0 from
    // gyro reset) instead of the auto's actual starting heading, causing the turret
    // to aim in the wrong direction.
    edu.wpi.first.math.geometry.Pose2d autoStartingPose = null;
    try {
      PathPlannerAuto ppAuto = new PathPlannerAuto(selectedName);
      autoStartingPose = ppAuto.getStartingPose();
      if (autoStartingPose != null) {
        edu.wpi.first.wpilibj.DriverStation.Alliance alliance =
            edu.wpi.first.wpilibj.DriverStation.getAlliance()
                .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);
        if (alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
          autoStartingPose = com.pathplanner.lib.util.FlippingUtil.flipFieldPose(autoStartingPose);
        }
      }
    } catch (Exception e) {
      // Auto doesn't have a valid starting pose - will skip reset
    }
    final edu.wpi.first.math.geometry.Pose2d startingPose = autoStartingPose;

    // Wrap with shooting setup and teardown using smartLaunchCommand
    return Commands.sequence(
            // 0. ODOM RESET: set robot pose to auto starting position so turret aiming
            //    in the initial SmartLaunch uses the correct heading.
            Commands.runOnce(
                () -> {
                  if (startingPose != null) {
                    drive.setPose(startingPose);
                  }
                }),
            // 1. SETUP: load fuel, sim reset, deploy intake
            Commands.runOnce(
                () -> {
                  // Set fuel count (sim visualizer)
                  if (shootingCoordinator != null && shootingCoordinator.getVisualizer() != null) {
                    shootingCoordinator.getVisualizer().setFuelCount(AUTO_START_FUEL_COUNT);
                  }
                  // Clear/respawn field fuel (sim)
                  if (Constants.currentMode == Constants.Mode.SIM) {
                    frc.robot.util.FuelSim.getInstance().clearFuel();
                    frc.robot.util.FuelSim.getInstance().spawnStartingFuel();
                  }
                  // Deploy and run intake
                  if (intake != null) {
                    intake.deploy();
                    intake.runIntake();
                  }
                }),
            // 2. WAIT FOR INTAKE DEPLOY: block until intake reaches deployed position
            //    before any shooting occurs — shooting with the intake stowed will rip it off.
            intake != null
                ? Commands.waitUntil(() -> intake.isDeployed()).withTimeout(0.25)
                : Commands.none(),
            // 3. INITIAL SHOT: fire preloaded balls with proper hood angle.
            //    smartLaunch handles turret+hood+launcher+motivator+spindexer+readiness.
            //    5s timeout controls duration (no fuel sensor yet).
            //    Wrapped in asProxy() so subsystem requirements don't leak into the
            //    outer sequence — otherwise the group holds hood/launcher/etc. for its
            //    entire lifetime, blocking default commands between steps.
            ShootingCommands.smartLaunchCommand(
                    launcher, shootingCoordinator, motivator, turret, hood, spindexer)
                .withTimeout(5.0)
                .asProxy(),
            // 4. HOOD STOW: explicitly drive hood to min angle (13 deg) and wait.
            //    We command the hood directly rather than relying on the default command
            //    so the hood moves regardless of subsystem scheduling.
            //    Wrapped in asProxy() so the outer sequence holds zero subsystem
            //    requirements — preventing conflicts when step 5's PathPlanner auto
            //    claims hood/launcher/etc. for SmartLaunch event zones.
            hood != null
                ? Commands.run(() -> hood.setHoodAngle(hood.getMinAngle()), hood)
                    .until(() -> hood.atTarget())
                    .withTimeout(0.75)
                    .asProxy()
                : Commands.none(),
            // 5. PATH: run auto path.
            //    PathPlanner "SmartLaunch" event zones trigger full smartLaunchCommand
            //    at path-designer-chosen safe moments (turret+hood+firing).
            //    When zone ends, hood stows back to 13 via default command.
            selectedAuto.asProxy(),
            // 6. POST-PATH: fire all collected fuel with proper hood angle.
            //    smartLaunch positions turret+hood and fires via spindexer.
            //    10s timeout controls duration (no fuel sensor yet).
            //    Wrapped in asProxy() to isolate subsystem requirements.
            ShootingCommands.smartLaunchCommand(
                    launcher, shootingCoordinator, motivator, turret, hood, spindexer)
                .withTimeout(10.0)
                .asProxy())
        .finallyDo(
            () -> {
              // 7. TEARDOWN: stop everything
              if (launcher != null) {
                launcher.stop();
              }
              if (motivator != null) {
                motivator.stopMotivator();
              }
              if (intake != null) {
                intake.stopRollers();
              }
            });
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

  /** Register NamedCommands for PathPlanner autos. Must be called before buildAutoChooser. */
  private void registerNamedCommands() {
    // Enable auto-shoot: spins up launcher + motivator, enables auto-shoot on
    // turret
    NamedCommands.registerCommand(
        "enableAutoShoot",
        Commands.runOnce(
            () -> {
              if (launcher != null) launcher.setVelocity(1700.0);
              if (motivator != null) motivator.setMotivatorVelocity(1000.0);
              if (shootingCoordinator != null) shootingCoordinator.enableAutoShoot();
            }));

    // Disable auto-shoot: stops auto-shoot and motors
    NamedCommands.registerCommand(
        "disableAutoShoot",
        Commands.runOnce(
            () -> {
              if (shootingCoordinator != null) shootingCoordinator.disableAutoShoot();
              if (launcher != null) launcher.stop();
              if (motivator != null) motivator.stopMotivator();
            }));

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

    // StartLauncher: spin up launcher + motivator + enable auto-shoot
    NamedCommands.registerCommand(
        "StartLauncher",
        Commands.runOnce(
            () -> {
              if (launcher != null) launcher.setVelocity(1700.0);
              if (motivator != null) motivator.setMotivatorVelocity(1000.0);
              if (shootingCoordinator != null) shootingCoordinator.enableAutoShoot();
            }));

    // SmartLaunch: full smartLaunch command for PathPlanner event zones
    NamedCommands.registerCommand(
        "SmartLaunch",
        ShootingCommands.smartLaunchCommand(
            launcher, shootingCoordinator, motivator, turret, hood, spindexer));
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
    String selectedAutoName = autoChooser.getSendableChooser().getSelected();
    edu.wpi.first.wpilibj.DriverStation.Alliance currentAlliance =
        edu.wpi.first.wpilibj.DriverStation.getAlliance()
            .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);

    // Update if either the auto selection or alliance changed
    boolean autoChanged =
        selectedAutoName != null && !selectedAutoName.equals(lastSelectedAutoName);
    boolean allianceChanged = currentAlliance != lastAlliance;

    if (autoChanged || allianceChanged) {
      lastSelectedAutoName = selectedAutoName;
      lastAlliance = currentAlliance;

      // Skip if no auto selected or "None" is selected
      if (selectedAutoName == null
          || selectedAutoName.isEmpty()
          || selectedAutoName.equals("None")) {
        return;
      }

      try {
        // Get the starting pose from the PathPlanner auto
        // PathPlannerAuto.getStartingPose() returns the pose relative to blue alliance
        // origin
        com.pathplanner.lib.commands.PathPlannerAuto auto =
            new com.pathplanner.lib.commands.PathPlannerAuto(selectedAutoName);
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
   * Build the auto chooser for the given mode. In competition mode, only Comp-folder autos are
   * included. In practice mode, all PathPlanner autos and SysId routines are included.
   */
  private LoggedDashboardChooser<Command> buildAutoChooserForMode(boolean competitionMode) {
    if (competitionMode) {
      SendableChooser<Command> sendable = new SendableChooser<>();
      sendable.setDefaultOption("None", Commands.none());
      for (String autoName : compAutos) {
        sendable.addOption(autoName, new PathPlannerAuto(autoName));
      }
      return new LoggedDashboardChooser<>("Auto Choices", sendable);
    } else {
      LoggedDashboardChooser<Command> chooser =
          new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
      // Add SysId routines only on real robot (not in simulation)
      if (Constants.currentMode != Constants.Mode.SIM) {
        chooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(drive));
        chooser.addOption(
            "Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterizationDrive(drive));
        chooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        chooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        chooser.addOption(
            "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        chooser.addOption(
            "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        if (launcher != null) {
          chooser.addOption("Launcher SysId (Quasistatic)", launcher.launcherSysIdQuasistatic());
          chooser.addOption("Launcher SysId (Dynamic)", launcher.launcherSysIdDynamic());
          chooser.addOption(
              "Launcher Simple FF Characterization",
              LauncherCommands.feedforwardCharacterization(launcher));
        }
        if (motivator != null) {
          chooser.addOption(
              "Motivator Simple FF Characterization",
              MotivatorCommands.feedforwardCharacterization(motivator));
        }

        if (spindexer != null) {
          chooser.addOption(
              "Spindexer Simple FF Characterization",
              SpindexerCommands.feedforwardCharacterization(spindexer));
        }
      }
      return chooser;
    }
  }

  /**
   * Scan PathPlanner auto files and return the names of autos in the "Comp" folder. Any auto placed
   * in the Comp folder in PathPlanner will automatically get the shooting wrap (fuel loading,
   * auto-shoot, intake, launcher spin-up/teardown).
   */
  private static Set<String> loadCompAutoNames() {
    Set<String> names = new HashSet<>();
    File autosDir =
        new File(edu.wpi.first.wpilibj.Filesystem.getDeployDirectory(), "pathplanner/autos");
    File[] autoFiles = autosDir.listFiles((dir, name) -> name.endsWith(".auto"));
    if (autoFiles == null) return names;

    for (File file : autoFiles) {
      try {
        String content = Files.readString(file.toPath());
        if (content.contains("\"folder\": \"Comp\"") || content.contains("\"folder\":\"Comp\"")) {
          // Auto name = filename without .auto extension
          String autoName = file.getName().replace(".auto", "");
          names.add(autoName);
        }
      } catch (IOException e) {
        // Skip unreadable files
      }
    }
    return names;
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

    // Default sim pose: midway between the tower (climbing bars) and the hub,
    // facing the hub
    double midX =
        (FieldConstants.Tower.centerPoint.getX() + FieldConstants.Hub.nearFace.getX()) / 2.0;
    double midY = FieldConstants.fieldWidth / 2.0;
    drive.setPose(
        new edu.wpi.first.math.geometry.Pose2d(
            midX, midY, new edu.wpi.first.math.geometry.Rotation2d()));

    // Start the simulation
    fuelSim.start();
  }

  /** Update the fuel simulation. Call this from robotPeriodic(). */
  public void updateFuelSim() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      FuelSim.getInstance().updateSim();
    }
  }
}
