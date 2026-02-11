// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootingCommands;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOVirtual;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeIOSparkMaxRollerOnly;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherIO;
import frc.robot.subsystems.launcher.LauncherIOSim;
import frc.robot.subsystems.launcher.LauncherIOSparkFlex;
import frc.robot.subsystems.motivator.Motivator;
import frc.robot.subsystems.motivator.MotivatorIO;
import frc.robot.subsystems.motivator.MotivatorIOSim;
import frc.robot.subsystems.motivator.MotivatorIOSparkFlex;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.FuelSim;
import frc.robot.util.MatchPhaseTracker;
import frc.robot.util.RobotStatus;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Turret turret; // Only instantiated for SquareBot, null for MainBot
  private final Vision vision; // Only instantiated for SquareBot, null for MainBot
  private final Intake intake; // Only instantiated for SquareBot, null for MainBot
  private final Launcher launcher; // Only instantiated for TurretBot, null for others
  private final Hood hood; // Only instantiated for robots with hood hardware, null for others
  private final Motivator motivator; // Only instantiated for robots with motivator, null for others
  private OperatorInterface oi = new OperatorInterface() {};

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Simulation: track last selected auto for pose updates
  private String lastSelectedAutoName = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Instantiate turret subsystem
    switch (Constants.currentMode) {
      case REAL:
        if (Constants.currentRobot == Constants.RobotType.TURRETBOT) {
          // TurretBot bench test: turret motor not connected, use no-op IO
          turret = new Turret(new TurretIO() {});
        } else {
          // SquareBot and MainBot use TalonFX
          turret = new Turret(new TurretIO() {});
        }
        break;

      case SIM:
        turret = new Turret(new TurretIOSim());
        break;

      default:
        // Replay mode - disable turret IO
        turret = new Turret(new TurretIO() {});
        break;
    }

    // Instantiate intake only for SquareBot and MainBot
    if (Constants.currentRobot == Constants.RobotType.SQUAREBOT
        || Constants.currentRobot == Constants.RobotType.MAINBOT) {
      switch (Constants.currentMode) {
        case REAL:
          if (Constants.currentRobot == Constants.RobotType.SQUAREBOT) {
            // SquareBot: roller-only intake (no deploy mechanism)
            intake = new Intake(new IntakeIOSparkMaxRollerOnly());
          } else {
            // MainBot: full intake with deploy + rollers
            intake = new Intake(new IntakeIOSparkMax());
          }
          break;

        case SIM:
          // Sim SquareBot - instantiate intake simulation
          intake = new Intake(new IntakeIOSim());
          break;

        default:
          // Replay mode - disable intake IO
          intake = new Intake(new IntakeIO() {});
          break;
      }
    } else {
      // RectangleBot and TurretBot do not have an intake
      intake = null;
    }

    // Instantiate launcher subsystem (TurretBot only)
    if (Constants.getRobotConfig().hasLauncher()) {
      switch (Constants.currentMode) {
        case REAL:
          launcher = new Launcher(new LauncherIOSparkFlex());
          break;

        case SIM:
          launcher = new Launcher(new LauncherIOSim());
          break;

        default:
          // Replay mode - disable launcher IO
          launcher = new Launcher(new LauncherIO() {});
          break;
      }
    } else {
      launcher = null;
    }

    // Instantiate hood subsystem (robots with hood hardware)
    if (Constants.getRobotConfig().hasHood()) {
      switch (Constants.currentMode) {
        case REAL:
          // TODO: Create HoodIOSparkMax when hardware is ready
          hood = new Hood(new HoodIOSparkMax()); // Use sim for now

          SmartDashboard.putData(
              "Hood/setAngle",
              Commands.runOnce(
                  () -> {
                    hood.setAngle(SmartDashboard.getNumber("Tuning/Hood/Hood Angle", 0.0));
                  },
                  hood));
          break;

        case SIM:
          hood = new Hood(new HoodIOSim());
          break;

        default:
          // Replay mode - disable hood IO
          hood = new Hood(new HoodIO() {});
          break;
      }
    } else {
      hood = null;
    }

    // Instantiate motivator subsystem (robots with motivator hardware)
    if (Constants.getRobotConfig().hasMotivator()) {
      switch (Constants.currentMode) {
        case REAL:
          motivator = new Motivator(new MotivatorIOSparkFlex());
          break;

        case SIM:
          motivator = new Motivator(new MotivatorIOSim());
          break;

        default:
          // Replay mode - disable motivator IO
          motivator = new Motivator(new MotivatorIO() {});
          break;
      }
    } else {
      motivator = null;
    }

    // Instantiate drive and vision subsystems
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        if (Constants.currentRobot == Constants.RobotType.TURRETBOT) {
          // TurretBot: Virtual drive modules (no physical drivetrain)
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOVirtual(),
                  new ModuleIOVirtual(),
                  new ModuleIOVirtual(),
                  new ModuleIOVirtual(),
                  turret);
          vision = null;
        } else {
          // SquareBot and MainBot: Full swerve drive
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOVirtual(),
                  new ModuleIOVirtual(),
                  new ModuleIOVirtual(),
                  new ModuleIOVirtual(),
                  turret);
          // Vision for SquareBot and MainBot
          // Camera order: A (FrontLeft), B (FrontRight), C (BackLeft), D (BackRight)
          if (Constants.currentRobot == Constants.RobotType.SQUAREBOT) {
            vision =
                new Vision(
                    drive::addVisionMeasurement,
                    new VisionIOPhotonVision(
                        VisionConstants.frontLeftCam, VisionConstants.robotToFrontLeftCam),
                    new VisionIOPhotonVision(
                        VisionConstants.frontRightCam, VisionConstants.robotToFrontRightCam),
                    new VisionIOPhotonVision(
                        VisionConstants.backLeftCam, VisionConstants.robotToBackLeftCam),
                    new VisionIOPhotonVision(
                        VisionConstants.backRightCam, VisionConstants.robotToBackRightCam));
          } else if (Constants.currentRobot == Constants.RobotType.MAINBOT) {
            // MainBot uses corner-mounted cameras aimed diagonally outward + front center
            // for
            // intake
            vision = null;
            // new Vision(
            // drive::addVisionMeasurement,
            // new VisionIOPhotonVision(
            // VisionConstants.frontLeftCam, VisionConstants.mainBotToFrontLeftCam),
            // new VisionIOPhotonVision(
            // VisionConstants.frontRightCam, VisionConstants.mainBotToFrontRightCam),
            // new VisionIOPhotonVision(
            // VisionConstants.backLeftCam, VisionConstants.mainBotToBackLeftCam),
            // new VisionIOPhotonVision(
            // VisionConstants.backRightCam, VisionConstants.mainBotToBackRightCam),
            // new VisionIOPhotonVision(
            // VisionConstants.objectDetectionFrontLeftCam,
            // VisionConstants.mainBotToObjectDetectionFrontLeftCam));
          } else {
            vision = null;
          }
        }
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        if (Constants.currentRobot == Constants.RobotType.TURRETBOT) {
          // TurretBot sim: Virtual modules for joystick-driven pose updates
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOVirtual(),
                  new ModuleIOVirtual(),
                  new ModuleIOVirtual(),
                  new ModuleIOVirtual(),
                  turret);
          vision = null;
        } else {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  turret);
          // Vision for SquareBot and MainBot in simulation
          // Camera order: A (FrontLeft), B (FrontRight), C (BackLeft), D (BackRight)
          // This order determines PhotonVision sim ports: A=1182, B=1183, C=1184, D=1185
          // Each camera has both current and recommended transforms for toggle comparison
          if (Constants.currentRobot == Constants.RobotType.SQUAREBOT) {
            vision =
                new Vision(
                    drive::addVisionMeasurement,
                    new VisionIOPhotonVisionSim(
                        VisionConstants.frontLeftCam,
                        VisionConstants.robotToFrontLeftCam,
                        RobotStatus::getRobotPose),
                    new VisionIOPhotonVisionSim(
                        VisionConstants.frontRightCam,
                        VisionConstants.robotToFrontRightCam,
                        RobotStatus::getRobotPose),
                    new VisionIOPhotonVisionSim(
                        VisionConstants.backLeftCam,
                        VisionConstants.robotToBackLeftCam,
                        RobotStatus::getRobotPose),
                    new VisionIOPhotonVisionSim(
                        VisionConstants.backRightCam,
                        VisionConstants.robotToBackRightCam,
                        RobotStatus::getRobotPose));
          } else if (Constants.currentRobot == Constants.RobotType.MAINBOT) {
            // MainBot uses corner-mounted cameras aimed diagonally outward + front center
            // for
            // intake
            vision =
                new Vision(
                    drive::addVisionMeasurement,
                    new VisionIOPhotonVisionSim(
                        VisionConstants.frontLeftCam,
                        VisionConstants.mainBotToFrontLeftCam,
                        RobotStatus::getRobotPose),
                    new VisionIOPhotonVisionSim(
                        VisionConstants.frontRightCam,
                        VisionConstants.mainBotToFrontRightCam,
                        RobotStatus::getRobotPose),
                    new VisionIOPhotonVisionSim(
                        VisionConstants.backLeftCam,
                        VisionConstants.mainBotToBackLeftCam,
                        RobotStatus::getRobotPose),
                    new VisionIOPhotonVisionSim(
                        VisionConstants.backRightCam,
                        VisionConstants.mainBotToBackRightCam,
                        RobotStatus::getRobotPose),
                    new VisionIOPhotonVisionSim(
                        VisionConstants.objectDetectionFrontLeftCam,
                        VisionConstants.mainBotToObjectDetectionFrontLeftCam,
                        RobotStatus::getRobotPose));
          } else {
            vision = null;
          }
        }
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                turret);
        // Vision for SquareBot and MainBot (replay mode)
        if (Constants.currentRobot == Constants.RobotType.SQUAREBOT) {
          vision =
              new Vision(
                  (pose, time, stdDevs) -> {},
                  new VisionIO() {},
                  new VisionIO() {},
                  new VisionIO() {},
                  new VisionIO() {});
        } else if (Constants.currentRobot == Constants.RobotType.MAINBOT) {
          // MainBot has 5 cameras
          vision =
              new Vision(
                  (pose, time, stdDevs) -> {},
                  new VisionIO() {},
                  new VisionIO() {},
                  new VisionIO() {},
                  new VisionIO() {},
                  new VisionIO() {});
        } else {
          vision = null;
        }
        break;
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

    // Initialize FuelSim for simulation mode
    if (Constants.currentMode == Constants.Mode.SIM) {
      initializeFuelSim();
    }

    // Initialize turret visualizer (works on both real robot and sim for debugging)
    if (turret != null && drive != null) {
      turret.initializeVisualizer(drive::getPose, () -> drive.getChassisSpeeds());
    }

    // Configure auto-shoot (turret fires automatically during autonomous when
    // conditions are met)
    if (turret != null && launcher != null) {
      turret.configureAutoShoot(launcher::atSetpoint, launcher::notifyBallFired);
    }

    // Register NamedCommands for PathPlanner autos (must be BEFORE
    // buildAutoChooser)
    registerNamedCommands();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Add SysId routines only on real robot (not in simulation)
    // In sim, we only want PathPlanner autos so we can set starting poses
    if (Constants.currentMode != Constants.Mode.SIM) {
      autoChooser.addOption(
          "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
      autoChooser.addOption(
          "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Forward)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Reverse)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooser.addOption(
          "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

      // Launcher SysId routines (TurretBot only) - forward only since launcher never
      // runs reverse
      if (launcher != null) {
        autoChooser.addOption("Launcher SysId (Quasistatic)", launcher.launcherSysIdQuasistatic());
        autoChooser.addOption("Launcher SysId (Dynamic)", launcher.launcherSysIdDynamic());
      }
    }

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
        oi, drive, vision, intake, turret, launcher, motivator, hood);
  }

  // Starting fuel count for autonomous (always 8)
  private static final int AUTO_START_FUEL_COUNT = 8;

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

    // Wrap with auto-shoot setup and teardown
    return Commands.sequence(
            // Setup: load fuel, clear field, spin up, enable auto-shoot
            Commands.runOnce(
                () -> {
                  // Set fuel count
                  if (turret != null && turret.getVisualizer() != null) {
                    turret.getVisualizer().setFuelCount(AUTO_START_FUEL_COUNT);
                  }
                  // Clear field fuel and respawn starting fuel for auto
                  if (Constants.currentMode == Constants.Mode.SIM) {
                    frc.robot.util.FuelSim.getInstance().clearFuel();
                    frc.robot.util.FuelSim.getInstance().spawnStartingFuel();
                  }
                  // Spin up launcher and motivator
                  if (launcher != null) {
                    launcher.setVelocity(1700.0);
                  }
                  if (motivator != null) {
                    motivator.setVelocities(1000.0, 1000.0); // motivators + prefeed
                  }
                  // Enable auto-shoot
                  if (turret != null) {
                    turret.enableAutoShoot();
                  }
                }),
            // Run the selected auto path (asProxy avoids "command already composed" on
            // re-run)
            selectedAuto.asProxy())
        .finallyDo(
            () -> {
              // Teardown: disable auto-shoot, stop motors
              if (turret != null) {
                turret.disableAutoShoot();
              }
              if (launcher != null) {
                launcher.stop();
              }
              if (motivator != null) {
                motivator.stop();
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
   * Get the launcher subsystem if it exists (TurretBot only).
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
              if (motivator != null) motivator.setMotivatorsVelocity(1000.0);
              if (turret != null) turret.enableAutoShoot();
            }));

    // Disable auto-shoot: stops auto-shoot and motors
    NamedCommands.registerCommand(
        "disableAutoShoot",
        Commands.runOnce(
            () -> {
              if (turret != null) turret.disableAutoShoot();
              if (launcher != null) launcher.stop();
              if (motivator != null) motivator.stop();
            }));

    // Set fuel count commands for testing
    NamedCommands.registerCommand(
        "setFuel40",
        Commands.runOnce(
            () -> {
              if (turret != null && turret.getVisualizer() != null) {
                turret.getVisualizer().setFuelCount(40);
              }
            }));

    NamedCommands.registerCommand(
        "setFuel25",
        Commands.runOnce(
            () -> {
              if (turret != null && turret.getVisualizer() != null) {
                turret.getVisualizer().setFuelCount(25);
              }
            }));

    // Wait until all fuel has been fired (with timeout for jams)
    // Use in autos that should empty their hopper before driving
    NamedCommands.registerCommand(
        "waitUntilFuelEmpty",
        Commands.waitUntil(
                () -> {
                  if (turret == null || turret.getVisualizer() == null) return true;
                  return turret.getVisualizer().getFuelCount() <= 0;
                })
            .withTimeout(5.0) // 5 second timeout in case of jams
            .withName("WaitUntilFuelEmpty"));

    // Reset simulation commands
    if (turret != null) {
      NamedCommands.registerCommand("resetSim", ShootingCommands.resetSimulationCommand(turret));
      NamedCommands.registerCommand(
          "resetStartingField", ShootingCommands.resetStartingFieldCommand(turret));
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
              if (motivator != null) motivator.setMotivatorsVelocity(1000.0);
              if (turret != null) turret.enableAutoShoot();
            }));
  }

  // Track last alliance to detect changes
  private edu.wpi.first.wpilibj.DriverStation.Alliance lastAlliance = null;

  /**
   * Updates the robot's simulation pose based on the currently selected auto. Call this from
   * disabledPeriodic() to automatically position the robot at the auto's starting location when the
   * auto selection changes or when the alliance changes.
   */
  public void updateSimulationPoseFromAuto() {
    if (Constants.currentMode != Constants.Mode.SIM) {
      return;
    }

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
        robotWidth, robotLength, bumperHeight, drive::getPose, () -> drive.getChassisSpeeds());

    // Register intake with fuel simulation for pickup collision detection
    // Intake zone: 10 inches (0.254m) from front frame, 30 inches (0.762m) wide
    // centered
    if (intake != null && turret != null) {
      fuelSim.registerIntake(
          0.35,
          0.604, // xMin, xMax (front bumper edge to 10" past it)
          -0.381,
          0.381, // yMin, yMax (30" wide, centered)
          () ->
              intake.isDeployed()
                  && turret.getVisualizer() != null
                  && turret.getVisualizer().canIntake(),
          () -> {
            if (turret.getVisualizer() != null) {
              turret.getVisualizer().queueFuel();
            }
          });
    }

    // Spawn the full match fuel layout
    fuelSim.spawnStartingFuel();

    // Start the simulation
    fuelSim.start();
  }

  /** Update the fuel simulation. Call this from robotPeriodic(). */
  public void updateFuelSim() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      FuelSim.getInstance().updateSim();
    }
  }

  /** Update the match phase tracker. Call this from robotPeriodic(). */
  public void updateMatchPhaseTracker() {
    MatchPhaseTracker.getInstance().periodic();
  }
}
