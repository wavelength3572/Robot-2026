// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.ModuleIOVirtual;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherIO;
import frc.robot.subsystems.launcher.LauncherIOSim;
import frc.robot.subsystems.launcher.LauncherIOSparkFlex;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOSparkMax;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.FuelSim;
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
          // TurretBot uses SparkMax + NEO 550
          turret = new Turret(new TurretIOSparkMax());
        } else {
          // SquareBot and MainBot use TalonFX
          turret = new Turret(new TurretIOTalonFX());
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

    // Instantiate intake only for MainBot (not present on MiniBot)
    if (Constants.currentRobot == Constants.RobotType.SQUAREBOT
        || Constants.currentRobot == Constants.RobotType.MAINBOT) {
      switch (Constants.currentMode) {
        case REAL:
          // Real SquareBot - instantiate intake hardware
          intake = new Intake(new IntakeIOSparkMax());
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
                  new GyroIOPigeon2(),
                  new ModuleIOSpark(0),
                  new ModuleIOSpark(1),
                  new ModuleIOSpark(2),
                  new ModuleIOSpark(3),
                  turret);
          // Vision only for SquareBot
          // Camera order: A (FrontLeft), B (FrontRight)
          // Cameras C (BackLeft) and D (BackRight) temporarily removed
          if (Constants.currentRobot == Constants.RobotType.SQUAREBOT) {
            vision =
                new Vision(
                    drive::addVisionMeasurement,
                    new VisionIOPhotonVision(
                        VisionConstants.frontLeftCam, VisionConstants.robotToFrontLeftCam),
                    new VisionIOPhotonVision(
                        VisionConstants.frontRightCam, VisionConstants.robotToFrontRightCam));
            // new VisionIOPhotonVision(
            //     VisionConstants.backLeftCam, VisionConstants.robotToBackLeftCam),
            // new VisionIOPhotonVision(
            //     VisionConstants.backRightCam, VisionConstants.robotToBackRightCam));
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
          // Vision only for SquareBot
          // Camera order: A (FrontLeft), B (FrontRight)
          // Cameras C (BackLeft) and D (BackRight) temporarily removed
          // This order determines PhotonVision sim ports: A=1182, B=1183
          // Each camera has both current and recommended transforms for toggle comparison
          if (Constants.currentRobot == Constants.RobotType.SQUAREBOT) {
            vision =
                new Vision(
                    drive::addVisionMeasurement,
                    new VisionIOPhotonVisionSim(
                        VisionConstants.frontLeftCam,
                        VisionConstants.robotToFrontLeftCam,
                        VisionConstants.recommendedFrontLeftCam,
                        RobotStatus::getRobotPose),
                    new VisionIOPhotonVisionSim(
                        VisionConstants.frontRightCam,
                        VisionConstants.robotToFrontRightCam,
                        VisionConstants.recommendedFrontRightCam,
                        RobotStatus::getRobotPose));
            // new VisionIOPhotonVisionSim(
            //     VisionConstants.backLeftCam,
            //     VisionConstants.robotToBackLeftCam,
            //     VisionConstants.recommendedBackLeftCam,
            //     RobotStatus::getRobotPose),
            // new VisionIOPhotonVisionSim(
            //     VisionConstants.backRightCam,
            //     VisionConstants.robotToBackRightCam,
            //     VisionConstants.recommendedBackRightCam,
            //     RobotStatus::getRobotPose));
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
        // Vision only for MainBot (replay mode)
        if (Constants.currentRobot == Constants.RobotType.SQUAREBOT) {
          vision =
              new Vision(
                  (pose, time, stdDevs) -> {},
                  new VisionIO() {},
                  new VisionIO() {},
                  new VisionIO() {},
                  new VisionIO() {});
        } else {
          vision = null;
        }
        break;
    }

    // Initialize RobotStatus with subsystem references (vision may be null for RectangleBot)
    RobotStatus.initialize(drive, vision);

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

      // Launcher SysId routines (TurretBot only)
      if (launcher != null) {
        autoChooser.addOption(
            "Launcher SysId (Quasistatic Forward)",
            launcher.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Launcher SysId (Quasistatic Reverse)",
            launcher.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
            "Launcher SysId (Dynamic Forward)",
            launcher.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Launcher SysId (Dynamic Reverse)",
            launcher.sysIdDynamic(SysIdRoutine.Direction.kReverse));
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
    ButtonsAndDashboardBindings.configureBindings(oi, drive, vision, intake, turret, launcher);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
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
        // PathPlannerAuto.getStartingPose() returns the pose relative to blue alliance origin
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

    // Robot dimensions (adjust for your robot)
    double robotWidth = 0.7; // meters (with bumpers)
    double robotLength = 0.7; // meters (with bumpers)
    double bumperHeight = 0.2; // meters

    // Register the robot with the simulation
    fuelSim.registerRobot(
        robotWidth, robotLength, bumperHeight, drive::getPose, () -> drive.getChassisSpeeds());

    // Spawn test fuel (smaller set for testing)
    fuelSim.spawnTestFuel();

    // Start the simulation
    fuelSim.start();

    // Initialize turret visualizer if turret exists
    if (turret != null) {
      turret.initializeVisualizer(drive::getPose, () -> drive.getChassisSpeeds());
    }
  }

  /** Update the fuel simulation. Call this from robotPeriodic(). */
  public void updateFuelSim() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      FuelSim.getInstance().updateSim();
    }
  }
}
