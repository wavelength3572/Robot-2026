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
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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
  private final Turret turret; // Only instantiated for MainBot, null for MiniBot
  private final Vision vision; // Only instantiated for MainBot, null for MiniBot
  private OperatorInterface oi = new OperatorInterface() {};

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Instantiate turret only for MainBot (not present on MiniBot)
    if (Constants.currentRobot == Constants.RobotType.MAINBOT) {
      switch (Constants.currentMode) {
        case REAL:
          // Real MainBot - instantiate turret hardware
          turret = new Turret(new TurretIOSim()); // TODO: Replace with real hardware IO when ready
          break;

        case SIM:
          // Sim MainBot - instantiate turret simulation
          turret = new Turret(new TurretIOSim());
          break;

        default:
          // Replay mode - disable turret IO
          turret = new Turret(new TurretIO() {});
          break;
      }
    } else {
      // MiniBot does not have a turret
      turret = null;
    }

    // Instantiate drive and vision subsystems
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                turret);
        // Vision only for MainBot
        if (Constants.currentRobot == Constants.RobotType.MAINBOT) {
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVision(
                      VisionConstants.frontRightCam, VisionConstants.robotToFrontRightCam),
                  new VisionIOPhotonVision(
                      VisionConstants.backRightCam, VisionConstants.robotToBackRightCam),
                  new VisionIOPhotonVision(
                      VisionConstants.frontLeftCam, VisionConstants.robotToFrontLeftCam),
                  new VisionIOPhotonVision(
                      VisionConstants.elevatorBackCam, VisionConstants.robotToElevatorBackCam));
        } else {
          vision = null;
        }
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                turret);
        // Vision only for MainBot
        if (Constants.currentRobot == Constants.RobotType.MAINBOT) {
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVisionSim(
                      VisionConstants.frontRightCam,
                      VisionConstants.robotToFrontRightCam,
                      RobotStatus::getRobotPose),
                  new VisionIOPhotonVisionSim(
                      VisionConstants.backRightCam,
                      VisionConstants.robotToBackRightCam,
                      RobotStatus::getRobotPose),
                  new VisionIOPhotonVisionSim(
                      VisionConstants.frontLeftCam,
                      VisionConstants.robotToFrontLeftCam,
                      RobotStatus::getRobotPose),
                  new VisionIOPhotonVisionSim(
                      VisionConstants.elevatorBackCam,
                      VisionConstants.robotToElevatorBackCam,
                      RobotStatus::getRobotPose));
        } else {
          vision = null;
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
        if (Constants.currentRobot == Constants.RobotType.MAINBOT) {
          vision =
              new Vision(
                  (pose, time, stdDevs) -> {},
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

    // Initialize RobotStatus with subsystem references (vision may be null for MiniBot)
    RobotStatus.initialize(drive, vision);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
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
    ButtonsAndDashboardBindings.configureBindings(oi, drive, vision);
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
   * Get the turret subsystem if it exists (MainBot only).
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
   * Get the vision subsystem if it exists (MainBot only).
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
}
