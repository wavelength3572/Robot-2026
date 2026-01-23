package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;

public class ButtonsAndDashboardBindings {

  private static OperatorInterface oi;
  private static Drive drive;
  private static Vision vision;
  private static Intake intake;
  private static Turret turret;

  public ButtonsAndDashboardBindings() {}

  public static void configureTestModeButtonBindings(
      OperatorInterface operatorInterface, Drive drive) {
    oi = operatorInterface;
    ButtonsAndDashboardBindings.drive = drive;
  }

  public static void configureBindings(
      OperatorInterface operatorInterface,
      Drive drive,
      Vision vision,
      Intake intake,
      Turret turret) {
    ButtonsAndDashboardBindings.oi = operatorInterface;
    ButtonsAndDashboardBindings.drive = drive;
    ButtonsAndDashboardBindings.vision = vision;
    ButtonsAndDashboardBindings.intake = intake;
    ButtonsAndDashboardBindings.turret = turret;

    configureDriverButtonBindings();
    configureOperatorButtonBindings();
    configureDashboardBindings();
  }

  // Legacy method without vision/intake/turret parameters
  public static void configureBindings(OperatorInterface operatorInterface, Drive drive) {
    configureBindings(operatorInterface, drive, null, null, null);
  }

  /****************************** */
  /*** DASHBOARD BINDINGS ****** */
  /****************************** */

  private static void configureDashboardBindings() {
    // Vision toggle on dashboard
    if (vision != null) {
      // Default vision OFF in simulation for performance
      if (Constants.currentMode == Constants.Mode.SIM) {
        vision.setVisionOff();
      }
      SmartDashboard.putBoolean("Vision/Enable", vision.isVisionOn());
      SmartDashboard.putData(
          "Vision/Toggle",
          Commands.runOnce(
                  () -> {
                    vision.toggleVision();
                    SmartDashboard.putBoolean("Vision/Enable", vision.isVisionOn());
                  })
              .ignoringDisable(true)
              .withName("Toggle Vision"));
    }

    // Intake controls on dashboard
    if (intake != null) {
      // Deploy/Retract buttons
      SmartDashboard.putData(
          "Intake/Deploy", Commands.runOnce(intake::deploy, intake).withName("Deploy Intake"));
      SmartDashboard.putData(
          "Intake/Retract", Commands.runOnce(intake::retract, intake).withName("Retract Intake"));

      // Roller on/off toggle
      SmartDashboard.putData(
          "Intake/Run", Commands.runOnce(intake::runIntake, intake).withName("Run Intake"));
      SmartDashboard.putData(
          "Intake/Stop", Commands.runOnce(intake::stopRollers, intake).withName("Stop Intake"));
    }

    // Turret controls on dashboard
    if (turret != null) {
      // Calculate shot to hub (enables trajectory visualization)
      SmartDashboard.putData(
          "Turret/CalculateShot",
          Commands.runOnce(
                  () -> {
                    boolean isBlue =
                        DriverStation.getAlliance()
                            .orElse(DriverStation.Alliance.Blue)
                            .equals(DriverStation.Alliance.Blue);
                    turret.calculateShotToHub(isBlue);
                  },
                  turret)
              .ignoringDisable(true)
              .withName("Calculate Shot"));

      // Launch fuel
      SmartDashboard.putData(
          "Turret/LaunchFuel",
          Commands.runOnce(turret::launchFuel, turret).withName("Launch Fuel"));
    }
  }

  /****************************** */
  /*** DRIVER BINDINGS ****** */
  /****************************** */

  private static void configureDriverButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, oi::getTranslateX, oi::getTranslateY, oi::getRotate));
    // Gyro Reset
    oi.getResetGyroButton()
        .onTrue(Commands.runOnce(drive::zeroGyroscope, drive).ignoringDisable(true));

    // Vision toggle on Button H
    if (vision != null) {
      oi.getButtonH()
          .onTrue(
              Commands.runOnce(
                      () -> {
                        vision.toggleVision();
                        SmartDashboard.putBoolean("Vision/Enable", vision.isVisionOn());
                      })
                  .ignoringDisable(true));
    }

    // Turret controls (only in simulation or if turret exists)
    if (turret != null) {
      // Button V: Calculate shot to hub (enables trajectory visualization)
      oi.getButtonV()
          .onTrue(
              Commands.runOnce(
                      () -> {
                        boolean isBlue =
                            DriverStation.getAlliance()
                                .orElse(DriverStation.Alliance.Blue)
                                .equals(DriverStation.Alliance.Blue);
                        turret.calculateShotToHub(isBlue);
                      },
                      turret)
                  .ignoringDisable(true));

      // Shoot button: Launch fuel while held
      oi.getShootButton().whileTrue(turret.repeatedlyLaunchFuelCommand());
    }
  }

  /****************************** */
  /*** BUTTON BOX BINDINGS ****** */
  /****************************** */

  private static void configureOperatorButtonBindings() {}
}
