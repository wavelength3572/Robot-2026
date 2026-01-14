package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class ButtonsAndDashboardBindings {

  private static OperatorInterface oi;
  private static Drive drive;
  private static Vision vision;

  public ButtonsAndDashboardBindings() {}

  public static void configureTestModeButtonBindings(
      OperatorInterface operatorInterface, Drive drive) {
    oi = operatorInterface;
    ButtonsAndDashboardBindings.drive = drive;
  }

  public static void configureBindings(
      OperatorInterface operatorInterface, Drive drive, Vision vision) {
    ButtonsAndDashboardBindings.oi = operatorInterface;
    ButtonsAndDashboardBindings.drive = drive;
    ButtonsAndDashboardBindings.vision = vision;

    configureDriverButtonBindings();
    configureOperatorButtonBindings();
    configureDashboardBindings();
  }

  // Legacy method without vision parameter
  public static void configureBindings(OperatorInterface operatorInterface, Drive drive) {
    configureBindings(operatorInterface, drive, null);
  }

  /****************************** */
  /*** DASHBOARD BINDINGS ****** */
  /****************************** */

  private static void configureDashboardBindings() {
    // Dashboard bindings can be added here as needed
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
  }

  /****************************** */
  /*** BUTTON BOX BINDINGS ****** */
  /****************************** */

  private static void configureOperatorButtonBindings() {}
}
