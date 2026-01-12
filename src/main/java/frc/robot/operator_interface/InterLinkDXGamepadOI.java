package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class InterLinkDXGamepadOI implements OperatorInterface {
  private final CommandGenericHID interLinkJoystick;
  private final Trigger[] interLinkJoystickJoystickButtons;

  private final CommandGenericHID gamePad;
  private final Trigger[] gamepadButtons;

  public InterLinkDXGamepadOI(int interlinkPort, int gamepadPort) {
    interLinkJoystick = new CommandGenericHID(interlinkPort);
    gamePad = new CommandGenericHID(gamepadPort);

    // buttons use 1-based indexing such that the index matches the button number;
    // leave index 0 set
    // to null
    this.gamepadButtons = new Trigger[11];

    for (int i = 1; i < gamepadButtons.length; i++) {
      gamepadButtons[i] = gamePad.button(i);
    }

    this.interLinkJoystickJoystickButtons = new Trigger[28];

    for (int i = 1; i < interLinkJoystickJoystickButtons.length; i++) {
      interLinkJoystickJoystickButtons[i] = interLinkJoystick.button(i);
    }
  }

  @Override
  public double getTranslateX() {
    return interLinkJoystick.getRawAxis(1);
  }

  @Override
  public double getTranslateY() {
    return -interLinkJoystick.getRawAxis(0);
  }

  @Override
  public double getRotate() {
    return -interLinkJoystick.getRawAxis(3);
  }

  @Override
  public double getDial() {
    return interLinkJoystick.getRawAxis(5); // Slider on Top Right Side
  }

  @Override
  public Trigger getFieldRelativeButton() {
    return interLinkJoystickJoystickButtons[1]; // A Button - near upper left top side
  }

  @Override
  public Trigger getResetGyroButton() {
    return interLinkJoystickJoystickButtons[14]; // RESET Button
  }

  @Override
  public Trigger getRightJoyUpButton() {
    return interLinkJoystickJoystickButtons[26];
  }

  @Override
  public Trigger getRightJoyDownButton() {
    return interLinkJoystickJoystickButtons[25];
  }

  @Override
  public Trigger getRightJoyLeftButton() {
    return interLinkJoystickJoystickButtons[23];
  }

  @Override
  public Trigger getRightJoyRightButton() {
    return interLinkJoystickJoystickButtons[24];
  }

  @Override
  public Trigger getGP_X() {
    return gamepadButtons[1];
  }

  @Override
  public Trigger getGP_Y() {
    return gamepadButtons[4];
  }

  @Override
  public Trigger getGP_Z() {
    return gamepadButtons[6];
  }

  @Override
  public Trigger getGP_A() {
    return gamepadButtons[2];
  }

  @Override
  public Trigger getGP_B() {
    return gamepadButtons[3];
  }

  @Override
  public Trigger getGP_C() {
    return gamepadButtons[5];
  }

  @Override
  public Trigger getGP_Y_Up() {
    double axisValue = gamePad.getRawAxis(4);
    if (axisValue < .5) {
      return new Trigger(() -> true);
    } else {
      return new Trigger(() -> false);
    }
  }

  @Override
  public Trigger getGP_Y_Down() {
    double axisValue = gamePad.getRawAxis(4);
    if (axisValue > .5) {
      return new Trigger(() -> true);
    } else {
      return new Trigger(() -> false);
    }
  }

  @Override
  public Trigger getGP_X_Left() {
    double axisValue = gamePad.getRawAxis(0);
    if (axisValue < .5) {
      return new Trigger(() -> true);
    } else {
      return new Trigger(() -> false);
    }
  }

  @Override
  public Trigger getGP_X_Right() {
    double axisValue = gamePad.getRawAxis(0);
    if (axisValue > .5) {
      return new Trigger(() -> true);
    } else {
      return new Trigger(() -> false);
    }
  }

  @Override
  public Trigger getGP_LB() {
    return gamepadButtons[7];
  }

  @Override
  public Trigger getGP_RB() {
    return gamepadButtons[8];
  }

  public Trigger getGP_Start() {
    return gamepadButtons[10];
  }
}
