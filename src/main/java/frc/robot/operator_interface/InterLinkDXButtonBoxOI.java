package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class InterLinkDXButtonBoxOI implements OperatorInterface {
  private final CommandGenericHID interLinkJoystick;
  private final Trigger[] interLinkJoystickJoystickButtons;
  private final CommandGenericHID player1ButtonBox;
  private final CommandGenericHID player2ButtonBox;
  private final Trigger[] Box1Buttons;
  private final Trigger[] Box2Buttons;

  public InterLinkDXButtonBoxOI(int interlinkPort, int buttonBox1Port, Integer buttonBox2Port) {
    interLinkJoystick = new CommandGenericHID(interlinkPort);
    player1ButtonBox = new CommandGenericHID(buttonBox1Port);
    player2ButtonBox = buttonBox2Port != null ? new CommandGenericHID(buttonBox2Port) : null;

    // buttons use 1-based indexing such that the index matches the button number;
    // leave index 0 set
    // to null
    this.Box1Buttons = new Trigger[13];
    for (int i = 1; i < Box1Buttons.length; i++) {
      Box1Buttons[i] = player1ButtonBox.button(i);
    }

    this.Box2Buttons = new Trigger[13];
    for (int i = 1; i < Box2Buttons.length; i++) {
      Box2Buttons[i] =
          player2ButtonBox != null ? player2ButtonBox.button(i) : new Trigger(() -> false);
    }

    this.interLinkJoystickJoystickButtons = new Trigger[28];

    for (int i = 1; i < interLinkJoystickJoystickButtons.length; i++) {
      interLinkJoystickJoystickButtons[i] = interLinkJoystick.button(i);
    }
  }

  // * INTERLINK BUTTONS */

  @Override
  public double getTranslateX() {
    return -interLinkJoystick.getRawAxis(1);
  }

  @Override
  public double getTranslateY() {
    return -interLinkJoystick.getRawAxis(0);
  }

  @Override
  public double getRotate() {
    return interLinkJoystick.getRawAxis(3);
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
  public Trigger getRightJoyLeftButton() {
    return interLinkJoystickJoystickButtons[23]; // slider directly below right axis - push to left
  }

  @Override
  public Trigger getLeftJoyLeftButton() {
    return interLinkJoystickJoystickButtons[19]; // slider directly below left axis - push left
  }

  @Override
  public Trigger getLeftJoyRightButton() {
    return interLinkJoystickJoystickButtons[20]; // slider directly below left axis - push right
  }

  @Override
  public Trigger getLeftJoyDownButton() {
    return interLinkJoystickJoystickButtons[21]; // slider directly right of left axis - push down
  }

  @Override
  public Trigger getLeftJoyUpButton() {
    return interLinkJoystickJoystickButtons[22]; // slider directly right of left axis - push up
  }

  @Override
  public Trigger getRightJoyRightButton() {
    return interLinkJoystickJoystickButtons[24]; // slider directly below right axis - push to right
  }

  @Override
  public Trigger getRightJoyDownButton() {
    return interLinkJoystickJoystickButtons[25];
  }

  @Override
  public Trigger getRightJoyUpButton() {
    return interLinkJoystickJoystickButtons[26];
  }

  @Override
  public Trigger getButtonFPosition1() {
    // Returns true when neither button[8] nor button[9] is active.
    return new Trigger(
        () ->
            !interLinkJoystickJoystickButtons[8].getAsBoolean()
                && !interLinkJoystickJoystickButtons[9].getAsBoolean());
  }

  @Override
  public Trigger getButtonI() {
    return interLinkJoystickJoystickButtons[13]; // Button I left side of top face - temp button
  }

  // * BUTTON BOX BUTTONS */

  @Override
  public Trigger getButtonBox2Button1() {
    return Box2Buttons[1]; //
  }

  @Override
  public Trigger getButtonBox2Button2() {
    return Box2Buttons[2]; //
  }

  @Override
  public Trigger getButtonBox2Button11() {
    return Box2Buttons[11]; //
  }

  @Override
  public Trigger getButtonBox2Button12() {
    return Box2Buttons[12]; //
  }

  @Override
  public Trigger getButtonBox2Button3() {
    return Box2Buttons[3]; //
  }

  @Override
  public Trigger getButtonBox2Button4() {
    return Box2Buttons[4]; //
  }

  @Override
  public Trigger getButtonBox2Button5() {
    return Box2Buttons[5]; //
  }

  @Override
  public Trigger getButtonBox2Button6() {
    return Box2Buttons[6]; //
  }

  @Override
  public Trigger getButtonBox2Button7() {
    return Box2Buttons[7]; //
  }

  @Override
  public Trigger getButtonBox1YAxisNegative() {
    return player1ButtonBox.axisLessThan(1, -0.5);
  }

  @Override
  public Trigger getButtonBox1YAxisPositive() {
    return player1ButtonBox.axisGreaterThan(1, 0.5);
  }

  @Override
  public Trigger getButtonBox1XAxisNegative() {
    return player1ButtonBox.axisLessThan(0, -0.5);
  }

  @Override
  public Trigger getButtonBox1XAxisPositive() {
    return player1ButtonBox.axisGreaterThan(0, 0.5);
  }

  @Override
  public Trigger getButtonBox1Button1() {
    return Box1Buttons[1]; //
  }

  @Override
  public Trigger getButtonBox1Button2() {
    return Box1Buttons[2]; //
  }

  @Override
  public Trigger getButtonBox1Button3() {
    return Box1Buttons[3]; //
  }

  @Override
  public Trigger getButtonBox1Button4() {
    return Box1Buttons[4]; //
  }

  @Override
  public Trigger getButtonBox1Button5() {
    return Box1Buttons[5]; //
  }

  @Override
  public Trigger getButtonBox1Button6() {
    return Box1Buttons[6]; //
  }

  @Override
  public Trigger getButtonBox1Button7() {
    return Box1Buttons[7]; //
  }

  @Override
  public Trigger getButtonBox1Button8() {
    return Box1Buttons[8]; //
  }

  @Override
  public Trigger getButtonBox1Button9() {
    return Box1Buttons[9]; //
  }

  @Override
  public Trigger getButtonBox1Button10() {
    return Box1Buttons[10]; //
  }

  @Override
  public Trigger getButtonBox1Button11() {
    return Box1Buttons[11]; //
  }

  @Override
  public Trigger getButtonBox1Button12() {
    return Box1Buttons[12]; //
  }

  public Trigger getButtonBox2Button8() {
    return Box2Buttons[8]; //
  }

  @Override
  public Trigger getPOVDown() {
    return player1ButtonBox.axisLessThan(1, .5);
  }

  @Override
  public Trigger getButtonFPosition2() {
    return interLinkJoystickJoystickButtons[8]; // Button F top right of front face - down position
  }

  @Override
  public Trigger getButtonFPosition0() {
    return interLinkJoystickJoystickButtons[9]; // Button F top right of front face - up position
  }

  @Override
  public Trigger getShootButton() {
    return interLinkJoystickJoystickButtons[13]; // Round Push Button on Top Left of Controller
  }

  @Override
  public Trigger getButtonDPosition0() {
    return interLinkJoystickJoystickButtons[6]; // Button D top left of front face - Up position
  }

  @Override
  public Trigger getButtonDPosition1() {
    return new Trigger(
        () ->
            !interLinkJoystickJoystickButtons[6].getAsBoolean()
                && !interLinkJoystickJoystickButtons[7].getAsBoolean());
  }

  @Override
  public Trigger getButtonDPosition2() {
    return interLinkJoystickJoystickButtons[7]; // Button D top left of front face - down position
  }

  @Override
  public Trigger getButtonCPosition0() {
    return interLinkJoystickJoystickButtons[4]; // Button C top left of front face - up position
  }

  @Override
  public Trigger getButtonCPosition2() {
    return interLinkJoystickJoystickButtons[5]; // Button C top left of front face - down position
  }
}
