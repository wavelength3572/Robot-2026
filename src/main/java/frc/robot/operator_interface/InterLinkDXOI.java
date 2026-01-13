// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two Xbox controllers. */
public class InterLinkDXOI implements OperatorInterface {
  private final CommandGenericHID interLinkJoystick;
  private final Trigger[] interLinkJoystickJoystickButtons;

  public InterLinkDXOI(int interlinkPort) {
    interLinkJoystick = new CommandGenericHID(interlinkPort);

    // buttons use 1-based indexing such that the index matches the button number;
    // leave index 0 set
    // to null
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
  public double getKnob() {
    return interLinkJoystick.getRawAxis(7); // Left Side Knob
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
  public Trigger getCancelButton() {
    return interLinkJoystickJoystickButtons[15]; // RESET Button
  }

  @Override
  public Trigger getShootButton() {
    return interLinkJoystickJoystickButtons[13]; // Round Push Button on Top Left of Controller
  }

  @Override
  public Trigger getRightJoyUpButton() {
    return interLinkJoystickJoystickButtons[26]; // slider directly left of right axis - push up
  }

  @Override
  public Trigger getRightJoyDownButton() {
    return interLinkJoystickJoystickButtons[25]; // slider directly left of right axis - push down
  }

  @Override
  public Trigger getRightJoyLeftButton() {
    return interLinkJoystickJoystickButtons[23]; // slider directly below right axis - push to left
  }

  @Override
  public Trigger getRightJoyRightButton() {
    return interLinkJoystickJoystickButtons[24]; // slider directly below right axis - push to right
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
  public Trigger getLeftJoyLeftButton() {
    return interLinkJoystickJoystickButtons[19]; // slider directly below left axis - push left
  }

  @Override
  public Trigger getLeftJoyRightButton() {
    return interLinkJoystickJoystickButtons[20]; // slider directly below left axis - push right
  }

  @Override
  public Trigger getButtonH() {
    return interLinkJoystickJoystickButtons[12]; // Button H bottom right of top face - up position
  }

  @Override
  public Trigger getButtonI() {
    return interLinkJoystickJoystickButtons[13]; // Button I left side of top face - temp button
  }

  @Override
  public Trigger getButtonV() {
    return interLinkJoystickJoystickButtons[1]; // Button v bottom left of top face - up position
  }

  @Override
  public Trigger getButtonFPosition2() {
    return interLinkJoystickJoystickButtons[8]; // Button F top right of front face - down position
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
  public Trigger getButtonFPosition0() {
    return interLinkJoystickJoystickButtons[9]; // Button F top right of front face - up position
  }

  @Override
  public Trigger getButtonGPosition2() {
    return interLinkJoystickJoystickButtons[10]; // Button G top right of front face - down position
  }

  @Override
  public Trigger getButtonGPosition1() {
    // Returns true when neither button[10] nor button[11] is active.
    return new Trigger(
        () ->
            !interLinkJoystickJoystickButtons[10].getAsBoolean()
                && !interLinkJoystickJoystickButtons[11].getAsBoolean());
  }

  @Override
  public Trigger getButtonGPosition0() {
    return interLinkJoystickJoystickButtons[11]; // Button G top right of front face - up position
  }

  @Override
  public Trigger getButtonCPosition0() {
    return interLinkJoystickJoystickButtons[4]; // Button C top left of front face - up position
  }

  @Override
  public Trigger getButtonCPosition2() {
    return interLinkJoystickJoystickButtons[5]; // Button C top left of front face - down position
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
}
