// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two Xbox controllers. */
public class KeyboardOI implements OperatorInterface {
  private final CommandGenericHID keyboard;

  public KeyboardOI(int keyboardPort) {
    keyboard = new CommandGenericHID(keyboardPort);
  }

  @Override
  public Trigger getPOVUp() {
    return keyboard.povUp();
  }

  @Override
  public Trigger getPOVDown() {
    return keyboard.povDown();
  }

  @Override
  public Trigger getPOVLeft() {
    return keyboard.povLeft();
  }

  @Override
  public Trigger getPOVRight() {
    return keyboard.povRight();
  }

  @Override
  public double getTranslateX() {
    if (keyboard.povUp().getAsBoolean() || keyboard.povDown().getAsBoolean()) {
      if (keyboard.povUp().getAsBoolean()) {
        return 0.5;
      } else {
        return -0.5;
      }
    } else {
      return 0.0;
    }
  }

  @Override
  public double getTranslateY() {
    if (keyboard.povLeft().getAsBoolean() || keyboard.povRight().getAsBoolean()) {
      if (keyboard.povRight().getAsBoolean()) {
        return 0.5;
      } else {
        return -0.5;
      }
    } else {
      return 0.0;
    }
  }

  @Override
  public Trigger getGP_Y() {
    return keyboard.button(1);
  }
}
