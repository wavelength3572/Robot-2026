// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single Xbox controller. */
public class SingleHandheldOI implements OperatorInterface {
  private final XboxController controller;

  public SingleHandheldOI(int port) {
    controller = new XboxController(port);
  }

  @Override
  public double getTranslateX() {
    return -controller.getLeftY();
  }

  @Override
  public double getTranslateY() {
    return -controller.getLeftX();
  }

  @Override
  public double getRotate() {
    if (controller.getAxisCount() == 4) {
      // Probably the logitech controller and not the XBox controller
      return -controller.getRawAxis(2);
    } else {
      // Probably XBox controller
      return -controller.getRightX();
    }
  }

  @Override
  public Trigger getAButton() {
    return new Trigger(controller::getAButton);
  }

  @Override
  public Trigger getBButton() {
    return new Trigger(controller::getBButton);
  }

  @Override
  public Trigger getYButton() {
    return new Trigger(controller::getYButton);
  }

  @Override
  public Trigger getXButton() {
    return new Trigger(controller::getXButton);
  }

  @Override
  public Trigger getBackButton() {
    return new Trigger(controller::getBackButton);
  }
}
