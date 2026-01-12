// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {

  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default double getRotate() {
    return 0.0;
  }

  public default double getDial() {
    return 0.0;
  }

  public default double getKnob() {
    return 0.0;
  }

  public default Trigger getFieldRelativeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getFButtonPosition0() {
    return new Trigger(() -> false);
  }

  public default Trigger getCancelButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox2Button1() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox2Button2() {
    return new Trigger(() -> false);
  }

  public default Trigger getShootButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRightJoyUpButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRightJoyDownButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRightJoyLeftButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRightJoyRightButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getPOVUp() {
    return new Trigger(() -> false);
  }

  public default double getPOVUp2() {
    return 0.0;
  }

  public default Trigger getPOVDown() {
    return new Trigger(() -> false);
  }

  public default Trigger getPOVLeft() {
    return new Trigger(() -> false);
  }

  public default Trigger getPOVRight() {
    return new Trigger(() -> false);
  }

  public default Trigger getGP_X() {
    return new Trigger(() -> false);
  }

  public default Trigger getGP_Y() {
    return new Trigger(() -> false);
  }

  public default Trigger getGP_Z() {
    return new Trigger(() -> false);
  }

  public default Trigger getGP_A() {
    return new Trigger(() -> false);
  }

  public default Trigger getGP_B() {
    return new Trigger(() -> false);
  }

  public default Trigger getGP_C() {
    return new Trigger(() -> false);
  }

  public default Trigger getGP_Y_Up() {
    return new Trigger(() -> false);
  }

  public default Trigger getGP_Y_Down() {
    return new Trigger(() -> false);
  }

  public default Trigger getGP_X_Left() {
    return new Trigger(() -> false);
  }

  public default Trigger getGP_X_Right() {
    return new Trigger(() -> false);
  }

  public default Trigger getGP_LB() {
    return new Trigger(() -> false);
  }

  public default Trigger getGP_RB() {
    return new Trigger(() -> false);
  }

  public default Trigger getGP_Start() {
    return new Trigger(() -> false);
  }

  public default Trigger getLockWheels() {
    return new Trigger(() -> false);
  }

  public default Trigger getLeftJoyUpButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLeftJoyDownButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLeftJoyRightButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLeftJoyLeftButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox2Button11() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox2Button12() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox2Button3() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox2Button4() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox2Button5() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox2Button6() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox2Button7() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox1YAxisNegative() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox1YAxisPositive() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox1XAxisNegative() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox1XAxisPositive() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox2Button8() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox1Button1() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox1Button2() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox1Button3() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox1Button4() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox1Button5() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox1Button6() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox1Button7() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox1Button8() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox1Button9() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox1Button10() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox1Button11() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonBox1Button12() {
    return new Trigger(() -> false);
  }

  public default Trigger getAButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getBButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getYButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getXButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getBackButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonH() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonV() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonFPosition2() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonFPosition1() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonFPosition0() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonGPosition2() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonGPosition1() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonGPosition0() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonCPosition0() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonCPosition2() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonDPosition0() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonDPosition1() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonDPosition2() {
    return new Trigger(() -> false);
  }

  public default Trigger getButtonI() {
    return new Trigger(() -> false);
  }
}
