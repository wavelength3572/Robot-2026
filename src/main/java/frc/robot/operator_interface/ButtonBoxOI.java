package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoxOI implements OperatorInterface {
  private final CommandGenericHID player1ButtonBox;
  private final CommandGenericHID player2ButtonBox;
  private final Trigger[] Box1Buttons;
  private final Trigger[] Box2Buttons;

  public ButtonBoxOI(int buttonBox1Port, int buttonBox2Port) {
    player1ButtonBox = new CommandGenericHID(buttonBox1Port);
    player2ButtonBox = new CommandGenericHID(buttonBox2Port);

    // buttons use 1-based indexing such that the index matches the button number;
    // leave index 0 set
    // to null
    this.Box1Buttons = new Trigger[13];
    for (int i = 1; i < Box1Buttons.length; i++) {
      Box1Buttons[i] = player1ButtonBox.button(i);
    }

    this.Box2Buttons = new Trigger[13];
    for (int i = 1; i < Box2Buttons.length; i++) {
      Box2Buttons[i] = player2ButtonBox.button(i);
    }
  }

  @Override
  public Trigger getLockWheels() {
    return Box2Buttons[4];
  }

  @Override
  public Trigger getPOVUp() {
    return player2ButtonBox.axisLessThan(1, -.5);
  }

  @Override
  public Trigger getPOVDown() {
    return player2ButtonBox.axisGreaterThan(1, .5);
  }

  @Override
  public Trigger getPOVLeft() {
    return player2ButtonBox.axisLessThan(0, -.5);
  }

  @Override
  public Trigger getPOVRight() {
    return player2ButtonBox.axisGreaterThan(0, .5);
  }

  @Override
  public Trigger getButtonBox2Button7() {
    return Box2Buttons[7]; //
  }

  @Override
  public Trigger getButtonBox1Button7() {
    return Box1Buttons[7]; //
  }

  @Override
  public Trigger getLeftJoyRightButton() {
    return Box1Buttons[2]; //
  }

  @Override
  public Trigger getLeftJoyLeftButton() {
    return Box1Buttons[1]; //
  }
}
