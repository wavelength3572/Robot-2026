package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GamepadOI implements OperatorInterface {
  private final CommandGenericHID gamePad;
  private final Trigger[] gamepadButtons;

  public GamepadOI(int gamepadPort) {
    gamePad = new CommandGenericHID(gamepadPort);

    // buttons use 1-based indexing such that the index matches the button number;
    // leave index 0 set
    // to null
    this.gamepadButtons = new Trigger[11];

    for (int i = 1; i < gamepadButtons.length; i++) {
      gamepadButtons[i] = gamePad.button(i);
    }
  }
}
