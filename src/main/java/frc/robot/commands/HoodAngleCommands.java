package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;

public class HoodAngleCommands {

  public static Command setHoodAngleCommand(Hood hood) {
    return Commands.runOnce(
        () -> {
          hood.setAngle(10);
        });
  }
}
