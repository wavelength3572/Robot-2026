package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.spindexer.Spindexer;

public class SpindexerCommands {

  private SpindexerCommands() {}

  /**
   * Measures the velocity feedforward constants for the Spindexer motor.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command spindexerCharacterization(Spindexer spindexer) {
    return FeedforwardCharacterization.run(
        spindexer,
        spindexer::runCharacterization,
        spindexer::getFFCharacterizationVelocity,
        "Spindexer");
  }
}
