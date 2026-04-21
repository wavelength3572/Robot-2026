package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.motivator.Motivator;

public class MotivatorCommands {

  private MotivatorCommands() {}

  /**
   * Measures the velocity feedforward constants for the Motivator motor.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Motivator motivator) {
    return FeedforwardCharacterization.run(
        motivator,
        motivator::runCharacterization,
        motivator::getFFCharacterizationVelocity,
        "Motivator");
  }
}
