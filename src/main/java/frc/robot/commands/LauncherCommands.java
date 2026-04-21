package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;

public class LauncherCommands {

  private LauncherCommands() {}

  /**
   * Measures the velocity feedforward constants for the Launcher motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Launcher launcher) {
    return FeedforwardCharacterization.run(
        launcher,
        launcher::runCharacterization,
        launcher::getFFCharacterizationVelocity,
        "Launcher");
  }
}
