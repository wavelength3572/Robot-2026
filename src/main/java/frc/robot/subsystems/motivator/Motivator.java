package frc.robot.subsystems.motivator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Motivator subsystem for feeding balls to the launcher. Controls three motors:
 *
 * <ul>
 *   <li>Main motivator: Leader/follower pair (motors 55, 56) for final feeding
 *   <li>Prefeed roller: Independent motor (motor 57) for ball staging
 * </ul>
 */
public class Motivator extends SubsystemBase {
  private final MotivatorIO io;
  private final MotivatorIOInputsAutoLogged inputs = new MotivatorIOInputsAutoLogged();

  // Tunable duty cycles for different operations
  private static final LoggedTunableNumber feedDutyCycle =
      new LoggedTunableNumber("Motivator/FeedDutyCycle", 0.8);
  private static final LoggedTunableNumber prefeedDutyCycle =
      new LoggedTunableNumber("Motivator/PrefeedDutyCycle", 0.6);
  private static final LoggedTunableNumber ejectDutyCycle =
      new LoggedTunableNumber("Motivator/EjectDutyCycle", -0.5);

  public Motivator(MotivatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Motivator", inputs);
  }

  // ========== Direct Control Methods ==========

  /**
   * Run the main motivator at a specific duty cycle.
   *
   * @param dutyCycle Duty cycle from -1.0 to 1.0
   */
  public void setMotivatorDutyCycle(double dutyCycle) {
    io.setMotivatorDutyCycle(dutyCycle);
  }

  /**
   * Run the prefeed roller at a specific duty cycle.
   *
   * @param dutyCycle Duty cycle from -1.0 to 1.0
   */
  public void setPrefeedDutyCycle(double dutyCycle) {
    io.setPrefeedDutyCycle(dutyCycle);
  }

  /** Run the main motivator at the default feed duty cycle. */
  public void feed() {
    io.setMotivatorDutyCycle(feedDutyCycle.get());
  }

  /** Run the prefeed roller at the default prefeed duty cycle. */
  public void prefeed() {
    io.setPrefeedDutyCycle(prefeedDutyCycle.get());
  }

  /** Run both motivator and prefeed at their default duty cycles. */
  public void feedAndPrefeed() {
    io.setMotivatorDutyCycle(feedDutyCycle.get());
    io.setPrefeedDutyCycle(prefeedDutyCycle.get());
  }

  /** Run both motors in reverse to eject balls. */
  public void eject() {
    io.setMotivatorDutyCycle(ejectDutyCycle.get());
    io.setPrefeedDutyCycle(ejectDutyCycle.get());
  }

  /** Stop all motors. */
  public void stop() {
    io.stop();
  }

  /** Stop only the main motivator. */
  public void stopMotivator() {
    io.stopMotivator();
  }

  /** Stop only the prefeed roller. */
  public void stopPrefeed() {
    io.stopPrefeed();
  }

  // ========== Status Methods ==========

  /**
   * Get the main motivator velocity.
   *
   * @return Leader motor velocity in RPM
   */
  public double getMotivatorVelocity() {
    return inputs.motivatorLeaderVelocityRPM;
  }

  /**
   * Get the prefeed roller velocity.
   *
   * @return Prefeed motor velocity in RPM
   */
  public double getPrefeedVelocity() {
    return inputs.prefeedVelocityRPM;
  }

  /**
   * Check if a ball is detected (future sensor).
   *
   * @return True if ball detected
   */
  public boolean isBallDetected() {
    return inputs.ballDetected;
  }

  /**
   * Check if all motors are connected.
   *
   * @return True if all motors are responding
   */
  public boolean isConnected() {
    return inputs.motivatorLeaderConnected
        && inputs.motivatorFollowerConnected
        && inputs.prefeedConnected;
  }

  // ========== Commands ==========

  /**
   * Command to run the main motivator while held.
   *
   * @return Command that feeds balls until interrupted
   */
  public Command feedCommand() {
    return run(this::feed).finallyDo(this::stopMotivator).withName("Motivator: Feed");
  }

  /**
   * Command to run the prefeed roller while held.
   *
   * @return Command that prefeeds balls until interrupted
   */
  public Command prefeedCommand() {
    return run(this::prefeed).finallyDo(this::stopPrefeed).withName("Motivator: Prefeed");
  }

  /**
   * Command to run both motivator and prefeed while held.
   *
   * @return Command that feeds and prefeeds until interrupted
   */
  public Command feedAndPrefeedCommand() {
    return run(this::feedAndPrefeed).finallyDo(this::stop).withName("Motivator: Feed+Prefeed");
  }

  /**
   * Command to eject balls (reverse) while held.
   *
   * @return Command that ejects balls until interrupted
   */
  public Command ejectCommand() {
    return run(this::eject).finallyDo(this::stop).withName("Motivator: Eject");
  }

  /**
   * Command to stop all motors.
   *
   * @return Instant command that stops all motors
   */
  public Command stopCommand() {
    return runOnce(this::stop).withName("Motivator: Stop");
  }

  /**
   * Command to run the motivator at a custom duty cycle while held.
   *
   * @param dutyCycle Duty cycle from -1.0 to 1.0
   * @return Command that runs until interrupted
   */
  public Command runMotivatorCommand(double dutyCycle) {
    return run(() -> setMotivatorDutyCycle(dutyCycle))
        .finallyDo(this::stopMotivator)
        .withName("Motivator: Run at " + (int) (dutyCycle * 100) + "%");
  }

  /**
   * Command to run the prefeed at a custom duty cycle while held.
   *
   * @param dutyCycle Duty cycle from -1.0 to 1.0
   * @return Command that runs until interrupted
   */
  public Command runPrefeedCommand(double dutyCycle) {
    return run(() -> setPrefeedDutyCycle(dutyCycle))
        .finallyDo(this::stopPrefeed)
        .withName("Prefeed: Run at " + (int) (dutyCycle * 100) + "%");
  }
}
