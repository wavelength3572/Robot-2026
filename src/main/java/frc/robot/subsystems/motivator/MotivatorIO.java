package frc.robot.subsystems.motivator;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the motivator subsystem. The motivator feeds balls to the launcher. Defines the
 * inputs that are logged and the methods for controlling the motivator motors.
 *
 * <p>Hardware configuration:
 *
 * <ul>
 *   <li>Motor 55 (SparkFlex + Vortex): Main motivator leader
 *   <li>Motor 56 (SparkFlex + Vortex): Main motivator follower (inverted)
 *   <li>Motor 57 (SparkFlex + Vortex): Prefeed roller (independent)
 * </ul>
 */
public interface MotivatorIO {
  @AutoLog
  public static class MotivatorIOInputs {
    // Connection status
    public boolean motivatorLeaderConnected = false;
    public boolean motivatorFollowerConnected = false;
    public boolean prefeedConnected = false;

    // Main motivator leader motor data
    public double motivatorLeaderVelocityRPM = 0.0;
    public double motivatorLeaderAppliedVolts = 0.0;
    public double motivatorLeaderCurrentAmps = 0.0;
    public double motivatorLeaderTempCelsius = 0.0;

    // Main motivator follower motor data (for monitoring)
    public double motivatorFollowerVelocityRPM = 0.0;
    public double motivatorFollowerAppliedVolts = 0.0;
    public double motivatorFollowerCurrentAmps = 0.0;
    public double motivatorFollowerTempCelsius = 0.0;

    // Prefeed motor data
    public double prefeedVelocityRPM = 0.0;
    public double prefeedAppliedVolts = 0.0;
    public double prefeedCurrentAmps = 0.0;
    public double prefeedTempCelsius = 0.0;

    // Future: ball detection sensor
    public boolean ballDetected = false;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(MotivatorIOInputs inputs) {}

  /**
   * Run the main motivator at the specified duty cycle.
   *
   * @param dutyCycle Duty cycle from -1.0 to 1.0
   */
  default void setMotivatorDutyCycle(double dutyCycle) {}

  /**
   * Run the prefeed roller at the specified duty cycle.
   *
   * @param dutyCycle Duty cycle from -1.0 to 1.0
   */
  default void setPrefeedDutyCycle(double dutyCycle) {}

  /** Stop all motivator motors. */
  default void stop() {}

  /** Stop only the main motivator motors (prefeed continues). */
  default void stopMotivator() {}

  /** Stop only the prefeed motor (main motivator continues). */
  default void stopPrefeed() {}
}
