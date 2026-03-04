package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the launcher subsystem. Defines the inputs that are logged and the methods for
 * controlling the launcher motors.
 */
public interface LauncherIO {
  @AutoLog
  public static class LauncherIOInputs {
    // Connection status
    public boolean leaderConnected = false;
    public boolean followerConnected = false;

    // Leader motor data (motor RPM, before gear ratio)
    public double leaderVelocityRPM = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double leaderCurrentAmps = 0.0;
    public double leaderPdhCurrentAmps = 0.0;
    public double leaderTempCelsius = 0.0;

    // Follower motor data (for monitoring even in follower mode)
    public double followerVelocityRPM = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerCurrentAmps = 0.0;
    public double followerPdhCurrentAmps = 0.0;
    public double followerTempCelsius = 0.0;

    // Wheel velocity (after gear ratio conversion)
    public double wheelVelocityRPM = 0.0;

    // PDH voltage
    public double pdhVoltage = 0.0;

    // Control state
    public double targetVelocityRPM = 0.0;
    public boolean atSetpoint = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LauncherIOInputs inputs) {}

  /** Set the target velocity in wheel RPM. */
  public default void setVelocity(double velocityRPM) {}

  /**
   * Set the target velocity with recovery boost parameters.
   *
   * @param velocityRPM Target velocity in wheel RPM
   * @param boostVolts Extra feedforward voltage to add during recovery
   * @param recoveryActive True to use recovery PID gains (higher P for faster response)
   */
  public default void setVelocityWithBoost(
      double velocityRPM, double boostVolts, boolean recoveryActive) {
    setVelocity(velocityRPM);
  }

  /** Run the launcher at the specified voltage (for characterization). */
  public default void setLauncherVoltage(double volts) {}

  /** Stop the launcher motors. */
  public default void stop() {}

  /**
   * Signal that a ball was fired (simulation only). In sim, this triggers a recovery period where
   * atSetpoint returns false until the configured recovery time elapses.
   */
  public default void notifyBallFired() {}

  /** Configure PID gains for velocity control. */
  default void configurePID(double kP, double kI, double kD, double recoveryKpBoost) {}

  /** Configure feedforward gains for velocity control. */
  default void configureFeedforward(double kS, double kV, double kA) {}

  /** Set the velocity tolerance for atSetpoint checks. */
  default void setVelocityTolerance(double toleranceRPM) {}

  /** Get the velocity of the launcher motor in motor. */
  public default double getFFCharacterizationVelocity() {
    return 0.0;
  }
}
