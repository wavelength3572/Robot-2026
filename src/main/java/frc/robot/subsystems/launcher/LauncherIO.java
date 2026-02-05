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
    public double leaderTempCelsius = 0.0;

    // Follower motor data (for monitoring even in follower mode)
    public double followerVelocityRPM = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerCurrentAmps = 0.0;
    public double followerTempCelsius = 0.0;

    // Wheel velocity (after gear ratio conversion)
    public double wheelVelocityRPM = 0.0;

    // Control state
    public double targetVelocityRPM = 0.0;
    public boolean atSetpoint = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LauncherIOInputs inputs) {}

  /** Set the target velocity in wheel RPM. */
  public default void setVelocity(double velocityRPM) {}

  /** Run the launcher at the specified voltage (for characterization). */
  public default void setVoltage(double volts) {}

  /** Stop the launcher motors. */
  public default void stop() {}

  /**
   * Signal that a ball was fired (simulation only). In sim, this triggers a recovery period where
   * atSetpoint returns false until the configured recovery time elapses.
   */
  public default void notifyBallFired() {}
}
