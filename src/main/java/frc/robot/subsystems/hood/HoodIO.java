package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the hood subsystem. The hood controls the launch angle by tilting the exit
 * mechanism. Used in hybrid RPM+hood trajectory control.
 */
public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    // Connection status
    public boolean connected = false;

    // Position data
    public double currentAngleDeg = 0.0;
    public double targetAngleDeg = 0.0;
    public double currentMotorRotations = 0.0;
    public double targetMotorRotations = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 25.0;

    // Control state
    public boolean atTarget = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodIOInputs inputs) {}

  /**
   * Set the target hood angle.
   *
   * @param angleDeg Target angle in degrees (0 = flat, positive = angled up)
   */
  public default void setAngle(double angleDeg) {}

  /**
   * Configure PID gains for the hood motor.
   *
   * @param kP Proportional gain
   * @param kD Derivative gain
   */
  public default void configurePID(double kP, double kD) {}

  /**
   * Set the angle tolerance used for atTarget calculation.
   *
   * @param toleranceDeg Tolerance in degrees
   */
  public default void setAngleTolerance(double toleranceDeg) {}
}
