package frc.robot.subsystems.motivator;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the motivator subsystem. The motivator feeds balls to the launcher. Defines the
 * inputs that are logged and the methods for controlling the motivator motors.
 *
 * <p>Hardware configuration:
 *
 * <ul>
 *   <li>Motor 55 (SparkMax + Neo): Motivator motor 1 (independent)
 * </ul>
 */
public interface MotivatorIO {
  /**
   * Reusable per-motor input class. Each motor gets its own instance logged under a subcategory.
   */
  @AutoLog
  public static class MotorInputs {
    public boolean connected = false;
    public double wheelRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double PdhCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double targetRPM = 0.0;
    public boolean atSetpoint = false;
  }

  /** Updates the set of loggable inputs for all three motors. */
  default void updateInputs(MotorInputs motivator1) {}

  // ========== Velocity Control ==========

  /**
   * Run motivator motor 1 at the specified velocity using closed-loop control.
   *
   * @param velocityRPM Target velocity in RPM
   */
  default void setMotivatorVelocity(double velocityRPM) {}

  default void setMotivatorVoltage(double volts) {}

  /** Stop only motivator motor 1. */
  default void stopMotivator() {}

  // ========== Configuration Methods ==========

  /** Configure PID gains for motivator motor 1. */
  default void configureMotivatorPID(double kP, double kI, double kD, double kS, double kV) {}

  /** Set velocity tolerances for atSetpoint checks. */
  default void setVelocityTolerance(double motivatorToleranceRPM) {}

  default double getFFCharacterizationVelocity() {
    return 0.0;
  }
}
