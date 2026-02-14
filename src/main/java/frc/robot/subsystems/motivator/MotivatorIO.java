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
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double PdhCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double targetVelocityRPM = 0.0;
    public boolean atSetpoint = false;
  }

  /** Updates the set of loggable inputs for all three motors. */
  default void updateInputs(MotorInputs motivator1) {}

  // ========== Duty Cycle Control ==========

  /**
   * Run motivator motor 1 at the specified duty cycle.
   *
   * @param dutyCycle Duty cycle from -1.0 to 1.0
   */
  default void setMotivator1DutyCycle(double dutyCycle) {}

  // ========== Velocity Control ==========

  /**
   * Run motivator motor 1 at the specified velocity using closed-loop control.
   *
   * @param velocityRPM Target velocity in RPM
   */
  default void setMotivator1Velocity(double velocityRPM) {}

  default void setMotivatorVoltage(double volts) {}

  // ========== Stop Methods ==========

  /** Stop all motivator motors. */
  default void stop() {}

  /** Stop only motivator motor 1. */
  default void stopMotivator1() {}

  /** Stop both motivator motors (1 and 2), but not prefeed. */
  default void stopMotivators() {}

  // ========== Configuration Methods ==========

  /** Configure PID gains for motivator motor 1. */
  default void configureMotivator1PID(double kP, double kI, double kD, double kS, double kV) {}

  /** Set velocity tolerances for atSetpoint checks. */
  default void setVelocityTolerances(double motivatorToleranceRPM) {}

  default double getFFCharacterizationVelocity() {
    return 0.0;
  }
}
