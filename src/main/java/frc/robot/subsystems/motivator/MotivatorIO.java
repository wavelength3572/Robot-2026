package frc.robot.subsystems.motivator;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the motivator subsystem. The motivator feeds balls to the launcher. Defines the
 * inputs that are logged and the methods for controlling the motivator motors.
 *
 * <p>Hardware configuration:
 *
 * <ul>
 *   <li>Motor 55 (SparkFlex + Vortex): Motivator motor 1 (independent)
 *   <li>Motor 56 (SparkFlex + Vortex): Motivator motor 2 (independent)
 *   <li>Motor 57 (SparkFlex + Vortex): Prefeed roller (independent)
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
    public double tempCelsius = 0.0;
    public double targetVelocityRPM = 0.0;
    public boolean atSetpoint = false;
  }

  /** Updates the set of loggable inputs for all three motors. */
  default void updateInputs(MotorInputs motivator1, MotorInputs motivator2, MotorInputs prefeed) {}

  // ========== Duty Cycle Control ==========

  /**
   * Run motivator motor 1 at the specified duty cycle.
   *
   * @param dutyCycle Duty cycle from -1.0 to 1.0
   */
  default void setMotivator1DutyCycle(double dutyCycle) {}

  /**
   * Run motivator motor 2 at the specified duty cycle.
   *
   * @param dutyCycle Duty cycle from -1.0 to 1.0
   */
  default void setMotivator2DutyCycle(double dutyCycle) {}

  /**
   * Run the prefeed roller at the specified duty cycle.
   *
   * @param dutyCycle Duty cycle from -1.0 to 1.0
   */
  default void setPrefeedDutyCycle(double dutyCycle) {}

  // ========== Velocity Control ==========

  /**
   * Run motivator motor 1 at the specified velocity using closed-loop control.
   *
   * @param velocityRPM Target velocity in RPM
   */
  default void setMotivator1Velocity(double velocityRPM) {}

  /**
   * Run motivator motor 2 at the specified velocity using closed-loop control.
   *
   * @param velocityRPM Target velocity in RPM
   */
  default void setMotivator2Velocity(double velocityRPM) {}

  /**
   * Run the prefeed roller at the specified velocity using closed-loop control.
   *
   * @param velocityRPM Target velocity in RPM
   */
  default void setPrefeedVelocity(double velocityRPM) {}

  // ========== Stop Methods ==========

  /** Stop all motivator motors. */
  default void stop() {}

  /** Stop only motivator motor 1. */
  default void stopMotivator1() {}

  /** Stop only motivator motor 2. */
  default void stopMotivator2() {}

  /** Stop both motivator motors (1 and 2), but not prefeed. */
  default void stopMotivators() {}

  /** Stop only the prefeed motor. */
  default void stopPrefeed() {}

  // ========== Configuration Methods ==========

  /** Configure PID gains for motivator motor 1. */
  default void configureMotivator1PID(double kP, double kI, double kD, double kFF) {}

  /** Configure PID gains for motivator motor 2. */
  default void configureMotivator2PID(double kP, double kI, double kD, double kFF) {}

  /** Configure PID gains for the prefeed motor. */
  default void configurePrefeedPID(double kP, double kI, double kD, double kFF) {}

  /** Set velocity tolerances for atSetpoint checks. */
  default void setVelocityTolerances(double motivatorToleranceRPM, double prefeedToleranceRPM) {}
}
