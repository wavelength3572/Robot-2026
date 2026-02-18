package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the motivator subsystem. The motivator feeds balls to the launcher. Defines the
 * inputs that are logged and the methods for controlling the Spindexer.
 *
 * <p>Hardware configuration:
 *
 * <ul>
 *   <li>Motor 55 (SparkMax + Neo):Spindexer (independent)
 * </ul>
 */
public interface SpindexerIO {
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
  default void updateInputs(MotorInputs spindexer) {}

  // ========== Velocity Control ==========

  /**
   * Run Spindexer at the specified velocity using closed-loop control.
   *
   * @param velocityRPM Target velocity in RPM
   */
  default void setSpindexerVelocity(double velocityRPM) {}

  default void setSpindexerVoltage(double volts) {}

  /** Stop only Spindexer */
  default void stopSpindexer() {}

  // ========== Configuration Methods ==========

  /** Configure PID gains for spindexer */
  default void configureSpindexerPID(double kP, double kI, double kD, double kS, double kV) {}

  /** Set velocity tolerances for atSetpoint checks. */
  default void setVelocityTolerance(double spindexerToleranceRPM) {}

  default double getFFCharacterizationVelocity() {
    return 0.0;
  }
}
