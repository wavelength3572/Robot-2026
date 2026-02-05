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
  @AutoLog
  public static class MotivatorIOInputs {
    // Connection status
    public boolean motivator1Connected = false;
    public boolean motivator2Connected = false;
    public boolean prefeedConnected = false;

    // Motivator motor 1 data (CAN ID 55)
    public double motivator1VelocityRPM = 0.0;
    public double motivator1AppliedVolts = 0.0;
    public double motivator1CurrentAmps = 0.0;
    public double motivator1TempCelsius = 0.0;
    public double motivator1TargetVelocityRPM = 0.0;
    public boolean motivator1AtSetpoint = false;

    // Motivator motor 2 data (CAN ID 56)
    public double motivator2VelocityRPM = 0.0;
    public double motivator2AppliedVolts = 0.0;
    public double motivator2CurrentAmps = 0.0;
    public double motivator2TempCelsius = 0.0;
    public double motivator2TargetVelocityRPM = 0.0;
    public boolean motivator2AtSetpoint = false;

    // Prefeed motor data (CAN ID 57)
    public double prefeedVelocityRPM = 0.0;
    public double prefeedAppliedVolts = 0.0;
    public double prefeedCurrentAmps = 0.0;
    public double prefeedTempCelsius = 0.0;
    public double prefeedTargetVelocityRPM = 0.0;
    public boolean prefeedAtSetpoint = false;

    // Future: ball detection sensor
    public boolean ballDetected = false;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(MotivatorIOInputs inputs) {}

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
}
