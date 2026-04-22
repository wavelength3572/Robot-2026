package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Climber subsystem. Despite the name, this is a pre-feed wheel that sits
 * before the spindexer — the "climber" label is a holdover from an earlier robot revision.
 */
public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean connected = false;
    public double wheelRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double targetRPM = 0.0;
    public boolean atSetpoint = false;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  /** Run the wheel at the specified velocity using closed-loop control. */
  default void setClimberVelocity(double wheelVelocityRPM) {}

  /** Run the motor at a raw voltage (for characterization / pit mode). */
  default void setClimberVoltage(double volts) {}

  /** Stop the motor. */
  default void stopClimber() {}

  /** Configure PID + feedforward gains. */
  default void configureClimberPID(double kP, double kI, double kD, double kS, double kV) {}

  /** Velocity tolerance for atSetpoint check. */
  default void setVelocityTolerance(double toleranceRPM) {}

  default double getFFCharacterizationVelocity() {
    return 0.0;
  }
}
