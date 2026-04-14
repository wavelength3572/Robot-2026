package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double positionRotations = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Read sensor data into inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Command the motor to a target position in motor rotations. */
  public default void setPosition(double motorRotations) {}

  /** Command the motor to a target position with an arbitrary feedforward voltage. */
  public default void setPosition(double motorRotations, double arbFFVolts) {}

  /** Run the motor at a raw voltage (for pit mode). */
  public default void setVoltage(double volts) {}

  /** Enable or disable soft limits (disable for pit recovery). */
  public default void setSoftLimitsEnabled(boolean enabled) {}

  /** Stop the motor (brake mode holds position). */
  public default void stop() {}

  /** Update PID gains and climb output cap on the motor controller. */
  public default void configurePID(double kP, double climbMaxOutput) {}

  /** Zero the encoder position (call in pit if robot rebooted with climber not stowed). */
  public default void zeroEncoder() {}

  /** Set the encoder to a specific position (for pit recovery). */
  public default void setEncoderPosition(double rotations) {}
}
