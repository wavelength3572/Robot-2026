package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // Deploy motor inputs
    public boolean deployConnected = false;
    public double deployPositionRotations = 0.0;
    public double deployVelocityRPM = 0.0;
    public double deployAppliedVolts = 0.0;
    public double deployCurrentAmps = 0.0;
    public double deployTargetPosition = 0.0;

    // Roller motor inputs
    public boolean rollerConnected = false;
    public double rollerVelocityRPM = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;
    public double rollerTargetSpeed = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Set the deploy motor to a target position (in mechanism rotations). */
  public default void setDeployPosition(double positionRotations) {}

  /** Set the roller motor speed as duty cycle (-1 to 1). */
  public default void setRollerDutyCycle(double dutyCycle) {}

  /** Stop both motors. */
  public default void stop() {}
}
