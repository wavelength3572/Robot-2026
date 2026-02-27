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
    public double rollerPdhCurrentAmps = 0.0;
    public double rollerPdhVoltage = 0.0;
    public double rollerTargetSpeed = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Set the deploy motor to a target position (in mechanism rotations). */
  public default void setDeployPosition(double positionRotations) {}

  /** Set the roller motor speed as duty cycle (-1 to 1). */
  public default void setRollerDutyCycle(double dutyCycle) {}

  /** Set the roller motor target velocity in RPM (closed-loop). */
  public default void setRollerVelocity(double rpm) {}

  /** Stop both motors. */
  public default void stop() {}

  /** Set the deploy motor duty cycle directly (-1 to 1). */
  public default void setDeployDutyCycle(double dutyCycle) {}

  /** Stop only the deploy motor and hold current position via MAXMotion. */
  public default void stopDeploy() {}

  /** Stop the deploy motor completely with no PID or position hold. */
  public default void disableDeploy() {}

  /** Set the deploy motor idle mode (brake or coast). */
  default void setDeployBrakeMode(boolean brake) {}

  /** Configure the deploy motor output range (caps voltage during tuning). */
  default void configureDeployOutputRange(double min, double max) {}

  /** Configure PID gains for the deploy motor. */
  default void configureDeployPID(double kP, double kI, double kD) {}

  /** Configure PID + FF gains for the roller motor velocity control. */
  default void configureRollerPID(double kP, double kI, double kD, double kFF) {}

  /** Configure MAXMotion parameters for deploy motor (trapezoidal motion profiling). */
  default void configureDeployMaxMotion(
      double maxVelocity, double maxAcceleration, double allowedError) {}
}
