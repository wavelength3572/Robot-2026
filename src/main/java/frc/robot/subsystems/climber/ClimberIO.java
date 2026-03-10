package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double targetPosition = 0.0;
    public double currentPosition = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean climbingFinished = false;
  }

  public default void setClimberVoltage(double volts) {
  }

  public default void setServoPosition(double position) {
  }

  public default void updateInputs(ClimberIOInputs inputs) {
  }

  public default void deployClimber() {
  }

  public default void climb() {
  }

  public default void stopClimber() {
  }

  public default boolean isClimberDeployed() {
    return false;
  }

  public default boolean isClimbingFinished() {
    return false;
  }
}
