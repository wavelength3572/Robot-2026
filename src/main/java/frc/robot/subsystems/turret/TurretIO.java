package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public double currentAngleDegrees = 0.0;
    public double targetAngleDegrees = 0.0;
    public double velocityDegreesPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TurretIOInputs inputs) {}

  /** Set the target angle for the turret in degrees relative to robot front */
  public default void setTargetAngle(double angleDegrees) {}

  /** Stop the turret motor */
  public default void stop() {}

  /** Reset the turret encoder to a specific angle */
  public default void resetEncoder(double angleDegrees) {}

  /** Set turret to brake or coast mode */
  public default void setBrakeMode(boolean enable) {}
}
