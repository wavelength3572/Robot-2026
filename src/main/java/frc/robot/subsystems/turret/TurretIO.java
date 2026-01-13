package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public double currentAngleDegrees = 0.0;
    public double currentAngleRadians = 0.0;
    public double targetAngleDegrees = 0.0;
    public double targetAngleRadians = 0.0;
    public double velocityDegreesPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TurretIOInputs inputs) {}

  /** Set the target angle for the turret in degrees relative to robot front */
  public default void setTargetAngle(Rotation2d rotation) {}
}
