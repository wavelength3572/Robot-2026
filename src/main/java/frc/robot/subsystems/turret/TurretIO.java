package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public double currentInsideAngleDeg = 0.0;
    public double targetInsideAngleDeg = 0.0;
    public double currentOutsideAngleDeg = 0.0;
    public double targetOutsideAngleDeg = 0.0;
    public double velocityDegreesPerSec = 0.0;
    public double velocityMotor = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double motorPosition = 0.0;
    public double absEncoder = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TurretIOInputs inputs) {}

  /** Set the target angle for the turret in degrees relative to robot front */
  public default void setOutsideTurretAngle(Rotation2d rotation) {}

  /** Set the target angle for the turret in degrees relative to robot front */
  public default void setInsideTurretAngle_ONLY_FOR_TESTING(Rotation2d rotation) {}

  public default Rotation2d getOutsideTargetAngle() {
    return Rotation2d.kZero;
  }

  public default Rotation2d getOutsideCurrentAngle() {
    return Rotation2d.kZero;
  }

  /** Set the target angle for the turret in degrees relative to robot front */
  public default void setTurretVolts(double volts) {}
}
