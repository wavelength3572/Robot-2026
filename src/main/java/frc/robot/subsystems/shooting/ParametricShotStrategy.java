package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Parametric (physics-based) shot strategy. Delegates directly to {@link
 * ShotCalculator#calculateHubShot}, preserving the existing ballistic trajectory calculation with
 * velocity compensation.
 *
 * <p>This is the default strategy — robot behavior is identical to before the strategy pattern was
 * introduced.
 */
public class ParametricShotStrategy implements ShotStrategy {

  @Override
  public ShotCalculator.ShotResult calculateShot(
      Pose2d robotPose,
      ChassisSpeeds fieldSpeeds,
      Translation3d target,
      ShotCalculator.TurretConfig config,
      double currentTurretAngleDeg,
      double effectiveMinDeg,
      double effectiveMaxDeg,
      double hoodMinAngleDeg,
      double hoodMaxAngleDeg) {
    return ShotCalculator.calculateHubShot(
        robotPose,
        fieldSpeeds,
        target,
        config,
        currentTurretAngleDeg,
        effectiveMinDeg,
        effectiveMaxDeg,
        hoodMinAngleDeg,
        hoodMaxAngleDeg);
  }

  @Override
  public String getName() {
    return "Parametric";
  }
}
