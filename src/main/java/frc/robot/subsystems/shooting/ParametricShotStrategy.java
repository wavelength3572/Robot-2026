package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Parametric (physics-based) shot strategy. Delegates directly to {@link
 * ShotCalculator#calculateHubShot}, using the ballistic trajectory calculation with velocity
 * compensation.
 *
 * <p>The single tunable knob is the launch efficiency constant in ShotCalculator
 * (Shots/SmartLaunch/Trajectory/LaunchEfficiency). This controls how RPM maps to exit velocity.
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
