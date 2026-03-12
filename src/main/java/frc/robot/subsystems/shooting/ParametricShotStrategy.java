package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Parametric (physics-based) shot strategy. Delegates directly to {@link
 * ShotCalculator#calculateHubShot}, using the ballistic trajectory calculation with velocity
 * compensation.
 *
 * <p>Two modes:
 *
 * <ul>
 *   <li><b>Calibrated</b> (default): Uses the RPM-dependent efficiency model derived from LUT data.
 *   <li><b>Raw</b>: No efficiency model at all — uses the static fallback efficiency. This is a
 *       true factory-reset mode with zero LUT influence.
 * </ul>
 */
public class ParametricShotStrategy implements ShotStrategy {

  private final boolean useRawEfficiency;
  private final String name;

  public ParametricShotStrategy() {
    this(false);
  }

  public ParametricShotStrategy(boolean useRawEfficiency) {
    this.useRawEfficiency = useRawEfficiency;
    this.name = useRawEfficiency ? "Parametric (Raw)" : "Parametric";
  }

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

    if (!useRawEfficiency) {
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

    // Raw mode: remove the fitted model, compute with fallback only, always restore.
    LaunchEfficiencyModel savedModel = ShotCalculator.getEfficiencyModel();
    ShotCalculator.setEfficiencyModel(null);
    try {
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
    } finally {
      ShotCalculator.setEfficiencyModel(savedModel);
    }
  }

  @Override
  public String getName() {
    return name;
  }
}
