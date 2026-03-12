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
 *       This is the normal mode — parametric physics with empirically-tuned efficiency.
 *   <li><b>Raw</b>: Bypasses the efficiency model entirely and uses a static tunable fallback. Use
 *       this as a factory reset if LUT data is bad or after a mechanical change.
 * </ul>
 */
public class ParametricShotStrategy implements ShotStrategy {

  private final boolean useRawEfficiency;
  private final String name;

  /** Create a calibrated parametric strategy (uses LUT-derived efficiency model). */
  public ParametricShotStrategy() {
    this(false);
  }

  /**
   * Create a parametric strategy.
   *
   * @param useRawEfficiency If true, bypasses the efficiency model and uses the static fallback
   */
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

    // Temporarily disable the efficiency model for raw mode
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

    // Raw mode: null the model, compute, ALWAYS restore (even on exception)
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
