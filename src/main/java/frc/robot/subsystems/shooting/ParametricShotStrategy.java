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
 *   <li><b>Raw</b>: Uses the same trajectory physics as calibrated (identical arc), but converts
 *       exit velocity to RPM using the static fallback efficiency instead of the fitted model.
 *       This shows what RPM the real robot would need without calibration data.
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
   * @param useRawEfficiency If true, recalculates RPM using the static fallback efficiency
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

    // Always compute the shot with the full efficiency model — trajectory physics are
    // identical regardless of efficiency. This avoids mutating global state and ensures
    // the velocity compensation loop converges the same way for both modes.
    ShotCalculator.ShotResult calibratedResult =
        ShotCalculator.calculateHubShot(
            robotPose,
            fieldSpeeds,
            target,
            config,
            currentTurretAngleDeg,
            effectiveMinDeg,
            effectiveMaxDeg,
            hoodMinAngleDeg,
            hoodMaxAngleDeg);

    if (!useRawEfficiency) {
      return calibratedResult;
    }

    // Raw mode: same trajectory, just recalculate RPM using the static fallback efficiency.
    // This shows what the real robot would need without any calibration data.
    double rawRPM = ShotCalculator.calculateRPMFromFallbackEfficiency(calibratedResult.exitVelocityMps());
    return new ShotCalculator.ShotResult(
        calibratedResult.exitVelocityMps(),
        rawRPM,
        calibratedResult.launchAngleRad(),
        calibratedResult.hoodAngleDeg(),
        calibratedResult.turretAngleDeg(),
        calibratedResult.aimTarget(),
        calibratedResult.achievable());
  }

  @Override
  public String getName() {
    return name;
  }
}
