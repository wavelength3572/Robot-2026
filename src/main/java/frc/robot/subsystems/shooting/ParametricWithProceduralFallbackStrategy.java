package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

/**
 * Graduated parametric fallback strategy. Tries progressively looser parametric approaches before
 * falling back to LUT:
 *
 * <ol>
 *   <li>Level 0: Normal parametric (descent angle sweep with standard RPM/peak limits)
 *   <li>Level 1: Fixed-hood parametric (clamp hood to nearest hardware limit, solve for RPM)
 *   <li>Level 2: Relaxed-constraints parametric (wider RPM range, higher peak height)
 *   <li>Level 3: LUT fallback (existing empirical lookup table)
 * </ol>
 *
 * <p>Levels 0-2 are handled by {@link ShotCalculator#calculateHubShotWithFallback}. Level 3 uses
 * the existing {@link LUTShotStrategy}.
 */
public class ParametricWithProceduralFallbackStrategy implements ShotStrategy {

  private final LUTShotStrategy lutFallback;

  public ParametricWithProceduralFallbackStrategy(LUTShotStrategy lutFallback) {
    this.lutFallback = lutFallback;
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

    // Levels 0-2: Graduated parametric fallback
    ShotCalculator.ShotResult parametricResult =
        ShotCalculator.calculateHubShotWithFallback(
            robotPose,
            fieldSpeeds,
            target,
            config,
            currentTurretAngleDeg,
            effectiveMinDeg,
            effectiveMaxDeg,
            hoodMinAngleDeg,
            hoodMaxAngleDeg);

    if (parametricResult.achievable()) {
      Logger.recordOutput("SmartLaunch/Fallback/UsedLUT", false);
      return parametricResult;
    }

    // Level 3: All parametric attempts failed — fall back to LUT
    Logger.recordOutput("SmartLaunch/Fallback/UsedLUT", true);
    Logger.recordOutput("SmartLaunch/Parametric/FallbackLevel", "LUT");
    return lutFallback.calculateShot(
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
    return "Parametric+ProceduralFallback";
  }
}
