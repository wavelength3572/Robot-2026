package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

/**
 * Tries parametric (physics-based) shot first. If the parametric result is unachievable (e.g., out
 * of RPM/angle range), falls back to the LUT strategy which clamps to its nearest entry.
 */
public class ParametricWithLUTFallbackStrategy implements ShotStrategy {

  private final ParametricShotStrategy parametric;
  private final LUTShotStrategy lutFallback;

  public ParametricWithLUTFallbackStrategy(
      ParametricShotStrategy parametric, LUTShotStrategy lutFallback) {
    this.parametric = parametric;
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

    ShotCalculator.ShotResult parametricResult =
        parametric.calculateShot(
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

    // Parametric unachievable — fall back to LUT
    Logger.recordOutput("SmartLaunch/Fallback/UsedLUT", true);
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
    return "Parametric+LUT";
  }
}
