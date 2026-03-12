package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

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
      ShotCalculator.ShotResult rawResult =
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

      // DEBUG: log raw result so we can compare with calibrated
      Logger.recordOutput("Shots/Debug/Raw/ExitVelocityMps", rawResult.exitVelocityMps());
      Logger.recordOutput("Shots/Debug/Raw/LaunchAngleDeg", rawResult.getLaunchAngleDegrees());
      Logger.recordOutput("Shots/Debug/Raw/RPM", rawResult.launcherRPM());
      Logger.recordOutput("Shots/Debug/Raw/HoodAngleDeg", rawResult.hoodAngleDeg());
      Logger.recordOutput("Shots/Debug/Raw/Achievable", rawResult.achievable());
      Logger.recordOutput(
          "Shots/Debug/Raw/AimTargetX", rawResult.aimTarget().getX());
      Logger.recordOutput(
          "Shots/Debug/Raw/AimTargetY", rawResult.aimTarget().getY());

      // Also compute calibrated for comparison (restore model, compute, null again)
      ShotCalculator.setEfficiencyModel(savedModel);
      ShotCalculator.ShotResult calResult =
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
      ShotCalculator.setEfficiencyModel(null); // re-null so finally restores correctly

      Logger.recordOutput("Shots/Debug/Cal/ExitVelocityMps", calResult.exitVelocityMps());
      Logger.recordOutput("Shots/Debug/Cal/LaunchAngleDeg", calResult.getLaunchAngleDegrees());
      Logger.recordOutput("Shots/Debug/Cal/RPM", calResult.launcherRPM());
      Logger.recordOutput("Shots/Debug/Cal/HoodAngleDeg", calResult.hoodAngleDeg());
      Logger.recordOutput("Shots/Debug/Cal/Achievable", calResult.achievable());
      Logger.recordOutput(
          "Shots/Debug/Cal/AimTargetX", calResult.aimTarget().getX());
      Logger.recordOutput(
          "Shots/Debug/Cal/AimTargetY", calResult.aimTarget().getY());

      return rawResult;
    } finally {
      ShotCalculator.setEfficiencyModel(savedModel);
    }
  }

  @Override
  public String getName() {
    return name;
  }
}
