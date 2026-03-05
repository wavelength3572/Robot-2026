package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Strategy interface for computing hub shot parameters. Implementations provide different ways to
 * determine RPM, hood angle, and turret angle — from physics equations (Parametric), empirical
 * lookup tables (LUT), or a blend of both (Hybrid).
 *
 * <p>All strategies receive the same inputs and produce a {@link ShotCalculator.ShotResult}.
 */
public interface ShotStrategy {

  /**
   * Calculate shot parameters to hit the given target.
   *
   * @param robotPose Current robot field pose
   * @param fieldSpeeds Field-relative chassis speeds (for velocity compensation)
   * @param target 3D target position (hub center)
   * @param config Turret geometry config
   * @param currentTurretAngleDeg Current turret angle for wrap optimization
   * @param effectiveMinDeg Turret min angle limit
   * @param effectiveMaxDeg Turret max angle limit
   * @param hoodMinAngleDeg Hood min angle limit
   * @param hoodMaxAngleDeg Hood max angle limit
   * @return Shot result with RPM, hood angle, turret angle, etc.
   */
  ShotCalculator.ShotResult calculateShot(
      Pose2d robotPose,
      ChassisSpeeds fieldSpeeds,
      Translation3d target,
      ShotCalculator.TurretConfig config,
      double currentTurretAngleDeg,
      double effectiveMinDeg,
      double effectiveMaxDeg,
      double hoodMinAngleDeg,
      double hoodMaxAngleDeg);

  /** Human-readable name for logging/dashboard. */
  String getName();
}
