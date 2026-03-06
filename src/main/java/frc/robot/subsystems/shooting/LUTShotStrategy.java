package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

/**
 * Pure lookup-table shot strategy. All shot parameters (RPM, hood angle, TOF) come from empirically
 * recorded data — no physics fudge factors involved.
 *
 * <p>Velocity compensation uses LUT-based time-of-flight, which captures real-world drag that the
 * parametric model ignores. The iteration loop (per Oblarg/CD) is:
 *
 * <ol>
 *   <li>Start with static distance to target
 *   <li>Look up TOF from LUT at that distance
 *   <li>Predict aim point (lead the target by TOF × robot velocity)
 *   <li>Re-compute distance to new aim point → re-look up TOF → repeat until converged
 *   <li>Final LUT lookup at converged distance gives RPM + hood angle
 * </ol>
 *
 * <p>The iteration includes early-exit when the aim point moves less than 1 cm, and tracks the
 * contraction rate (ratio of successive corrections) as a shot quality signal. See {@link
 * ShotCalculator.ShotResult#contractionRate} for interpretation.
 *
 * <p>Falls back to {@link ParametricShotStrategy} if the LUT has fewer than 2 data points.
 */
public class LUTShotStrategy implements ShotStrategy {

  private final ShotLookupTable lookupTable;
  private final ParametricShotStrategy fallback = new ParametricShotStrategy();

  /**
   * Max iterations for the velocity-compensation fixed-point loop. See {@link
   * ShotCalculator#calculateHubShot} for a detailed explanation of why this works and what the
   * contraction rate means.
   */
  private static final int VELOCITY_COMP_MAX_ITERATIONS = 3;

  /** Early-exit threshold — if the aim point moved less than this, stop iterating (meters). */
  private static final double CONVERGENCE_THRESHOLD_METERS = 0.01;

  public LUTShotStrategy(ShotLookupTable lookupTable) {
    this.lookupTable = lookupTable;
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

    // Fall back to parametric if not enough LUT data
    if (!lookupTable.hasEnoughData()) {
      Logger.recordOutput("Shots/Strategy/LUT/Fallback", true);
      return fallback.calculateShot(
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
    Logger.recordOutput("Shots/Strategy/LUT/Fallback", false);

    // Get turret field position
    double robotHeadingRad = robotPose.getRotation().getRadians();
    double[] turretFieldPos =
        ShotCalculator.getTurretFieldPosition(
            robotPose.getX(), robotPose.getY(), robotHeadingRad, config);
    double turretX = turretFieldPos[0];
    double turretY = turretFieldPos[1];

    // Static distance to target
    double staticDistance =
        Math.sqrt(Math.pow(target.getX() - turretX, 2) + Math.pow(target.getY() - turretY, 2));

    // --- Velocity compensation iteration using LUT TOF ---
    // Same fixed-point approach as ShotCalculator.calculateHubShot(), but using empirical
    // TOF from the lookup table instead of physics-based TOF. The LUT TOF captures real-world
    // drag that the parametric model ignores, which is the whole point of this strategy.
    Translation3d aimTarget = target;
    double contractionRate = -1.0; // -1 = not applicable (robot stationary)
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    if (robotSpeed > 0.1) {
      double tof = lookupTable.lookupTOF(staticDistance);
      if (tof > 0) {
        double prevCorrectionDist = 0.0;

        for (int i = 0; i < VELOCITY_COMP_MAX_ITERATIONS; i++) {
          Translation3d newAimTarget =
              ShotCalculator.clampAimOffset(
                  ShotCalculator.predictTargetPos(target, fieldSpeeds, tof), target);

          // Track convergence — how much the aim point moved this iteration
          double correctionDist =
              Math.sqrt(
                  Math.pow(newAimTarget.getX() - aimTarget.getX(), 2)
                      + Math.pow(newAimTarget.getY() - aimTarget.getY(), 2));

          if (i > 0 && prevCorrectionDist > CONVERGENCE_THRESHOLD_METERS) {
            contractionRate = correctionDist / prevCorrectionDist;
          }

          aimTarget = newAimTarget;

          if (correctionDist < CONVERGENCE_THRESHOLD_METERS) {
            break; // converged — further iterations won't move the aim point meaningfully
          }

          prevCorrectionDist = correctionDist;

          double aimDistance =
              Math.sqrt(
                  Math.pow(aimTarget.getX() - turretX, 2)
                      + Math.pow(aimTarget.getY() - turretY, 2));
          double newTof = lookupTable.lookupTOF(aimDistance);
          if (newTof > 0) {
            tof = newTof;
          }
        }
      }
    }

    // Final distance to the (possibly velocity-compensated) aim point
    double finalDistance =
        Math.sqrt(
            Math.pow(aimTarget.getX() - turretX, 2) + Math.pow(aimTarget.getY() - turretY, 2));

    // Look up shot params at the final distance
    ShotLookupTable.ShotEntry entry = lookupTable.lookup(finalDistance);
    if (entry == null) {
      // Shouldn't happen if hasEnoughData() passed, but be safe
      return fallback.calculateShot(
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

    // Calculate turret angle to aim at the target
    double turretAngleDeg =
        ShotCalculator.calculateOutsideTurretAngle(
            robotPose.getX(),
            robotPose.getY(),
            robotPose.getRotation().getDegrees(),
            aimTarget.getX(),
            aimTarget.getY(),
            currentTurretAngleDeg,
            effectiveMinDeg,
            effectiveMaxDeg,
            config);

    // Convert hood angle to launch angle for the result
    double launchAngleRad = Math.toRadians(90.0 - entry.hoodAngleDeg());

    // RPM comes directly from LUT — no efficiency conversion needed
    // The LUT stores actual RPMs that worked, bypassing the fudge factor entirely
    double exitVelocityMps = ShotCalculator.calculateExitVelocityFromRPM(entry.rpm());

    // Check hood angle is achievable
    boolean achievable =
        entry.hoodAngleDeg() >= hoodMinAngleDeg && entry.hoodAngleDeg() <= hoodMaxAngleDeg;

    Logger.recordOutput("Shots/Strategy/LUT/StaticDistanceM", staticDistance);
    Logger.recordOutput("Shots/Strategy/LUT/FinalDistanceM", finalDistance);
    Logger.recordOutput("Shots/Strategy/LUT/LookupRPM", entry.rpm());
    Logger.recordOutput("Shots/Strategy/LUT/LookupHoodDeg", entry.hoodAngleDeg());
    Logger.recordOutput("Shots/Strategy/LUT/LookupTOF", entry.timeOfFlightS());
    Logger.recordOutput("Shots/Strategy/LUT/ContractionRate", contractionRate);

    return new ShotCalculator.ShotResult(
        exitVelocityMps,
        entry.rpm(),
        launchAngleRad,
        entry.hoodAngleDeg(),
        turretAngleDeg,
        aimTarget,
        achievable,
        contractionRate);
  }

  @Override
  public String getName() {
    return "LUT";
  }
}
