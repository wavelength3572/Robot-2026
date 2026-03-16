package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Pure lookup-table shot strategy. All shot parameters (RPM, hood angle, TOF) come from empirically
 * recorded data — no physics involved at all.
 *
 * <p>Completely independent from the parametric system. Does NOT fall back to parametric — if the
 * LUT has insufficient data, returns a non-achievable result. Outside the data range, clamps to the
 * nearest entry (like 6328's approach).
 *
 * <p>Velocity compensation uses LUT-based time-of-flight, which captures real-world drag that the
 * parametric model ignores. The iteration loop (per Oblarg/CD) is:
 *
 * <ol>
 *   <li>Start with static distance to target
 *   <li>Look up TOF from LUT at that distance
 *   <li>Predict aim point (lead the target by TOF * robot velocity)
 *   <li>Re-compute distance to new aim point → re-look up TOF → repeat 3x
 *   <li>Final LUT lookup at converged distance gives RPM + hood angle
 * </ol>
 */
public class LUTShotStrategy implements ShotStrategy {

  private final ShotLookupTable lookupTable;

  private static final int VELOCITY_COMP_ITERATIONS = 3;

  /**
   * Max hood angle deviation (degrees) we can compensate with RPM. If the LUT wants an angle more
   * than this far outside the allowed range, the shot is unachievable. Within this range, we clamp
   * the hood and add RPM to compensate.
   *
   * <p>TODO: Tune this to support larger compensation (2-5°). There's no physical reason we
   * couldn't clamp e.g. a 24° shot down to 18° and compensate with RPM — the ball just needs more
   * speed on a steeper trajectory. The RPM-per-degree ratio may need to be non-linear at larger
   * deltas. For now 1° is conservative and safe.
   */
  private static final LoggedTunableNumber maxCompensationDeg =
      new LoggedTunableNumber("Shots/LUT/MaxCompensationDeg", 1.0);

  /** RPM added per degree of hood compensation (scales linearly with the clamped delta). */
  private static final LoggedTunableNumber rpmPerDegCompensation =
      new LoggedTunableNumber("Shots/LUT/RPMPerDegCompensation", 100.0);

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

    // If not enough LUT data, return non-achievable — don't silently fall back to parametric
    if (!lookupTable.hasEnoughData()) {
      return new ShotCalculator.ShotResult(0.0, 0.0, 0.0, 0.0, 0.0, target, false);
    }

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

    // Velocity compensation iteration using LUT TOF
    // Outside the data range, lookup() clamps to nearest entry — no fallback needed
    Translation3d aimTarget = target;
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    if (robotSpeed > 0.1) {
      double tof = lookupTable.lookupTOF(staticDistance);
      if (tof > 0) {
        for (int i = 0; i < VELOCITY_COMP_ITERATIONS; i++) {
          aimTarget =
              ShotCalculator.clampAimOffset(
                  ShotCalculator.predictTargetPos(target, fieldSpeeds, tof), target);
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

    // Look up shot params at the final distance (clamps at boundaries)
    ShotLookupTable.ShotEntry entry = lookupTable.lookup(finalDistance);
    if (entry == null) {
      return new ShotCalculator.ShotResult(0.0, 0.0, 0.0, 0.0, 0.0, target, false);
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

    // Clamp hood angle to the max limit (trench safety ceiling). The min limit is
    // hardware-only — we never need to clamp upward toward the trench.
    double rawHoodDeg = entry.hoodAngleDeg();
    double clampedHoodDeg = Math.min(hoodMaxAngleDeg, rawHoodDeg);
    // positive delta = LUT wanted a flatter angle than allowed, we clamped down
    //   → steeper launch, ball falls short → add RPM to compensate
    double hoodDelta = rawHoodDeg - clampedHoodDeg;
    double rpmCompensation = hoodDelta * rpmPerDegCompensation.get();
    double effectiveRPM = entry.rpm() + rpmCompensation;

    // Achievable if the hood is within hardware limits AND any trench clamp is
    // within our RPM compensation budget
    boolean achievable = rawHoodDeg >= hoodMinAngleDeg && hoodDelta <= maxCompensationDeg.get();

    Logger.recordOutput("Shots/Strategy/LUT/InRange", lookupTable.isInRange(finalDistance));
    Logger.recordOutput("Shots/Strategy/LUT/RawHoodDeg", rawHoodDeg);
    Logger.recordOutput("Shots/Strategy/LUT/ClampedHoodDeg", clampedHoodDeg);
    Logger.recordOutput("Shots/Strategy/LUT/RPMCompensation", rpmCompensation);

    // Convert clamped hood angle to launch angle for the result
    double launchAngleRad = Math.toRadians(90.0 - clampedHoodDeg);

    // RPM comes directly from LUT (+ any clamp compensation)
    double exitVelocityMps =
        ShotCalculator.calculateExitVelocityFromRPM(effectiveRPM, finalDistance);

    return new ShotCalculator.ShotResult(
        exitVelocityMps,
        effectiveRPM,
        launchAngleRad,
        clampedHoodDeg,
        turretAngleDeg,
        aimTarget,
        achievable);
  }

  @Override
  public String getName() {
    return "LUT";
  }
}
