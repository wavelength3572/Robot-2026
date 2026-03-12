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
 *   <li>Predict aim point (lead the target by TOF * robot velocity)
 *   <li>Re-compute distance to new aim point → re-look up TOF → repeat 3x
 *   <li>Final LUT lookup at converged distance gives RPM + hood angle
 * </ol>
 *
 * <p>Falls back to {@link ParametricShotStrategy} if the LUT has fewer than 2 data points or the
 * target distance is outside the empirical data range.
 */
public class LUTShotStrategy implements ShotStrategy {

  private final ShotLookupTable lookupTable;
  private final ParametricShotStrategy fallback = new ParametricShotStrategy();

  private static final int VELOCITY_COMP_ITERATIONS = 3;

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
      Logger.recordOutput("Shots/Strategy/LUT/FallbackReason", "Not enough data");
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

    // Fall back to parametric if outside the empirical data range
    if (!lookupTable.isInRange(staticDistance)) {
      Logger.recordOutput("Shots/Strategy/LUT/Fallback", true);
      Logger.recordOutput(
          "Shots/Strategy/LUT/FallbackReason",
          String.format(
              "Distance %.2fm outside LUT range [%.2f-%.2f]",
              staticDistance, lookupTable.getMinDistance(), lookupTable.getMaxDistance()));
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
    Logger.recordOutput("Shots/Strategy/LUT/FallbackReason", "");

    // Velocity compensation iteration using LUT TOF
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

    return new ShotCalculator.ShotResult(
        exitVelocityMps,
        entry.rpm(),
        launchAngleRad,
        entry.hoodAngleDeg(),
        turretAngleDeg,
        aimTarget,
        achievable);
  }

  @Override
  public String getName() {
    return "LUT";
  }
}
