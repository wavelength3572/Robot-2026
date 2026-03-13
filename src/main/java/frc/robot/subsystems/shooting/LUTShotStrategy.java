package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
      Logger.recordOutput("Shots/Strategy/LUT/Status", "NO_DATA");
      return new ShotCalculator.ShotResult(0.0, 0.0, 0.0, 0.0, 0.0, target, false, 0.0, 0.0);
    }

    Logger.recordOutput("Shots/Strategy/LUT/Status", "OK");

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
      return new ShotCalculator.ShotResult(0.0, 0.0, 0.0, 0.0, 0.0, target, false, 0.0, 0.0);
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
    // The LUT stores actual RPMs that worked, bypassing efficiency entirely
    double exitVelocityMps =
        ShotCalculator.calculateExitVelocityFromRPM(entry.rpm(), finalDistance);

    // Check hood angle is achievable
    boolean achievable =
        entry.hoodAngleDeg() >= hoodMinAngleDeg && entry.hoodAngleDeg() <= hoodMaxAngleDeg;

    Logger.recordOutput("Shots/Strategy/LUT/StaticDistanceM", staticDistance);
    Logger.recordOutput("Shots/Strategy/LUT/FinalDistanceM", finalDistance);
    Logger.recordOutput("Shots/Strategy/LUT/InRange", lookupTable.isInRange(finalDistance));
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
        achievable,
        entry.motivatorRPM(),
        entry.spindexerRPM());
  }

  @Override
  public String getName() {
    return "LUT";
  }
}
