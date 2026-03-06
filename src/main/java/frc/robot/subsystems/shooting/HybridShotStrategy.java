package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.hood.TrajectoryOptimizer;
import org.littletonrobotics.junction.Logger;

/**
 * Hybrid shot strategy: uses LUT time-of-flight for velocity compensation (captures real-world
 * drag) but parametric TrajectoryOptimizer for RPM and hood angle (works at any distance without
 * needing dense data).
 *
 * <p>This is the best choice when you have sparse LUT data (a few practice distances) but want
 * better shoot-on-the-move accuracy than pure parametric. The LUT TOF corrects the velocity
 * compensation lead, while the physics engine handles the actual shot parameters.
 *
 * <p>The velocity compensation loop uses the same fixed-point iteration as the other strategies
 * (see {@link ShotCalculator#calculateHubShot} for a detailed explanation), but the TOF comes from
 * the LUT while the RPM/hood come from TrajectoryOptimizer. This blend captures real-world drag in
 * the TOF without needing dense LUT data for every distance.
 *
 * <p>Falls back to pure parametric if the LUT has fewer than 2 data points.
 */
public class HybridShotStrategy implements ShotStrategy {

  private final ShotLookupTable lookupTable;
  private final ParametricShotStrategy fallback = new ParametricShotStrategy();

  /** See {@link ShotCalculator#calculateHubShot} for why 3 iterations is sufficient. */
  private static final int VELOCITY_COMP_MAX_ITERATIONS = 3;

  /** Early-exit threshold — if the aim point moved less than this, stop iterating (meters). */
  private static final double CONVERGENCE_THRESHOLD_METERS = 0.01;

  public HybridShotStrategy(ShotLookupTable lookupTable) {
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
      Logger.recordOutput("Shots/Strategy/Hybrid/UsingLUTTof", false);
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
    Translation3d turretPos = new Translation3d(turretX, turretY, config.heightMeters());

    // Static distance
    double staticDistance =
        Math.sqrt(Math.pow(target.getX() - turretX, 2) + Math.pow(target.getY() - turretY, 2));

    // --- Velocity compensation using LUT TOF (the key hybrid advantage) ---
    // TOF from the LUT captures real-world drag. RPM/hood from TrajectoryOptimizer works at
    // any distance. We keep the last optimizer result to avoid a redundant call after the loop.
    Translation3d aimTarget = target;
    double contractionRate = -1.0;
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    boolean usedLutTof = false;

    // Pre-compute the initial optimizer result (needed whether we iterate or not)
    TrajectoryOptimizer.OptimalShot lastShot =
        TrajectoryOptimizer.calculateOptimalShot(
            turretPos, target, hoodMinAngleDeg, hoodMaxAngleDeg);

    if (robotSpeed > 0.1) {
      double tof = lookupTable.lookupTOF(staticDistance);
      if (tof > 0) {
        usedLutTof = true;
        double prevCorrectionDist = 0.0;

        for (int i = 0; i < VELOCITY_COMP_MAX_ITERATIONS; i++) {
          Translation3d newAimTarget =
              ShotCalculator.clampAimOffset(
                  ShotCalculator.predictTargetPos(target, fieldSpeeds, tof), target);

          // Track convergence
          double correctionDist =
              Math.sqrt(
                  Math.pow(newAimTarget.getX() - aimTarget.getX(), 2)
                      + Math.pow(newAimTarget.getY() - aimTarget.getY(), 2));

          if (i > 0 && prevCorrectionDist > CONVERGENCE_THRESHOLD_METERS) {
            contractionRate = correctionDist / prevCorrectionDist;
          }

          aimTarget = newAimTarget;

          if (correctionDist < CONVERGENCE_THRESHOLD_METERS) {
            break;
          }

          prevCorrectionDist = correctionDist;

          double aimDistance =
              Math.sqrt(
                  Math.pow(aimTarget.getX() - turretX, 2)
                      + Math.pow(aimTarget.getY() - turretY, 2));

          // Update TOF from LUT at the new distance
          double newTof = lookupTable.lookupTOF(aimDistance);
          if (newTof > 0) {
            tof = newTof;
          }

          // Re-solve RPM/hood from parametric at the new aim point
          TrajectoryOptimizer.OptimalShot refined =
              TrajectoryOptimizer.calculateOptimalShot(
                  turretPos, aimTarget, hoodMinAngleDeg, hoodMaxAngleDeg);
          if (!refined.achievable) {
            // Parametric can't solve at this aim point — fall back to raw target
            aimTarget = target;
            usedLutTof = false;
            contractionRate = -1.0;
            // lastShot stays as the initial (static target) result
            break;
          }
          lastShot = refined;
        }
      }
    }

    Logger.recordOutput("Shots/Strategy/Hybrid/UsingLUTTof", usedLutTof);
    Logger.recordOutput("Shots/Strategy/Hybrid/StaticDistanceM", staticDistance);
    Logger.recordOutput("Shots/Strategy/Hybrid/ContractionRate", contractionRate);

    // lastShot already corresponds to aimTarget — no redundant optimizer call needed.

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

    Logger.recordOutput("Shots/Strategy/Hybrid/FinalRPM", lastShot.rpm);
    Logger.recordOutput("Shots/Strategy/Hybrid/FinalHoodDeg", lastShot.hoodAngleDeg);

    return new ShotCalculator.ShotResult(
        lastShot.exitVelocityMps,
        lastShot.rpm,
        Math.toRadians(lastShot.launchAngleDeg),
        lastShot.hoodAngleDeg,
        turretAngleDeg,
        aimTarget,
        lastShot.achievable,
        contractionRate);
  }

  @Override
  public String getName() {
    return "Hybrid";
  }
}
