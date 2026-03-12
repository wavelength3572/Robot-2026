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
 * <p>Falls back to pure parametric if the LUT has fewer than 2 data points.
 */
public class HybridShotStrategy implements ShotStrategy {

  private final ShotLookupTable lookupTable;
  private final ParametricShotStrategy fallback = new ParametricShotStrategy();

  private static final int VELOCITY_COMP_ITERATIONS = 3;

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

    // Velocity compensation using LUT TOF (the key hybrid advantage)
    Translation3d aimTarget = target;
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    boolean usedLutTof = false;

    if (robotSpeed > 0.1) {
      double tof = lookupTable.lookupTOF(staticDistance);
      if (tof > 0) {
        usedLutTof = true;
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
          // Refine RPM/hood from parametric at the new aim point to keep iteration consistent
          TrajectoryOptimizer.OptimalShot refined =
              TrajectoryOptimizer.calculateOptimalShot(
                  turretPos, aimTarget, hoodMinAngleDeg, hoodMaxAngleDeg);
          if (!refined.achievable) {
            // Parametric can't solve at this aim point — fall back to raw target
            aimTarget = target;
            usedLutTof = false;
            break;
          }
        }
      }
    }

    Logger.recordOutput("Shots/Strategy/Hybrid/UsingLUTTof", usedLutTof);
    Logger.recordOutput("Shots/Strategy/Hybrid/StaticDistanceM", staticDistance);

    // Final shot parameters from parametric TrajectoryOptimizer at the converged aim point
    TrajectoryOptimizer.OptimalShot optimalShot =
        TrajectoryOptimizer.calculateOptimalShot(
            turretPos, aimTarget, hoodMinAngleDeg, hoodMaxAngleDeg);

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

    Logger.recordOutput("Shots/Strategy/Hybrid/FinalRPM", optimalShot.rpm);
    Logger.recordOutput("Shots/Strategy/Hybrid/FinalHoodDeg", optimalShot.hoodAngleDeg);

    // Get feed speeds from LUT if available
    double finalDistance =
        Math.sqrt(
            Math.pow(aimTarget.getX() - turretX, 2) + Math.pow(aimTarget.getY() - turretY, 2));
    double motivatorRPM = 0.0;
    double spindexerRPM = 0.0;
    ShotLookupTable.ShotEntry feedEntry = lookupTable.lookup(finalDistance);
    if (feedEntry != null) {
      motivatorRPM = feedEntry.motivatorRPM();
      spindexerRPM = feedEntry.spindexerRPM();
    }

    return new ShotCalculator.ShotResult(
        optimalShot.exitVelocityMps,
        optimalShot.rpm,
        Math.toRadians(optimalShot.launchAngleDeg),
        optimalShot.hoodAngleDeg,
        turretAngleDeg,
        aimTarget,
        optimalShot.achievable,
        motivatorRPM,
        spindexerRPM);
  }

  @Override
  public String getName() {
    return "Hybrid";
  }
}
