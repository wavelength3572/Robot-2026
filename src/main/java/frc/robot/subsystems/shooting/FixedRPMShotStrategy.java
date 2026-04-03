package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Simple shot strategy: fixed launcher RPM, hood angle varies by distance only.
 *
 * <p>Instead of computing exit velocity from parabola physics and deriving RPM, this strategy uses a
 * constant launcher RPM and a distance-to-hood-angle lookup table. This eliminates
 * orientation-dependent shot variation caused by motivator energy modeling errors.
 *
 * <p>Tuning: set the launcher RPM high enough to reach the farthest shot, then tune hood angle at
 * each distance breakpoint until shots land consistently.
 */
public class FixedRPMShotStrategy implements ShotStrategy {

  // Fixed launcher RPM — tune this once so the farthest shot has enough energy
  private static final LoggedTunableNumber launcherRPM =
      new LoggedTunableNumber(
          "Shots/FixedRPM/LauncherRPM",
          Constants.getRobotConfig().getFixedHeightMaxRPM());

  // Distance breakpoints (meters) and corresponding hood angles (degrees).
  // Hood angle: lower = steeper/loftier shot, higher = flatter shot.
  // Hood range is typically 13° (steep) to 46° (flat).
  private static final LoggedTunableNumber dist1M =
      new LoggedTunableNumber("Shots/FixedRPM/Dist1M", 1.2);
  private static final LoggedTunableNumber hood1Deg =
      new LoggedTunableNumber("Shots/FixedRPM/Hood1Deg", 13.0);

  private static final LoggedTunableNumber dist2M =
      new LoggedTunableNumber("Shots/FixedRPM/Dist2M", 2.0);
  private static final LoggedTunableNumber hood2Deg =
      new LoggedTunableNumber("Shots/FixedRPM/Hood2Deg", 20.0);

  private static final LoggedTunableNumber dist3M =
      new LoggedTunableNumber("Shots/FixedRPM/Dist3M", 3.0);
  private static final LoggedTunableNumber hood3Deg =
      new LoggedTunableNumber("Shots/FixedRPM/Hood3Deg", 28.0);

  private static final LoggedTunableNumber dist4M =
      new LoggedTunableNumber("Shots/FixedRPM/Dist4M", 4.0);
  private static final LoggedTunableNumber hood4Deg =
      new LoggedTunableNumber("Shots/FixedRPM/Hood4Deg", 35.0);

  private static final LoggedTunableNumber dist5M =
      new LoggedTunableNumber("Shots/FixedRPM/Dist5M", 5.5);
  private static final LoggedTunableNumber hood5Deg =
      new LoggedTunableNumber("Shots/FixedRPM/Hood5Deg", 42.0);

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

    // Get turret field position
    double robotHeadingRad = robotPose.getRotation().getRadians();
    double[] turretFieldPos =
        ShotCalculator.getTurretFieldPosition(
            robotPose.getX(), robotPose.getY(), robotHeadingRad, config);
    double turretX = turretFieldPos[0];
    double turretY = turretFieldPos[1];

    // Distance from turret to target
    double D = Math.hypot(target.getX() - turretX, target.getY() - turretY);

    // Interpolate hood angle from distance lookup table
    double hoodAngleDeg = interpolateHoodAngle(D);

    // Clamp hood to mechanical limits
    hoodAngleDeg = Math.max(hoodMinAngleDeg, Math.min(hoodMaxAngleDeg, hoodAngleDeg));

    // Calculate turret angle
    double turretAngleDeg =
        ShotCalculator.calculateOutsideTurretAngle(
            robotPose.getX(),
            robotPose.getY(),
            robotPose.getRotation().getDegrees(),
            target.getX(),
            target.getY(),
            currentTurretAngleDeg,
            effectiveMinDeg,
            effectiveMaxDeg,
            config);

    double rpm = launcherRPM.get();
    double theta = Math.toRadians(90.0 - hoodAngleDeg);
    double exitVelocity = ShotCalculator.calculateExitVelocityFromRPM(rpm, D);

    // Log
    Logger.recordOutput("Shots/FixedRPM/DistanceM", D);
    Logger.recordOutput("Shots/FixedRPM/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("Shots/FixedRPM/LauncherRPM", rpm);
    Logger.recordOutput("Shots/FixedRPM/ExitVelocityMps", exitVelocity);

    return new ShotCalculator.ShotResult(
        exitVelocity, rpm, theta, hoodAngleDeg, turretAngleDeg, target, true);
  }

  /** Piecewise-linear interpolation of hood angle from 5 distance breakpoints. */
  private static double interpolateHoodAngle(double distanceM) {
    double[] dists = {dist1M.get(), dist2M.get(), dist3M.get(), dist4M.get(), dist5M.get()};
    double[] hoods = {hood1Deg.get(), hood2Deg.get(), hood3Deg.get(), hood4Deg.get(), hood5Deg.get()};

    // Below first breakpoint — clamp
    if (distanceM <= dists[0]) return hoods[0];
    // Above last breakpoint — clamp
    if (distanceM >= dists[dists.length - 1]) return hoods[hoods.length - 1];

    // Find segment and interpolate
    for (int i = 0; i < dists.length - 1; i++) {
      if (distanceM <= dists[i + 1]) {
        double t = (distanceM - dists[i]) / (dists[i + 1] - dists[i]);
        return hoods[i] + t * (hoods[i + 1] - hoods[i]);
      }
    }

    return hoods[hoods.length - 1];
  }

  @Override
  public String getName() {
    return "FixedRPM";
  }
}
