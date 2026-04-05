package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Simple shot strategy: two-point linear interpolation for both RPM and hood angle by distance.
 *
 * <p>No physics model — just two tunable (distance, RPM, hoodAngle) points with linear
 * interpolation between them. Start with close/far RPM identical to test hood-only, then split them
 * if close shots arc too high.
 *
 * <p>Includes velocity compensation for shoot-on-the-move using the same approach as
 * FixedHeightShotStrategy: estimate time-of-flight, shift target by robot velocity.
 */
public class FixedRPMShotStrategy implements ShotStrategy {

  // Close point (nearest shot distance)
  private static final LoggedTunableNumber closeDistM =
      new LoggedTunableNumber("Shots/FixedRPM/CloseDistM", 1.2);
  private static final LoggedTunableNumber closeRPM =
      new LoggedTunableNumber("Shots/FixedRPM/CloseRPM", 3000.0);
  private static final LoggedTunableNumber closeHoodDeg =
      new LoggedTunableNumber("Shots/FixedRPM/CloseHoodDeg", 13.0);

  // Far point (farthest shot distance)
  private static final LoggedTunableNumber farDistM =
      new LoggedTunableNumber("Shots/FixedRPM/FarDistM", 5.5);
  private static final LoggedTunableNumber farRPM =
      new LoggedTunableNumber("Shots/FixedRPM/FarRPM", 3000.0);
  private static final LoggedTunableNumber farHoodDeg =
      new LoggedTunableNumber("Shots/FixedRPM/FarHoodDeg", 42.0);

  // Motivator and spindexer RPM (also linear close/far)
  private static final LoggedTunableNumber closeMotivatorRPM =
      new LoggedTunableNumber(
          "Shots/FixedRPM/CloseMotivatorRPM",
          Constants.getRobotConfig().getHubShotMotivatorRPM());
  private static final LoggedTunableNumber farMotivatorRPM =
      new LoggedTunableNumber(
          "Shots/FixedRPM/FarMotivatorRPM",
          Constants.getRobotConfig().getHubShotMotivatorRPM());
  private static final LoggedTunableNumber closeSpindexerRPM =
      new LoggedTunableNumber(
          "Shots/FixedRPM/CloseSpindexerRPM",
          Constants.getRobotConfig().getSpindexerCloseRPM());
  private static final LoggedTunableNumber farSpindexerRPM =
      new LoggedTunableNumber(
          "Shots/FixedRPM/FarSpindexerRPM", Constants.getRobotConfig().getSpindexerFarRPM());

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

    // Velocity compensation for shoot-on-the-move:
    // Static solve → estimate TOF → shift target → final solve
    Translation3d compensatedTarget = target;
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    if (robotSpeed > 0.1) {
      double staticD = Math.hypot(target.getX() - turretX, target.getY() - turretY);
      double staticHoodDeg = lerp(staticD, closeHoodDeg.get(), farHoodDeg.get());
      staticHoodDeg = Math.max(hoodMinAngleDeg, Math.min(hoodMaxAngleDeg, staticHoodDeg));
      double staticTheta = Math.toRadians(90.0 - staticHoodDeg);
      double staticRPM = lerp(staticD, closeRPM.get(), farRPM.get());
      double staticExitVelocity =
          ShotCalculator.calculateExitVelocityFromRPM(staticRPM, staticD);
      double tof =
          ShotCalculator.calculateTimeOfFlight(staticExitVelocity, staticTheta, staticD);

      if (tof > 0 && tof < Double.MAX_VALUE) {
        compensatedTarget =
            ShotCalculator.clampAimOffset(
                ShotCalculator.predictTargetPos(target, fieldSpeeds, tof), target);
      }
    }

    // Compute distance to the (possibly shifted) target
    double D = Math.hypot(compensatedTarget.getX() - turretX, compensatedTarget.getY() - turretY);

    // Linear interpolation between close and far points
    double rpm = lerp(D, closeRPM.get(), farRPM.get());
    double hoodAngleDeg = lerp(D, closeHoodDeg.get(), farHoodDeg.get());

    // Clamp hood to mechanical limits
    hoodAngleDeg = Math.max(hoodMinAngleDeg, Math.min(hoodMaxAngleDeg, hoodAngleDeg));

    // Calculate turret angle to the velocity-compensated target
    double turretAngleDeg =
        ShotCalculator.calculateOutsideTurretAngle(
            robotPose.getX(),
            robotPose.getY(),
            robotPose.getRotation().getDegrees(),
            compensatedTarget.getX(),
            compensatedTarget.getY(),
            currentTurretAngleDeg,
            effectiveMinDeg,
            effectiveMaxDeg,
            config);

    double motivator = lerp(D, closeMotivatorRPM.get(), farMotivatorRPM.get());
    double spindexer = lerp(D, closeSpindexerRPM.get(), farSpindexerRPM.get());
    double theta = Math.toRadians(90.0 - hoodAngleDeg);
    double exitVelocity = ShotCalculator.calculateExitVelocityFromRPM(rpm, D);

    // Log
    Logger.recordOutput("Shots/FixedRPM/DistanceM", D);
    Logger.recordOutput("Shots/FixedRPM/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("Shots/FixedRPM/RPM", rpm);
    Logger.recordOutput("Shots/FixedRPM/MotivatorRPM", motivator);
    Logger.recordOutput("Shots/FixedRPM/SpindexerRPM", spindexer);
    Logger.recordOutput("Shots/FixedRPM/ExitVelocityMps", exitVelocity);
    Logger.recordOutput("Shots/FixedRPM/RobotSpeedMps", robotSpeed);

    return new ShotCalculator.ShotResult(
        exitVelocity, rpm, theta, hoodAngleDeg, turretAngleDeg,
        motivator, spindexer, compensatedTarget, true);
  }

  /**
   * Linear interpolation between close and far values based on distance. Clamps at both ends (no
   * extrapolation).
   */
  private double lerp(double distanceM, double closeVal, double farVal) {
    double dClose = closeDistM.get();
    double dFar = farDistM.get();
    if (distanceM <= dClose) return closeVal;
    if (distanceM >= dFar) return farVal;
    double t = (distanceM - dClose) / (dFar - dClose);
    return closeVal + t * (farVal - closeVal);
  }

  @Override
  public String getName() {
    return "FixedRPM";
  }
}
