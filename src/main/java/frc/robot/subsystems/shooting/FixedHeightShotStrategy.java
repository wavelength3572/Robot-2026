package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Fixed-height parabola shot strategy for hub shots. Defines a unique trajectory using three
 * constraints:
 *
 * <ol>
 *   <li>Launch point (turret position)
 *   <li>Fixed peak height (same for every shot regardless of distance)
 *   <li>Pass-through point on the descent, at a fixed height and horizontal offset from hub center
 * </ol>
 *
 * <p>Delegates core parabola math to {@link ShotCalculator#solveFixedHeightParabola}. When the hood
 * must be clamped to mechanical limits, the solver re-derives velocity so the shot still hits the
 * pass-through point — peak height shifts but the ball still lands.
 */
public class FixedHeightShotStrategy implements ShotStrategy {

  private static final double INCHES_TO_METERS = 0.0254;
  private static final double GRAVITY = 9.81;

  // All spatial tunables in inches
  private static final LoggedTunableNumber peakHeightIn =
      new LoggedTunableNumber("Shots/FixedHeight/PeakHeightIn", 110.0);

  private static final LoggedTunableNumber passThroughHeightIn =
      new LoggedTunableNumber("Shots/FixedHeight/PassThroughHeightIn", 72.0); // 6 feet

  private static final LoggedTunableNumber horizontalOffsetIn =
      new LoggedTunableNumber("Shots/FixedHeight/HorizontalOffsetIn", 10.0);

  // RPM limits
  private static final LoggedTunableNumber minRPM =
      new LoggedTunableNumber("Shots/FixedHeight/MinRPM", 1500.0);

  private static final LoggedTunableNumber maxRPM =
      new LoggedTunableNumber("Shots/FixedHeight/MaxRPM", 4000.0);

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

    // Convert tunable inches to meters
    double peakHeightM = peakHeightIn.get() * INCHES_TO_METERS;
    double passThroughHeightM = passThroughHeightIn.get() * INCHES_TO_METERS;
    double horizontalOffsetM = horizontalOffsetIn.get() * INCHES_TO_METERS;
    double h0 = config.heightMeters();

    // Velocity compensation: single-pass solve static → get TOF → shift target → final solve.
    Translation3d compensatedTarget = target;
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    if (robotSpeed > 0.1) {
      double staticD = Math.hypot(target.getX() - turretX, target.getY() - turretY);
      double staticXp = staticD - horizontalOffsetM;
      if (staticXp > 0) {
        ShotCalculator.FixedHeightResult staticResult =
            ShotCalculator.solveFixedHeightParabola(
                h0, peakHeightM, passThroughHeightM, staticXp, hoodMinAngleDeg, hoodMaxAngleDeg);
        if (staticResult.achievable()) {
          double staticTheta = Math.toRadians(staticResult.launchAngleDeg());
          double tof =
              ShotCalculator.calculateTimeOfFlight(
                  staticResult.exitVelocityMps(), staticTheta, staticXp);
          if (tof > 0 && tof < Double.MAX_VALUE) {
            compensatedTarget =
                ShotCalculator.clampAimOffset(
                    ShotCalculator.predictTargetPos(target, fieldSpeeds, tof), target);
          }
        }
      }
    }

    // Compute pass-through distance for the (possibly shifted) target
    double D = Math.hypot(compensatedTarget.getX() - turretX, compensatedTarget.getY() - turretY);
    double x_p = D - horizontalOffsetM;

    // Log inputs
    Logger.recordOutput("Shots/FixedHeight/DistanceM", D);
    Logger.recordOutput("Shots/FixedHeight/DistanceIn", D / INCHES_TO_METERS);
    Logger.recordOutput("Shots/FixedHeight/TurretHeightIn", h0 / INCHES_TO_METERS);
    Logger.recordOutput("Shots/FixedHeight/PeakHeightIn", peakHeightIn.get());
    Logger.recordOutput("Shots/FixedHeight/PassThroughHeightIn", passThroughHeightIn.get());
    Logger.recordOutput("Shots/FixedHeight/PassThroughDistM", x_p);

    // Solve the parabola
    ShotCalculator.FixedHeightResult result =
        ShotCalculator.solveFixedHeightParabola(
            h0, peakHeightM, passThroughHeightM, x_p, hoodMinAngleDeg, hoodMaxAngleDeg);

    if (!result.achievable()) {
      logFailure("%s at D=%.2fm", result.failureReason(), D);
      return new ShotCalculator.ShotResult(0, 0, 0, 0, 0, target, false);
    }

    double thetaDeg = result.launchAngleDeg();
    double theta = Math.toRadians(thetaDeg);
    double hoodAngleDeg = 90.0 - thetaDeg;
    double velocity = result.exitVelocityMps();

    // Log trajectory values
    Logger.recordOutput("Shots/FixedHeight/VertexXM", result.vertexXM());
    Logger.recordOutput("Shots/FixedHeight/LaunchAngleDeg", thetaDeg);
    Logger.recordOutput("Shots/FixedHeight/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("Shots/FixedHeight/Clamped", result.clamped());
    if (result.clamped()) {
      Logger.recordOutput(
          "Shots/FixedHeight/ActualPeakHeightIn", result.actualPeakHeightM() / INCHES_TO_METERS);
    }

    // Convert to RPM
    double rpm = ShotCalculator.calculateRPMForVelocity(velocity, D);

    Logger.recordOutput("Shots/FixedHeight/ExitVelocityMps", velocity);
    Logger.recordOutput("Shots/FixedHeight/RPM", rpm);

    // Check RPM limits
    if (rpm < minRPM.get() || rpm > maxRPM.get()) {
      logFailure("RPM %.0f outside [%.0f-%.0f] at D=%.2fm", rpm, minRPM.get(), maxRPM.get(), D);
      return new ShotCalculator.ShotResult(0, 0, 0, hoodAngleDeg, 0, target, false);
    }

    // Build the pass-through point in field coordinates for the visualizer
    double dirX = (compensatedTarget.getX() - turretX) / D;
    double dirY = (compensatedTarget.getY() - turretY) / D;
    Translation3d aimTarget =
        new Translation3d(turretX + dirX * x_p, turretY + dirY * x_p, passThroughHeightM);

    // Calculate turret angle to the velocity-compensated hub center
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

    // Status logging
    if (result.clamped()) {
      Logger.recordOutput(
          "Shots/FixedHeight/Status",
          String.format(
              "CLAMPED: hood at %s %.0f°, peak %s to %.0fin (tuned %.0fin) at D=%.2fm",
              result.clampedLow() ? "min" : "max",
              hoodAngleDeg,
              result.clampedLow() ? "raised" : "lowered",
              result.actualPeakHeightM() / INCHES_TO_METERS,
              peakHeightIn.get(),
              D));
    } else {
      Logger.recordOutput("Shots/FixedHeight/Status", "OK");
    }
    Logger.recordOutput("Shots/FixedHeight/Achievable", true);

    return new ShotCalculator.ShotResult(
        velocity, rpm, theta, hoodAngleDeg, turretAngleDeg, aimTarget, true);
  }

  private static void logFailure(String format, Object... args) {
    String msg = "FAIL: " + String.format(format, args);
    Logger.recordOutput("Shots/FixedHeight/Status", msg);
    Logger.recordOutput("Shots/FixedHeight/Achievable", false);
  }

  @Override
  public String getName() {
    return "FixedHeight";
  }
}
