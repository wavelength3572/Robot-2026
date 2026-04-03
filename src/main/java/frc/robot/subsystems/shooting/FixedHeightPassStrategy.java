package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Fixed-height parabola strategy for passes. Same vertex-form math as {@link
 * FixedHeightShotStrategy} but tuned for passing: the ball arcs to a fixed peak height and passes
 * through a target point (e.g. above the bump) at a specified height, then continues on the
 * parabola to land beyond it.
 *
 * <p>The {@code target} parameter is the pass-through point — its Z coordinate is the clearance
 * height, and its X/Y define where the ball must fly through. The ball lands past this point.
 *
 * <p>Graceful degradation matches the hub strategy: when the hood must clamp to mechanical limits,
 * velocity is re-derived to still hit the pass-through point. Peak height shifts but the pass still
 * clears the obstacle.
 */
public class FixedHeightPassStrategy implements ShotStrategy {

  private static final double INCHES_TO_METERS = 0.0254;

  // Peak height for passes — lower than hub shots since we're clearing an obstacle, not scoring
  private static final LoggedTunableNumber peakHeightIn =
      new LoggedTunableNumber(
          "Shots/FixedHeightPass/PeakHeightIn",
          Constants.getRobotConfig().getFixedHeightPassPeakHeightIn());

  // RPM limits for passes
  private static final LoggedTunableNumber minRPM =
      new LoggedTunableNumber(
          "Shots/FixedHeightPass/MinRPM", Constants.getRobotConfig().getFixedHeightPassMinRPM());

  private static final LoggedTunableNumber maxRPM =
      new LoggedTunableNumber(
          "Shots/FixedHeightPass/MaxRPM", Constants.getRobotConfig().getFixedHeightPassMaxRPM());

  // Hood floor for passes — prevents near-vertical launches
  private static final LoggedTunableNumber hoodMinFloor =
      new LoggedTunableNumber(
          "Shots/FixedHeightPass/HoodMinDeg",
          Constants.getRobotConfig().getFixedHeightPassHoodMinDeg());

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

    double peakHeightM = peakHeightIn.get() * INCHES_TO_METERS;
    double h0 = config.heightMeters();
    // Pass-through height comes from the target's Z coordinate (set by the coordinator)
    double passThroughHeightM = target.getZ();

    // Apply pass-specific hood floor
    double effectiveHoodMin = Math.max(hoodMinAngleDeg, hoodMinFloor.get());

    // Velocity compensation: single-pass solve static → get TOF → shift target → final solve.
    Translation3d compensatedTarget = target;
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    if (robotSpeed > 0.1) {
      double staticD = Math.hypot(target.getX() - turretX, target.getY() - turretY);
      if (staticD > 0) {
        ShotCalculator.FixedHeightResult staticResult =
            ShotCalculator.solveFixedHeightParabola(
                h0, peakHeightM, passThroughHeightM, staticD, effectiveHoodMin, hoodMaxAngleDeg);
        if (staticResult.achievable()) {
          double staticTheta = Math.toRadians(staticResult.launchAngleDeg());
          double tof =
              ShotCalculator.calculateTimeOfFlight(
                  staticResult.exitVelocityMps(), staticTheta, staticD);
          if (tof > 0 && tof < Double.MAX_VALUE) {
            compensatedTarget =
                ShotCalculator.clampAimOffset(
                    ShotCalculator.predictTargetPos(target, fieldSpeeds, tof), target);
          }
        }
      }
    }

    // Distance to the (possibly shifted) pass-through point — no offset, target IS the
    // pass-through
    double D = Math.hypot(compensatedTarget.getX() - turretX, compensatedTarget.getY() - turretY);

    // Log inputs
    Logger.recordOutput("Shots/FixedHeightPass/DistanceM", D);
    Logger.recordOutput("Shots/FixedHeightPass/DistanceIn", D / INCHES_TO_METERS);
    Logger.recordOutput("Shots/FixedHeightPass/PeakHeightIn", peakHeightIn.get());
    Logger.recordOutput(
        "Shots/FixedHeightPass/PassThroughHeightIn", passThroughHeightM / INCHES_TO_METERS);

    // Solve the parabola
    ShotCalculator.FixedHeightResult result =
        ShotCalculator.solveFixedHeightParabola(
            h0, peakHeightM, passThroughHeightM, D, effectiveHoodMin, hoodMaxAngleDeg);

    if (!result.achievable()) {
      logFailure("%s at D=%.2fm", result.failureReason(), D);
      return new ShotCalculator.ShotResult(0, 0, 0, 0, 0, target, false);
    }

    double thetaDeg = result.launchAngleDeg();
    double theta = Math.toRadians(thetaDeg);
    double hoodAngleDeg = 90.0 - thetaDeg;
    double velocity = result.exitVelocityMps();

    // Log trajectory values
    Logger.recordOutput("Shots/FixedHeightPass/VertexXM", result.vertexXM());
    Logger.recordOutput("Shots/FixedHeightPass/LaunchAngleDeg", thetaDeg);
    Logger.recordOutput("Shots/FixedHeightPass/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("Shots/FixedHeightPass/Clamped", result.clamped());
    if (result.clamped()) {
      Logger.recordOutput(
          "Shots/FixedHeightPass/ActualPeakHeightIn",
          result.actualPeakHeightM() / INCHES_TO_METERS);
    }

    // Calculate turret angle (needed for turret-dependent hood fudge)
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

    // Apply turret-angle-dependent hood fudge to compensate for ball entry angle changes
    double hoodTurretFudge = ShotCalculator.getHoodAngleTurretFudge(turretAngleDeg);
    hoodAngleDeg += hoodTurretFudge;

    // Recalculate the actual launch angle after turret fudge
    theta = Math.toRadians(90.0 - hoodAngleDeg);

    Logger.recordOutput("Shots/FixedHeightPass/HoodTurretFudgeDeg", hoodTurretFudge);

    // Convert to RPM
    double rpm = ShotCalculator.calculateRPMForVelocity(velocity, D);

    Logger.recordOutput("Shots/FixedHeightPass/ExitVelocityMps", velocity);
    Logger.recordOutput("Shots/FixedHeightPass/RPM", rpm);

    // Check RPM limits
    if (rpm < minRPM.get() || rpm > maxRPM.get()) {
      logFailure("RPM %.0f outside [%.0f-%.0f] at D=%.2fm", rpm, minRPM.get(), maxRPM.get(), D);
      return new ShotCalculator.ShotResult(0, 0, 0, hoodAngleDeg, 0, target, false);
    }

    // Status logging
    if (result.clamped()) {
      Logger.recordOutput(
          "Shots/FixedHeightPass/Status",
          String.format(
              "CLAMPED: hood at %s %.0f°, peak %s to %.0fin (tuned %.0fin) at D=%.2fm",
              result.clampedLow() ? "min" : "max",
              hoodAngleDeg,
              result.clampedLow() ? "raised" : "lowered",
              result.actualPeakHeightM() / INCHES_TO_METERS,
              peakHeightIn.get(),
              D));
    } else {
      Logger.recordOutput("Shots/FixedHeightPass/Status", "OK");
    }
    Logger.recordOutput("Shots/FixedHeightPass/Achievable", true);

    return new ShotCalculator.ShotResult(
        velocity, rpm, theta, hoodAngleDeg, turretAngleDeg, compensatedTarget, true);
  }

  private static void logFailure(String format, Object... args) {
    String msg = "FAIL: " + String.format(format, args);
    Logger.recordOutput("Shots/FixedHeightPass/Status", msg);
    Logger.recordOutput("Shots/FixedHeightPass/Achievable", false);
  }

  @Override
  public String getName() {
    return "FixedHeightPass";
  }
}
