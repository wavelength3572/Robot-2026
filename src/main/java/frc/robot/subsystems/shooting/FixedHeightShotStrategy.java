package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Fixed-height parabola shot strategy. Defines a unique trajectory using three constraints:
 *
 * <ol>
 *   <li>Launch point (turret position)
 *   <li>Fixed peak height (same for every shot regardless of distance)
 *   <li>Pass-through point on the descent, at a fixed height and horizontal offset from hub center
 * </ol>
 *
 * <p>Uses the vertex form of a parabola: y = a(x - h)^2 + k, where (h, k) is the peak. Given the
 * launch point and pass-through point, the vertex x-position and parabola coefficient are uniquely
 * determined. Launch angle and exit velocity follow from the parabola shape and gravity.
 */
public class FixedHeightShotStrategy implements ShotStrategy {

  private static final double GRAVITY = 9.81; // m/s^2
  private static final double INCHES_TO_METERS = 0.0254;

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
    double h0 = config.heightMeters(); // turret height (launch point y)
    double k = peakHeightM; // vertex y (peak height)
    double y_p = passThroughHeightM; // pass-through point y

    // Height validations (don't depend on distance)
    if (k <= h0) {
      logFailure(
          "peak %.1fin <= turret height %.1fin", k / INCHES_TO_METERS, h0 / INCHES_TO_METERS);
      return new ShotCalculator.ShotResult(0, 0, 0, 0, 0, target, false);
    }
    if (k <= y_p) {
      logFailure(
          "peak %.1fin <= pass-through height %.1fin",
          k / INCHES_TO_METERS, y_p / INCHES_TO_METERS);
      return new ShotCalculator.ShotResult(0, 0, 0, 0, 0, target, false);
    }

    // r is constant for fixed heights — doesn't depend on distance
    double r = Math.sqrt((y_p - k) / (h0 - k));

    // Velocity compensation: shift the hub target to account for robot movement
    // during flight, then solve the parabola for the shifted geometry.
    // Uses iterative refinement (like LUT strategy): solve static → get TOF → shift → re-solve.
    Translation3d compensatedTarget = target;
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    if (robotSpeed > 0.1) {
      // Initial solve at static distance for TOF estimate
      double staticD = Math.hypot(target.getX() - turretX, target.getY() - turretY);
      double staticXp = staticD - horizontalOffsetM;
      if (staticXp > 0) {
        double staticH = staticXp / (1.0 + r);
        double staticA = (h0 - k) / (staticH * staticH);
        double staticTanTheta = -2.0 * staticA * staticH;
        double staticTheta = Math.atan(staticTanTheta);
        double staticCos = Math.cos(staticTheta);
        double staticVSq = -GRAVITY / (2.0 * staticA * staticCos * staticCos);
        if (staticVSq > 0) {
          double staticV = Math.sqrt(staticVSq);
          double tof = ShotCalculator.calculateTimeOfFlight(staticV, staticTheta, staticXp);
          if (tof > 0 && tof < Double.MAX_VALUE) {
            compensatedTarget =
                ShotCalculator.clampAimOffset(
                    ShotCalculator.predictTargetPos(target, fieldSpeeds, tof), target);
          }
        }
      }
    }

    // Solve the parabola for the (possibly velocity-compensated) target
    double D = Math.hypot(compensatedTarget.getX() - turretX, compensatedTarget.getY() - turretY);
    double x_p = D - horizontalOffsetM; // pass-through point x

    // Always log inputs so we can debug failures
    Logger.recordOutput("Shots/FixedHeight/DistanceM", D);
    Logger.recordOutput("Shots/FixedHeight/DistanceIn", D / INCHES_TO_METERS);
    Logger.recordOutput("Shots/FixedHeight/TurretHeightIn", h0 / INCHES_TO_METERS);
    Logger.recordOutput("Shots/FixedHeight/PeakHeightIn", peakHeightIn.get());
    Logger.recordOutput("Shots/FixedHeight/PassThroughHeightIn", passThroughHeightIn.get());
    Logger.recordOutput("Shots/FixedHeight/PassThroughDistM", x_p);

    if (x_p <= 0) {
      logFailure(
          "pass-through dist %.2fm <= 0 (too close, D=%.2fm, offset=%.1fin)",
          x_p, D, horizontalOffsetIn.get());
      return new ShotCalculator.ShotResult(0, 0, 0, 0, 0, target, false);
    }

    // Solve for vertex x-position (h) using the ratio method
    double h = x_p / (1.0 + r); // vertex x-position (between launch and pass-through)

    // Parabola coefficient: a = (h0 - k) / h^2
    double a = (h0 - k) / (h * h);

    // Launch angle from slope at x=0: tan(theta) = dy/dx|_{x=0} = 2a(0 - h) = -2ah
    double tanTheta = -2.0 * a * h;
    double theta = Math.atan(tanTheta); // launch angle in radians
    double thetaDeg = Math.toDegrees(theta);

    // Hood angle (mechanical) = 90 - launch angle
    double hoodAngleDeg = 90.0 - thetaDeg;
    boolean clamped = false;

    double velocity;
    double cosTheta;

    if (hoodAngleDeg < hoodMinAngleDeg) {
      // Too close — fixed peak height requires a steeper angle than the hood allows.
      // Clamp hood to min angle and solve for velocity to hit the pass-through point.
      // Peak height will be higher than tuned, but the shot still lands.
      clamped = true;
      hoodAngleDeg = hoodMinAngleDeg;
      thetaDeg = 90.0 - hoodAngleDeg;
      theta = Math.toRadians(thetaDeg);
      cosTheta = Math.cos(theta);
      double sinTheta = Math.sin(theta);
      double tanTh = Math.tan(theta);

      // Projectile: y_p = h0 + x_p*tan(θ) - g*x_p²/(2*v²*cos²θ)
      // Solve for v²: v² = g*x_p² / (2*cos²θ*(h0 + x_p*tan(θ) - y_p))
      double denom = 2.0 * cosTheta * cosTheta * (h0 + x_p * tanTh - y_p);
      if (denom <= 0) {
        logFailure(
            "clamped hood %.0f°: unreachable (denom=%.3f) at D=%.2fm", hoodAngleDeg, denom, D);
        return new ShotCalculator.ShotResult(0, 0, 0, hoodAngleDeg, 0, target, false);
      }
      double vSquared = GRAVITY * x_p * x_p / denom;
      velocity = Math.sqrt(vSquared);

      // Compute actual peak height for logging
      double vy0 = velocity * sinTheta;
      double actualPeakM = h0 + (vy0 * vy0) / (2.0 * GRAVITY);
      Logger.recordOutput("Shots/FixedHeight/ActualPeakHeightIn", actualPeakM / INCHES_TO_METERS);
    } else if (hoodAngleDeg > hoodMaxAngleDeg) {
      // Too far — fixed peak height requires a flatter angle than the hood allows.
      // Clamp hood to max angle and solve for velocity to hit the pass-through point.
      // Peak height will be lower than tuned, but the shot still lands.
      clamped = true;
      hoodAngleDeg = hoodMaxAngleDeg;
      thetaDeg = 90.0 - hoodAngleDeg;
      theta = Math.toRadians(thetaDeg);
      cosTheta = Math.cos(theta);
      double sinTheta = Math.sin(theta);
      double tanTh = Math.tan(theta);

      double denom = 2.0 * cosTheta * cosTheta * (h0 + x_p * tanTh - y_p);
      if (denom <= 0) {
        logFailure(
            "clamped hood %.0f°: unreachable (denom=%.3f) at D=%.2fm", hoodAngleDeg, denom, D);
        return new ShotCalculator.ShotResult(0, 0, 0, hoodAngleDeg, 0, target, false);
      }
      double vSquared = GRAVITY * x_p * x_p / denom;
      velocity = Math.sqrt(vSquared);

      // Compute actual peak height for logging
      double vy0 = velocity * sinTheta;
      double actualPeakM = h0 + (vy0 * vy0) / (2.0 * GRAVITY);
      Logger.recordOutput("Shots/FixedHeight/ActualPeakHeightIn", actualPeakM / INCHES_TO_METERS);
    } else {
      // Normal case — hood angle is achievable, use the vertex-form solution
      cosTheta = Math.cos(theta);
      double vSquared = -GRAVITY / (2.0 * a * cosTheta * cosTheta);
      if (vSquared <= 0) {
        logFailure("invalid velocity (v^2=%.3f) at D=%.2fm, angle=%.1f°", vSquared, D, thetaDeg);
        return new ShotCalculator.ShotResult(0, 0, 0, hoodAngleDeg, 0, target, false);
      }
      velocity = Math.sqrt(vSquared);
    }

    // Always log computed trajectory values
    Logger.recordOutput("Shots/FixedHeight/VertexXM", h);
    Logger.recordOutput("Shots/FixedHeight/LaunchAngleDeg", thetaDeg);
    Logger.recordOutput("Shots/FixedHeight/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("Shots/FixedHeight/Clamped", clamped);

    // Convert to RPM
    double rpm = ShotCalculator.calculateRPMForVelocity(velocity, D);

    // Always log velocity/RPM even if out of range
    Logger.recordOutput("Shots/FixedHeight/ExitVelocityMps", velocity);
    Logger.recordOutput("Shots/FixedHeight/RPM", rpm);

    // Check RPM limits
    if (rpm < minRPM.get() || rpm > maxRPM.get()) {
      logFailure("RPM %.0f outside [%.0f-%.0f] at D=%.2fm", rpm, minRPM.get(), maxRPM.get(), D);
      return new ShotCalculator.ShotResult(0, 0, 0, hoodAngleDeg, 0, target, false);
    }

    // Build the pass-through point in field coordinates for the visualizer.
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

    if (clamped) {
      double vy0 = velocity * Math.sin(theta);
      double actualPeakIn = (h0 + (vy0 * vy0) / (2.0 * GRAVITY)) / INCHES_TO_METERS;
      boolean clampedLow = hoodAngleDeg <= hoodMinAngleDeg + 0.1;
      Logger.recordOutput(
          "Shots/FixedHeight/Status",
          String.format(
              "CLAMPED: hood at %s %.0f°, peak %s to %.0fin (tuned %.0fin) at D=%.2fm",
              clampedLow ? "min" : "max",
              hoodAngleDeg,
              clampedLow ? "raised" : "lowered",
              actualPeakIn,
              peakHeightIn.get(),
              D));
    } else {
      Logger.recordOutput("Shots/FixedHeight/Status", "OK");
    }
    Logger.recordOutput("Shots/FixedHeight/Achievable", true);

    double launchAngleRad = theta;
    return new ShotCalculator.ShotResult(
        velocity, rpm, launchAngleRad, hoodAngleDeg, turretAngleDeg, aimTarget, true);
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
