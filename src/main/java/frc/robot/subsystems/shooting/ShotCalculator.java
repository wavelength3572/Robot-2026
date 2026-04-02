package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ZoneDetector;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class for all shot-related calculations. Converts robot pose + target into shot
 * parameters (exit velocity, launch angle, hood angle, turret angle). Also provides launcher
 * RPM↔velocity conversions and projectile physics utilities.
 */
public final class ShotCalculator {

  // ========== Physical Constants ==========
  private static final double GRAVITY = 9.81; // m/s^2

  // Two-roller launcher model:
  // - Main wheel: 3" diameter (1.5" radius)
  // - Hood rollers: 2" diameter, surface speed is 1/1.41 of main wheel (chained together)
  // Ball exit velocity ≈ average of both surface speeds × slip efficiency
  private static final double MAIN_WHEEL_RADIUS_METERS = 0.0381; // 3" diameter = 1.5" radius
  private static final double HOOD_SURFACE_SPEED_RATIO = 1.0 / 1.41; // hood rolls slower

  // Mechanical roller-to-ball transfer efficiency, interpolated by distance.
  // Derived from calibration data by back-calculating the exit velocity each entry's
  // arc (hood angle) requires to hit the hub, then dividing by the average surface velocity
  // at that RPM. Efficiency varies with distance/RPM — higher RPM (longer range) tends to have
  // more ball compression and slip, reducing effective efficiency.
  //
  // Distance breakpoints come from ZoneDetector's zone boundaries (Shots/Zones/CloseDist,
  // MidDist, FarDist) — single source of truth for both zone classification and efficiency.
  // Efficiency values are tunable via NetworkTables under Shots/SmartLaunch/Efficiency/.
  private static final LoggedTunableNumber efficiencyClose =
      new LoggedTunableNumber(
          "Shots/SmartLaunch/Efficiency/Close",
          Constants.getRobotConfig().getShotEfficiencyClose()); // was .774
  private static final LoggedTunableNumber efficiencyMid =
      new LoggedTunableNumber(
          "Shots/SmartLaunch/Efficiency/Mid",
          Constants.getRobotConfig().getShotEfficiencyMid()); // was .774
  private static final LoggedTunableNumber efficiencyFar =
      new LoggedTunableNumber(
          "Shots/SmartLaunch/Efficiency/Far",
          Constants.getRobotConfig().getShotEfficiencyFar()); // was .75
  private static final LoggedTunableNumber efficiencyCorner =
      new LoggedTunableNumber(
          "Shots/SmartLaunch/Efficiency/Corner",
          Constants.getRobotConfig().getShotEfficiencyCorner());

  // Hood angle fudge factor (degrees), interpolated by distance.
  // Positive values increase hood angle (flatter shot), negative values decrease it (steeper shot).
  // Use this to calibrate the arc to match what you see on screen.
  private static final LoggedTunableNumber hoodAngleFudgeClose =
      new LoggedTunableNumber(
          "Shots/SmartLaunch/HoodAngleFudge/Close",
          Constants.getRobotConfig().getShotHoodAngleFudgeClose());
  private static final LoggedTunableNumber hoodAngleFudgeMid =
      new LoggedTunableNumber(
          "Shots/SmartLaunch/HoodAngleFudge/Mid",
          Constants.getRobotConfig().getShotHoodAngleFudgeMid());
  private static final LoggedTunableNumber hoodAngleFudgeFar =
      new LoggedTunableNumber(
          "Shots/SmartLaunch/HoodAngleFudge/Far",
          Constants.getRobotConfig().getShotHoodAngleFudgeFar());
  private static final LoggedTunableNumber hoodAngleFudgeCorner =
      new LoggedTunableNumber(
          "Shots/SmartLaunch/HoodAngleFudge/Corner",
          Constants.getRobotConfig().getShotHoodAngleFudgeCorner());

  // Velocity limits for safety
  private static final double MIN_EXIT_VELOCITY = 3.0; // m/s
  private static final double MAX_EXIT_VELOCITY = 15.0; // m/s

  // Velocity compensation multipliers for shoot-while-moving (0.0 = no compensation, 1.0 = full)
  private static final LoggedTunableNumber velocityCompX =
      new LoggedTunableNumber(
          "Shots/VelocityComp/X", Constants.getRobotConfig().getShotVelocityCompX());
  private static final LoggedTunableNumber velocityCompY =
      new LoggedTunableNumber(
          "Shots/VelocityComp/Y", Constants.getRobotConfig().getShotVelocityCompY());

  // ========== Launcher RPM Tracking ==========
  // currentLauncherRPM = what the launcher is actually doing right now
  // targetLauncherRPM = what we're commanding the launcher to do (setpoint)
  private static double currentLauncherRPM = 0.0;
  private static double targetLauncherRPM = 0.0;

  /** Turret geometry config (immutable, set once at startup). */
  public record TurretConfig(double heightMeters, double xOffset, double yOffset) {}

  /** Result of a shot calculation. */
  public record ShotResult(
      double exitVelocityMps,
      double launcherRPM, // pre-computed RPM (uses launch efficiency)
      double launchAngleRad,
      double hoodAngleDeg,
      double turretAngleDeg, // robot-relative
      Translation3d aimTarget, // velocity-compensated
      boolean achievable) {

    /** Get the launch angle converted to degrees. */
    public double getLaunchAngleDegrees() {
      return Math.toDegrees(launchAngleRad);
    }
  }

  private ShotCalculator() {} // Static utility class

  // ========== Launcher RPM Methods ==========

  /** Set the current launcher wheel RPM (called by Launcher periodic). */
  public static void setLauncherRPM(double rpm) {
    currentLauncherRPM = rpm;
  }

  /** Set the target launcher wheel RPM (called when commanding a new velocity). */
  public static void setTargetLauncherRPM(double rpm) {
    targetLauncherRPM = rpm;
  }

  /**
   * Get the launch efficiency interpolated by distance. Linearly interpolates between four tunable
   * breakpoints (close, mid, far, corner). Clamps outside the breakpoint range.
   */
  public static double getEfficiency(double distanceMeters) {
    double dClose = ZoneDetector.getZoneBoundaryClose();
    double dMid = ZoneDetector.getZoneBoundaryMid();
    double dFar = ZoneDetector.getZoneBoundaryFar();
    double dCorner = ZoneDetector.getZoneBoundaryCorner();
    double eClose = efficiencyClose.get();
    double eMid = efficiencyMid.get();
    double eFar = efficiencyFar.get();
    double eCorner = efficiencyCorner.get();

    if (distanceMeters <= dClose) {
      return eClose;
    } else if (distanceMeters <= dMid) {
      double t = (distanceMeters - dClose) / (dMid - dClose);
      return eClose + t * (eMid - eClose);
    } else if (distanceMeters <= dFar) {
      double t = (distanceMeters - dMid) / (dFar - dMid);
      return eMid + t * (eFar - eMid);
    } else if (distanceMeters <= dCorner) {
      double t = (distanceMeters - dFar) / (dCorner - dFar);
      return eFar + t * (eCorner - eFar);
    } else {
      return eCorner;
    }
  }

  /** Get the launch efficiency at default mid-range distance. */
  public static double getEfficiency() {
    return getEfficiency(3.0);
  }

  /**
   * Get the hood angle fudge factor (degrees) interpolated by distance. Same zone breakpoints as
   * efficiency. Positive values increase hood angle (flatter shot), negative decreases (steeper).
   */
  public static double getHoodAngleFudge(double distanceMeters) {
    double dClose = ZoneDetector.getZoneBoundaryClose();
    double dMid = ZoneDetector.getZoneBoundaryMid();
    double dFar = ZoneDetector.getZoneBoundaryFar();
    double dCorner = ZoneDetector.getZoneBoundaryCorner();
    double fClose = hoodAngleFudgeClose.get();
    double fMid = hoodAngleFudgeMid.get();
    double fFar = hoodAngleFudgeFar.get();
    double fCorner = hoodAngleFudgeCorner.get();

    if (distanceMeters <= dClose) {
      return fClose;
    } else if (distanceMeters <= dMid) {
      double t = (distanceMeters - dClose) / (dMid - dClose);
      return fClose + t * (fMid - fClose);
    } else if (distanceMeters <= dFar) {
      double t = (distanceMeters - dMid) / (dFar - dMid);
      return fMid + t * (fFar - fMid);
    } else if (distanceMeters <= dCorner) {
      double t = (distanceMeters - dFar) / (dCorner - dFar);
      return fFar + t * (fCorner - fFar);
    } else {
      return fCorner;
    }
  }

  /**
   * Calculate ball exit velocity from a given wheel RPM using the two-roller model.
   *
   * <p>Exit velocity = average of main wheel and hood roller surface speeds × distance-dependent
   * efficiency.
   *
   * @param rpm Launcher wheel RPM
   * @param distanceMeters Horizontal distance to target (used for distance-dependent efficiency)
   */
  public static double calculateExitVelocityFromRPM(double rpm, double distanceMeters) {
    double mainSurfaceVelocity = (rpm * 2.0 * Math.PI * MAIN_WHEEL_RADIUS_METERS) / 60.0;
    double hoodSurfaceVelocity = mainSurfaceVelocity * HOOD_SURFACE_SPEED_RATIO;
    double averageSurfaceVelocity = (mainSurfaceVelocity + hoodSurfaceVelocity) / 2.0;
    return averageSurfaceVelocity * getEfficiency(distanceMeters);
  }

  /** Overload using default mid-range efficiency (for call sites without distance context). */
  public static double calculateExitVelocityFromRPM(double rpm) {
    return calculateExitVelocityFromRPM(rpm, 3.0);
  }

  /** Calculate ball exit velocity from current launcher RPM. */
  public static double calculateExitVelocityFromRPM() {
    return calculateExitVelocityFromRPM(currentLauncherRPM);
  }

  /** Calculate ball exit velocity from target (setpoint) launcher RPM. */
  public static double calculateSetpointExitVelocity() {
    return calculateExitVelocityFromRPM(targetLauncherRPM);
  }

  /**
   * Get what RPM would be needed to achieve a target exit velocity at a given distance.
   *
   * @param targetExitVelocity Desired exit velocity in m/s
   * @param distanceMeters Horizontal distance to target (used for distance-dependent efficiency)
   * @return Required wheel RPM
   */
  public static double calculateRPMForVelocity(double targetExitVelocity, double distanceMeters) {
    double averageSurfaceVelocity = targetExitVelocity / getEfficiency(distanceMeters);
    // Reverse the two-roller average: avg = main × (1 + hoodRatio) / 2
    double mainSurfaceVelocity = averageSurfaceVelocity * 2.0 / (1.0 + HOOD_SURFACE_SPEED_RATIO);
    return (mainSurfaceVelocity * 60.0) / (2.0 * Math.PI * MAIN_WHEEL_RADIUS_METERS);
  }

  /** Overload using default mid-range efficiency. */
  public static double calculateRPMForVelocity(double targetExitVelocity) {
    return calculateRPMForVelocity(targetExitVelocity, 3.0);
  }

  // ========== Physics Utilities ==========

  /**
   * Calculate time of flight for a projectile.
   *
   * @param exitVelocity Exit velocity in m/s
   * @param launchAngle Launch angle in radians
   * @param distance Horizontal distance to target in meters
   * @return Time of flight in seconds
   */
  public static double calculateTimeOfFlight(
      double exitVelocity, double launchAngle, double distance) {
    double horizontalVelocity = exitVelocity * Math.cos(launchAngle);
    if (horizontalVelocity <= 0) return Double.MAX_VALUE;
    return distance / horizontalVelocity;
  }

  /**
   * Predict where the target will be after a given time, accounting for robot motion.
   *
   * @param target Current target position
   * @param fieldSpeeds Field-relative robot speeds
   * @param timeOfFlight Expected time of flight in seconds
   * @return Predicted target position (accounting for robot movement)
   */
  public static Translation3d predictTargetPos(
      Translation3d target, ChassisSpeeds fieldSpeeds, double timeOfFlight) {
    double predictedX =
        target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight * velocityCompX.get();
    double predictedY =
        target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight * velocityCompY.get();
    return new Translation3d(predictedX, predictedY, target.getZ());
  }

  // ========== Field Position Helpers ==========

  /**
   * Pass through a velocity-compensated aim target. The smart launch speed gates already prevent
   * shooting at speeds where the offset would be unreasonable, so no clamping is needed.
   */
  public static Translation3d clampAimOffset(
      Translation3d aimTarget, Translation3d originalTarget) {
    return aimTarget;
  }

  /**
   * Calculate the turret's field position from robot pose, accounting for turret offset.
   *
   * @return double[] {turretFieldX, turretFieldY}
   */
  public static double[] getTurretFieldPosition(
      double robotX, double robotY, double robotHeadingRad, TurretConfig config) {
    double tx =
        robotX
            + (config.xOffset() * Math.cos(robotHeadingRad)
                - config.yOffset() * Math.sin(robotHeadingRad));
    double ty =
        robotY
            + (config.xOffset() * Math.sin(robotHeadingRad)
                + config.yOffset() * Math.cos(robotHeadingRad));
    return new double[] {tx, ty};
  }

  /**
   * Calculate robot-relative turret angle to point at a field target. Accounts for turret offset,
   * wraps to minimize rotation from current position, and clamps to effective limits.
   */
  public static double calculateOutsideTurretAngle(
      double robotX,
      double robotY,
      double robotHeadingDeg,
      double targetX,
      double targetY,
      double currentTurretAngleDeg,
      double effectiveMinDeg,
      double effectiveMaxDeg,
      TurretConfig config) {
    // Get current turret angle
    double currentAngle = currentTurretAngleDeg;

    // Normalize robotOmega to -180 to +180 range
    // This code actually probably doesn't do anything
    // Since our robot omega is always -180 to +180
    double robotOmegaNormalized = robotHeadingDeg % 360;
    if (robotOmegaNormalized > 180) {
      robotOmegaNormalized -= 360;
    } else if (robotOmegaNormalized <= -180) {
      robotOmegaNormalized += 360;
    }
    // Convert robot heading to radians for rotation calculations
    double robotOmegaRad = Math.toRadians(robotHeadingDeg);

    // Calculate turret's actual position on the field
    // The turret offset is in robot-relative coordinates, so we need to rotate it
    // to field coordinates based on the robot's heading
    double turretFieldX =
        robotX
            + (config.xOffset() * Math.cos(robotOmegaRad)
                - config.yOffset() * Math.sin(robotOmegaRad));

    double turretFieldY =
        robotY
            + (config.xOffset() * Math.sin(robotOmegaRad)
                + config.yOffset() * Math.cos(robotOmegaRad));

    // Calculate vector from turret position to target
    double deltaX = targetX - turretFieldX;
    double deltaY = targetY - turretFieldY;

    // Calculate absolute angle to target from field coordinates
    double absoluteAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

    // Calculate relative (desired) angle (turret angle relative to robot heading)
    double relativeAngle = absoluteAngle - robotHeadingDeg;

    // Find the equivalent angle closest to current position
    // Check desiredAngle and its ±360° versions
    double[] candidates = {relativeAngle, relativeAngle + 360.0, relativeAngle - 360.0};

    double bestOutsideAngle =
        relativeAngle; // doesn't matter what we set this to, it's just for initalization
    double smallestMove = 1000000.0; // Set this high do first viable candidate becomes the best.

    for (double candidate : candidates) {
      // Check if this candidate is within physical limits
      if (candidate >= effectiveMinDeg && candidate <= effectiveMaxDeg) {

        double moveDistance = Math.abs(candidate - currentAngle);
        if (moveDistance < smallestMove) {
          smallestMove = moveDistance;
          bestOutsideAngle = candidate;
        }
      }
    }

    // If bestAngle is still out of range, clamp to nearest limit
    // This should actually never come into play since one of the candidates
    // should always work and be within range.
    if (bestOutsideAngle < effectiveMinDeg) {
      bestOutsideAngle = effectiveMinDeg;
    } else if (bestOutsideAngle > effectiveMaxDeg) {
      bestOutsideAngle = effectiveMaxDeg;
    }

    // Log commanded angle and how close we are to a flip
    Logger.recordOutput("Turret/FlipCalc/CommandedAngleDeg", bestOutsideAngle);
    double marginToMin = bestOutsideAngle - effectiveMinDeg;
    double marginToMax = effectiveMaxDeg - bestOutsideAngle;
    Logger.recordOutput("Turret/FlipCalc/FlipMarginDeg", Math.min(marginToMin, marginToMax));

    return bestOutsideAngle;
  }

  // ========== Fixed-Height Parabola Solver ==========

  /**
   * Result from the fixed-height parabola solver. Contains the raw physics output (launch angle,
   * exit velocity) and whether the hood was clamped to a mechanical limit.
   */
  public record FixedHeightResult(
      double launchAngleDeg,
      double exitVelocityMps,
      boolean clamped,
      boolean clampedLow, // true = clamped to hood min (too close), false = hood max (too far)
      double actualPeakHeightM, // actual peak (differs from tuned when clamped)
      double vertexXM, // vertex x-position for diagnostics
      boolean achievable,
      String failureReason) {

    static FixedHeightResult failure(String reason) {
      return new FixedHeightResult(0, 0, false, false, 0, 0, false, reason);
    }
  }

  /**
   * Solve a fixed-height parabola trajectory. Given a launch height, fixed peak height, and a
   * pass-through point (horizontal distance + height), computes the unique launch angle and exit
   * velocity. If the required hood angle exceeds mechanical limits, clamps the hood and re-solves
   * for velocity to still hit the pass-through point.
   *
   * <p>Uses vertex form: y = a(x - h)² + k, where (h, k) is the peak.
   *
   * @param h0 Launch height (turret height) in meters
   * @param k Peak height in meters (must be > h0 and > y_p)
   * @param y_p Pass-through point height in meters
   * @param x_p Pass-through point horizontal distance from turret in meters
   * @param hoodMinAngleDeg Mechanical hood minimum angle
   * @param hoodMaxAngleDeg Mechanical hood maximum angle
   * @return Solver result with launch angle, velocity, and clamping info
   */
  public static FixedHeightResult solveFixedHeightParabola(
      double h0, double k, double y_p, double x_p, double hoodMinAngleDeg, double hoodMaxAngleDeg) {

    // Height validations
    if (k <= h0) {
      return FixedHeightResult.failure(
          String.format("peak %.1fin <= turret height %.1fin", k / 0.0254, h0 / 0.0254));
    }
    if (k <= y_p) {
      return FixedHeightResult.failure(
          String.format("peak %.1fin <= pass-through height %.1fin", k / 0.0254, y_p / 0.0254));
    }
    if (x_p <= 0) {
      return FixedHeightResult.failure(String.format("pass-through dist %.2fm <= 0", x_p));
    }

    // r = ratio that locates the vertex between launch and pass-through
    double r = Math.sqrt((y_p - k) / (h0 - k));

    // Vertex x-position
    double h = x_p / (1.0 + r);

    // Parabola coefficient
    double a = (h0 - k) / (h * h);

    // Launch angle from slope at x=0: tan(theta) = -2ah
    double tanTheta = -2.0 * a * h;
    double theta = Math.atan(tanTheta);
    double thetaDeg = Math.toDegrees(theta);

    // Hood angle (mechanical) = 90 - launch angle
    double hoodAngleDeg = 90.0 - thetaDeg;
    boolean clamped = false;
    boolean clampedLow = false;
    double velocity;
    double cosTheta;
    double actualPeakM = k; // default to tuned peak

    if (hoodAngleDeg < hoodMinAngleDeg) {
      // Too close — fixed peak height requires steeper angle than hood allows.
      // Clamp hood to min, re-solve velocity to still hit pass-through point.
      // Peak will be higher than tuned, but the shot still works.
      clamped = true;
      clampedLow = true;
      hoodAngleDeg = hoodMinAngleDeg;
      thetaDeg = 90.0 - hoodAngleDeg;
      theta = Math.toRadians(thetaDeg);
      cosTheta = Math.cos(theta);
      double sinTheta = Math.sin(theta);
      double tanTh = Math.tan(theta);

      // Projectile: y_p = h0 + x_p*tan(θ) - g*x_p²/(2*v²*cos²θ)
      double denom = 2.0 * cosTheta * cosTheta * (h0 + x_p * tanTh - y_p);
      if (denom <= 0) {
        return FixedHeightResult.failure(
            String.format("clamped hood %.0f°: unreachable (denom=%.3f)", hoodAngleDeg, denom));
      }
      velocity = Math.sqrt(GRAVITY * x_p * x_p / denom);
      double vy0 = velocity * sinTheta;
      actualPeakM = h0 + (vy0 * vy0) / (2.0 * GRAVITY);

    } else if (hoodAngleDeg > hoodMaxAngleDeg) {
      // Too far — fixed peak height requires flatter angle than hood allows.
      // Clamp hood to max, re-solve velocity. Peak will be lower than tuned.
      clamped = true;
      clampedLow = false;
      hoodAngleDeg = hoodMaxAngleDeg;
      thetaDeg = 90.0 - hoodAngleDeg;
      theta = Math.toRadians(thetaDeg);
      cosTheta = Math.cos(theta);
      double sinTheta = Math.sin(theta);
      double tanTh = Math.tan(theta);

      double denom = 2.0 * cosTheta * cosTheta * (h0 + x_p * tanTh - y_p);
      if (denom <= 0) {
        return FixedHeightResult.failure(
            String.format("clamped hood %.0f°: unreachable (denom=%.3f)", hoodAngleDeg, denom));
      }
      velocity = Math.sqrt(GRAVITY * x_p * x_p / denom);
      double vy0 = velocity * sinTheta;
      actualPeakM = h0 + (vy0 * vy0) / (2.0 * GRAVITY);

    } else {
      // Normal case — hood angle is achievable, use vertex-form solution
      cosTheta = Math.cos(theta);
      double vSquared = -GRAVITY / (2.0 * a * cosTheta * cosTheta);
      if (vSquared <= 0) {
        return FixedHeightResult.failure(
            String.format("invalid velocity (v^2=%.3f) at angle=%.1f°", vSquared, thetaDeg));
      }
      velocity = Math.sqrt(vSquared);
    }

    return new FixedHeightResult(
        thetaDeg, velocity, clamped, clampedLow, actualPeakM, h, true, null);
  }

  /**
   * Calculate shot parameters for test mode using manual RPM, hood angle, and turret angle.
   * Projects a target 5m in the aim direction for trajectory visualization.
   */
  public static ShotResult calculateManualShot(
      Pose2d robotPose,
      TurretConfig config,
      double launcherRPM,
      double hoodAngleDeg,
      double turretAngleDeg) {

    double exitVelocity = calculateExitVelocityFromRPM(launcherRPM);
    double launchAngleRad = Math.toRadians(90.0 - hoodAngleDeg);
    double robotHeadingRad = robotPose.getRotation().getRadians();

    double[] turretFieldPos =
        getTurretFieldPosition(robotPose.getX(), robotPose.getY(), robotHeadingRad, config);
    double turretX = turretFieldPos[0];
    double turretY = turretFieldPos[1];

    // Calculate field-relative turret aim direction using TARGET angle
    double turretAimRad = robotHeadingRad + Math.toRadians(turretAngleDeg);

    // Project target 5 meters in aim direction at appropriate height
    double targetDistance = 5.0;
    double targetX = turretX + targetDistance * Math.cos(turretAimRad);
    double targetY = turretY + targetDistance * Math.sin(turretAimRad);

    // Calculate target height based on trajectory
    double timeOfFlight = calculateTimeOfFlight(exitVelocity, launchAngleRad, targetDistance);
    double verticalVelocity = exitVelocity * Math.sin(launchAngleRad);
    double targetZ =
        config.heightMeters()
            + verticalVelocity * timeOfFlight
            - 0.5 * GRAVITY * timeOfFlight * timeOfFlight;
    targetZ = Math.max(0, targetZ);

    Translation3d target = new Translation3d(targetX, targetY, targetZ);

    return new ShotResult(
        exitVelocity,
        calculateRPMForVelocity(exitVelocity),
        launchAngleRad,
        hoodAngleDeg,
        turretAngleDeg,
        target,
        true);
  }
}
