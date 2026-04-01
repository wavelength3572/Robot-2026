package frc.robot.subsystems.shooting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.hood.TrajectoryOptimizer;
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
  // Derived from LUT calibration data by back-calculating the exit velocity each LUT entry's
  // own arc (hood angle) requires to hit the hub, then dividing by the average surface velocity
  // at that RPM. Efficiency varies with distance/RPM — higher RPM (longer range) tends to have
  // more ball compression and slip, reducing effective efficiency.
  //
  // Distance breakpoints come from ZoneDetector's zone boundaries (Shots/Zones/CloseDist,
  // MidDist, FarDist) — single source of truth for both zone classification and efficiency.
  // Efficiency values are tunable via NetworkTables under Shots/SmartLaunch/Efficiency/.
  private static final LoggedTunableNumber efficiencyClose =
      new LoggedTunableNumber("Shots/SmartLaunch/Efficiency/Close", 0.7); // was .774
  private static final LoggedTunableNumber efficiencyMid =
      new LoggedTunableNumber("Shots/SmartLaunch/Efficiency/Mid", 0.7); // was .774
  private static final LoggedTunableNumber efficiencyFar =
      new LoggedTunableNumber("Shots/SmartLaunch/Efficiency/Far", 0.7); // was .75
  private static final LoggedTunableNumber efficiencyCorner =
      new LoggedTunableNumber("Shots/SmartLaunch/Efficiency/Corner", 0.69);

  // Velocity limits for safety
  private static final double MIN_EXIT_VELOCITY = 3.0; // m/s
  private static final double MAX_EXIT_VELOCITY = 15.0; // m/s

  // Velocity compensation multipliers for shoot-while-moving (0.0 = no compensation, 1.0 = full)
  private static final LoggedTunableNumber velocityCompX =
      new LoggedTunableNumber("Shots/VelocityComp/X", 1.0);
  private static final LoggedTunableNumber velocityCompY =
      new LoggedTunableNumber("Shots/VelocityComp/Y", 1.0);

  // When true, use single-pass horizontal TOF for velocity compensation (stable, no iteration).
  // When false, use the legacy 3-iteration total TOF refinement loop (kept for comparison/testing).
  private static final boolean USE_HORIZONTAL_TOF_DEFAULT = true;

  static {
    SmartDashboard.putBoolean("Shots/VelocityComp/UseHorizontalTOF", USE_HORIZONTAL_TOF_DEFAULT);
  }

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
   * Get the horizontal distance from robot to target.
   *
   * @param robot Current robot pose
   * @param target Target position (3D)
   * @return Distance in meters
   */
  public static double getDistanceToTarget(Pose2d robot, Translation3d target) {
    return robot.getTranslation().getDistance(target.toTranslation2d());
  }

  /**
   * Calculate the optimal launch angle to hit a target given a fixed exit velocity.
   *
   * @param robot Current robot pose
   * @param velocity Exit velocity in m/s
   * @param target Target position (3D)
   * @param turretHeightMeters Height of the turret above ground in meters
   * @return Launch angle in radians (from horizontal)
   */
  public static double calculateAngleFromVelocity(
      Pose2d robot, double velocity, Translation3d target, double turretHeightMeters) {
    double xDist = getDistanceToTarget(robot, target);
    double yDist = target.getZ() - turretHeightMeters;

    double v2 = velocity * velocity;
    double v4 = v2 * v2;
    double discriminant = v4 - GRAVITY * (GRAVITY * xDist * xDist + 2 * yDist * v2);

    if (discriminant < 0) {
      return Math.PI / 4; // 45 degrees fallback
    }

    return Math.atan((v2 + Math.sqrt(discriminant)) / (GRAVITY * xDist));
  }

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
   * Calculate the azimuth angle (turret rotation) to point at a target.
   *
   * @param robot Current robot pose
   * @param target Target position (3D)
   * @return Azimuth angle in radians (relative to robot heading, 0-2π)
   */
  public static double calculateAzimuthAngle(Pose2d robot, Translation3d target) {
    Translation2d direction = target.toTranslation2d().minus(robot.getTranslation());
    double fieldAngle = direction.getAngle().getRadians();
    double robotAngle = robot.getRotation().getRadians();
    return MathUtil.inputModulus(fieldAngle - robotAngle, 0, 2 * Math.PI);
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

  // ========== Shot Calculations ==========

  /**
   * Calculate shot parameters for the hub (uses TrajectoryOptimizer for optimal RPM/angle).
   * Includes velocity compensation for robot movement.
   */
  public static ShotResult calculateHubShot(
      Pose2d robotPose,
      ChassisSpeeds fieldSpeeds,
      Translation3d hubTarget,
      TurretConfig config,
      double currentTurretAngleDeg,
      double effectiveMinDeg,
      double effectiveMaxDeg,
      double hoodMinAngleDeg,
      double hoodMaxAngleDeg) {

    double robotHeadingRad = robotPose.getRotation().getRadians();
    double[] turretFieldPos =
        getTurretFieldPosition(robotPose.getX(), robotPose.getY(), robotHeadingRad, config);
    double turretX = turretFieldPos[0];
    double turretY = turretFieldPos[1];
    Translation3d turretPos = new Translation3d(turretX, turretY, config.heightMeters());

    // Diagnostic: log exact optimizer inputs so orientation-dependence can be verified
    double turretToTargetD =
        Math.sqrt(
            Math.pow(hubTarget.getX() - turretX, 2) + Math.pow(hubTarget.getY() - turretY, 2));
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    Logger.recordOutput("SmartLaunch/Debug/RobotHeadingDeg", Math.toDegrees(robotHeadingRad));
    Logger.recordOutput("SmartLaunch/Debug/TurretFieldX", turretX);
    Logger.recordOutput("SmartLaunch/Debug/TurretFieldY", turretY);
    Logger.recordOutput("SmartLaunch/Debug/TurretToTargetD", turretToTargetD);
    Logger.recordOutput("SmartLaunch/Debug/RobotSpeedMps", robotSpeed);
    Logger.recordOutput("SmartLaunch/Debug/VelocityCompActive", robotSpeed > 0.1);

    // Velocity compensation: adjust aim point to counteract robot movement during flight.
    // Only apply when the initial shot is achievable — if the optimizer fails (returns
    // exitVelocityMps=0), the ToF calculation produces Double.MAX_VALUE and predictTargetPos
    // generates infinity coordinates, causing the turret to snap to a garbage angle.
    Translation3d aimTarget = hubTarget;
    if (robotSpeed > 0.1) {
      TrajectoryOptimizer.OptimalShot initialShot =
          TrajectoryOptimizer.calculateOptimalShotWithFallback(
              turretPos, hubTarget, hoodMinAngleDeg, hoodMaxAngleDeg);

      if (initialShot.achievable) {
        double distanceToTarget =
            Math.sqrt(
                Math.pow(hubTarget.getX() - turretX, 2) + Math.pow(hubTarget.getY() - turretY, 2));

        if (SmartDashboard.getBoolean(
            "Shots/VelocityComp/UseHorizontalTOF", USE_HORIZONTAL_TOF_DEFAULT)) {
          // Single-pass horizontal TOF: lateral drift depends on horizontal flight time,
          // not total arc time. Stable and physically correct.
          double vx =
              initialShot.exitVelocityMps * Math.cos(Math.toRadians(initialShot.launchAngleDeg));
          double horizontalTof = (vx > 0.1) ? distanceToTarget / vx : 0.0;
          aimTarget = predictTargetPos(hubTarget, fieldSpeeds, horizontalTof);
        } else {
          // Legacy 3-iteration refinement loop using total TOF. Kept for comparison/testing
          // via the Shots/VelocityComp/UseHorizontalTOF dashboard toggle. Can diverge at
          // steep launch angles where total TOF is much longer than horizontal flight time.
          double tof =
              calculateTimeOfFlight(
                  initialShot.exitVelocityMps,
                  Math.toRadians(initialShot.launchAngleDeg),
                  distanceToTarget);

          for (int i = 0; i < 3; i++) {
            Translation3d candidate =
                clampAimOffset(predictTargetPos(hubTarget, fieldSpeeds, tof), hubTarget);
            double aimDistance =
                Math.sqrt(
                    Math.pow(candidate.getX() - turretX, 2)
                        + Math.pow(candidate.getY() - turretY, 2));
            TrajectoryOptimizer.OptimalShot refinedShot =
                TrajectoryOptimizer.calculateOptimalShotWithFallback(
                    turretPos, candidate, hoodMinAngleDeg, hoodMaxAngleDeg);
            if (!refinedShot.achievable) {
              break;
            }
            aimTarget = candidate;
            tof =
                calculateTimeOfFlight(
                    refinedShot.exitVelocityMps,
                    Math.toRadians(refinedShot.launchAngleDeg),
                    aimDistance);
          }
        }
      }
    }

    TrajectoryOptimizer.OptimalShot optimalShot =
        TrajectoryOptimizer.calculateOptimalShotWithFallback(
            turretPos, aimTarget, hoodMinAngleDeg, hoodMaxAngleDeg);

    double turretAngleDeg =
        calculateOutsideTurretAngle(
            robotPose.getX(),
            robotPose.getY(),
            robotPose.getRotation().getDegrees(),
            aimTarget.getX(),
            aimTarget.getY(),
            currentTurretAngleDeg,
            effectiveMinDeg,
            effectiveMaxDeg,
            config);

    return new ShotResult(
        optimalShot.exitVelocityMps,
        optimalShot.rpm,
        Math.toRadians(optimalShot.launchAngleDeg),
        optimalShot.hoodAngleDeg,
        turretAngleDeg,
        aimTarget,
        optimalShot.achievable);
  }

  /**
   * Calculate shot parameters for the hub with procedural fallback. Tries normal parametric first,
   * then fixed-hood at hood limits, then relaxed constraints (wider RPM/peak height). Use this when
   * you want the best achievable shot without falling through to the LUT.
   *
   * <p>Same signature and velocity compensation logic as {@link #calculateHubShot}, but uses {@link
   * TrajectoryOptimizer#calculateOptimalShotWithFallback} instead of the strict solver.
   */
  public static ShotResult calculateHubShotWithFallback(
      Pose2d robotPose,
      ChassisSpeeds fieldSpeeds,
      Translation3d hubTarget,
      ShotCalculator.TurretConfig config,
      double currentTurretAngleDeg,
      double effectiveMinDeg,
      double effectiveMaxDeg,
      double hoodMinAngleDeg,
      double hoodMaxAngleDeg) {

    double robotHeadingRad = robotPose.getRotation().getRadians();
    double[] turretFieldPos =
        getTurretFieldPosition(robotPose.getX(), robotPose.getY(), robotHeadingRad, config);
    double turretX = turretFieldPos[0];
    double turretY = turretFieldPos[1];
    Translation3d turretPos = new Translation3d(turretX, turretY, config.heightMeters());

    // Velocity compensation: single-pass horizontal TOF (same logic as calculateHubShot)
    Translation3d aimTarget = hubTarget;
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    // Diagnostic logging shared with calculateHubShot — values are identical for same inputs
    double turretToTargetD =
        Math.sqrt(
            Math.pow(hubTarget.getX() - turretX, 2) + Math.pow(hubTarget.getY() - turretY, 2));
    Logger.recordOutput("SmartLaunch/Debug/RobotHeadingDeg", Math.toDegrees(robotHeadingRad));
    Logger.recordOutput("SmartLaunch/Debug/TurretFieldX", turretX);
    Logger.recordOutput("SmartLaunch/Debug/TurretFieldY", turretY);
    Logger.recordOutput("SmartLaunch/Debug/TurretToTargetD", turretToTargetD);
    Logger.recordOutput("SmartLaunch/Debug/RobotSpeedMps", robotSpeed);
    Logger.recordOutput("SmartLaunch/Debug/VelocityCompActive", robotSpeed > 0.1);
    if (robotSpeed > 0.1) {
      TrajectoryOptimizer.OptimalShot initialShot =
          TrajectoryOptimizer.calculateOptimalShotWithFallback(
              turretPos, hubTarget, hoodMinAngleDeg, hoodMaxAngleDeg);

      if (initialShot.achievable) {
        double distanceToTarget =
            Math.sqrt(
                Math.pow(hubTarget.getX() - turretX, 2) + Math.pow(hubTarget.getY() - turretY, 2));

        if (SmartDashboard.getBoolean(
            "Shots/VelocityComp/UseHorizontalTOF", USE_HORIZONTAL_TOF_DEFAULT)) {
          double vx =
              initialShot.exitVelocityMps * Math.cos(Math.toRadians(initialShot.launchAngleDeg));
          double horizontalTof = (vx > 0.1) ? distanceToTarget / vx : 0.0;
          aimTarget = predictTargetPos(hubTarget, fieldSpeeds, horizontalTof);
        } else {
          double tof =
              calculateTimeOfFlight(
                  initialShot.exitVelocityMps,
                  Math.toRadians(initialShot.launchAngleDeg),
                  distanceToTarget);

          for (int i = 0; i < 3; i++) {
            Translation3d candidate =
                clampAimOffset(predictTargetPos(hubTarget, fieldSpeeds, tof), hubTarget);
            double aimDistance =
                Math.sqrt(
                    Math.pow(candidate.getX() - turretX, 2)
                        + Math.pow(candidate.getY() - turretY, 2));
            TrajectoryOptimizer.OptimalShot refinedShot =
                TrajectoryOptimizer.calculateOptimalShotWithFallback(
                    turretPos, candidate, hoodMinAngleDeg, hoodMaxAngleDeg);
            if (!refinedShot.achievable) {
              break;
            }
            aimTarget = candidate;
            tof =
                calculateTimeOfFlight(
                    refinedShot.exitVelocityMps,
                    Math.toRadians(refinedShot.launchAngleDeg),
                    aimDistance);
          }
        }
      }
    }

    TrajectoryOptimizer.OptimalShot optimalShot =
        TrajectoryOptimizer.calculateOptimalShotWithFallback(
            turretPos, aimTarget, hoodMinAngleDeg, hoodMaxAngleDeg);

    double turretAngleDeg =
        calculateOutsideTurretAngle(
            robotPose.getX(),
            robotPose.getY(),
            robotPose.getRotation().getDegrees(),
            aimTarget.getX(),
            aimTarget.getY(),
            currentTurretAngleDeg,
            effectiveMinDeg,
            effectiveMaxDeg,
            config);

    return new ShotResult(
        optimalShot.exitVelocityMps,
        optimalShot.rpm,
        Math.toRadians(optimalShot.launchAngleDeg),
        optimalShot.hoodAngleDeg,
        turretAngleDeg,
        aimTarget,
        optimalShot.achievable);
  }

  /**
   * Calculate shot parameters for the hub using a fixed (locked) hood angle. Used in trench mode
   * where the hood is clamped to a single value and we solve only for RPM. Includes velocity
   * compensation for robot movement.
   *
   * @param fixedHoodAngleDeg The locked hood angle (e.g., trench max of 18°)
   */
  public static ShotResult calculateTrenchHubShot(
      Pose2d robotPose,
      ChassisSpeeds fieldSpeeds,
      Translation3d hubTarget,
      TurretConfig config,
      double currentTurretAngleDeg,
      double effectiveMinDeg,
      double effectiveMaxDeg,
      double fixedHoodAngleDeg) {

    double robotHeadingRad = robotPose.getRotation().getRadians();
    double[] turretFieldPos =
        getTurretFieldPosition(robotPose.getX(), robotPose.getY(), robotHeadingRad, config);
    double turretX = turretFieldPos[0];
    double turretY = turretFieldPos[1];
    Translation3d turretPos = new Translation3d(turretX, turretY, config.heightMeters());

    // Single-pass velocity compensation using horizontal TOF (distance / horizontal velocity).
    // The fixed 18° hood creates a steep trajectory where total TOF is much longer than
    // horizontal flight time, making total-TOF-based compensation overshoot.
    // Horizontal TOF is stable and physically more correct — lateral drift depends on horizontal
    // time, not the total time the ball spends going up and coming down.
    Translation3d aimTarget = hubTarget;
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    if (robotSpeed > 0.1) {
      TrajectoryOptimizer.OptimalShot initialShot =
          TrajectoryOptimizer.calculateFixedHoodShot(turretPos, hubTarget, fixedHoodAngleDeg);
      if (initialShot.achievable) {
        double distanceToTarget =
            Math.sqrt(
                Math.pow(hubTarget.getX() - turretX, 2) + Math.pow(hubTarget.getY() - turretY, 2));
        // Use horizontal distance / horizontal velocity for a more stable estimate
        // than total TOF (which is inflated by the steep launch angle).
        double vx =
            initialShot.exitVelocityMps * Math.cos(Math.toRadians(initialShot.launchAngleDeg));
        double horizontalTof = (vx > 0.1) ? distanceToTarget / vx : 0.0;
        aimTarget = predictTargetPos(hubTarget, fieldSpeeds, horizontalTof);
      }
    }

    TrajectoryOptimizer.OptimalShot optimalShot =
        TrajectoryOptimizer.calculateFixedHoodShot(turretPos, aimTarget, fixedHoodAngleDeg);

    double turretAngleDeg =
        calculateOutsideTurretAngle(
            robotPose.getX(),
            robotPose.getY(),
            robotPose.getRotation().getDegrees(),
            aimTarget.getX(),
            aimTarget.getY(),
            currentTurretAngleDeg,
            effectiveMinDeg,
            effectiveMaxDeg,
            config);

    return new ShotResult(
        optimalShot.exitVelocityMps,
        optimalShot.rpm,
        Math.toRadians(optimalShot.launchAngleDeg),
        optimalShot.hoodAngleDeg,
        turretAngleDeg,
        aimTarget,
        optimalShot.achievable);
  }

  /**
   * Calculate shot parameters for a pass to a ground-level target. Uses simple projectile physics
   * with a fixed launch angle instead of the hub-specific TrajectoryOptimizer.
   */
  public static ShotResult calculatePassShot(
      Pose2d robotPose,
      ChassisSpeeds fieldSpeeds,
      Translation3d passTarget,
      TurretConfig config,
      double launchAngleDeg,
      double currentTurretAngleDeg,
      double effectiveMinDeg,
      double effectiveMaxDeg) {

    double robotHeadingRad = robotPose.getRotation().getRadians();
    double[] turretFieldPos =
        getTurretFieldPosition(robotPose.getX(), robotPose.getY(), robotHeadingRad, config);
    double turretX = turretFieldPos[0];
    double turretY = turretFieldPos[1];

    double launchAngleRad = Math.toRadians(launchAngleDeg);
    double h = config.heightMeters() - passTarget.getZ();
    double horizontalDist =
        Math.sqrt(
            Math.pow(passTarget.getX() - turretX, 2) + Math.pow(passTarget.getY() - turretY, 2));

    // Solve for velocity: v^2 = g*D^2 / (2*cos^2(theta) * (D*tan(theta) + h))
    double cosTheta = Math.cos(launchAngleRad);
    double tanTheta = Math.tan(launchAngleRad);
    double denominator = 2.0 * cosTheta * cosTheta * (horizontalDist * tanTheta + h);

    double exitVelocity;
    if (denominator > 0) {
      exitVelocity = Math.sqrt(GRAVITY * horizontalDist * horizontalDist / denominator);
    } else {
      exitVelocity = 10.0; // fallback
    }

    // Velocity compensation for robot movement (3-iteration refinement using total TOF).
    // Pass shots recalculate exit velocity at each shifted distance, so iteration is needed.
    Translation3d aimTarget = passTarget;
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    if (robotSpeed > 0.1) {
      double tof = calculateTimeOfFlight(exitVelocity, launchAngleRad, horizontalDist);
      for (int i = 0; i < 3; i++) {
        aimTarget = clampAimOffset(predictTargetPos(passTarget, fieldSpeeds, tof), passTarget);
        double aimDist =
            Math.sqrt(
                Math.pow(aimTarget.getX() - turretX, 2) + Math.pow(aimTarget.getY() - turretY, 2));
        double aimH = config.heightMeters() - aimTarget.getZ();
        double aimDenom = 2.0 * cosTheta * cosTheta * (aimDist * tanTheta + aimH);
        if (aimDenom > 0) {
          exitVelocity = Math.sqrt(GRAVITY * aimDist * aimDist / aimDenom);
        }
        tof = calculateTimeOfFlight(exitVelocity, launchAngleRad, aimDist);
      }
    }

    double turretAngleDeg =
        calculateOutsideTurretAngle(
            robotPose.getX(),
            robotPose.getY(),
            robotPose.getRotation().getDegrees(),
            aimTarget.getX(),
            aimTarget.getY(),
            currentTurretAngleDeg,
            effectiveMinDeg,
            effectiveMaxDeg,
            config);

    double finalDist =
        Math.sqrt(
            Math.pow(aimTarget.getX() - turretX, 2) + Math.pow(aimTarget.getY() - turretY, 2));

    return new ShotResult(
        exitVelocity,
        calculateRPMForVelocity(exitVelocity, finalDist),
        launchAngleRad,
        90.0 - launchAngleDeg, // convert launch angle to hood angle
        turretAngleDeg,
        aimTarget,
        true);
  }

  /**
   * Calculate pass shot using two-point trajectory math (same approach as TrajectoryOptimizer).
   * Solves for the unique parabola through a clearance point and the landing target, then validates
   * hood angle, RPM, and peak height.
   *
   * @param robotPose Current robot pose
   * @param fieldSpeeds Field-relative chassis speeds
   * @param passTarget Landing target (ground level)
   * @param config Turret geometry config
   * @param constraintX Horizontal distance along shot line to clearance point (meters)
   * @param constraintH Required height above ground at clearance point (meters)
   * @param maxPeakHeightM Maximum allowed peak height (meters)
   * @param currentTurretAngleDeg Current turret angle for wrapping
   * @param effectiveMinDeg Turret min limit
   * @param effectiveMaxDeg Turret max limit
   * @param hoodMinAngleDeg Hood mechanical min (e.g. 16)
   * @param hoodMaxAngleDeg Hood mechanical max (e.g. 46)
   */
  public static ShotResult calculatePassShotTwoPoint(
      Pose2d robotPose,
      ChassisSpeeds fieldSpeeds,
      Translation3d passTarget,
      TurretConfig config,
      double constraintX,
      double constraintH,
      double maxPeakHeightM,
      double currentTurretAngleDeg,
      double effectiveMinDeg,
      double effectiveMaxDeg,
      double hoodMinAngleDeg,
      double hoodMaxAngleDeg,
      double rpmPerDegCompensation,
      double maxRpmCompensation) {

    double robotHeadingRad = robotPose.getRotation().getRadians();
    double[] turretFieldPos =
        getTurretFieldPosition(robotPose.getX(), robotPose.getY(), robotHeadingRad, config);
    double turretX = turretFieldPos[0];
    double turretY = turretFieldPos[1];

    double horizontalDist =
        Math.sqrt(
            Math.pow(passTarget.getX() - turretX, 2) + Math.pow(passTarget.getY() - turretY, 2));

    // Two constraint points relative to turret launch height:
    //   Point 1 (clearance): (x1, y1) where y1 = constraintH - turretHeight
    //   Point 2 (landing):   (x2, y2) where y2 = passTarget.getZ() - turretHeight
    double x1 = constraintX;
    double y1 = constraintH - config.heightMeters();
    double x2 = horizontalDist;
    double y2 = passTarget.getZ() - config.heightMeters();

    // Log constraint points for debugging
    Logger.recordOutput("SmartLaunch/Pass/TwoPoint/ConstraintX", x1);
    Logger.recordOutput("SmartLaunch/Pass/TwoPoint/ConstraintH", constraintH);
    Logger.recordOutput("SmartLaunch/Pass/TwoPoint/HorizontalDist", horizontalDist);

    // Solve for launch angle: tanTheta = (y1*x2^2 - y2*x1^2) / (x1*x2*(x2 - x1))
    // Reject when x2 is too close to x1 — the solver becomes near-singular and produces
    // extreme launch angles. Require at least 30% of horizontal distance as separation.
    if (Math.abs(x2 - x1) < horizontalDist * 0.3) {
      Logger.recordOutput(
          "SmartLaunch/Pass/TwoPoint/RejectReason",
          String.format("x2-x1 gap %.2fm < 30%% of %.2fm", Math.abs(x2 - x1), horizontalDist));
      return unachievablePassResult(
          robotPose, passTarget, config, currentTurretAngleDeg, effectiveMinDeg, effectiveMaxDeg);
    }
    double denominator = x1 * x2 * (x2 - x1);
    if (Math.abs(denominator) < 0.001) {
      return unachievablePassResult(
          robotPose, passTarget, config, currentTurretAngleDeg, effectiveMinDeg, effectiveMaxDeg);
    }

    double tanTheta = (y1 * x2 * x2 - y2 * x1 * x1) / denominator;
    double theta = Math.atan(tanTheta);
    double thetaDeg = Math.toDegrees(theta);

    // Hard cap: reject launch angles above 75° to prevent near-vertical shots
    if (thetaDeg > 75.0) {
      Logger.recordOutput(
          "SmartLaunch/Pass/TwoPoint/RejectReason",
          String.format("Launch angle %.1f° > 75° cap", thetaDeg));
      return unachievablePassResult(
          robotPose, passTarget, config, currentTurretAngleDeg, effectiveMinDeg, effectiveMaxDeg);
    }

    double hoodAngleDeg = 90.0 - thetaDeg;

    // Clamp hood to mechanical limits and compensate RPM — passes always fire
    double rawHoodAngleDeg = hoodAngleDeg;
    hoodAngleDeg = Math.max(hoodMinAngleDeg, Math.min(hoodMaxAngleDeg, hoodAngleDeg));
    double hoodDeltaDeg = rawHoodAngleDeg - hoodAngleDeg;
    if (Math.abs(hoodDeltaDeg) > 0.01) {
      Logger.recordOutput(
          "SmartLaunch/Pass/TwoPoint/HoodClamped",
          String.format("%.1f -> %.1f (delta %.1f)", rawHoodAngleDeg, hoodAngleDeg, hoodDeltaDeg));
      // Recalculate launch angle and tan from clamped hood
      thetaDeg = 90.0 - hoodAngleDeg;
      theta = Math.toRadians(thetaDeg);
      tanTheta = Math.tan(theta);
    } else {
      Logger.recordOutput("SmartLaunch/Pass/TwoPoint/HoodClamped", "none");
    }

    // Solve for K
    double K = (x1 * tanTheta - y1) / (x1 * x1);
    if (K <= 0) {
      Logger.recordOutput("SmartLaunch/Pass/TwoPoint/RejectReason", "K <= 0");
      return unachievablePassResult(
          robotPose, passTarget, config, currentTurretAngleDeg, effectiveMinDeg, effectiveMaxDeg);
    }

    // Solve for velocity
    double cosTheta = Math.cos(theta);
    double vSquared = GRAVITY / (2 * K * cosTheta * cosTheta);
    if (vSquared <= 0) {
      Logger.recordOutput("SmartLaunch/Pass/TwoPoint/RejectReason", "v^2 <= 0");
      return unachievablePassResult(
          robotPose, passTarget, config, currentTurretAngleDeg, effectiveMinDeg, effectiveMaxDeg);
    }
    double exitVelocity = Math.sqrt(vSquared);

    // Add RPM compensation for clamped hood angle, capped to prevent overshooting
    double rpm = calculateRPMForVelocity(exitVelocity, horizontalDist);
    double rpmComp = Math.min(Math.abs(hoodDeltaDeg) * rpmPerDegCompensation, maxRpmCompensation);
    rpm += rpmComp;
    Logger.recordOutput("SmartLaunch/Pass/TwoPoint/RPMCompensation", rpmComp);

    // Log RPM but don't reject — clamp to safe range instead
    if (rpm > 4500) {
      Logger.recordOutput(
          "SmartLaunch/Pass/TwoPoint/RejectReason", String.format("RPM %.0f clamped to 4500", rpm));
      rpm = 4500;
    } else if (rpm < 1500) {
      Logger.recordOutput(
          "SmartLaunch/Pass/TwoPoint/RejectReason", String.format("RPM %.0f clamped to 1500", rpm));
      rpm = 1500;
    }

    // Reject if peak height exceeds max — prevents balls going way up in the air
    double sinTheta = Math.sin(theta);
    double vy0 = exitVelocity * sinTheta;
    double peakHeight = config.heightMeters() + (vy0 * vy0) / (2 * GRAVITY);
    if (peakHeight > maxPeakHeightM) {
      Logger.recordOutput(
          "SmartLaunch/Pass/TwoPoint/RejectReason",
          String.format("Peak %.1fm > max %.1fm", peakHeight, maxPeakHeightM));
      return unachievablePassResult(
          robotPose, passTarget, config, currentTurretAngleDeg, effectiveMinDeg, effectiveMaxDeg);
    }

    // Check if ball is still rising at clearance point (peak after x1)
    // This is fine for passes (ball still clears the height), just log it
    double vx = exitVelocity * cosTheta;
    double timeToPeak = vy0 / GRAVITY;
    double distanceToPeak = vx * timeToPeak;
    if (distanceToPeak >= x1) {
      Logger.recordOutput(
          "SmartLaunch/Pass/TwoPoint/RejectReason",
          String.format(
              "OK (rising at constraint: peak %.2fm, constraint %.2fm)", distanceToPeak, x1));
    } else {
      Logger.recordOutput("SmartLaunch/Pass/TwoPoint/RejectReason", "OK");
    }
    Logger.recordOutput("SmartLaunch/Pass/TwoPoint/PeakHeightM", peakHeight);
    Logger.recordOutput("SmartLaunch/Pass/TwoPoint/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("SmartLaunch/Pass/TwoPoint/RPM", rpm);

    // Single-pass velocity compensation: shift aim target using total TOF.
    // The arc guards (75° cap, peak height, x2-x1 gap, hood min) prevent bad trajectories.
    Translation3d aimTarget = passTarget;
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    if (robotSpeed > 0.1) {
      double tof = calculateTimeOfFlight(exitVelocity, theta, horizontalDist);
      aimTarget = clampAimOffset(predictTargetPos(passTarget, fieldSpeeds, tof), passTarget);
    }

    double turretAngleDeg =
        calculateOutsideTurretAngle(
            robotPose.getX(),
            robotPose.getY(),
            robotPose.getRotation().getDegrees(),
            aimTarget.getX(),
            aimTarget.getY(),
            currentTurretAngleDeg,
            effectiveMinDeg,
            effectiveMaxDeg,
            config);

    return new ShotResult(
        exitVelocity, rpm, Math.toRadians(thetaDeg), hoodAngleDeg, turretAngleDeg, aimTarget, true);
  }

  /** Helper to build an unachievable pass result with valid turret angle. */
  private static ShotResult unachievablePassResult(
      Pose2d robotPose,
      Translation3d passTarget,
      TurretConfig config,
      double currentTurretAngleDeg,
      double effectiveMinDeg,
      double effectiveMaxDeg) {
    double turretAngleDeg =
        calculateOutsideTurretAngle(
            robotPose.getX(),
            robotPose.getY(),
            robotPose.getRotation().getDegrees(),
            passTarget.getX(),
            passTarget.getY(),
            currentTurretAngleDeg,
            effectiveMinDeg,
            effectiveMaxDeg,
            config);
    return new ShotResult(0, 0, 0, 0, turretAngleDeg, passTarget, false);
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
