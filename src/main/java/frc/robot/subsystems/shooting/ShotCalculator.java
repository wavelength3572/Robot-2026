package frc.robot.subsystems.shooting;

import org.littletonrobotics.junction.Logger;

/**
 * Shared utility class for shot-related calculations. Contains physical constants, the parabola
 * solver, turret geometry helpers, and time-of-flight computation.
 *
 * <p>Strategy-specific concerns (efficiency, hood angle offsets, RPM↔velocity conversion) live in
 * each {@link ShotStrategy} implementation, not here.
 */
public final class ShotCalculator {

  // ========== Physical Constants (package-private for strategy access) ==========
  static final double GRAVITY = 9.81; // m/s^2

  // Two-roller launcher model:
  // - Main wheel: 3" diameter (1.5" radius)
  // - Hood rollers: 2" diameter, surface speed is 1/1.41 of main wheel (chained together)
  static final double MAIN_WHEEL_RADIUS_METERS = 0.0381; // 3" diameter = 1.5" radius
  static final double HOOD_SURFACE_SPEED_RATIO = 1.0 / 1.41; // hood rolls slower

  // Motivator wheel: 3" diameter (same as main wheel)
  static final double MOTIVATOR_WHEEL_RADIUS_METERS = 0.0381;

  // ========== Launcher RPM Tracking ==========
  // currentLauncherRPM = what the launcher is actually doing right now
  // targetLauncherRPM = what we're commanding the launcher to do (setpoint)
  private static double currentLauncherRPM = 0.0;
  private static double targetLauncherRPM = 0.0;

  /** Turret geometry config (immutable, set once at startup). */
  public record TurretConfig(double heightMeters, double xOffset, double yOffset) {}

  /** Result of a shot calculation — the 4 mechanical commands. */
  public record ShotResult(
      double launcherRPM, double hoodAngleDeg, double motivatorRPM, double spindexerRPM) {

    /** Derived launch angle from hood angle: theta = 90° - hoodAngle. */
    public double launchAngleRad() {
      return Math.toRadians(90.0 - hoodAngleDeg);
    }

    /** Get the launch angle converted to degrees. */
    public double getLaunchAngleDegrees() {
      return 90.0 - hoodAngleDeg;
    }
  }

  private ShotCalculator() {} // Static utility class

  // ========== Launcher RPM Tracking ==========

  /** Set the current launcher wheel RPM (called by Launcher periodic). */
  public static void setLauncherRPM(double rpm) {
    currentLauncherRPM = rpm;
  }

  /** Get the current launcher wheel RPM. */
  public static double getLauncherRPM() {
    return currentLauncherRPM;
  }

  /** Set the target launcher wheel RPM (called when commanding a new velocity). */
  public static void setTargetLauncherRPM(double rpm) {
    targetLauncherRPM = rpm;
  }

  /** Get the target launcher wheel RPM. */
  public static double getTargetLauncherRPM() {
    return targetLauncherRPM;
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

  // ========== Field Position Helpers ==========

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
    double currentAngle = currentTurretAngleDeg;

    double robotOmegaNormalized = robotHeadingDeg % 360;
    if (robotOmegaNormalized > 180) {
      robotOmegaNormalized -= 360;
    } else if (robotOmegaNormalized <= -180) {
      robotOmegaNormalized += 360;
    }
    double robotOmegaRad = Math.toRadians(robotHeadingDeg);

    double turretFieldX =
        robotX
            + (config.xOffset() * Math.cos(robotOmegaRad)
                - config.yOffset() * Math.sin(robotOmegaRad));

    double turretFieldY =
        robotY
            + (config.xOffset() * Math.sin(robotOmegaRad)
                + config.yOffset() * Math.cos(robotOmegaRad));

    double deltaX = targetX - turretFieldX;
    double deltaY = targetY - turretFieldY;

    double absoluteAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
    double relativeAngle = absoluteAngle - robotHeadingDeg;

    double[] candidates = {relativeAngle, relativeAngle + 360.0, relativeAngle - 360.0};

    double bestOutsideAngle = relativeAngle;
    double smallestMove = 1000000.0;

    for (double candidate : candidates) {
      if (candidate >= effectiveMinDeg && candidate <= effectiveMaxDeg) {
        double moveDistance = Math.abs(candidate - currentAngle);
        if (moveDistance < smallestMove) {
          smallestMove = moveDistance;
          bestOutsideAngle = candidate;
        }
      }
    }

    if (bestOutsideAngle < effectiveMinDeg) {
      bestOutsideAngle = effectiveMinDeg;
    } else if (bestOutsideAngle > effectiveMaxDeg) {
      bestOutsideAngle = effectiveMaxDeg;
    }

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
      boolean clampedLow,
      double actualPeakHeightM,
      double vertexXM,
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

    double r = Math.sqrt((y_p - k) / (h0 - k));
    double h = x_p / (1.0 + r);
    double a = (h0 - k) / (h * h);

    double tanTheta = -2.0 * a * h;
    double theta = Math.atan(tanTheta);
    double thetaDeg = Math.toDegrees(theta);

    double hoodAngleDeg = 90.0 - thetaDeg;
    boolean clamped = false;
    boolean clampedLow = false;
    double velocity;
    double cosTheta;
    double actualPeakM = k;

    if (hoodAngleDeg < hoodMinAngleDeg) {
      clamped = true;
      clampedLow = true;
      hoodAngleDeg = hoodMinAngleDeg;
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

    } else if (hoodAngleDeg > hoodMaxAngleDeg) {
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
   * Calculate shot parameters for test mode using manual RPM, hood angle, and turret angle. Returns
   * a ShotResult with the given RPM and hood angle; motivator/spindexer default to 0 (test mode).
   */
  public static ShotResult calculateManualShot(double launcherRPM, double hoodAngleDeg) {
    return new ShotResult(launcherRPM, hoodAngleDeg, 0, 0);
  }
}
