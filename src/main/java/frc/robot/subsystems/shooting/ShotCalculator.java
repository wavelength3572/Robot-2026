package frc.robot.subsystems.shooting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.hood.TrajectoryOptimizer;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class for all shot-related calculations. Converts robot pose + target into shot
 * parameters (exit velocity, launch angle, hood angle, turret angle). Also provides launcher
 * RPM↔velocity conversions and projectile physics utilities.
 */
public final class ShotCalculator {

  // ========== Physical Constants ==========
  private static final double GRAVITY = 9.81; // m/s^2
  private static final double WHEEL_RADIUS_METERS = 0.0508; // 4" diameter = 2" radius

  // Efficiency factor: how much of wheel surface velocity transfers to ball (0.0-1.0)
  // Single value — no distance interpolation. Keep it simple so the LUT and parametric
  // systems are easy to reason about. Tune this once, then use LUT for fine-tuning per distance.
  private static final LoggedTunableNumber launchEfficiency =
      new LoggedTunableNumber("Shots/SmartLaunch/Trajectory/LaunchEfficiency", 0.50);

  // Velocity limits for safety
  private static final double MIN_EXIT_VELOCITY = 3.0; // m/s
  private static final double MAX_EXIT_VELOCITY = 15.0; // m/s

  // Maximum velocity compensation aim offset (meters from original target)
  // Prevents the aim target from drifting off-field when TOF is large
  private static final double MAX_AIM_OFFSET_METERS = 1.5;

  // ========== Launcher RPM Tracking ==========
  // currentLauncherRPM = what the launcher is actually doing right now
  // targetLauncherRPM = what we're commanding the launcher to do (setpoint)
  private static double currentLauncherRPM = 0.0;
  private static double targetLauncherRPM = 0.0;

  /** Turret geometry config (immutable, set once at startup). */
  public record TurretConfig(double heightMeters, double xOffset, double yOffset) {}

  /**
   * Result of a shot calculation.
   *
   * @param contractionRate How fast the velocity-compensation iteration is converging. This is a
   *     ratio of successive aim-point corrections: if iteration N moved the aim point by 20 cm and
   *     iteration N+1 moved it by 2 cm, the contraction rate is 0.1. Lower is better:
   *     <ul>
   *       <li>&lt; 0.3 — converging quickly, shot is reliable
   *       <li>0.3–0.7 — converging, but residual error may be significant
   *       <li>&gt; 0.7 — barely converging, aim point is unstable (consider not shooting)
   *       <li>-1.0 — not applicable (robot stationary, so no iteration was needed)
   *     </ul>
   *     This comes from fixed-point iteration theory: if the contraction rate is r, the remaining
   *     error after the final iteration is roughly r × (last correction). So a rate of 0.1 means
   *     your answer is ~10× better than your last correction — very good. A rate near 1.0 means
   *     you're not converging and the answer is unreliable.
   */
  public record ShotResult(
      double exitVelocityMps,
      double launcherRPM, // pre-computed RPM (uses launch efficiency)
      double launchAngleRad,
      double hoodAngleDeg,
      double turretAngleDeg, // robot-relative
      Translation3d aimTarget, // velocity-compensated
      boolean achievable,
      double contractionRate) {

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

  /** Get the current launcher RPM. */
  public static double getCurrentLauncherRPM() {
    return currentLauncherRPM;
  }

  /** Get the target launcher RPM. */
  public static double getTargetLauncherRPM() {
    return targetLauncherRPM;
  }

  /**
   * Get the launch efficiency. Single constant value — no distance interpolation. Tune once, then
   * use LUT for per-distance fine-tuning.
   */
  public static double getEfficiency() {
    return launchEfficiency.get();
  }

  /**
   * Calculate ball exit velocity from a given wheel RPM.
   *
   * <p>Formula: exitVelocity = (RPM × 2π × radius / 60) × efficiency
   */
  public static double calculateExitVelocityFromRPM(double rpm) {
    double surfaceVelocity = (rpm * 2.0 * Math.PI * WHEEL_RADIUS_METERS) / 60.0;
    return surfaceVelocity * getEfficiency();
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
   * Get what RPM would be needed to achieve a target exit velocity (no distance context).
   *
   * @param targetExitVelocity Desired exit velocity in m/s
   * @return Required wheel RPM
   */
  public static double calculateRPMForVelocity(double targetExitVelocity) {
    double surfaceVelocity = targetExitVelocity / getEfficiency();
    return (surfaceVelocity * 60.0) / (2.0 * Math.PI * WHEEL_RADIUS_METERS);
  }

  /**
   * Get what RPM would be needed to achieve a target exit velocity at a given distance. Distance
   * parameter kept for API compatibility but efficiency is now a single constant.
   *
   * @param targetExitVelocity Desired exit velocity in m/s
   * @param distanceM Horizontal distance to target in meters (unused)
   * @return Required wheel RPM
   */
  public static double calculateRPMForVelocity(double targetExitVelocity, double distanceM) {
    return calculateRPMForVelocity(targetExitVelocity);
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
    double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight;
    double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight;
    return new Translation3d(predictedX, predictedY, target.getZ());
  }

  // ========== Field Position Helpers ==========

  /**
   * Clamp a velocity-compensated aim target so it doesn't drift too far from the original target.
   * Prevents the aim point from going off-field when TOF is large.
   */
  public static Translation3d clampAimOffset(
      Translation3d aimTarget, Translation3d originalTarget) {
    double offsetX = aimTarget.getX() - originalTarget.getX();
    double offsetY = aimTarget.getY() - originalTarget.getY();
    double offsetDist = Math.hypot(offsetX, offsetY);
    if (offsetDist > MAX_AIM_OFFSET_METERS) {
      double scale = MAX_AIM_OFFSET_METERS / offsetDist;
      return new Translation3d(
          originalTarget.getX() + offsetX * scale,
          originalTarget.getY() + offsetY * scale,
          originalTarget.getZ());
    }
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

    // Log calculation values
    Logger.recordOutput("Turret/RobotOmegaNormalized", robotOmegaNormalized);
    Logger.recordOutput("Turret/AbsoluteAngle", absoluteAngle);
    Logger.recordOutput("Turret/RelativeAngle", relativeAngle);

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

    Logger.recordOutput("Turret/bestOutsideAngle", bestOutsideAngle);

    return bestOutsideAngle;
  }

  // ========== Convergence Constants ==========

  /**
   * Maximum number of velocity-compensation iterations (fixed-point iteration).
   *
   * <p>Each iteration: estimate time-of-flight → predict where the target appears from the robot's
   * moving frame → re-solve the shot to the new "virtual goal" → get a new TOF → repeat. With 3
   * iterations and a typical contraction rate of 0.1–0.3, the residual error is negligible.
   */
  private static final int VELOCITY_COMP_MAX_ITERATIONS = 3;

  /**
   * If the aim point moves less than this between iterations, stop early (meters). 1 cm is well
   * below any mechanical tolerance on the turret or hood, so further refinement is pointless.
   */
  private static final double CONVERGENCE_THRESHOLD_METERS = 0.01;

  // ========== Shot Calculations ==========

  /**
   * Calculate shot parameters for the hub (uses TrajectoryOptimizer for optimal RPM/angle).
   * Includes velocity compensation for robot movement.
   *
   * <h3>How velocity compensation works (the "time-of-flight recursion"):</h3>
   *
   * <p>When the robot is moving, the ball's starting position shifts during flight. We compensate
   * by aiming at a "virtual goal" — the point where the real goal *appears* to be in the robot's
   * moving reference frame. Finding this virtual goal requires knowing the flight time, but the
   * flight time depends on the distance to the virtual goal, which we haven't found yet. This
   * circular dependency is resolved by <b>fixed-point iteration</b>:
   *
   * <ol>
   *   <li>Solve the shot to the real target → get initial time-of-flight (TOF)
   *   <li>Offset the target by {@code -TOF * robotVelocity} → "virtual goal v1"
   *   <li>Re-solve the shot to v1 → get a new TOF (distance changed, so TOF changed)
   *   <li>Offset the *original* target by the new TOF → "virtual goal v2"
   *   <li>Repeat until the virtual goal stops moving (converges)
   * </ol>
   *
   * <p>We track how much the aim point moved on each iteration. The ratio of successive movements
   * (the "contraction rate") tells us how reliable the result is — see {@link
   * ShotResult#contractionRate}.
   *
   * <p><b>Why not Newton's method?</b> Fixed-point iteration naturally *fails to converge* when the
   * shot is infeasible (robot too fast, target behind you). Newton's method can converge to
   * pathological solutions in those cases. The contraction rate gives us a free "shot quality"
   * signal that Newton's method doesn't provide. (Source: CD recursive TOF simulator thread.)
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

    // --- Velocity compensation iteration ---
    //
    // We keep track of the *last* optimizer result so we can reuse it after the loop exits,
    // avoiding a redundant 4th optimizer call. The optimizer is the most expensive part of
    // the shot calculation (it solves a constrained trajectory through two 3D points), so
    // skipping one call per cycle is a meaningful savings on a 50 Hz control loop.
    Translation3d aimTarget = hubTarget;
    double contractionRate = -1.0; // -1 means "not applicable" (robot stationary)
    TrajectoryOptimizer.OptimalShot lastShot =
        TrajectoryOptimizer.calculateOptimalShot(
            turretPos, hubTarget, hoodMinAngleDeg, hoodMaxAngleDeg);

    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    if (robotSpeed > 0.1 && lastShot.achievable) {
      // Seed the iteration with the static shot's time-of-flight.
      double distanceToTarget =
          Math.sqrt(
              Math.pow(hubTarget.getX() - turretX, 2) + Math.pow(hubTarget.getY() - turretY, 2));
      double tof =
          calculateTimeOfFlight(
              lastShot.exitVelocityMps,
              Math.toRadians(lastShot.launchAngleDeg),
              distanceToTarget);

      double prevCorrectionDist = 0.0; // how far the aim point moved on the previous iteration

      for (int i = 0; i < VELOCITY_COMP_MAX_ITERATIONS; i++) {
        // Predict where the target *appears* to be from the robot's moving frame.
        // We always offset from the ORIGINAL target, not from the last iteration's aim point.
        // This is important: offsetting from the previous aim point would compound errors.
        Translation3d newAimTarget =
            clampAimOffset(predictTargetPos(hubTarget, fieldSpeeds, tof), hubTarget);

        // --- Early exit check ---
        // Measure how far the aim point moved this iteration compared to last iteration.
        double correctionDist =
            Math.sqrt(
                Math.pow(newAimTarget.getX() - aimTarget.getX(), 2)
                    + Math.pow(newAimTarget.getY() - aimTarget.getY(), 2));

        // Compute contraction rate = (this correction) / (previous correction).
        // Only meaningful from iteration 1 onward (need two corrections to compare).
        if (i > 0 && prevCorrectionDist > CONVERGENCE_THRESHOLD_METERS) {
          contractionRate = correctionDist / prevCorrectionDist;
        }

        aimTarget = newAimTarget;

        // If the aim point barely moved, we've converged — no need for more iterations.
        if (correctionDist < CONVERGENCE_THRESHOLD_METERS) {
          break;
        }

        prevCorrectionDist = correctionDist;

        // Re-solve the shot to the new aim point to get an updated TOF.
        double aimDistance =
            Math.sqrt(
                Math.pow(aimTarget.getX() - turretX, 2)
                    + Math.pow(aimTarget.getY() - turretY, 2));
        TrajectoryOptimizer.OptimalShot refinedShot =
            TrajectoryOptimizer.calculateOptimalShot(
                turretPos, aimTarget, hoodMinAngleDeg, hoodMaxAngleDeg);
        if (!refinedShot.achievable) {
          // Optimizer can't solve at this aim point — the virtual goal is too far off.
          // Fall back to the static (no velocity compensation) shot.
          aimTarget = hubTarget;
          contractionRate = -1.0;
          break;
        }

        lastShot = refinedShot;
        tof =
            calculateTimeOfFlight(
                refinedShot.exitVelocityMps,
                Math.toRadians(refinedShot.launchAngleDeg),
                aimDistance);
      }

      // If we exited the loop normally (didn't break on !achievable), lastShot already
      // corresponds to aimTarget. No need for a redundant optimizer call.
    }

    // Log the contraction rate for diagnostics. Check this in AdvantageScope —
    // if it's consistently > 0.7 during matches, the iteration isn't converging
    // and you may want to slow down or approach the target from a different angle.
    Logger.recordOutput("Shots/VelocityComp/ContractionRate", contractionRate);

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
        lastShot.exitVelocityMps,
        lastShot.rpm,
        Math.toRadians(lastShot.launchAngleDeg),
        lastShot.hoodAngleDeg,
        turretAngleDeg,
        aimTarget,
        lastShot.achievable,
        contractionRate);
  }

  /**
   * Calculate shot parameters for a pass to a ground-level target. Uses simple projectile physics
   * with a fixed launch angle instead of the hub-specific TrajectoryOptimizer.
   *
   * <p>Velocity compensation uses the same fixed-point iteration as {@link #calculateHubShot}, but
   * the "solver" here is just the projectile velocity formula instead of TrajectoryOptimizer.
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

    // Velocity compensation — same fixed-point iteration as calculateHubShot.
    // See that method's Javadoc for the full explanation.
    Translation3d aimTarget = passTarget;
    double contractionRate = -1.0;
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    if (robotSpeed > 0.1) {
      double tof = calculateTimeOfFlight(exitVelocity, launchAngleRad, horizontalDist);
      double prevCorrectionDist = 0.0;

      for (int i = 0; i < VELOCITY_COMP_MAX_ITERATIONS; i++) {
        Translation3d newAimTarget =
            clampAimOffset(predictTargetPos(passTarget, fieldSpeeds, tof), passTarget);

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

    return new ShotResult(
        exitVelocity,
        calculateRPMForVelocity(exitVelocity),
        launchAngleRad,
        90.0 - launchAngleDeg, // convert launch angle to hood angle
        turretAngleDeg,
        aimTarget,
        true,
        contractionRate);
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
        true,
        -1.0); // manual shot — no velocity compensation iteration
  }
}
