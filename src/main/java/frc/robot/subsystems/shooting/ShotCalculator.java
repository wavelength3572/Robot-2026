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

  // Two-roller launcher model:
  // - Main wheel: 3" diameter (1.5" radius)
  // - Hood rollers: 2" diameter, surface speed is 1/1.41 of main wheel (chained together)
  // Ball exit velocity ≈ average of both surface speeds × slip efficiency
  private static final double MAIN_WHEEL_RADIUS_METERS = 0.0381; // 3" diameter = 1.5" radius
  private static final double HOOD_SURFACE_SPEED_RATIO = 1.0 / 1.41; // hood rolls slower

  // Distance-dependent slip/compression efficiency, derived from LUT calibration data.
  // Formula: efficiency = A*d^2 + B*d + C, where d = distance to target in meters.
  // Peaks ~0.785 at mid-range, drops at close range (steep angle) and long range (flat/high slip).
  // Recalculate these coefficients when updating ShotTableConstants baseline data.
  private static final double EFFICIENCY_A = -0.0232;
  private static final double EFFICIENCY_B = 0.1311;
  private static final double EFFICIENCY_C = 0.5929;
  private static final double EFFICIENCY_MIN = 0.50; // floor to prevent nonsense at extreme range
  private static final double EFFICIENCY_MAX = 0.85; // ceiling

  // Velocity limits for safety
  private static final double MIN_EXIT_VELOCITY = 3.0; // m/s
  private static final double MAX_EXIT_VELOCITY = 15.0; // m/s

  // Velocity compensation multipliers for shoot-while-moving (0.0 = no compensation, 1.0 = full)
  private static final LoggedTunableNumber velocityCompX =
      new LoggedTunableNumber("Shots/VelocityComp/X", 1.0);
  private static final LoggedTunableNumber velocityCompY =
      new LoggedTunableNumber("Shots/VelocityComp/Y", 1.0);

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

  /** Get the current launcher RPM. */
  public static double getCurrentLauncherRPM() {
    return currentLauncherRPM;
  }

  /** Get the target launcher RPM. */
  public static double getTargetLauncherRPM() {
    return targetLauncherRPM;
  }

  /**
   * Get the distance-dependent launch efficiency.
   *
   * @param distanceMeters Horizontal distance to target
   * @return Efficiency factor (clamped to safe range)
   */
  public static double getEfficiency(double distanceMeters) {
    double eff =
        EFFICIENCY_A * distanceMeters * distanceMeters
            + EFFICIENCY_B * distanceMeters
            + EFFICIENCY_C;
    return Math.max(EFFICIENCY_MIN, Math.min(EFFICIENCY_MAX, eff));
  }

  /** Get efficiency at a default mid-range distance (for call sites without distance context). */
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
   * @param distanceMeters Horizontal distance to target (for efficiency curve)
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
   * @param distanceMeters Horizontal distance to target (for efficiency curve)
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

    // Velocity compensation: adjust aim point to counteract robot movement during flight.
    // Only apply when the initial shot is achievable — if the optimizer fails (returns
    // exitVelocityMps=0), the ToF calculation produces Double.MAX_VALUE and predictTargetPos
    // generates infinity coordinates, causing the turret to snap to a garbage angle.
    Translation3d aimTarget = hubTarget;
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    if (robotSpeed > 0.1) {
      TrajectoryOptimizer.OptimalShot initialShot =
          TrajectoryOptimizer.calculateOptimalShot(
              turretPos, hubTarget, hoodMinAngleDeg, hoodMaxAngleDeg);

      if (initialShot.achievable) {
        double distanceToTarget =
            Math.sqrt(
                Math.pow(hubTarget.getX() - turretX, 2) + Math.pow(hubTarget.getY() - turretY, 2));
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
              TrajectoryOptimizer.calculateOptimalShot(
                  turretPos, candidate, hoodMinAngleDeg, hoodMaxAngleDeg);
          if (!refinedShot.achievable) {
            // Refinement diverged — keep last successful aimTarget
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

    TrajectoryOptimizer.OptimalShot optimalShot =
        TrajectoryOptimizer.calculateOptimalShot(
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

    // Velocity compensation for robot movement
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
      double hoodMaxAngleDeg) {

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
    Logger.recordOutput("Turret/Pass/TwoPoint/ConstraintX", x1);
    Logger.recordOutput("Turret/Pass/TwoPoint/ConstraintH", constraintH);
    Logger.recordOutput("Turret/Pass/TwoPoint/HorizontalDist", horizontalDist);

    // Solve for launch angle: tanTheta = (y1*x2^2 - y2*x1^2) / (x1*x2*(x2 - x1))
    double denominator = x1 * x2 * (x2 - x1);
    if (Math.abs(denominator) < 0.001) {
      return unachievablePassResult(
          robotPose, passTarget, config, currentTurretAngleDeg, effectiveMinDeg, effectiveMaxDeg);
    }

    double tanTheta = (y1 * x2 * x2 - y2 * x1 * x1) / denominator;
    double theta = Math.atan(tanTheta);
    double thetaDeg = Math.toDegrees(theta);
    double hoodAngleDeg = 90.0 - thetaDeg;

    // Check hood limits
    if (hoodAngleDeg < hoodMinAngleDeg || hoodAngleDeg > hoodMaxAngleDeg) {
      Logger.recordOutput(
          "Turret/Pass/TwoPoint/RejectReason",
          String.format(
              "Hood %.1f outside [%.0f-%.0f]", hoodAngleDeg, hoodMinAngleDeg, hoodMaxAngleDeg));
      return unachievablePassResult(
          robotPose, passTarget, config, currentTurretAngleDeg, effectiveMinDeg, effectiveMaxDeg);
    }

    // Solve for K
    double K = (x1 * tanTheta - y1) / (x1 * x1);
    if (K <= 0) {
      Logger.recordOutput("Turret/Pass/TwoPoint/RejectReason", "K <= 0");
      return unachievablePassResult(
          robotPose, passTarget, config, currentTurretAngleDeg, effectiveMinDeg, effectiveMaxDeg);
    }

    // Solve for velocity
    double cosTheta = Math.cos(theta);
    double vSquared = GRAVITY / (2 * K * cosTheta * cosTheta);
    if (vSquared <= 0) {
      Logger.recordOutput("Turret/Pass/TwoPoint/RejectReason", "v^2 <= 0");
      return unachievablePassResult(
          robotPose, passTarget, config, currentTurretAngleDeg, effectiveMinDeg, effectiveMaxDeg);
    }
    double exitVelocity = Math.sqrt(vSquared);

    // Check RPM limits
    double rpm = calculateRPMForVelocity(exitVelocity, horizontalDist);
    if (rpm < 1500 || rpm > 5000) {
      Logger.recordOutput(
          "Turret/Pass/TwoPoint/RejectReason", String.format("RPM %.0f outside [1500-5000]", rpm));
      return unachievablePassResult(
          robotPose, passTarget, config, currentTurretAngleDeg, effectiveMinDeg, effectiveMaxDeg);
    }

    // Check peak height
    double sinTheta = Math.sin(theta);
    double vy0 = exitVelocity * sinTheta;
    double peakHeight = config.heightMeters() + (vy0 * vy0) / (2 * GRAVITY);
    if (peakHeight > maxPeakHeightM) {
      Logger.recordOutput(
          "Turret/Pass/TwoPoint/RejectReason",
          String.format("Peak %.1fm > max %.1fm", peakHeight, maxPeakHeightM));
      return unachievablePassResult(
          robotPose, passTarget, config, currentTurretAngleDeg, effectiveMinDeg, effectiveMaxDeg);
    }

    // Check that ball is descending at clearance point (peak before x1)
    double vx = exitVelocity * cosTheta;
    double timeToPeak = vy0 / GRAVITY;
    double distanceToPeak = vx * timeToPeak;
    if (distanceToPeak >= x1) {
      Logger.recordOutput(
          "Turret/Pass/TwoPoint/RejectReason",
          String.format("Peak at %.2fm, constraint at %.2fm", distanceToPeak, x1));
      return unachievablePassResult(
          robotPose, passTarget, config, currentTurretAngleDeg, effectiveMinDeg, effectiveMaxDeg);
    }

    Logger.recordOutput("Turret/Pass/TwoPoint/RejectReason", "OK");
    Logger.recordOutput("Turret/Pass/TwoPoint/PeakHeightM", peakHeight);
    Logger.recordOutput("Turret/Pass/TwoPoint/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("Turret/Pass/TwoPoint/RPM", rpm);

    // Velocity compensation: re-solve with adjusted aim target, keeping clearance point fixed
    Translation3d aimTarget = passTarget;
    double robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    if (robotSpeed > 0.1) {
      double tof = calculateTimeOfFlight(exitVelocity, theta, horizontalDist);
      for (int i = 0; i < 3; i++) {
        Translation3d candidate =
            clampAimOffset(predictTargetPos(passTarget, fieldSpeeds, tof), passTarget);
        double aimDist =
            Math.sqrt(
                Math.pow(candidate.getX() - turretX, 2) + Math.pow(candidate.getY() - turretY, 2));

        // Re-solve two-point with updated x2
        double newX2 = aimDist;
        double newY2 = candidate.getZ() - config.heightMeters();
        double newDenom = x1 * newX2 * (newX2 - x1);
        if (Math.abs(newDenom) < 0.001) break;

        double newTanTheta = (y1 * newX2 * newX2 - newY2 * x1 * x1) / newDenom;
        double newTheta = Math.atan(newTanTheta);
        double newCosTheta = Math.cos(newTheta);
        double newK = (x1 * newTanTheta - y1) / (x1 * x1);
        if (newK <= 0) break;

        double newVSquared = GRAVITY / (2 * newK * newCosTheta * newCosTheta);
        if (newVSquared <= 0) break;

        exitVelocity = Math.sqrt(newVSquared);
        theta = newTheta;
        thetaDeg = Math.toDegrees(theta);
        hoodAngleDeg = 90.0 - thetaDeg;
        rpm = calculateRPMForVelocity(exitVelocity, aimDist);
        aimTarget = candidate;
        tof = calculateTimeOfFlight(exitVelocity, theta, aimDist);
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
