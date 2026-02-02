// Adapted from FRC Team 5000 Hammerheads
// https://github.com/hammerheads5000/2026Rebuilt
// Open source under WPILib BSD license

package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.LoggedTunableNumber;

/**
 * Utility class for turret ballistics calculations. Provides methods to calculate optimal launch
 * angles, exit velocities, and time of flight for projectile shots.
 */
public class TurretCalculator {
  // Physical constants
  private static final double GRAVITY = 9.81; // m/s^2

  // Tunable shooter parameters
  // Launcher wheel radius in meters (4" diameter = 2" radius = 0.0508m)
  private static final LoggedTunableNumber wheelRadiusMeters =
      new LoggedTunableNumber("Shooter/WheelRadiusMeters", 0.0508);

  // Efficiency factor: how much of wheel surface velocity transfers to ball (0.0-1.0)
  // Accounts for slip, compression, etc. Typical values: 0.6-0.9
  private static final LoggedTunableNumber launchEfficiency =
      new LoggedTunableNumber("Shooter/LaunchEfficiency", 0.70);

  // Launcher RPM tracking
  // currentLauncherRPM = what the launcher is actually doing right now
  // targetLauncherRPM = what we're commanding the launcher to do (setpoint)
  private static double currentLauncherRPM = 0.0;
  private static double targetLauncherRPM = 0.0;

  // Launch angle override: set to override the physics-calculated angle
  // -1 = use physics calculation (default), any other value = use that fixed angle in degrees
  private static final LoggedTunableNumber launchAngleOverrideDeg =
      new LoggedTunableNumber("Shooter/LaunchAngleOverrideDeg", -1.0);

  // Velocity limits for safety
  private static final double MIN_EXIT_VELOCITY = 3.0; // m/s
  private static final double MAX_EXIT_VELOCITY = 15.0; // m/s

  /**
   * Set the current launcher wheel RPM. Call this from the shooting system so trajectory
   * calculations use the actual wheel speed.
   *
   * @param rpm Current launcher wheel RPM
   */
  public static void setLauncherRPM(double rpm) {
    currentLauncherRPM = rpm;
  }

  /**
   * Set the target launcher wheel RPM. Call this when commanding a new velocity so trajectory
   * setpoint calculations use the commanded speed.
   *
   * @param rpm Target launcher wheel RPM
   */
  public static void setTargetLauncherRPM(double rpm) {
    targetLauncherRPM = rpm;
  }

  /**
   * Get the current launcher RPM.
   *
   * @return Current launcher wheel RPM
   */
  public static double getCurrentLauncherRPM() {
    return currentLauncherRPM;
  }

  /**
   * Get the target launcher RPM.
   *
   * @return Target launcher wheel RPM
   */
  public static double getTargetLauncherRPM() {
    return targetLauncherRPM;
  }

  /**
   * Calculate ball exit velocity from current wheel RPM using physics.
   *
   * <p>Formula: exitVelocity = (RPM × 2π × radius / 60) × efficiency
   *
   * @return Exit velocity in m/s based on CURRENT launcher RPM
   */
  public static double calculateExitVelocityFromRPM() {
    return calculateExitVelocityFromRPM(currentLauncherRPM);
  }

  /**
   * Calculate ball exit velocity from TARGET wheel RPM using physics.
   *
   * @return Exit velocity in m/s based on TARGET launcher RPM (what we're commanding)
   */
  public static double calculateSetpointExitVelocity() {
    return calculateExitVelocityFromRPM(targetLauncherRPM);
  }

  /**
   * Calculate ball exit velocity from a given wheel RPM using physics.
   *
   * <p>Formula: exitVelocity = (RPM × 2π × radius / 60) × efficiency
   *
   * @param rpm Wheel RPM to calculate from
   * @return Exit velocity in m/s
   */
  public static double calculateExitVelocityFromRPM(double rpm) {
    double radius = wheelRadiusMeters.get();
    double efficiency = launchEfficiency.get();

    // Wheel surface velocity in m/s
    double surfaceVelocity = (rpm * 2.0 * Math.PI * radius) / 60.0;

    // Apply efficiency factor
    return surfaceVelocity * efficiency;
  }

  /**
   * Get what RPM would be needed to achieve a target exit velocity.
   *
   * @param targetExitVelocity Desired exit velocity in m/s
   * @return Required wheel RPM
   */
  public static double calculateRPMForVelocity(double targetExitVelocity) {
    double radius = wheelRadiusMeters.get();
    double efficiency = launchEfficiency.get();

    // Reverse the formula: RPM = (velocity / efficiency) × 60 / (2π × radius)
    double surfaceVelocity = targetExitVelocity / efficiency;
    return (surfaceVelocity * 60.0) / (2.0 * Math.PI * radius);
  }

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
   * Calculate the optimal launch angle to hit a target given a fixed exit velocity. Uses the
   * standard projectile motion equation with gravity.
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

    // Solve quadratic for launch angle: tan(theta) = (v^2 +/- sqrt(v^4 - g*(g*x^2 + 2*y*v^2))) /
    // (g*x)
    double v2 = velocity * velocity;
    double v4 = v2 * v2;
    double discriminant = v4 - GRAVITY * (GRAVITY * xDist * xDist + 2 * yDist * v2);

    if (discriminant < 0) {
      // Target is unreachable with given velocity - return max angle
      return Math.PI / 4; // 45 degrees
    }

    // Use the higher angle (more arc) for better clearance
    double angle = Math.atan((v2 + Math.sqrt(discriminant)) / (GRAVITY * xDist));
    return angle;
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
    // Target stays fixed, but we need to lead the shot based on our movement
    double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight;
    double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight;
    return new Translation3d(predictedX, predictedY, target.getZ());
  }

  /**
   * Get the exit velocity for shots based on current launcher RPM.
   *
   * @param distanceToTarget Distance to target in meters (unused, kept for API compatibility)
   * @return Exit velocity in m/s
   */
  public static double scaleExitVelocity(double distanceToTarget) {
    double velocity = calculateExitVelocityFromRPM();

    // Debug logging
    org.littletonrobotics.junction.Logger.recordOutput(
        "Shooter/Debug/CurrentRPM", currentLauncherRPM);
    org.littletonrobotics.junction.Logger.recordOutput("Shooter/Debug/RawExitVelocity", velocity);
    org.littletonrobotics.junction.Logger.recordOutput(
        "Shooter/Debug/WheelRadius", wheelRadiusMeters.get());
    org.littletonrobotics.junction.Logger.recordOutput(
        "Shooter/Debug/Efficiency", launchEfficiency.get());

    double clampedVelocity = MathUtil.clamp(velocity, MIN_EXIT_VELOCITY, MAX_EXIT_VELOCITY);
    org.littletonrobotics.junction.Logger.recordOutput(
        "Shooter/Debug/ClampedVelocity", clampedVelocity);
    org.littletonrobotics.junction.Logger.recordOutput(
        "Shooter/Debug/WasClamped", velocity != clampedVelocity);

    return clampedVelocity;
  }

  /**
   * Get the launch angle for shots. If override is set (>= 0), uses that fixed angle. Otherwise
   * calculates optimal angle from physics.
   *
   * @param robot Current robot pose
   * @param velocity Exit velocity in m/s
   * @param target Target position (3D)
   * @param turretHeightMeters Height of turret above ground
   * @return Launch angle in radians
   */
  public static double getLaunchAngle(
      Pose2d robot, double velocity, Translation3d target, double turretHeightMeters) {
    double overrideAngle = launchAngleOverrideDeg.get();
    if (overrideAngle >= 0) {
      // Use override angle (convert degrees to radians)
      return Math.toRadians(overrideAngle);
    }
    // Calculate optimal angle from physics
    return calculateAngleFromVelocity(robot, velocity, target, turretHeightMeters);
  }

  /**
   * Calculate shot parameters using an iterative approach for a moving robot. Refines the shot
   * prediction over multiple iterations for better accuracy.
   *
   * @param robot Current robot pose
   * @param fieldSpeeds Field-relative chassis speeds
   * @param target Target position
   * @param iterations Number of refinement iterations (3-5 recommended)
   * @param turretHeightMeters Height of the turret above ground in meters
   * @return ShotData containing exit velocity, hood angle, and predicted target
   */
  public static ShotData calculateMovingShot(
      Pose2d robot,
      ChassisSpeeds fieldSpeeds,
      Translation3d target,
      int iterations,
      double turretHeightMeters) {
    // Initial estimation assuming stationary robot
    double distance = getDistanceToTarget(robot, target);
    double velocity = scaleExitVelocity(distance);
    double launchAngle = getLaunchAngle(robot, velocity, target, turretHeightMeters);
    double timeOfFlight = calculateTimeOfFlight(velocity, launchAngle, distance);

    Translation3d predictedTarget = target;

    // Iteratively refine the prediction
    for (int i = 0; i < iterations; i++) {
      predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
      distance = getDistanceToTarget(robot, predictedTarget);
      velocity = scaleExitVelocity(distance);
      launchAngle = getLaunchAngle(robot, velocity, predictedTarget, turretHeightMeters);
      timeOfFlight = calculateTimeOfFlight(velocity, launchAngle, distance);
    }

    return new ShotData(velocity, launchAngle, predictedTarget);
  }

  /**
   * Calculate shot parameters for a stationary shot.
   *
   * @param robot Current robot pose
   * @param target Target position
   * @param turretHeightMeters Height of the turret above ground in meters
   * @return ShotData containing exit velocity, hood angle, and target
   */
  public static ShotData calculateStationaryShot(
      Pose2d robot, Translation3d target, double turretHeightMeters) {
    double distance = getDistanceToTarget(robot, target);
    double velocity = scaleExitVelocity(distance);
    double launchAngle = getLaunchAngle(robot, velocity, target, turretHeightMeters);
    return new ShotData(velocity, launchAngle, target);
  }

  /** Record containing all parameters needed for a shot. */
  public record ShotData(double exitVelocity, double launchAngle, Translation3d target) {
    /**
     * Get the exit velocity in m/s.
     *
     * @return Exit velocity
     */
    public double getExitVelocity() {
      return exitVelocity;
    }

    /**
     * Get the launch angle in radians.
     *
     * @return Launch angle from horizontal
     */
    public double getLaunchAngle() {
      return launchAngle;
    }

    /**
     * Get the launch angle in degrees.
     *
     * @return Launch angle from horizontal in degrees
     */
    public double getLaunchAngleDegrees() {
      return Math.toDegrees(launchAngle);
    }

    /**
     * Get the target position.
     *
     * @return Target Translation3d
     */
    public Translation3d getTarget() {
      return target;
    }
  }
}
