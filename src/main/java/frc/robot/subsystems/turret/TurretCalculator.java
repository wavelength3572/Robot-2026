// Adapted from FRC Team 5000 Hammerheads
// https://github.com/hammerheads5000/2026Rebuilt
// Open source under WPILib BSD license

package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

/**
 * Utility class for turret ballistics calculations. Provides methods to calculate optimal launch
 * angles, exit velocities, and time of flight for projectile shots.
 */
public class TurretCalculator {
  // Physical constants
  private static final double GRAVITY = 9.81; // m/s^2

  // Shooter parameters (from CAD - 75° launch angle, 6.5 m/s at 2m from hopper)
  private static final double DEFAULT_EXIT_VELOCITY = 6.5; // m/s
  private static final double MIN_EXIT_VELOCITY = 6.0; // m/s
  private static final double MAX_EXIT_VELOCITY = 8.0; // m/s

  // Velocity scaling parameters
  // Tuned so velocity ≈ 6.5 m/s at 2m distance, slight increase for longer shots
  private static final double BASE_VELOCITY = 6.0; // m/s base velocity
  private static final double VELOCITY_MULTIPLIER = 0.4; // minimal scaling with distance
  private static final double VELOCITY_POWER = 0.3; // gradual increase

  private static final frc.robot.RobotConfig config = Constants.getRobotConfig();

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
   * @return Launch angle in radians (from horizontal)
   */
  public static double calculateAngleFromVelocity(
      Pose2d robot, double velocity, Translation3d target) {
    double xDist = getDistanceToTarget(robot, target);
    double yDist = target.getZ() - config.getTurretHeightMeters();

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
   * Scale exit velocity based on distance to target. Uses a power-law scaling to minimize flywheel
   * speed changes.
   *
   * @param distanceToTarget Distance to target in meters
   * @return Scaled exit velocity in m/s
   */
  public static double scaleExitVelocity(double distanceToTarget) {
    double velocity =
        BASE_VELOCITY + VELOCITY_MULTIPLIER * Math.pow(distanceToTarget, VELOCITY_POWER);
    return MathUtil.clamp(velocity, MIN_EXIT_VELOCITY, MAX_EXIT_VELOCITY);
  }

  /**
   * Calculate shot parameters using an iterative approach for a moving robot. Refines the shot
   * prediction over multiple iterations for better accuracy.
   *
   * @param robot Current robot pose
   * @param fieldSpeeds Field-relative chassis speeds
   * @param target Target position
   * @param iterations Number of refinement iterations (3-5 recommended)
   * @return ShotData containing exit velocity, hood angle, and predicted target
   */
  public static ShotData calculateMovingShot(
      Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
    // Initial estimation assuming stationary robot
    double distance = getDistanceToTarget(robot, target);
    double exitVelocity = scaleExitVelocity(distance);
    double launchAngle = calculateAngleFromVelocity(robot, exitVelocity, target);
    double timeOfFlight = calculateTimeOfFlight(exitVelocity, launchAngle, distance);

    Translation3d predictedTarget = target;

    // Iteratively refine the prediction
    for (int i = 0; i < iterations; i++) {
      predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
      distance = getDistanceToTarget(robot, predictedTarget);
      exitVelocity = scaleExitVelocity(distance);
      launchAngle = calculateAngleFromVelocity(robot, exitVelocity, predictedTarget);
      timeOfFlight = calculateTimeOfFlight(exitVelocity, launchAngle, distance);
    }

    return new ShotData(exitVelocity, launchAngle, predictedTarget);
  }

  /**
   * Calculate shot parameters for a stationary shot.
   *
   * @param robot Current robot pose
   * @param target Target position
   * @return ShotData containing exit velocity, hood angle, and target
   */
  public static ShotData calculateStationaryShot(Pose2d robot, Translation3d target) {
    double distance = getDistanceToTarget(robot, target);
    double exitVelocity = scaleExitVelocity(distance);
    double launchAngle = calculateAngleFromVelocity(robot, exitVelocity, target);
    return new ShotData(exitVelocity, launchAngle, target);
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
