package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

/**
 * Utility class providing global access to robot subsystems.
 *
 * <p>This breaks circular dependencies and enables subsystems to communicate without tight
 * coupling.
 */
public class RobotStatus {
  private static Drive driveSystem;
  private static Vision visionSystem;

  // Cached alliance to avoid Optional allocation every cycle
  private static DriverStation.Alliance cachedAlliance = DriverStation.Alliance.Blue;
  private static boolean allianceResolved = false;

  /** Initialize RobotStatus with subsystem references. Called once from RobotContainer. */
  public static void initialize(Drive drive, Vision vision) {
    driveSystem = drive;
    visionSystem = vision;
  }

  /** Returns the robot's current pose from the drive subsystem. */
  public static Pose2d getRobotPose() {
    if (driveSystem == null) {
      return new Pose2d(); // Return origin if not initialized (for early sim calls)
    }
    return driveSystem.getPose();
  }

  /** Returns the Drive subsystem instance. */
  public static Drive getDrive() {
    return driveSystem;
  }

  /** Returns the Vision subsystem instance. */
  public static Vision getVision() {
    return visionSystem;
  }

  /**
   * Returns the cached alliance color. Call {@link #refreshAlliance()} periodically (e.g., in
   * Robot.robotPeriodic) to keep this up to date without allocating an Optional every cycle.
   */
  public static DriverStation.Alliance getAlliance() {
    return cachedAlliance;
  }

  /** Returns true if the robot is on the blue alliance. */
  public static boolean isBlueAlliance() {
    return cachedAlliance == DriverStation.Alliance.Blue;
  }

  /**
   * Refresh the cached alliance from DriverStation. Call once per loop cycle (e.g., in
   * Robot.robotPeriodic) rather than from every subsystem.
   */
  public static void refreshAlliance() {
    // Only re-query until resolved, then lock in
    if (!allianceResolved) {
      var optional = DriverStation.getAlliance();
      if (optional.isPresent()) {
        cachedAlliance = optional.get();
        allianceResolved = true;
      }
    }
  }

  /** Returns whether vision is currently enabled. */
  public static boolean isVisionOn() {
    if (visionSystem == null) {
      return false;
    }
    return visionSystem.isVisionOn();
  }
}
