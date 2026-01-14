package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
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
}
