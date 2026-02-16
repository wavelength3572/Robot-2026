package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.fueldetection.FuelDetection;
import frc.robot.subsystems.intake.Intake;
import java.util.Optional;

/**
 * Commands for autonomous fuel pickup using Pi-based fuel detection and PathPlanner on-the-fly
 * pathfinding.
 */
public class FuelDetectionCommands {
  // Conservative constraints for approaching fuel (slower than normal driving)
  private static final PathConstraints FUEL_APPROACH_CONSTRAINTS =
      new PathConstraints(2.0, 2.0, 360.0, 540.0); // m/s, m/s², deg/s, deg/s²

  /**
   * Assessment position near the depot — robot backs up here for a good camera view of the depot
   * area while the turret shoots collected fuel. Blue-alliance coordinates; PathPlanner flips
   * automatically for red.
   */
  private static final Pose2d DEPOT_ASSESSMENT_POSE =
      new Pose2d(2.0, 5.5, Rotation2d.fromDegrees(180.0));

  private FuelDetectionCommands() {}

  /**
   * Creates a command that pathfinds to the nearest detected fuel and picks it up. The command:
   *
   * <ol>
   *   <li>Waits until the Pi detects a fuel target
   *   <li>Computes an approach pose (offset so the intake faces the fuel)
   *   <li>Uses PathPlanner pathfindToPose to drive there with obstacle avoidance
   *   <li>Deploys the intake and runs rollers during approach
   * </ol>
   *
   * Ends when the robot reaches the target pose, or can be cancelled if the fuel disappears.
   *
   * @param drive The drive subsystem
   * @param fuelDetection The fuel detection subsystem
   * @param intake The intake subsystem (may be null if not present)
   * @return Command to drive to and pick up fuel
   */
  public static Command driveToFuel(Drive drive, FuelDetection fuelDetection, Intake intake) {
    return Commands.sequence(
            // Wait until we have a valid fuel target
            Commands.waitUntil(fuelDetection::hasValidTarget).withTimeout(3.0),

            // Snapshot the target pose and pathfind to it
            Commands.defer(
                () -> {
                  Optional<Pose2d> targetPose =
                      fuelDetection.getPickupPose(drive.getPose().getTranslation());
                  if (targetPose.isEmpty()) {
                    return Commands.none();
                  }

                  Command pathfind =
                      AutoBuilder.pathfindToPose(targetPose.get(), FUEL_APPROACH_CONSTRAINTS);

                  // If intake is available, deploy and run rollers during approach
                  if (intake != null) {
                    return Commands.parallel(
                        pathfind,
                        Commands.runOnce(
                            () -> {
                              intake.deploy();
                              intake.runIntake();
                            },
                            intake));
                  }
                  return pathfind;
                },
                drive.getDefaultCommand() != null
                    ? drive.getDefaultCommand().getRequirements()
                    : java.util.Set.of(drive)))
        .finallyDo(
            () -> {
              if (intake != null) {
                intake.stopRollers();
              }
            })
        .withName("DriveToFuel");
  }

  /**
   * Creates a command that continuously tracks and drives toward fuel. Unlike {@link
   * #driveToFuel}, this command re-evaluates the fuel position each cycle, making it responsive to
   * fuel that moves or new detections. Ends when the robot is within pickup range.
   *
   * @param drive The drive subsystem
   * @param fuelDetection The fuel detection subsystem
   * @param intake The intake subsystem (may be null)
   * @return Command that tracks and drives to fuel
   */
  public static Command trackAndDriveToFuel(
      Drive drive, FuelDetection fuelDetection, Intake intake) {
    return Commands.sequence(
            Commands.waitUntil(fuelDetection::hasValidTarget).withTimeout(3.0),
            Commands.defer(
                () -> {
                  Optional<Pose2d> targetPose =
                      fuelDetection.getPickupPose(drive.getPose().getTranslation());
                  if (targetPose.isEmpty()) {
                    return Commands.none();
                  }

                  Command pathfind =
                      AutoBuilder.pathfindToPose(targetPose.get(), FUEL_APPROACH_CONSTRAINTS);

                  if (intake != null) {
                    return Commands.parallel(
                        pathfind,
                        Commands.runOnce(
                            () -> {
                              intake.deploy();
                              intake.runIntake();
                            },
                            intake));
                  }
                  return pathfind;
                },
                drive.getDefaultCommand() != null
                    ? drive.getDefaultCommand().getRequirements()
                    : java.util.Set.of(drive)))
        .finallyDo(
            () -> {
              if (intake != null) {
                intake.stopRollers();
              }
            })
        .withName("TrackAndDriveToFuel");
  }

  /**
   * Creates a command for the depot cleanup phase. After the initial pre-planned depot sweep, this
   * command:
   *
   * <ol>
   *   <li>Pathfinds back to an assessment position (good camera view of the depot)
   *   <li>Pauses briefly for the camera to stabilize while the turret shoots
   *   <li>Checks if the Pi camera detects any remaining fuel at the depot
   *   <li>If fuel is detected: drives to pick it up with intake running, then returns
   *   <li>If no fuel detected: ends immediately
   * </ol>
   *
   * <p>The turret's auto-shoot should stay enabled throughout, so the robot shoots while stationary
   * and while driving to pick up leftovers.
   *
   * @param drive The drive subsystem
   * @param fuelDetection The fuel detection subsystem
   * @param intake The intake subsystem (may be null if not present)
   * @return Command for depot assessment and cleanup
   */
  public static Command assessAndCleanupDepot(
      Drive drive, FuelDetection fuelDetection, Intake intake) {
    return Commands.sequence(
            // Back up to assessment position (gives camera a wide view of the depot)
            AutoBuilder.pathfindToPose(DEPOT_ASSESSMENT_POSE, FUEL_APPROACH_CONSTRAINTS),

            // Brief pause for camera to stabilize while stationary
            Commands.waitSeconds(0.5),

            // Check camera and pick up remaining fuel if detected
            Commands.defer(
                () -> {
                  if (!fuelDetection.hasValidTarget()) {
                    System.out.println("[DepotCleanup] No remaining fuel detected at depot");
                    return Commands.none();
                  }

                  Optional<Pose2d> targetPose =
                      fuelDetection.getPickupPose(drive.getPose().getTranslation());
                  if (targetPose.isEmpty()) {
                    return Commands.none();
                  }

                  System.out.println("[DepotCleanup] Remaining fuel detected — driving to pick up");

                  Command pickupDrive =
                      AutoBuilder.pathfindToPose(targetPose.get(), FUEL_APPROACH_CONSTRAINTS);

                  // Drive to fuel with intake running, then return to assessment pose
                  return Commands.sequence(
                      Commands.parallel(
                          pickupDrive,
                          intake != null
                              ? Commands.runOnce(
                                  () -> {
                                    intake.deploy();
                                    intake.runIntake();
                                  },
                                  intake)
                              : Commands.none()),
                      // Return to assessment position so turret can finish shooting
                      AutoBuilder.pathfindToPose(DEPOT_ASSESSMENT_POSE, FUEL_APPROACH_CONSTRAINTS));
                },
                drive.getDefaultCommand() != null
                    ? drive.getDefaultCommand().getRequirements()
                    : java.util.Set.of(drive)))
        .finallyDo(
            () -> {
              if (intake != null) {
                intake.stopRollers();
              }
            })
        .withName("AssessAndCleanupDepot");
  }
}
