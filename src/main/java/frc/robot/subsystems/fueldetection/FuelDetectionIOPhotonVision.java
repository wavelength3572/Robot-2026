package frc.robot.subsystems.fueldetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

/**
 * Reads fuel detection data from PhotonVision's object detection pipeline. Uses the game piece
 * detection ML model running on a coprocessor (Raspberry Pi) with PhotonVision.
 *
 * <p>The camera's yaw/pitch to the detected game piece, combined with the camera's known mounting
 * position and the robot's field pose, is used to compute the fuel's field-relative position.
 */
public class FuelDetectionIOPhotonVision implements FuelDetectionIO {
  private final PhotonCamera camera;
  private final Transform3d robotToCamera;
  private final Supplier<Pose2d> robotPoseSupplier;

  /**
   * @param cameraName The name of the PhotonVision camera running object detection
   * @param robotToCamera Transform from robot center to the camera
   * @param robotPoseSupplier Supplies the robot's current field-relative pose (for projecting
   *     detections to field coordinates)
   */
  public FuelDetectionIOPhotonVision(
      String cameraName, Transform3d robotToCamera, Supplier<Pose2d> robotPoseSupplier) {
    this.camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
    this.robotPoseSupplier = robotPoseSupplier;
  }

  @Override
  public void updateInputs(FuelDetectionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    var results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      // No new frames - keep previous hasFuelTarget state but it will age out via timestamp
      return;
    }

    // Use the most recent result
    var latestResult = results.get(results.size() - 1);

    if (!latestResult.hasTargets()) {
      inputs.hasFuelTarget = false;
      inputs.confidence = 0.0;
      return;
    }

    var bestTarget = latestResult.getBestTarget();
    double yawDeg = bestTarget.getYaw();
    double pitchDeg = bestTarget.getPitch();

    // Camera mounting parameters
    double cameraHeightMeters = robotToCamera.getZ();
    double cameraPitchRadians = robotToCamera.getRotation().getY();

    // Fuel sits on the ground at ~0.075m radius (ball center height)
    double fuelHeightMeters = 0.075;

    // Calculate distance using pinhole camera geometry:
    // distance = (cameraHeight - fuelHeight) / tan(cameraPitch + targetPitch)
    double totalPitchRadians = cameraPitchRadians + Math.toRadians(pitchDeg);
    double heightDifference = cameraHeightMeters - fuelHeightMeters;

    // Guard against zero/negative pitch (target at or above camera horizon)
    if (totalPitchRadians <= 0.01) {
      inputs.hasFuelTarget = false;
      inputs.confidence = 0.0;
      return;
    }

    double groundDistance = heightDifference / Math.tan(totalPitchRadians);

    // Reject unreasonable distances
    if (groundDistance <= 0.0 || groundDistance > 6.0) {
      inputs.hasFuelTarget = false;
      inputs.confidence = 0.0;
      return;
    }

    // Compute field-relative position of the fuel
    Pose2d robotPose = robotPoseSupplier.get();
    double yawRadians = Math.toRadians(yawDeg);

    // Camera-relative offset: forward = groundDistance, left = groundDistance * tan(yaw)
    // The yaw from PhotonVision is positive-left (counterclockwise)
    double cameraForward = groundDistance;
    double cameraLeft = groundDistance * Math.tan(yawRadians);

    // Transform from camera-relative to robot-relative
    double cameraYawOnRobot = robotToCamera.getRotation().getZ();
    double cosYaw = Math.cos(cameraYawOnRobot);
    double sinYaw = Math.sin(cameraYawOnRobot);
    double robotRelX = robotToCamera.getX() + (cameraForward * cosYaw - cameraLeft * sinYaw);
    double robotRelY = robotToCamera.getY() + (cameraForward * sinYaw + cameraLeft * cosYaw);

    // Transform from robot-relative to field-relative
    Translation2d robotRelative = new Translation2d(robotRelX, robotRelY);
    Rotation2d robotHeading = robotPose.getRotation();
    Translation2d fieldRelative =
        robotRelative.rotateBy(robotHeading).plus(robotPose.getTranslation());

    inputs.hasFuelTarget = true;
    inputs.fuelFieldX = fieldRelative.getX();
    inputs.fuelFieldY = fieldRelative.getY();
    inputs.confidence = bestTarget.getArea() / 100.0; // area as rough confidence proxy
    inputs.timestamp = latestResult.getTimestampSeconds();
    inputs.distanceMeters = groundDistance;
    inputs.angleRadians = yawRadians;
  }
}
