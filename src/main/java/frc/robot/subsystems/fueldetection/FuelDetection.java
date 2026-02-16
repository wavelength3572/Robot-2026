package frc.robot.subsystems.fueldetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that receives fuel position data from a Raspberry Pi coprocessor. The Pi runs a trained
 * object detection model, calculates the detected fuel's field-relative position using camera
 * geometry and the robot's known pose, and publishes the result to NetworkTables.
 *
 * <p>This subsystem provides the detected fuel position to commands that can use PathPlanner's
 * on-the-fly pathfinding to drive the robot directly to the fuel.
 */
public class FuelDetection extends SubsystemBase {
  /** Minimum confidence threshold to trust a detection. */
  private static final double MIN_CONFIDENCE = 0.5;

  /** Maximum age (seconds) before a detection is considered stale. */
  private static final double MAX_DETECTION_AGE_SECONDS = 0.5;

  /** Approach offset: stop this far from the fuel center so the intake can grab it. */
  private static final double APPROACH_OFFSET_METERS = 0.4;

  private final FuelDetectionIO io;
  private final FuelDetectionIOInputsAutoLogged inputs = new FuelDetectionIOInputsAutoLogged();
  private final Alert disconnectedAlert =
      new Alert("Fuel detection coprocessor disconnected.", AlertType.kWarning);

  public FuelDetection(FuelDetectionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("FuelDetection", inputs);
    disconnectedAlert.set(!inputs.connected);
  }

  /** Returns whether the coprocessor is connected. */
  @AutoLogOutput(key = "FuelDetection/Connected")
  public boolean isConnected() {
    return inputs.connected;
  }

  /** Returns whether a valid, recent fuel target is available. */
  @AutoLogOutput(key = "FuelDetection/HasValidTarget")
  public boolean hasValidTarget() {
    if (!inputs.connected || !inputs.hasFuelTarget) return false;
    if (inputs.confidence < MIN_CONFIDENCE) return false;
    double age = Timer.getFPGATimestamp() - inputs.timestamp;
    return age < MAX_DETECTION_AGE_SECONDS;
  }

  /** Returns the field-relative position of the detected fuel, or empty if no valid target. */
  public Optional<Translation2d> getFuelPosition() {
    if (!hasValidTarget()) return Optional.empty();
    return Optional.of(new Translation2d(inputs.fuelFieldX, inputs.fuelFieldY));
  }

  /** Returns the distance to the detected fuel in meters, or 0 if no valid target. */
  @AutoLogOutput(key = "FuelDetection/DistanceMeters")
  public double getDistanceMeters() {
    return hasValidTarget() ? inputs.distanceMeters : 0.0;
  }

  /** Returns the detection confidence, or 0 if no valid target. */
  @AutoLogOutput(key = "FuelDetection/Confidence")
  public double getConfidence() {
    return inputs.confidence;
  }

  /**
   * Computes the target pose for the robot to drive to in order to pick up the detected fuel. The
   * robot is oriented to face the fuel so the intake (front of robot) points at it.
   *
   * @param robotPosition Current robot position (used to compute approach heading)
   * @return Target Pose2d for pathfinding, or empty if no valid target
   */
  public Optional<Pose2d> getPickupPose(Translation2d robotPosition) {
    Optional<Translation2d> fuelPos = getFuelPosition();
    if (fuelPos.isEmpty()) return Optional.empty();

    Translation2d fuel = fuelPos.get();
    // Heading from robot toward fuel
    Rotation2d heading = fuel.minus(robotPosition).getAngle();

    // Offset position: stop APPROACH_OFFSET_METERS before the fuel center
    Translation2d offset =
        new Translation2d(APPROACH_OFFSET_METERS, heading.plus(Rotation2d.kPi));
    Translation2d targetPosition = fuel.plus(offset);

    return Optional.of(new Pose2d(targetPosition, heading));
  }
}
