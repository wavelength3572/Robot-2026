// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RobotStatus;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final String[] cameraNames;

  private boolean isVisionOn = true;

  // Robot speed supplier for adaptive std dev scaling.
  // Higher speeds increase measurement uncertainty from motion blur.
  private Supplier<Double> robotSpeedSupplier = () -> 0.0;

  private final Map<Integer, Double> aprilTagTimestamps = new ConcurrentHashMap<>();

  // Pre-allocated lists to reduce GC pressure (cleared each cycle)
  private final List<Pose3d> allTagPoses = new ArrayList<>(32);
  private final List<Pose3d> allRobotPoses = new ArrayList<>(16);
  private final List<Pose3d> allRobotPosesAccepted = new ArrayList<>(16);
  private final List<Pose3d> allRobotPosesRejected = new ArrayList<>(16);
  private final List<Pose3d> tagPosesAccepted = new ArrayList<>(32);
  private final List<Pose3d> tagPosesRejected = new ArrayList<>(32);
  private final List<List<Pose3d>> perCameraTagPoses;
  private final List<List<Pose3d>> perCameraRobotPoses;
  private final List<List<Pose3d>> perCameraRobotPosesAccepted;
  private final List<List<Pose3d>> perCameraRobotPosesRejected;
  private final List<List<Pose3d>> perCameraContributingTags;

  public Vision(VisionConsumer consumer, String[] cameraNames, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;
    this.cameraNames = cameraNames;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert("Vision camera " + cameraNames[i] + " is disconnected.", AlertType.kWarning);
    }

    // Initialize per-camera lists
    perCameraTagPoses = new ArrayList<>(io.length);
    perCameraRobotPoses = new ArrayList<>(io.length);
    perCameraRobotPosesAccepted = new ArrayList<>(io.length);
    perCameraRobotPosesRejected = new ArrayList<>(io.length);
    perCameraContributingTags = new ArrayList<>(io.length);
    for (int i = 0; i < io.length; i++) {
      perCameraTagPoses.add(new ArrayList<>(8));
      perCameraRobotPoses.add(new ArrayList<>(4));
      perCameraRobotPosesAccepted.add(new ArrayList<>(4));
      perCameraRobotPosesRejected.add(new ArrayList<>(4));
      perCameraContributingTags.add(new ArrayList<>(8));
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Vision/isVisionOn", isVisionOn);

    // Skip all vision processing if vision is disabled
    if (!isVisionOn) {
      return;
    }

    // Main pipeline using MegaTag 2 pose estimation - best for multi-tag scenarios
    updateMainPipeline();
  }

  /** Returns whether vision processing is currently enabled. */
  public boolean isVisionOn() {
    return isVisionOn;
  }

  /** Toggles vision processing on/off. */
  public void toggleVision() {
    isVisionOn = !isVisionOn;
  }

  /** Enables vision processing. */
  public void setVisionOn() {
    isVisionOn = true;
  }

  /** Disables vision processing. */
  public void setVisionOff() {
    isVisionOn = false;
  }

  /**
   * Set the robot speed supplier for adaptive standard deviation scaling. Higher robot speeds
   * increase vision measurement uncertainty due to motion blur.
   *
   * @param speedSupplier Supplies robot speed in m/s (magnitude of chassis velocity)
   */
  public void setRobotSpeedSupplier(Supplier<Double> speedSupplier) {
    this.robotSpeedSupplier = speedSupplier;
  }

  private void updateMainPipeline() {
    // In simulation, update the vision sim ONCE before processing any cameras
    // (VisionSystemSim.update() simulates ALL cameras at once)
    if (frc.robot.Constants.currentMode == frc.robot.Constants.Mode.SIM) {
      VisionIOPhotonVisionSim.updateSim();
    }

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + cameraNames[i], inputs[i]);
    }

    // Clear pre-allocated lists for this cycle
    allTagPoses.clear();
    allRobotPoses.clear();
    allRobotPosesAccepted.clear();
    allRobotPosesRejected.clear();
    tagPosesAccepted.clear();
    tagPosesRejected.clear();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Get pre-allocated per-camera lists and clear them
      List<Pose3d> tagPoses = perCameraTagPoses.get(cameraIndex);
      List<Pose3d> robotPoses = perCameraRobotPoses.get(cameraIndex);
      List<Pose3d> robotPosesAccepted = perCameraRobotPosesAccepted.get(cameraIndex);
      List<Pose3d> robotPosesRejected = perCameraRobotPosesRejected.get(cameraIndex);
      List<Pose3d> contributingTagsForCamera = perCameraContributingTags.get(cameraIndex);
      tagPoses.clear();
      robotPoses.clear();
      robotPosesAccepted.clear();
      robotPosesRejected.clear();
      contributingTagsForCamera.clear();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      Logger.recordOutput(
          "Vision/" + cameraNames[cameraIndex] + "/ObservationCount",
          inputs[cameraIndex].poseObservations.length);

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Rejection conditions — broken out for per-camera diagnostics
        boolean noTags = observation.tagCount() == 0;
        boolean tooAmbiguous =
            observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity;
        boolean zError = Math.abs(observation.pose().getZ()) > maxZError;
        boolean outsideFieldX =
            observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength();
        boolean outsideFieldY =
            observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();
        boolean singleTagTooFar =
            observation.tagCount() == 1
                && observation.closestTagDistance() > MAX_SINGLE_TAG_DISTANCE;
        boolean multiTagTooFar =
            observation.tagCount() > 1 && observation.closestTagDistance() > MAX_MULTI_TAG_DISTANCE;

        boolean rejectPose =
            noTags
                || tooAmbiguous
                || zError
                || outsideFieldX
                || outsideFieldY
                || singleTagTooFar
                || multiTagTooFar;

        // Log rejection reasons per camera
        String rejKey = "Vision/" + cameraNames[cameraIndex] + "/Rejection/";
        Logger.recordOutput(rejKey + "NoTags", noTags);
        Logger.recordOutput(rejKey + "TooAmbiguous", tooAmbiguous);
        Logger.recordOutput(rejKey + "ZError", zError);
        Logger.recordOutput(rejKey + "OutsideFieldX", outsideFieldX);
        Logger.recordOutput(rejKey + "OutsideFieldY", outsideFieldY);
        Logger.recordOutput(rejKey + "SingleTagTooFar", singleTagTooFar);
        Logger.recordOutput(rejKey + "MultiTagTooFar", multiTagTooFar);
        Logger.recordOutput(rejKey + "Rejected", rejectPose);
        // Log raw values for debugging thresholds
        Logger.recordOutput(rejKey + "Ambiguity", observation.ambiguity());
        Logger.recordOutput(rejKey + "TagCount", observation.tagCount());
        Logger.recordOutput(rejKey + "ClosestTagDist", observation.closestTagDistance());
        Logger.recordOutput(rejKey + "PoseZ", observation.pose().getZ());

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
          // Add contributing tags to rejected list
          for (int tagId : inputs[cameraIndex].tagIds) {
            var tagPose = aprilTagLayout.getTagPose(tagId);
            if (tagPose.isPresent()) {
              tagPosesRejected.add(tagPose.get());
            }
          }
          continue;
        }

        // Accepted pose - add contributing tags
        for (int tagId : inputs[cameraIndex].tagIds) {
          var tagPose = aprilTagLayout.getTagPose(tagId);
          if (tagPose.isPresent()) {
            tagPosesAccepted.add(tagPose.get());
            contributingTagsForCamera.add(tagPose.get());
          }
          logAprilTagDetection(tagId);
        }
        robotPosesAccepted.add(observation.pose());

        // Calculate standard deviations using adaptive scaling:
        // - Base: distance² / tagCount (existing heuristic)
        // - Ambiguity: scale up for ambiguous single-tag observations
        // - Speed: scale up at high robot speeds (motion blur degrades image quality)
        double stdDevFactor =
            Math.pow(observation.closestTagDistance(), 2.0) / observation.tagCount();

        // Ambiguity scaling: observations with higher ambiguity are less trustworthy.
        // For multi-tag (ambiguity ~0), this adds ~0%. For single-tag at 0.25 ambiguity,
        // this adds ~50% uncertainty. Scales linearly from 0 to 2x at maxAmbiguity.
        double ambiguityScale = 1.0 + (observation.ambiguity() / maxAmbiguity) * 2.0;
        stdDevFactor *= ambiguityScale;

        // Speed scaling: at high speeds, camera images are blurred and less reliable.
        // Adds up to 2x uncertainty at 3+ m/s. No effect when stationary.
        double robotSpeed = robotSpeedSupplier.get();
        double speedScale = 1.0 + Math.min(robotSpeed / 3.0, 1.0) * 2.0;
        stdDevFactor *= speedScale;

        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        Logger.recordOutput("Vision/" + cameraNames[cameraIndex] + "/StdDev/Linear", linearStdDev);
        Logger.recordOutput(
            "Vision/" + cameraNames[cameraIndex] + "/StdDev/Angular", angularStdDev);
        Logger.recordOutput(
            "Vision/" + cameraNames[cameraIndex] + "/StdDev/AmbiguityScale", ambiguityScale);
        Logger.recordOutput(
            "Vision/" + cameraNames[cameraIndex] + "/StdDev/SpeedScale", speedScale);

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera data
      Logger.recordOutput(
          "Vision/" + cameraNames[cameraIndex] + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/" + cameraNames[cameraIndex] + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/" + cameraNames[cameraIndex] + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/" + cameraNames[cameraIndex] + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

    // Log contributing tags per camera
    for (int i = 0; i < io.length; i++) {
      Logger.recordOutput(
          "Vision/Summary/" + cameraNames[i] + "/ContributingTags",
          perCameraContributingTags
              .get(i)
              .toArray(new Pose3d[perCameraContributingTags.get(i).size()]));
    }

    // New logging: Tags used for accepted/rejected poses
    Logger.recordOutput(
        "Vision/Summary/TagPosesAccepted",
        tagPosesAccepted.toArray(new Pose3d[tagPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/TagPosesRejected",
        tagPosesRejected.toArray(new Pose3d[tagPosesRejected.size()]));

    // Get robot pose for camera visualization (transforms cameras to world coordinates)
    Pose2d currentRobotPose = RobotStatus.getRobotPose();
    Pose3d robotPose3d = new Pose3d(currentRobotPose);

    // Select camera transforms based on robot type
    Transform3d centerRearCamTransform;
    Transform3d rightFrontCamTransform;
    Transform3d leftRearCamTransform;
    Transform3d rightRearCamTransform;
    if (frc.robot.Constants.currentRobot == frc.robot.Constants.RobotType.MAINBOT) {
      centerRearCamTransform = VisionConstants.mainBotToCenterRearCam;
      rightFrontCamTransform = VisionConstants.mainBotToRightFrontCam;
      leftRearCamTransform = VisionConstants.mainBotToLeftRearCam;
      rightRearCamTransform = VisionConstants.mainBotToRightRearCam;
    } else {
      centerRearCamTransform = robotToCenterRearCam;
      rightFrontCamTransform = robotToRightFrontCam;
      leftRearCamTransform = robotToLeftRearCam;
      rightRearCamTransform = robotToRightRearCam;
    }

    // Log all cameras as array for combined visualization
    Pose3d centerRearPose = robotPose3d.transformBy(centerRearCamTransform);
    Pose3d rightFrontPose = robotPose3d.transformBy(rightFrontCamTransform);
    Pose3d leftRearPose = robotPose3d.transformBy(leftRearCamTransform);
    Pose3d rightRearPose = robotPose3d.transformBy(rightRearCamTransform);

    Logger.recordOutput(
        "Vision/CameraViz/AllCameras",
        new Pose3d[] {centerRearPose, rightFrontPose, leftRearPose, rightRearPose});
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  /**
   * Logs an AprilTag detection with the current timestamp (using FPGATimestamp for consistency).
   */
  public void logAprilTagDetection(int tagId) {
    aprilTagTimestamps.put(tagId, Timer.getFPGATimestamp()); // Use FPGA time
  }

  /** Checks if the given tagId was seen within the last maxAgeSeconds. */
  public boolean hasRecentlySeenAprilTag(int tagId, double maxAgeSeconds) {
    Double lastSeenTime = aprilTagTimestamps.get(tagId);
    if (lastSeenTime == null) {
      return false;
    }
    return (Timer.getFPGATimestamp() - lastSeenTime) <= maxAgeSeconds;
  }
}
