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
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private boolean isVisionOn = true;

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

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
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

  private void updateMainPipeline() {
    // In simulation, update the vision sim ONCE before processing any cameras
    // (VisionSystemSim.update() simulates ALL cameras at once)
    if (frc.robot.Constants.currentMode == frc.robot.Constants.Mode.SIM) {
      VisionIOPhotonVisionSim.updateSim();
    }

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
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
          "Vision/Camera" + cameraIndex + "/ObservationCount",
          inputs[cameraIndex].poseObservations.length);

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Rejection conditions
        boolean rejectPose =
            observation.tagCount() == 0
                || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
                || Math.abs(observation.pose().getZ()) > maxZError
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth()
                || observation.closestTagDistance() > MAX_TAG_DISTANCE;

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

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.closestTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        Logger.recordOutput("Vision/StdDev/LinearStdDev" + cameraIndex, linearStdDev);
        Logger.recordOutput("Vision/StdDev/AngularStdDev" + cameraIndex, angularStdDev);

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera data
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));

    // Log contributing tags per camera
    for (int i = 0; i < io.length; i++) {
      Logger.recordOutput(
          "Vision/Summary/Camera" + i + "/ContributingTags",
          perCameraContributingTags.get(i).toArray(new Pose3d[0]));
    }

    // New logging: Tags used for accepted/rejected poses
    Logger.recordOutput("Vision/Summary/TagPosesAccepted", tagPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/TagPosesRejected", tagPosesRejected.toArray(new Pose3d[0]));

    // Get robot pose for camera visualization (transforms cameras to world coordinates)
    Pose3d robotPose3d = new Pose3d(RobotStatus.getRobotPose());

    // Check if using proposed positions (in sim mode)
    boolean usingProposed =
        frc.robot.Constants.currentMode == frc.robot.Constants.Mode.SIM
            && VisionIOPhotonVisionSim.isUsingRecommendedPositions();
    Logger.recordOutput("Vision/CameraViz/UsingProposedPositions", usingProposed);

    // Select camera transforms based on robot type
    Transform3d frontLeftCamTransform;
    Transform3d frontRightCamTransform;
    Transform3d backLeftCamTransform;
    Transform3d backRightCamTransform;
    Transform3d frontCenterCamTransform = null; // Only for MainBot

    if (frc.robot.Constants.currentRobot == frc.robot.Constants.RobotType.MAINBOT) {
      // MainBot camera transforms (5 cameras)
      frontLeftCamTransform = VisionConstants.mainBotToFrontLeftCam;
      frontRightCamTransform = VisionConstants.mainBotToFrontRightCam;
      backLeftCamTransform = VisionConstants.mainBotToBackLeftCam;
      backRightCamTransform = VisionConstants.mainBotToBackRightCam;
      frontCenterCamTransform = VisionConstants.mainBotToFrontCenterCam;
    } else {
      // SquareBot camera transforms (4 cameras)
      frontLeftCamTransform = robotToFrontLeftCam;
      frontRightCamTransform = robotToFrontRightCam;
      backLeftCamTransform = robotToBackLeftCam;
      backRightCamTransform = robotToBackRightCam;
    }

    // Log each camera position individually with meaningful names for AdvantageScope
    Pose3d frontLeftPose = robotPose3d.transformBy(frontLeftCamTransform);
    Pose3d frontRightPose = robotPose3d.transformBy(frontRightCamTransform);
    Pose3d backLeftPose = robotPose3d.transformBy(backLeftCamTransform);
    Pose3d backRightPose = robotPose3d.transformBy(backRightCamTransform);

    Logger.recordOutput("Vision/Cameras/FrontLeft", frontLeftPose);
    Logger.recordOutput("Vision/Cameras/FrontRight", frontRightPose);
    Logger.recordOutput("Vision/Cameras/BackLeft", backLeftPose);
    Logger.recordOutput("Vision/Cameras/BackRight", backRightPose);

    // Log all corner cameras as array for combined visualization
    if (frc.robot.Constants.currentRobot == frc.robot.Constants.RobotType.MAINBOT
        && frontCenterCamTransform != null) {
      // MainBot has 5 cameras
      Pose3d frontCenterPose = robotPose3d.transformBy(frontCenterCamTransform);
      Logger.recordOutput("Vision/Cameras/FrontCenter", frontCenterPose);
      Logger.recordOutput(
          "Vision/CameraViz/AllCameras",
          new Pose3d[] {
            frontLeftPose, frontRightPose, backLeftPose, backRightPose, frontCenterPose
          });
    } else {
      // SquareBot has 4 cameras
      Logger.recordOutput(
          "Vision/CameraViz/AllCameras",
          new Pose3d[] {frontLeftPose, frontRightPose, backLeftPose, backRightPose});
    }
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
