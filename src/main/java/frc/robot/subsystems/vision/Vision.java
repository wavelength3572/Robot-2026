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
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionConsumer consumer2;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private Boolean isVisionOn = true;

  private final Map<Integer, Double> aprilTagTimestamps = new ConcurrentHashMap<>();
  private static double lastLogTime = 0;
  private static final double LOG_INTERVAL_SECONDS = 10.0; // Log every 10 seconds

  // An array of PhotonPoseEstimators, one per camera
  private final PhotonPoseEstimator[] photonEstimators;

  public Vision(VisionConsumer consumer, VisionConsumer consumer2, VisionIO... io) {
    this.consumer = consumer;
    this.consumer2 = consumer2;
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

    // ----------------------------------------------------------------
    // 1) Create PhotonPoseEstimators, one per camera
    // ----------------------------------------------------------------
    photonEstimators = new PhotonPoseEstimator[io.length];
    for (int i = 0; i < io.length; i++) {
      photonEstimators[i] = createPhotonEstimatorForCamera(i);
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

    // Existing pipeline for consumer
    updateMainPipeline(); // your existing code

    // Separate pipeline for consumer2
    updatePhotonVisionEstimates();
  }

  private void updateMainPipeline() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    List<Pose3d> tagPosesAccepted = new LinkedList<>();
    List<Pose3d> tagPosesRejected = new LinkedList<>();

    // Track tag contributions per camera
    Map<Integer, List<Pose3d>> tagContributionsByCamera = new HashMap<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      List<Pose3d> contributingTagsForCamera = new LinkedList<>();

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
        boolean rejectPose = false;
        String rejectReason = null;

        // Rejection conditions
        if (observation.tagCount() == 0) {
          rejectReason = "No tags detected";
        } else if (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) {
          rejectReason = "Single tag with high ambiguity: " + observation.ambiguity();
        } else if (Math.abs(observation.pose().getZ()) > maxZError) {
          rejectReason = "Pose Z out of bounds: " + observation.pose().getZ();
        } else if (observation.pose().getX() < 0.0) {
          rejectReason = "Pose X is negative: " + observation.pose().getX();
        } else if (observation.pose().getX() > aprilTagLayout.getFieldLength()) {
          rejectReason = "Pose X exceeds field length: " + observation.pose().getX();
        } else if (observation.pose().getY() < 0.0) {
          rejectReason = "Pose Y is negative: " + observation.pose().getY();
        } else if (observation.pose().getY() > aprilTagLayout.getFieldWidth()) {
          rejectReason = "Pose Y exceeds field width: " + observation.pose().getY();
        } else if (observation.closestTagDistance() > MAX_TAG_DISTANCE) {
          rejectReason = "Closest tag distance too high: " + observation.closestTagDistance();
        }

        // Aggregate rejections per cycle
        List<String> rejectionReasons = new ArrayList<>();

        if (rejectReason != null) {
          rejectionReasons.add(rejectReason);
          rejectPose = true;
        }

        // Log summary once per cycle
        // if (shouldLogSummary()) {
        //   Logger.recordOutput(
        //       "Vision/RejectSummary",
        //       "Cycle Rejections: "
        //           + rejectionReasons.size()
        //           + " | Unique Reasons: "
        //           + new HashSet<>(rejectionReasons));
        //   rejectionReasons.clear();
        // }

        // Track which tags contributed to this observation
        List<Pose3d> contributingTags = new LinkedList<>();
        for (int tagId : inputs[cameraIndex].tagIds) {
          var tagPose = aprilTagLayout.getTagPose(tagId);
          if (tagPose.isPresent()) {
            contributingTags.add(tagPose.get());
          }
        }

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
          tagPosesRejected.addAll(contributingTags);
        } else {
          robotPosesAccepted.add(observation.pose());
          tagPosesAccepted.addAll(contributingTags);
          contributingTagsForCamera.addAll(contributingTags); // Add to per-camera tracker
          // ✅ Log AprilTag timestamps ONLY if the pose was ACCEPTED
          for (int tagId : inputs[cameraIndex].tagIds) {
            logAprilTagDetection(tagId);
          }
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

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

      // Save contributing tags per camera
      tagContributionsByCamera.put(cameraIndex, contributingTagsForCamera);

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

    // New logging: Summary of contributing tags per camera
    for (var entry : tagContributionsByCamera.entrySet()) {
      int cameraIndex = entry.getKey();
      List<Pose3d> contributingTags = entry.getValue();
      Logger.recordOutput(
          "Vision/Summary/Camera" + cameraIndex + "/ContributingTags",
          contributingTags.toArray(new Pose3d[0]));
    }

    // New logging: Tags used for accepted/rejected poses
    Logger.recordOutput("Vision/Summary/TagPosesAccepted", tagPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/TagPosesRejected", tagPosesRejected.toArray(new Pose3d[0]));
  }

  private void updatePhotonVisionEstimates() {

    // Example logging structures
    List<Pose3d> allPhotonRobotPoses = new LinkedList<>();
    List<Pose3d> allPhotonRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allPhotonRobotPosesRejected = new LinkedList<>();

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // We only get pipeline results if the IO is actually a VisionIOPhotonVision
      if (!(io[cameraIndex] instanceof VisionIOPhotonVision)) {
        continue;
      }
      VisionIOPhotonVision photonVisionIO = (VisionIOPhotonVision) io[cameraIndex];

      // Now fetch any new pipeline results
      var unreadResults = photonVisionIO.camera.getAllUnreadResults();

      // If we have a valid PhotonPoseEstimator
      PhotonPoseEstimator estimator = photonEstimators[cameraIndex];

      if (estimator == null) {
        continue;
      }

      // Process each pipeline result
      for (var result : unreadResults) {
        Rotation2d gyroAngle = RobotStatus.getRobotPose().getRotation();
        estimator.addHeadingData(result.getTimestampSeconds(), gyroAngle);
        // Let PhotonPoseEstimator produce a pose estimate
        var maybeEstimatedRobotPose = estimator.update(result);
        if (maybeEstimatedRobotPose.isEmpty()) {
          // No valid pose was found (no targets, or something else)
          continue;
        }

        // We have an estimated pose
        var estimatedRobotPose = maybeEstimatedRobotPose.get();
        Pose3d pose3d = estimatedRobotPose.estimatedPose;
        double timestamp = estimatedRobotPose.timestampSeconds;

        // *** Acceptance Logic (very similar to your main pipeline) ***
        int tagCount = estimatedRobotPose.targetsUsed.size();
        double averageTagDistance = computeAverageTagDistance(pose3d, estimatedRobotPose);

        boolean rejectPose =
            (tagCount == 0)
                || (tagCount == 1 && /* e.g. an ambiguity check, if you want */ false)
                || Math.abs(pose3d.getZ()) > maxZError
                || pose3d.getX() < 0.0
                || pose3d.getX() > aprilTagLayout.getFieldLength()
                || pose3d.getY() < 0.0
                || pose3d.getY() > aprilTagLayout.getFieldWidth();

        allPhotonRobotPoses.add(pose3d);

        if (rejectPose) {
          allPhotonRobotPosesRejected.add(pose3d);
        } else {
          allPhotonRobotPosesAccepted.add(pose3d);

          // Optionally compute standard deviations the same way you do in your main
          // pipeline
          double stdDevFactor = Math.pow(averageTagDistance, 2.0) / Math.max(tagCount, 1);
          double linearStdDev = linearStdDevBaseline * stdDevFactor;
          double angularStdDev = angularStdDevBaseline * stdDevFactor;
          if (cameraIndex < cameraStdDevFactors.length) {
            linearStdDev *= cameraStdDevFactors[cameraIndex];
            angularStdDev *= cameraStdDevFactors[cameraIndex];
          }

          Matrix<N3, N1> stdDevs = VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);

          // Forward accepted pose to consumer2
          consumer2.accept(pose3d.toPose2d(), timestamp, stdDevs);
        }
      }
    }

    // (Optional) Log summary
    Logger.recordOutput(
        "Vision/PhotonEstimator/Summary/RobotPoses", allPhotonRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/PhotonEstimator/Summary/RobotPosesAccepted",
        allPhotonRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/PhotonEstimator/Summary/RobotPosesRejected",
        allPhotonRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public boolean isVisionOn() {
    return isVisionOn;
  }

  public void setVisionOn() {
    isVisionOn = true;
  }

  public void setVisionOff() {
    isVisionOn = false;
  }

  public void toggleVision() {
    isVisionOn = !isVisionOn;
  }

  // Create an estimator for each camera using the names and transforms from
  // VisionConstants
  private PhotonPoseEstimator createPhotonEstimatorForCamera(int cameraIndex) {
    String cameraName;
    Transform3d cameraTransform;

    // Map index -> camera name & transform
    switch (cameraIndex) {
      case 0:
        cameraName = VisionConstants.frontRightCam;
        cameraTransform = VisionConstants.robotToFrontRightCam;
        break;
      case 1:
        cameraName = VisionConstants.backRightCam;
        cameraTransform = VisionConstants.robotToBackRightCam;
        break;
      case 2:
        cameraName = VisionConstants.frontLeftCam;
        cameraTransform = VisionConstants.robotToFrontLeftCam;
        break;
      case 3:
        cameraName = VisionConstants.elevatorBackCam;
        cameraTransform = VisionConstants.robotToElevatorBackCam;
        break;
      default:
        cameraName = "UnknownCamera";
        cameraTransform = new Transform3d();
        break;
    }

    PhotonPoseEstimator estimator =
        new PhotonPoseEstimator(
            VisionConstants.aprilTagLayout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, cameraTransform);

    return estimator;
  }

  /**
   * Example helper: compute average distance from the robot to each recognized AprilTag. Because
   * the official EstimatedRobotPose doesn't store 'averageTagDistance' directly, we can
   * re-calculate it or skip it if you don't need it.
   */
  private double computeAverageTagDistance(
      Pose3d robotPose, EstimatedRobotPose estimatedRobotPose) {
    if (estimatedRobotPose.targetsUsed.isEmpty()) {
      return 9999.0;
    }
    double sum = 0.0;
    for (var t : estimatedRobotPose.targetsUsed) {
      // If we want the distance from the robot to the actual field tag:
      var fieldTagPoseOpt = aprilTagLayout.getTagPose(t.getFiducialId());
      if (fieldTagPoseOpt.isPresent()) {
        Pose3d fieldTagPose = fieldTagPoseOpt.get();
        double dist = robotPose.getTranslation().getDistance(fieldTagPose.getTranslation());
        sum += dist;
      }
    }
    return sum / estimatedRobotPose.targetsUsed.size();
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

  private static boolean shouldLogSummary() {
    double currentTime = Timer.getFPGATimestamp();
    if (currentTime - lastLogTime >= LOG_INTERVAL_SECONDS) {
      lastLogTime = currentTime;
      return true;
    }
    return false;
  }
}
