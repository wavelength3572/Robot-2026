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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  // Vision sim throttle — update sim at 25Hz instead of 50Hz (simulates ALL cameras, expensive)
  private boolean visionSimFrame = false;

  // Pre-computed camera key prefixes to avoid repeated string concatenation
  private final String[] cameraKeyPrefixes;
  private final String[] cameraRejectionPrefixes;

  // Robot speed supplier for adaptive std dev scaling.
  // Higher speeds increase measurement uncertainty from motion blur.
  private Supplier<Double> robotSpeedSupplier = () -> 0.0;

  private final Map<Integer, Double> aprilTagTimestamps = new ConcurrentHashMap<>();

  // Debug toggle: when true, per-observation rejection and std dev diagnostics are logged.
  // Default false for competition performance; enable via SmartDashboard for debugging.
  private static final String VERBOSE_VISION_KEY = "Debug/VerboseVisionLogging";

  static {
    SmartDashboard.putBoolean(VERBOSE_VISION_KEY, false);
  }

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

    // Pre-compute camera key prefixes
    cameraKeyPrefixes = new String[cameraNames.length];
    cameraRejectionPrefixes = new String[cameraNames.length];
    for (int i = 0; i < cameraNames.length; i++) {
      cameraKeyPrefixes[i] = "Vision/" + cameraNames[i] + "/";
      cameraRejectionPrefixes[i] = "Vision/" + cameraNames[i] + "/Rejection/";
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
    // In simulation, update the vision sim at 25Hz (every other cycle).
    // VisionSystemSim.update() simulates ALL cameras and is expensive (10-20ms).
    if (frc.robot.Constants.currentMode == frc.robot.Constants.Mode.SIM) {
      visionSimFrame = !visionSimFrame;
      if (visionSimFrame) {
        VisionIOPhotonVisionSim.updateSim();
      }
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

    // Read debug toggle once per cycle
    boolean verboseLogging = SmartDashboard.getBoolean(VERBOSE_VISION_KEY, false);

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

      String camPrefix = cameraKeyPrefixes[cameraIndex];
      String rejKey = cameraRejectionPrefixes[cameraIndex];

      Logger.recordOutput(
          camPrefix + "ObservationCount", inputs[cameraIndex].poseObservations.length);

      // Pre-compute speed scaling once per camera (same for all observations this cycle)
      double robotSpeed = robotSpeedSupplier.get();
      double speedScale = 1.0 + Math.min(robotSpeed / 3.0, 1.0) * 2.0;

      // Track last observation's rejection reasons for per-camera logging
      boolean lastNoTags = false;
      boolean lastTooAmbiguous = false;
      boolean lastZError = false;
      boolean lastOutsideFieldX = false;
      boolean lastOutsideFieldY = false;
      boolean lastSingleTagTooFar = false;
      boolean lastMultiTagTooFar = false;
      boolean lastRejectPose = false;

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Rejection conditions — broken out for per-camera diagnostics
        lastNoTags = observation.tagCount() == 0;
        lastTooAmbiguous = observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity;
        lastZError = Math.abs(observation.pose().getZ()) > maxZError;
        lastOutsideFieldX =
            observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength();
        lastOutsideFieldY =
            observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();
        lastSingleTagTooFar =
            observation.tagCount() == 1
                && observation.closestTagDistance() > MAX_SINGLE_TAG_DISTANCE;
        lastMultiTagTooFar =
            observation.tagCount() > 1 && observation.closestTagDistance() > MAX_MULTI_TAG_DISTANCE;

        lastRejectPose =
            lastNoTags
                || lastTooAmbiguous
                || lastZError
                || lastOutsideFieldX
                || lastOutsideFieldY
                || lastSingleTagTooFar
                || lastMultiTagTooFar;

        // Add pose to log
        robotPoses.add(observation.pose());
        if (lastRejectPose) {
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
        double closestDist = observation.closestTagDistance();
        double stdDevFactor = (closestDist * closestDist) / observation.tagCount();

        // Ambiguity scaling: observations with higher ambiguity are less trustworthy.
        double ambiguityScale = 1.0 + (observation.ambiguity() / maxAmbiguity) * 2.0;
        stdDevFactor *= ambiguityScale;

        // Speed scaling (pre-computed above)
        stdDevFactor *= speedScale;

        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        if (verboseLogging) {
          Logger.recordOutput(camPrefix + "StdDev/Linear", linearStdDev);
          Logger.recordOutput(camPrefix + "StdDev/Angular", angularStdDev);
          Logger.recordOutput(camPrefix + "StdDev/AmbiguityScale", ambiguityScale);
          Logger.recordOutput(camPrefix + "StdDev/SpeedScale", speedScale);
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log rejection reasons once per camera (last observation's state, gated for performance)
      if (verboseLogging) {
        Logger.recordOutput(rejKey + "NoTags", lastNoTags);
        Logger.recordOutput(rejKey + "TooAmbiguous", lastTooAmbiguous);
        Logger.recordOutput(rejKey + "ZError", lastZError);
        Logger.recordOutput(rejKey + "OutsideFieldX", lastOutsideFieldX);
        Logger.recordOutput(rejKey + "OutsideFieldY", lastOutsideFieldY);
        Logger.recordOutput(rejKey + "SingleTagTooFar", lastSingleTagTooFar);
        Logger.recordOutput(rejKey + "MultiTagTooFar", lastMultiTagTooFar);
        Logger.recordOutput(rejKey + "Rejected", lastRejectPose);
      }

      // Log camera data
      Logger.recordOutput(camPrefix + "TagPoses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          camPrefix + "RobotPoses", robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          camPrefix + "RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          camPrefix + "RobotPosesRejected",
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

    // Log contributing tags per camera (verbose — gated for performance)
    if (verboseLogging) {
      for (int i = 0; i < io.length; i++) {
        Logger.recordOutput(
            "Vision/Summary/" + cameraNames[i] + "/ContributingTags",
            perCameraContributingTags
                .get(i)
                .toArray(new Pose3d[perCameraContributingTags.get(i).size()]));
      }
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
