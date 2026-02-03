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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public final class VisionConstants {

  public static double MAX_TAG_DISTANCE = 6.0; // Accept tags within 6 meters (was 1.5m)

  // AprilTag layout for 2026 Rebuilt field
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // _________________________________________________________________________________________________

  // Camera names, must match names configured on coprocessor
  // TODO: Update these camera names to match your PhotonVision coprocessor configuration
  public static String frontRightCam = "CAMERA_B";
  public static String backRightCam = "CAMERA_D";
  public static String frontLeftCam = "CAMERA_A";
  public static String backLeftCam = "CAMERA_C";
  public static String frontCenterCam = "CAMERA_E"; // Intake/object detection camera

  // Robot to camera transforms
  // TODO: Calibrate these transforms for your 2026 robot - these are placeholders
  // All cameras are mounted near swerve pods at the four corners of the robot
  public static Transform3d robotToFrontLeftCam =
      new Transform3d(
          0.26985,
          0.26981,
          0.22155,
          new Rotation3d(0.0, Rotation2d.fromDegrees(-15).getRadians(), 0.0));

  public static Transform3d robotToFrontRightCam =
      new Transform3d(
          0.26985,
          -0.26981,
          0.22155,
          new Rotation3d(0.0, Rotation2d.fromDegrees(-15).getRadians(), 0.0));

  public static Transform3d robotToBackLeftCam =
      new Transform3d(
          -0.26985,
          0.26981,
          0.22155,
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-15).getRadians(),
              Rotation2d.fromDegrees(180).getRadians()));

  public static Transform3d robotToBackRightCam =
      new Transform3d(
          -0.26985,
          -0.26981,
          0.22155,
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-15).getRadians(),
              Rotation2d.fromDegrees(180).getRadians()));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  // Order matches camera instantiation: A (FrontLeft), B (FrontRight), C (BackLeft), D (BackRight),
  // E
  // (FrontCenter)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // FrontLeft (CAMERA_A)
        1.0, // FrontRight (CAMERA_B)
        1.0, // BackLeft (CAMERA_C)
        1.0, // BackRight (CAMERA_D)
        1.0 // FrontCenter (CAMERA_E) - MainBot only
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  // _________________________________________________________________________________________________
  // MAINBOT camera transforms - rectangular chassis (31" wide x 23.5" deep)
  // Cameras at corners, aimed diagonally outward at 45° angles
  // Height: 20.25" = 0.5144m, slight downward pitch (-10°) to see tags at various heights

  // MainBot corner positions: wheelBase/2 = 0.298m, trackWidth/2 = 0.394m
  public static Transform3d mainBotToFrontLeftCam =
      new Transform3d(
          0.298, // X: front of robot
          0.394, // Y: left side
          0.5144, // Z: 20.25 inches
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-10).getRadians(), // Pitch down 10°
              Rotation2d.fromDegrees(45).getRadians())); // Yaw outward 45°

  public static Transform3d mainBotToFrontRightCam =
      new Transform3d(
          0.298, // X: front of robot
          -0.394, // Y: right side
          0.5144, // Z: 20.25 inches
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-10).getRadians(), // Pitch down 10°
              Rotation2d.fromDegrees(-45).getRadians())); // Yaw outward -45°

  public static Transform3d mainBotToBackLeftCam =
      new Transform3d(
          -0.298, // X: back of robot
          0.394, // Y: left side
          0.5144, // Z: 20.25 inches
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-10).getRadians(), // Pitch down 10°
              Rotation2d.fromDegrees(135).getRadians())); // Yaw outward 135°

  public static Transform3d mainBotToBackRightCam =
      new Transform3d(
          -0.298, // X: back of robot
          -0.394, // Y: right side
          0.5144, // Z: 20.25 inches
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-10).getRadians(), // Pitch down 10°
              Rotation2d.fromDegrees(-135).getRadians())); // Yaw outward -135°

  // MainBot front center camera - for intake/object detection
  // Positioned at front center, looking straight ahead with downward pitch
  public static Transform3d mainBotToFrontCenterCam =
      new Transform3d(
          0.298, // X: front of robot
          0.0, // Y: centered
          0.5144, // Z: 20.25 inches
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(10).getRadians(), // Pitch DOWN 10° (positive = down)
              Rotation2d.fromDegrees(0).getRadians())); // Yaw straight ahead

  // MainBot recommended transforms (for testing/comparison in sim)
  public static Transform3d mainBotRecommendedFrontLeftCam = mainBotToFrontLeftCam;
  public static Transform3d mainBotRecommendedFrontRightCam = mainBotToFrontRightCam;
  public static Transform3d mainBotRecommendedBackLeftCam = mainBotToBackLeftCam;
  public static Transform3d mainBotRecommendedBackRightCam = mainBotToBackRightCam;
  public static Transform3d mainBotRecommendedFrontCenterCam = mainBotToFrontCenterCam;

  // _________________________________________________________________________________________________
  // SQUAREBOT RECOMMENDED camera transforms for 2026 Rebuilt field (for visualization/comparison)
  // These are positioned to maximize AprilTag visibility across the field:
  // - Front cameras: pitched UP (+10°) to see HUB tags at 1.12m height
  // - Rear cameras: pitched DOWN (-10°) to see OUTPOST/TOWER tags at 0.55m height
  // - All cameras angled outward for ~360° coverage

  public static Transform3d recommendedFrontLeftCam =
      new Transform3d(
          0.26985,
          0.26981,
          0.43,
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-10).getRadians(), // Pitch UP 10° (negative pitch = look up)
              Rotation2d.fromDegrees(120).getRadians())); // Yaw left 30°

  public static Transform3d recommendedFrontRightCam =
      new Transform3d(
          0.26985,
          -0.26981,
          0.43,
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-10).getRadians(), // Pitch UP 10°
              Rotation2d.fromDegrees(-120).getRadians())); // Yaw right 30°

  public static Transform3d recommendedBackLeftCam =
      new Transform3d(
          -0.26985,
          0.26981,
          0.43,
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-10)
                  .getRadians(), // Pitch DOWN 10° (positive pitch = look down)
              Rotation2d.fromDegrees(60).getRadians())); // Yaw back-left 150°

  public static Transform3d recommendedBackRightCam =
      new Transform3d(
          -0.26985,
          -0.26981,
          0.43,
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-10).getRadians(), // Pitch DOWN 10°
              Rotation2d.fromDegrees(-60).getRadians())); // Yaw back-right 150°

  private VisionConstants() {}
}
