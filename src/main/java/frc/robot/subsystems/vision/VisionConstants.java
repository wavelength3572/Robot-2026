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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class VisionConstants {

  public static double MAX_TAG_DISTANCE = 6.0; // Accept tags within 6 meters (was 1.5m)

  // AprilTag layout for 2026 Rebuilt field
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // _________________________________________________________________________________________________

  // Camera names, must match names configured on coprocessor
  public static String frontRightCam = "CAMERA_B";
  public static String backRightCam = "BackRight";
  public static String frontLeftCam = "CAMERA_A";
  public static String backLeftCam = "BackLeft";
  public static String objectDetectionFrontLeftCam =
      "ObjectDetectionFrontLeft"; // Object detection camera

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
  // Order matches camera instantiation: FrontLeft, FrontRight, BackLeft, BackRight,
  // ObjectDetectionFrontLeft
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // FrontLeft
        1.0, // FrontRight
        1.0, // BackLeft
        1.0, // BackRight
        1.0 // ObjectDetectionFrontLeft - MainBot only
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  // The standard deviations of our vision estimated poses, which affect
  // correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

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
              Rotation2d.fromDegrees(0).getRadians())); // Yaw straight forward

  public static Transform3d mainBotToFrontRightCam =
      new Transform3d(
          0.298, // X: front of robot
          -0.394, // Y: right side
          0.5144, // Z: 20.25 inches
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-16).getRadians(), // Pitch up 16°
              Rotation2d.fromDegrees(-83).getRadians())); // Yaw perpendicular right -83°

  public static Transform3d mainBotToBackLeftCam =
      new Transform3d(
          -0.2726, // X: back of robot (1" toward center)
          0.394, // Y: left side
          0.37465, // Z: 14.75 inches
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-21).getRadians(), // Pitch up 21°
              Rotation2d.fromDegrees(83).getRadians())); // Yaw left 83°

  public static Transform3d mainBotToBackRightCam =
      new Transform3d(
          -0.26670, // X: 10.5" behind center (from CAD)
          -0.35111, // Y: 13.823" right of center (from CAD)
          0.52093, // Z: 20.509" height (from CAD)
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-10).getRadians(), // Pitch 10° up from horizontal
              Rotation2d.fromDegrees(180).getRadians())); // Yaw straight backward

  // MainBot object detection camera - front left
  // Mounted just below frame near front left swerve pod, looking straight ahead
  public static Transform3d mainBotToObjectDetectionFrontLeftCam =
      new Transform3d(
          0.2726, // X: 1" back from front of robot
          0.1744, // Y: ~6.9" left of center
          0.0381, // Z: 1.5 inches off ground (just below bumper line for intake detection)
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(0).getRadians(), // Pitch level
              Rotation2d.fromDegrees(0).getRadians())); // Yaw straight ahead

  private VisionConstants() {}
}
