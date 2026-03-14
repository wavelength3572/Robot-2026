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

  public static double MAX_SINGLE_TAG_DISTANCE = 3.0; // Single-tag: conservative (ambiguity flips)
  public static double MAX_MULTI_TAG_DISTANCE =
      5.0; // Multi-tag: lenient (geometry constrains pose)

  // AprilTag layout for 2026 Rebuilt field
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // _________________________________________________________________________________________________

  // Camera names, must match names configured on coprocessor
  public static String rightFrontCam = "RightFront";
  public static String rightRearCam = "RightRear";
  public static String centerRearCam = "CenterRear";
  public static String leftRearCam = "LeftRear";
  // Robot to camera transforms (generic/SquareBot - used only for sim)
  // All cameras are mounted near swerve pods at the four corners of the robot
  public static Transform3d robotToCenterRearCam =
      new Transform3d(
          0.26985,
          0.26981,
          0.22155,
          new Rotation3d(0.0, Rotation2d.fromDegrees(-15).getRadians(), 0.0));

  public static Transform3d robotToRightFrontCam =
      new Transform3d(
          0.26985,
          -0.26981,
          0.22155,
          new Rotation3d(0.0, Rotation2d.fromDegrees(-15).getRadians(), 0.0));

  public static Transform3d robotToLeftRearCam =
      new Transform3d(
          -0.26985,
          0.26981,
          0.22155,
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-15).getRadians(),
              Rotation2d.fromDegrees(180).getRadians()));

  public static Transform3d robotToRightRearCam =
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
  public static double linearStdDevBaseline = 0.25; // Meters (was 0.15, increased to reduce jitter)
  public static double angularStdDevBaseline = 0.3; // Radians (was 0.06, increased to reduce jumps)

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  // Order matches camera instantiation: CenterRear, RightFront, LeftRear, RightRear
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // CenterRear
        1.0, // RightFront
        1.0, // LeftRear
        1.0 // RightRear
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
  // MAINBOT camera transforms - from CAD measurements (4 March 2026 PDF)
  // Positions measured from robot center origin, converted inches → meters (×0.0254)

  // Center Rear camera (forward facing) - mounted at rear center, looking forward
  public static Transform3d mainBotToCenterRearCam =
      new Transform3d(
          -0.26538, // X: 10.448" to rear
          -0.04445, // Y: 1.750" to the right
          0.52093, // Z: 20.509" up
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-10).getRadians(), // Pitch 10° up from horizontal
              Rotation2d.fromDegrees(0).getRadians())); // Yaw: facing forward

  // Right Front camera (rightward facing) - mounted at front right, -97° yaw (7° inward)
  public static Transform3d mainBotToRightFrontCam =
      new Transform3d(
          0.23904, // X: 9.411" to front
          -0.38273, // Y: 15.068" to the right
          0.52339, // Z: 20.606" up
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-16).getRadians(), // Pitch 16° up from horizontal
              Rotation2d.fromDegrees(-97).getRadians())); // Yaw: 97° right (7° toward rear/inward)

  // Left Rear camera (leftward facing) - mounted at rear left
  public static Transform3d mainBotToLeftRearCam =
      new Transform3d(
          -0.22962, // X: 9.040" to rear
          0.38002, // Y: 14.960" to the left
          0.36449, // Z: 14.350" up
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-21).getRadians(), // Pitch 21° up from horizontal
              Rotation2d.fromDegrees(83).getRadians())); // Yaw: 83° left (7° toward front)

  // Right Rear camera (rearward facing) - mounted at rear right
  public static Transform3d mainBotToRightRearCam =
      new Transform3d(
          -0.26670, // X: 10.5" to rear
          -0.35111, // Y: 13.823" to the right
          0.52410, // Z: 20.634" up
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-10).getRadians(), // Pitch 10° up from horizontal
              Rotation2d.fromDegrees(180).getRadians())); // Yaw: facing rearward

  private VisionConstants() {}
}
