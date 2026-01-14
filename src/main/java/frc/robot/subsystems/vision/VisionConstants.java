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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.io.IOException;
import java.nio.file.Path;

public final class VisionConstants {

  public static double MAX_TAG_DISTANCE = 5.0; // Only accept tags within 5 meters

  // AprilTag layout for 2026 Rebuilt field
  // TODO: Once WPILib releases the official 2026 field, replace with:
  //   public static AprilTagFieldLayout aprilTagLayout =
  //       AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  public static AprilTagFieldLayout aprilTagLayout = load2026FieldLayout();

  private static AprilTagFieldLayout load2026FieldLayout() {
    try {
      return new AprilTagFieldLayout(
          Path.of(
              edu.wpi.first.wpilibj.Filesystem.getDeployDirectory().getPath(),
              "2026-rebuilt-welded.json"));
    } catch (IOException e) {
      throw new RuntimeException("Failed to load 2026 AprilTag field layout", e);
    }
  }

  // _________________________________________________________________________________________________

  // Camera names, must match names configured on coprocessor
  // TODO: Update these camera names to match your PhotonVision coprocessor configuration
  public static String frontRightCam = "CAMERA_B";
  public static String backRightCam = "CAMERA_D";
  public static String frontLeftCam = "CAMERA_A";
  public static String backLeftCam = "CAMERA_C";

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
  // Order matches camera instantiation: A (FrontLeft), B (FrontRight), C (BackLeft), D (BackRight)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // FrontLeft (CAMERA_A)
        1.0, // FrontRight (CAMERA_B)
        1.0, // BackLeft (CAMERA_C)
        1.0 // BackRight (CAMERA_D)
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  // _________________________________________________________________________________________________
  // RECOMMENDED camera transforms for 2026 Rebuilt field (for visualization/comparison)
  // These are positioned to maximize AprilTag visibility across the field:
  // - Front cameras: pitched UP (+10°) to see HUB tags at 1.12m height
  // - Rear cameras: pitched DOWN (-10°) to see OUTPOST/TOWER tags at 0.55m height
  // - All cameras angled outward for ~360° coverage

  public static Transform3d recommendedFrontLeftCam =
      new Transform3d(
          0.26985,
          0.26981,
          0.22155,
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-10).getRadians(), // Pitch UP 10° (negative pitch = look up)
              Rotation2d.fromDegrees(30).getRadians())); // Yaw left 30°

  public static Transform3d recommendedFrontRightCam =
      new Transform3d(
          0.26985,
          -0.26981,
          0.22155,
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-10).getRadians(), // Pitch UP 10°
              Rotation2d.fromDegrees(-30).getRadians())); // Yaw right 30°

  public static Transform3d recommendedBackLeftCam =
      new Transform3d(
          -0.26985,
          0.26981,
          0.22155,
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(10)
                  .getRadians(), // Pitch DOWN 10° (positive pitch = look down)
              Rotation2d.fromDegrees(150).getRadians())); // Yaw back-left 150°

  public static Transform3d recommendedBackRightCam =
      new Transform3d(
          -0.26985,
          -0.26981,
          0.22155,
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(10).getRadians(), // Pitch DOWN 10°
              Rotation2d.fromDegrees(-150).getRadians())); // Yaw back-right 150°

  private VisionConstants() {}
}
