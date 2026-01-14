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

  public static double MAX_TAG_DISTANCE = 1.5; // Only accept tags within 1.5 meters

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
  public static String elevatorBackCam = "CAMERA_C";

  // Robot to camera transforms
  // TODO: Calibrate these transforms for your 2026 robot - these are placeholders from 2025
  public static Transform3d robotToFrontRightCam =
      new Transform3d(
          0.26985,
          -0.26981,
          0.22155,
          new Rotation3d(0.0, Rotation2d.fromDegrees(-15).getRadians(), 0.0));

  public static Transform3d robotToBackRightCam =
      new Transform3d(
          -0.26985,
          -0.26981,
          0.22155,
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(-15).getRadians(),
              Rotation2d.fromDegrees(180).getRadians()));

  public static Transform3d robotToFrontLeftCam =
      new Transform3d(
          0.26985,
          0.26981,
          0.22155,
          new Rotation3d(0.0, Rotation2d.fromDegrees(-15).getRadians(), 0.0));

  public static Transform3d robotToElevatorBackCam =
      new Transform3d(
          -0.273075,
          -0.088951,
          0.74201,
          new Rotation3d(0.0, Math.toRadians(-38), Math.toRadians(180)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // FrontRight
        1.0, // BackRight
        1.0, // FrontLeft
        1.0 // ElevatorBack
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  private VisionConstants() {}
}
