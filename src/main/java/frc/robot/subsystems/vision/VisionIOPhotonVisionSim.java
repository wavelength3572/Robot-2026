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

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;
  private static Supplier<Pose2d> sharedPoseSupplier;

  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);

    // Initialize vision sim (only once, shared across all cameras)
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
      sharedPoseSupplier = poseSupplier;
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    // cameraProperties.setCalibration(800, 600, Rotation2d.fromDegrees(70));
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(true);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  /**
   * Updates the vision simulation. Call this ONCE per cycle before calling updateInputs on any
   * camera. VisionSystemSim.update() simulates ALL cameras at once, so calling it multiple times
   * per cycle is wasteful.
   */
  public static void updateSim() {
    if (visionSim != null && sharedPoseSupplier != null) {
      visionSim.update(sharedPoseSupplier.get());
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Note: visionSim.update() is now called once via updateSim() before any camera's updateInputs
    super.updateInputs(inputs);
  }
}
