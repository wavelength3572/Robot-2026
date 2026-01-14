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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;
  private static Supplier<Pose2d> sharedPoseSupplier;

  // Camera position toggle system
  private static final List<EditableCameraConfig> cameraConfigs = new ArrayList<>();
  private static BooleanEntry useProposedEntry;
  private static boolean lastUseProposed = false;
  private static boolean initialized = false;

  private final PhotonCameraSim cameraSim;
  private final Transform3d currentTransform;

  /** Holds camera sim reference, current transform, and editable proposed parameters */
  private static class EditableCameraConfig {
    final String name;
    final PhotonCameraSim cameraSim;
    final Transform3d currentTransform;
    final VisionIOPhotonVisionSim
        parentIO; // Reference to update robotToCamera for pose calculation

    // Editable proposed values (NetworkTables entries)
    final DoubleEntry proposedXEntry;
    final DoubleEntry proposedYEntry;
    final DoubleEntry proposedZEntry;
    final DoubleEntry proposedPitchEntry;
    final DoubleEntry proposedYawEntry;

    // Cached values for this cycle (read once, use multiple times)
    double cachedX, cachedY, cachedZ;
    double cachedPitch, cachedYaw;

    // Previous cycle's values for change detection
    double prevX, prevY, prevZ;
    double prevPitch, prevYaw;

    // Cached transform (rebuilt only when values change)
    Transform3d cachedProposedTransform;

    // Epsilon for floating point comparison (avoid spurious changes from float noise)
    private static final double EPSILON = 0.0001;

    EditableCameraConfig(
        String name,
        PhotonCameraSim sim,
        Transform3d current,
        Transform3d initialProposed,
        NetworkTable table,
        VisionIOPhotonVisionSim parentIO) {
      this.name = name;
      this.cameraSim = sim;
      this.currentTransform = current;
      this.parentIO = parentIO;

      // Extract initial proposed position (rounded to 3 decimal places for meters)
      double initialX = roundTo(initialProposed.getTranslation().getX(), 3);
      double initialY = roundTo(initialProposed.getTranslation().getY(), 3);
      double initialZ = roundTo(initialProposed.getTranslation().getZ(), 3);

      // Extract initial proposed rotation in degrees (rounded to 1 decimal place)
      double initialPitch = roundTo(Math.toDegrees(initialProposed.getRotation().getY()), 1);
      double initialYaw = roundTo(Math.toDegrees(initialProposed.getRotation().getZ()), 1);

      // Create editable NT entries for this camera's proposed position
      this.proposedXEntry = table.getDoubleTopic(name + "/ProposedX").getEntry(initialX);
      this.proposedYEntry = table.getDoubleTopic(name + "/ProposedY").getEntry(initialY);
      this.proposedZEntry = table.getDoubleTopic(name + "/ProposedZ").getEntry(initialZ);
      this.proposedPitchEntry =
          table.getDoubleTopic(name + "/ProposedPitch").getEntry(initialPitch);
      this.proposedYawEntry = table.getDoubleTopic(name + "/ProposedYaw").getEntry(initialYaw);

      // Set initial values
      this.proposedXEntry.set(initialX);
      this.proposedYEntry.set(initialY);
      this.proposedZEntry.set(initialZ);
      this.proposedPitchEntry.set(initialPitch);
      this.proposedYawEntry.set(initialYaw);

      // Initialize cached and previous values
      this.cachedX = this.prevX = initialX;
      this.cachedY = this.prevY = initialY;
      this.cachedZ = this.prevZ = initialZ;
      this.cachedPitch = this.prevPitch = initialPitch;
      this.cachedYaw = this.prevYaw = initialYaw;

      // Build initial transform
      this.cachedProposedTransform =
          buildTransform(initialX, initialY, initialZ, initialPitch, initialYaw);
    }

    /** Reads NT values once per cycle and caches them. Call this at start of updateSim(). */
    void cacheCurrentValues() {
      cachedX = proposedXEntry.get();
      cachedY = proposedYEntry.get();
      cachedZ = proposedZEntry.get();
      cachedPitch = proposedPitchEntry.get();
      cachedYaw = proposedYawEntry.get();
    }

    /** Returns true if cached values differ from previous cycle (using epsilon comparison) */
    boolean hasProposedChanged() {
      return Math.abs(cachedX - prevX) > EPSILON
          || Math.abs(cachedY - prevY) > EPSILON
          || Math.abs(cachedZ - prevZ) > EPSILON
          || Math.abs(cachedPitch - prevPitch) > EPSILON
          || Math.abs(cachedYaw - prevYaw) > EPSILON;
    }

    /** Updates previous values to current cached values. Call after applying changes. */
    void commitValues() {
      prevX = cachedX;
      prevY = cachedY;
      prevZ = cachedZ;
      prevPitch = cachedPitch;
      prevYaw = cachedYaw;
      // Rebuild transform with committed values
      cachedProposedTransform = buildTransform(cachedX, cachedY, cachedZ, cachedPitch, cachedYaw);
    }

    /** Returns the cached proposed transform (consistent within a cycle) */
    Transform3d getProposedTransform() {
      return cachedProposedTransform;
    }

    /** Builds a Transform3d from the given values */
    private static Transform3d buildTransform(
        double x, double y, double z, double pitchDeg, double yawDeg) {
      return new Transform3d(
          new Translation3d(x, y, z),
          new Rotation3d(
              0.0,
              Rotation2d.fromDegrees(pitchDeg).getRadians(),
              Rotation2d.fromDegrees(yawDeg).getRadians()));
    }
  }

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param robotToCamera The current/physical camera transform.
   * @param recommendedTransform The recommended camera transform for comparison.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name,
      Transform3d robotToCamera,
      Transform3d recommendedTransform,
      Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.currentTransform = robotToCamera;

    // Initialize vision sim (only once, shared across all cameras)
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
      sharedPoseSupplier = poseSupplier;

      // Set up NetworkTables for camera position editing
      var table = NetworkTableInstance.getDefault().getTable("Vision/CameraEditor");
      useProposedEntry = table.getBooleanTopic("UseProposedPositions").getEntry(false);
      useProposedEntry.set(false);

      // Also publish to SmartDashboard for easy toggle button
      SmartDashboard.putBoolean("Vision/UseProposedCameras", false);

      initialized = true;
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(true);
    visionSim.addCamera(cameraSim, robotToCamera);

    // Store config with editable proposed parameters
    var table = NetworkTableInstance.getDefault().getTable("Vision/CameraEditor");
    cameraConfigs.add(
        new EditableCameraConfig(
            name, cameraSim, robotToCamera, recommendedTransform, table, this));
  }

  /**
   * Updates the vision simulation. Call this ONCE per cycle before calling updateInputs on any
   * camera. VisionSystemSim.update() simulates ALL cameras at once, so calling it multiple times
   * per cycle is wasteful.
   */
  public static void updateSim() {
    if (visionSim != null && sharedPoseSupplier != null) {
      // Step 1: Cache all NT values at the start of the cycle (read once, use consistently)
      for (EditableCameraConfig config : cameraConfigs) {
        config.cacheCurrentValues();
      }

      // Step 2: Check SmartDashboard toggle
      boolean useProposed = SmartDashboard.getBoolean("Vision/UseProposedCameras", false);
      boolean toggleChanged = useProposed != lastUseProposed;

      if (toggleChanged) {
        lastUseProposed = useProposed;
        useProposedEntry.set(useProposed);
      }

      // Step 3: Apply camera transforms as needed
      for (EditableCameraConfig config : cameraConfigs) {
        boolean needsUpdate = false;

        if (toggleChanged) {
          // Toggle changed - always update
          needsUpdate = true;
        } else if (useProposed && config.hasProposedChanged()) {
          // Using proposed and values changed - update and commit
          config.commitValues();
          needsUpdate = true;
        }

        if (needsUpdate) {
          Transform3d transform =
              useProposed ? config.getProposedTransform() : config.currentTransform;
          // Update both the sim camera position AND the parent's robotToCamera for pose calculation
          visionSim.adjustCamera(config.cameraSim, transform);
          config.parentIO.robotToCamera = transform;
        }
      }

      // Step 4: Update the simulation
      visionSim.update(sharedPoseSupplier.get());
    }
  }

  /** Returns whether the simulation is currently using proposed camera positions. */
  public static boolean isUsingRecommendedPositions() {
    return lastUseProposed;
  }

  /** Sets whether to use proposed camera positions. */
  public static void setUseRecommendedPositions(boolean useProposed) {
    SmartDashboard.putBoolean("Vision/UseProposedCameras", useProposed);
  }

  /** Gets the current proposed pitch for a camera (for logging/display). */
  public static double getProposedPitch(int cameraIndex) {
    if (cameraIndex < cameraConfigs.size()) {
      return cameraConfigs.get(cameraIndex).proposedPitchEntry.get();
    }
    return 0.0;
  }

  /** Gets the current proposed yaw for a camera (for logging/display). */
  public static double getProposedYaw(int cameraIndex) {
    if (cameraIndex < cameraConfigs.size()) {
      return cameraConfigs.get(cameraIndex).proposedYawEntry.get();
    }
    return 0.0;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Note: visionSim.update() is now called once via updateSim() before any camera's updateInputs
    super.updateInputs(inputs);
  }

  /** Rounds a value to a specified number of decimal places. */
  private static double roundTo(double value, int decimalPlaces) {
    double scale = Math.pow(10, decimalPlaces);
    return Math.round(value * scale) / scale;
  }
}
