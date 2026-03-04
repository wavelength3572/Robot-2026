# Fuel Detection & Multi-Fuel Collection System

## Overview

Add PhotonVision ML-based fuel detection and autonomous/teleop collection using the existing `ObjectDetectionFrontLeft` camera. The system detects fuel on the ground, tracks positions in field coordinates, and drives the robot to collect multiple fuel in sequence.

---

## Architecture

```
PhotonVision ML Camera
        │
  ObjectDetectionIO          (new IO interface)
        │
  ObjectDetection subsystem  (new subsystem - processes detections, owns FuelTracker)
        │
  FuelTracker utility        (new - maintains tracked fuel positions in field space)
        │
  FuelCollectionCommands     (new - drive-to-fuel, multi-collect, teleop assist)
        │
  Drive + Intake subsystems  (existing)
```

---

## Step 1: Object Detection IO Layer

### New: `subsystems/vision/ObjectDetectionIO.java`

```java
public interface ObjectDetectionIO {
  @AutoLog
  public static class ObjectDetectionIOInputs {
    public boolean connected = false;
    public DetectedObject[] detectedObjects = new DetectedObject[0];
  }

  public static record DetectedObject(
      double yawDegrees,        // horizontal angle to object center
      double pitchDegrees,      // vertical angle to object center
      double area,              // fraction of image area (0-1)
      double confidence,        // ML confidence (0-1)
      int classId               // detected class ID
  ) {}

  public default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
```

### New: `subsystems/vision/ObjectDetectionIOPhotonVision.java`

- Connects to `PhotonCamera("ObjectDetectionFrontLeft")`
- In `updateInputs()`: iterates `camera.getAllUnreadResults()`, extracts ALL targets (not just best)
- For each `PhotonTrackedTarget`: reads `getYaw()`, `getPitch()`, `getArea()`, `getDetectedObjectClassId()`, `getDetectedObjectConfidence()`
- Filters by minimum confidence threshold (e.g., 0.5)

### New: `subsystems/vision/ObjectDetectionIOSim.java`

- Simulation implementation that queries `FuelSim.getInstance()` for fuel positions
- Projects fuel positions into camera space using the camera transform
- Generates synthetic `DetectedObject` entries for fuel within camera FOV
- Enables full testing without real hardware

### Modify: `VisionConstants.java`

- Add object detection constants: `MIN_DETECTION_CONFIDENCE = 0.5`, `FUEL_RADIUS = 0.075`, `FUEL_CLASS_ID = 0`
- Camera FOV constants: `OBJ_DET_HFOV_DEG = 70.0`, `OBJ_DET_VFOV_DEG = 43.75`, `OBJ_DET_RES_WIDTH = 1280`, `OBJ_DET_RES_HEIGHT = 800`

---

## Step 2: Distance Estimation

### Challenge
Camera at 0.0381m height, level pitch. Fuel center at 0.075m. Pitch angles are tiny at range (~2° at 1m, ~0.7° at 3m), making pitch-only estimation noisy.

### Approach: Area-based distance estimation (primary) + pitch-based (secondary)

**Area-based method (more robust at range):**
- Known fuel diameter = 0.15m
- At distance `d`, the fuel subtends angular width ≈ `2 * atan(0.075 / d)` radians
- Apparent pixel width ≈ `(angular_width / hfov_rad) * image_width`
- Area fraction ≈ `(π * (pixel_radius)²) / (image_width * image_height)`
- Invert: `distance ≈ K / sqrt(area)` where K is calibrated from known measurements
- K can be computed: `K = fuel_radius / (sqrt(area_at_1m) * 1.0m)` — calibrate once on the field

**Pitch-based method (better at close range):**
- `distance = (fuel_center_height - camera_height) / tan(pitch_radians)`
- `distance = 0.0369 / tan(pitch_radians)`
- Only reliable when pitch > ~3° (distance < ~0.7m)

**Fusion:** Use area-based for distance > 0.5m, pitch-based for < 0.5m, weighted blend in between. Implemented in a `FuelDistanceEstimator` utility method.

---

## Step 3: Fuel Tracking

### New: `util/FuelTracker.java`

Maintains a list of `TrackedFuel` objects in field coordinates.

```java
public class FuelTracker {
  public static record TrackedFuel(
      Translation2d position,    // field-relative position
      double lastSeenTimestamp,  // FPGA timestamp
      int observationCount,      // number of times seen
      double confidence          // rolling average confidence
  ) {}

  // Core methods:
  void addObservations(List<Translation2d> positions, double timestamp);
  List<TrackedFuel> getTrackedFuel();
  TrackedFuel getNearestFuel(Translation2d robotPosition);
  void markCollected(Translation2d position, double radius);
  void periodic();  // expires stale tracks
}
```

**Data association:** For each new observation, find nearest existing track within 0.3m. If found, update with exponential moving average (alpha=0.3). If not found, create new track.

**Expiry:** Remove tracks not seen for 2.0 seconds.

**Collection detection:** When robot pose is within ~0.3m of a tracked fuel AND intake current spikes, mark it collected and remove.

---

## Step 4: Object Detection Subsystem

### New: `subsystems/vision/ObjectDetection.java`

```java
public class ObjectDetection extends SubsystemBase {
  private final ObjectDetectionIO io;
  private final FuelTracker fuelTracker;
  private Supplier<Pose2d> robotPoseSupplier;

  // In periodic():
  //   1. Read IO inputs
  //   2. For each detected object:
  //      a. Estimate distance using area + pitch
  //      b. Compute field-relative position:
  //         - cameraRelativeX = distance * cos(yaw)
  //         - cameraRelativeY = distance * sin(yaw)
  //         - Apply camera-to-robot transform
  //         - Apply robot pose to get field coordinates
  //      c. Pass to FuelTracker
  //   3. FuelTracker.periodic() to expire stale tracks
  //   4. Log tracked fuel positions for AdvantageScope visualization

  // Public API:
  Translation2d getNearestFuelPosition();
  List<Translation2d> getAllTrackedFuel();
  boolean hasFuelInView();
  Rotation2d getYawToNearestFuel();  // camera-relative, for final approach
}
```

**Field-space projection math:**
```
camRelX = distance * cos(toRadians(yawDeg))
camRelY = distance * sin(toRadians(yawDeg))

// Camera transform: ObjectDetectionFrontLeft at (0.2726, 0.1744, 0.0381)
robotRelX = camRelX + 0.2726
robotRelY = camRelY + 0.1744

// Rotate by robot heading and add robot position
fieldX = robotPose.getX() + robotRelX * cos(heading) - robotRelY * sin(heading)
fieldY = robotPose.getY() + robotRelX * sin(heading) + robotRelY * cos(heading)
```

---

## Step 5: Collection Commands

### New: `commands/FuelCollectionCommands.java`

#### `driveToFuel(Drive, ObjectDetection, Intake)` — Single fuel pickup

Two-phase command:

**Phase 1 — Approach (field-coordinate PID):**
- Get target fuel position from FuelTracker (field coordinates)
- Use PID controllers on X and Y to drive toward target
- Orient robot so intake faces the fuel (rotation PID)
- Deploy intake when distance < 1.0m
- Transition to Phase 2 when distance < 0.5m

**Phase 2 — Final alignment (camera-relative):**
- Use `getYawToNearestFuel()` to steer (rotation PID on yaw error)
- Drive forward at controlled speed (0.5-1.0 m/s)
- Run intake rollers
- End when: fuel disappears from tracking (collected) OR timeout (3s)

```java
public static Command driveToFuel(Drive drive, ObjectDetection objDet, Intake intake) {
  // PID controllers for X, Y, and rotation
  // Returns a SequentialCommandGroup or state-machine command
}
```

#### `collectMultipleFuel(Drive, ObjectDetection, Intake, int maxCount)` — Multi-fuel

- Repeating command: `driveToFuel` → check if more fuel visible → repeat
- Stops when `maxCount` reached, no fuel visible, or cancelled
- Between collections, brief scan (rotate in place) if no fuel currently tracked

```java
public static Command collectMultipleFuel(Drive drive, ObjectDetection objDet, Intake intake, int maxCount) {
  return Commands.sequence(
    driveToFuel(drive, objDet, intake),
    // Check if more fuel available and repeat
  ).repeatedly().until(() -> collectedCount >= maxCount || !objDet.hasFuelInView());
}
```

#### `teleopFuelAssist(Drive, ObjectDetection, Intake, joystick suppliers)` — Teleop

- Like `joystickDriveAtAngle` but rotation locks onto nearest fuel
- Driver controls speed/direction, robot auto-rotates to face fuel
- Intake auto-deploys and runs when fuel is close
- Driver can override by releasing the assist button

---

## Step 6: Wiring in RobotContainer

### Modify: `RobotContainer.java`

1. **Separate the ObjectDetection camera** from the Vision AprilTag array
   - Remove `ObjectDetectionFrontLeft` from the `VisionIO[]` array passed to `Vision`
   - Create `ObjectDetection` subsystem with its own `ObjectDetectionIOPhotonVision` instance
   - In sim mode, create `ObjectDetectionIOSim` instead

2. **Wire commands:**
   - Teleop button binding: hold trigger → `teleopFuelAssist` command
   - Auto named command: `"CollectFuel"` → `collectMultipleFuel` for PathPlanner autos
   - Dashboard command: manual trigger for testing

3. **Dependency injection:**
   - `ObjectDetection` needs `Drive::getPose` as robot pose supplier
   - `FuelCollectionCommands` needs `Drive`, `ObjectDetection`, `Intake`

---

## Step 7: Simulation Support

### `ObjectDetectionIOSim.java`

- Query `FuelSim.getInstance()` for all fuel positions each cycle
- For each fuel, compute camera-relative angles:
  - Transform fuel field position → robot-relative → camera-relative
  - Check if within camera FOV (±35° horizontal, ±22° vertical)
  - Compute area from distance
  - Add noise to yaw/pitch/area for realism
- Return synthetic `DetectedObject` array
- This enables full closed-loop testing: robot drives to simulated fuel, FuelSim removes it when intake catches it

---

## Files Summary

| Action | File | Purpose |
|--------|------|---------|
| **Create** | `subsystems/vision/ObjectDetectionIO.java` | IO interface with DetectedObject record |
| **Create** | `subsystems/vision/ObjectDetectionIOPhotonVision.java` | Real hardware - reads ML detections |
| **Create** | `subsystems/vision/ObjectDetectionIOSim.java` | Simulation - projects FuelSim fuel into camera |
| **Create** | `subsystems/vision/ObjectDetection.java` | Subsystem - processes detections, owns FuelTracker |
| **Create** | `util/FuelTracker.java` | Tracks fuel positions across frames |
| **Create** | `commands/FuelCollectionCommands.java` | Drive-to-fuel, multi-collect, teleop assist |
| **Modify** | `subsystems/vision/VisionConstants.java` | Add object detection constants |
| **Modify** | `RobotContainer.java` | Wire ObjectDetection subsystem + commands |

---

## Verification Plan

1. **Unit test FuelTracker:** Add observations, verify tracking, expiry, and nearest-fuel selection
2. **Simulation test:** Run in sim with FuelSim spawning fuel, verify:
   - ObjectDetectionIOSim generates detections for visible fuel
   - ObjectDetection subsystem tracks fuel positions in field space (check AdvantageScope logs)
   - `driveToFuel` command drives robot to fuel and intake collects it
   - `collectMultipleFuel` picks up multiple fuel in sequence
3. **AdvantageScope visualization:** Log tracked fuel positions as `Translation2d[]` for overlay on field view
4. **Teleop test:** Verify fuel assist rotation locks onto nearest fuel while driver controls translation
5. **Calibrate distance estimation:** Place fuel at known distances (0.5m, 1m, 2m, 3m), log estimated vs actual distances, tune K constant

---

## Open Questions / Considerations

- **Distance estimation calibration:** The area-based K constant needs to be calibrated on the real field. Plan to make it a `LoggedTunableNumber`.
- **Multiple cameras:** If you add more object detection cameras later, the architecture supports it (just add more ObjectDetectionIO instances).
- **Fuel vs other objects:** The `classId` filter ensures we only track fuel, not other detected objects.
- **Field half awareness:** During competition, you may want to only collect fuel on your alliance's half. FuelTracker could accept a field-region filter.
