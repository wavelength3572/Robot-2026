package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldPositions;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final double turretHeightMeters;

  // Visualizer for trajectory display (initialized when suppliers are set)
  private TurretVisualizer visualizer = null;
  private Supplier<Pose2d> robotPoseSupplier = null;
  private Supplier<ChassisSpeeds> fieldSpeedsSupplier = null;

  // Current shot data
  private TurretCalculator.ShotData currentShot = null;

  /**
   * Creates a new Turret subsystem.
   *
   * @param io The IO implementation to use (real hardware or simulation)
   */
  public Turret(TurretIO io) {
    this.io = io;
    this.turretHeightMeters = Constants.getRobotConfig().getTurretHeightMeters();

    // Log turret configuration for debugging (logged once at startup)
    var config = Constants.getRobotConfig();
    Logger.recordOutput("Turret/Config/heightMeters", turretHeightMeters);
    Logger.recordOutput("Turret/Config/gearRatio", config.getTurretGearRatio());
    Logger.recordOutput("Turret/Config/maxAngleDegrees", config.getTurretMaxAngleDegrees());
    Logger.recordOutput("Turret/Config/minAngleDegrees", config.getTurretMinAngleDegrees());
    Logger.recordOutput("Turret/Config/kP", config.getTurretKp());
    Logger.recordOutput("Turret/Config/kI", config.getTurretKi());
    Logger.recordOutput("Turret/Config/kD", config.getTurretKd());
    Logger.recordOutput("Turret/Config/currentLimitAmps", config.getTurretCurrentLimitAmps());
    Logger.recordOutput("Turret/Config/motorCanId", config.getTurretMotorCanId());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // Log additional useful debugging values
    double angleError = inputs.targetAngleDegrees - inputs.currentAngleDegrees;
    Logger.recordOutput("Turret/angleError", angleError);
    Logger.recordOutput("Turret/atTarget", atTarget());

    // Update visualizer if initialized
    if (visualizer != null) {
      visualizer.logFuelStatus();
      visualizer.update3dPose(Math.toRadians(getCurrentAngle()));

      // Update trajectory visualization if we have a current shot
      if (currentShot != null) {
        visualizer.visualizeShot(currentShot);
      }
    }
  }

  /**
   * Initialize the turret visualizer with robot pose and speed suppliers.
   *
   * @param poseSupplier Supplier for robot's 2D pose
   * @param speedsSupplier Supplier for field-relative chassis speeds
   */
  public void initializeVisualizer(
      Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
    this.robotPoseSupplier = poseSupplier;
    this.fieldSpeedsSupplier = speedsSupplier;

    // Create 3D pose supplier from 2D pose
    Supplier<Pose3d> pose3dSupplier =
        () -> {
          Pose2d pose2d = poseSupplier.get();
          return new Pose3d(
              pose2d.getX(),
              pose2d.getY(),
              0,
              new Rotation3d(0, 0, pose2d.getRotation().getRadians()));
        };

    this.visualizer = new TurretVisualizer(pose3dSupplier, speedsSupplier, turretHeightMeters);
  }

  /**
   * Calculate shot parameters for the given target and update the trajectory visualization. Also
   * aims the turret at the target.
   *
   * @param target Target position (3D)
   */
  public void calculateShotToTarget(Translation3d target) {
    if (robotPoseSupplier == null || fieldSpeedsSupplier == null) return;

    Pose2d robotPose = robotPoseSupplier.get();
    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

    // Calculate shot with motion compensation
    currentShot =
        TurretCalculator.calculateMovingShot(robotPose, fieldSpeeds, target, 3, turretHeightMeters);

    // Aim turret at the target
    double azimuthRad = TurretCalculator.calculateAzimuthAngle(robotPose, target);
    double azimuthDeg = Math.toDegrees(azimuthRad);
    // Convert from 0-360 to -180 to 180 range
    if (azimuthDeg > 180) {
      azimuthDeg -= 360;
    }
    setAngle(azimuthDeg);

    // Log shot data
    Logger.recordOutput("Turret/ShotExitVelocity", currentShot.getExitVelocity());
    Logger.recordOutput("Turret/ShotLaunchAngleDeg", currentShot.getLaunchAngleDegrees());
    Logger.recordOutput("Turret/ShotAzimuthDeg", azimuthDeg);
    Logger.recordOutput("Turret/ShotTargetX", currentShot.getTarget().getX());
    Logger.recordOutput("Turret/ShotTargetY", currentShot.getTarget().getY());
    Logger.recordOutput("Turret/ShotTargetZ", currentShot.getTarget().getZ());
  }

  /**
   * Calculate shot to the alliance hub.
   *
   * @param isBlueAlliance True if targeting blue hub, false for red
   */
  public void calculateShotToHub(boolean isBlueAlliance) {
    Translation3d hubTarget;
    if (isBlueAlliance) {
      hubTarget =
          new Translation3d(
              FieldPositions.BLUE_HUB_X, FieldPositions.BLUE_HUB_Y, FieldPositions.HUB_HEIGHT);
    } else {
      hubTarget =
          new Translation3d(
              FieldPositions.RED_HUB_X, FieldPositions.RED_HUB_Y, FieldPositions.HUB_HEIGHT);
    }
    calculateShotToTarget(hubTarget);
  }

  /** Launch a fuel ball using the current shot parameters. */
  public void launchFuel() {
    if (visualizer != null && currentShot != null && robotPoseSupplier != null) {
      // Calculate azimuth angle (field-relative direction from robot to target)
      Pose2d robot = robotPoseSupplier.get();
      Translation3d target = currentShot.getTarget();
      double azimuthAngle = Math.atan2(target.getY() - robot.getY(), target.getX() - robot.getX());

      visualizer.launchFuel(
          currentShot.getExitVelocity(), currentShot.getLaunchAngle(), azimuthAngle);
    }
  }

  /**
   * Get a command that repeatedly launches fuel at the current target.
   *
   * @return Command that launches fuel every 0.25 seconds
   */
  public Command repeatedlyLaunchFuelCommand() {
    if (visualizer == null) {
      return runOnce(() -> {}); // No-op if visualizer not initialized
    }
    // Use suppliers so currentShot is checked at runtime, not binding time
    return visualizer.repeatedlyLaunchFuel(
        () -> currentShot != null ? currentShot.getExitVelocity() : 0.0,
        () -> currentShot != null ? currentShot.getLaunchAngle() : 0.0,
        this);
  }

  /**
   * Get the current shot data.
   *
   * @return Current shot parameters, or null if not calculated
   */
  public TurretCalculator.ShotData getCurrentShot() {
    return currentShot;
  }

  /**
   * Get the visualizer instance.
   *
   * @return TurretVisualizer or null if not initialized
   */
  public TurretVisualizer getVisualizer() {
    return visualizer;
  }

  /**
   * Set the turret to point at a specific angle relative to the robot's front.
   *
   * @param angleDegrees Angle in degrees (-180 to 180, positive = counter-clockwise)
   */
  public void setAngle(double angleDegrees) {
    io.setTargetAngle(new Rotation2d(Rotation2d.fromDegrees(angleDegrees).getRadians()));
  }

  /**
   * Calculate and set the turret angle to point at a field position.
   *
   * @param robotX Robot's X position on field (meters)
   * @param robotY Robot's Y position on field (meters)
   * @param robotOmega Robot's heading in degrees (0-360)
   * @param targetX Target X position on field (meters)
   * @param targetY Target Y position on field (meters)
   */
  public void aimAtFieldPosition(
      double robotX, double robotY, double robotOmega, double targetX, double targetY) {
    double turretAngle = calculateTurretAngle(robotX, robotY, robotOmega, targetX, targetY);
    setAngle(turretAngle);
  }

  /**
   * Calculate the turret angle needed to point at a target location on the field, accounting for
   * the turret's offset from the robot's center.
   *
   * @param robotX Robot center's X position on field (meters)
   * @param robotY Robot center's Y position on field (meters)
   * @param robotOmega Robot's heading in degrees (0-360, counter-clockwise from +X axis)
   * @param targetX Target X position on field (meters)
   * @param targetY Target Y position on field (meters)
   * @return Angle in degrees the turret needs to rotate relative to robot heading Range: -180 to
   *     +180 degrees
   */
  private double calculateTurretAngle(
      double robotX, double robotY, double robotOmega, double targetX, double targetY) {
    // Convert robot heading to radians for rotation calculations
    double robotOmegaRad = Math.toRadians(robotOmega);

    // Calculate turret's actual position on the field
    // The turret offset is in robot-relative coordinates, so we need to rotate it
    // to field coordinates based on the robot's heading
    double turretFieldX =
        robotX
            + (TurretConstants.TURRET_X_OFFSET * Math.cos(robotOmegaRad)
                - TurretConstants.TURRET_Y_OFFSET * Math.sin(robotOmegaRad));

    double turretFieldY =
        robotY
            + (TurretConstants.TURRET_X_OFFSET * Math.sin(robotOmegaRad)
                + TurretConstants.TURRET_Y_OFFSET * Math.cos(robotOmegaRad));

    // Calculate vector from turret position to target
    double deltaX = targetX - turretFieldX;
    double deltaY = targetY - turretFieldY;

    // Calculate absolute angle to target from field coordinates
    double absoluteAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

    // Normalize to 0-360 range
    if (absoluteAngle < 0) {
      absoluteAngle += 360;
    }

    // Calculate relative angle (turret angle relative to robot heading)
    double relativeAngle = absoluteAngle - robotOmega;

    // Normalize to -180 to +180 range for shortest rotation
    if (relativeAngle > 180) {
      relativeAngle -= 360;
    } else if (relativeAngle < -180) {
      relativeAngle += 360;
    }

    return relativeAngle;
  }

  /**
   * Get the current turret angle.
   *
   * @return Current angle in degrees (-180 to 180)
   */
  public double getCurrentAngle() {
    return inputs.currentAngleDegrees;
  }

  /**
   * Get the target turret angle.
   *
   * @return Target angle in degrees
   */
  public double getTargetAngle() {
    return inputs.targetAngleDegrees;
  }

  /**
   * Check if the turret is at the target angle within tolerance.
   *
   * @return True if at target
   */
  public boolean atTarget() {
    return Math.abs(inputs.currentAngleDegrees - inputs.targetAngleDegrees)
        <= TurretConstants.ANGLE_TOLERANCE_DEGREES;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Turret")
  public Pose3d getPose() {
    return new Pose3d(
        TurretConstants.TURRET_X_OFFSET,
        TurretConstants.TURRET_Y_OFFSET,
        TurretConstants.TURRET_HEIGHT_METERS,
        new Rotation3d(0.0, 0.0, Rotation2d.fromDegrees(getCurrentAngle()).getRadians()));
  }

  /**
   * Get the turret height from configuration.
   *
   * @return Turret height in meters
   */
  public double getTurretHeightMeters() {
    return turretHeightMeters;
  }
}
