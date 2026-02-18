// Adapted from FRC Team 5000 Hammerheads
// https://github.com/hammerheads5000/2026Rebuilt
// Open source under WPILib BSD license

package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.util.FuelSim;
import frc.robot.util.LoggedTunableNumber;
import java.util.LinkedList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Launcher status for trajectory color coding. */
enum LauncherStatus {
  UNPOWERED, // Red - launcher not running
  SPINNING_UP, // Yellow - powered but not at setpoint
  AT_SETPOINT // Green - at setpoint, ready to fire
}

/**
 * Handles visualization of shot trajectory and fuel simulation integration. Calculates and logs
 * trajectory points for AdvantageScope visualization.
 */
public class ShotVisualizer {
  private static final int TRAJECTORY_POINTS = 50;
  private static final double TRAJECTORY_TIME_STEP = 0.04; // seconds between points
  private static final double GRAVITY = 9.81; // m/s^2

  // Trajectory for visualization (where the ball would actually go)
  private Translation3d[] actualTrajectory = new Translation3d[TRAJECTORY_POINTS];
  private Translation3d[] whatIfTrajectory = new Translation3d[TRAJECTORY_POINTS];
  private static final Translation3d[] EMPTY_TRAJECTORY = new Translation3d[0];

  // Threshold for determining launcher status
  private static final double RPM_VELOCITY_THRESHOLD = 50.0;
  private static final double RPM_SETPOINT_TOLERANCE = 100.0; // RPM tolerance for "at setpoint"

  private final Supplier<Pose3d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;
  private final double turretHeightMeters;

  // Turret offset from robot center (robot-relative coordinates)
  private final double turretXOffset;
  private final double turretYOffset;

  // Fuel inventory management
  private static final int FUEL_CAPACITY = 40;
  private int fuelStored = 25;

  // Intake-to-prefeed transit delay simulation
  private final LinkedList<Double> pendingFuelTimestamps = new LinkedList<>();
  private static final LoggedTunableNumber transitDelay =
      new LoggedTunableNumber("Sim/Intake/TransitDelaySeconds", 0.6);
  private static final LoggedTunableNumber transitDelayRandom =
      new LoggedTunableNumber("Sim/Intake/TransitDelayRandomSeconds", 0.15);

  // Current azimuth angle for launching (set by visualizeShot)
  private double currentAzimuthAngle = 0.0;

  /**
   * Creates a new ShotVisualizer.
   *
   * @param robotPoseSupplier Supplier for robot's 3D pose
   * @param fieldSpeedsSupplier Supplier for field-relative chassis speeds
   * @param turretHeightMeters Height of the turret in meters
   * @param turretXOffset Turret X offset from robot center (robot-relative, positive = forward)
   * @param turretYOffset Turret Y offset from robot center (robot-relative, positive = left)
   */
  public ShotVisualizer(
      Supplier<Pose3d> robotPoseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier,
      double turretHeightMeters,
      double turretXOffset,
      double turretYOffset) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.fieldSpeedsSupplier = fieldSpeedsSupplier;
    this.turretHeightMeters = turretHeightMeters;
    this.turretXOffset = turretXOffset;
    this.turretYOffset = turretYOffset;

    // Initialize trajectory arrays
    for (int i = 0; i < TRAJECTORY_POINTS; i++) {
      actualTrajectory[i] = new Translation3d();
      whatIfTrajectory[i] = new Translation3d();
    }
  }

  /**
   * Calculate the turret's field-relative position accounting for robot pose and turret offset.
   *
   * @return 3D position of turret in field coordinates
   */
  private Translation3d getTurretFieldPosition() {
    Pose3d robot = robotPoseSupplier.get();
    double robotHeadingRad = robot.getRotation().getZ();

    double turretFieldX =
        robot.getX()
            + (turretXOffset * Math.cos(robotHeadingRad)
                - turretYOffset * Math.sin(robotHeadingRad));
    double turretFieldY =
        robot.getY()
            + (turretXOffset * Math.sin(robotHeadingRad)
                + turretYOffset * Math.cos(robotHeadingRad));
    double turretFieldZ = robot.getZ() + turretHeightMeters;

    return new Translation3d(turretFieldX, turretFieldY, turretFieldZ);
  }

  /**
   * Calculate the launch velocity vector accounting for turret azimuth and robot movement.
   *
   * @param exitVelocity Exit velocity magnitude in m/s
   * @param launchAngle Launch angle in radians from horizontal
   * @param azimuthAngle Turret azimuth angle in radians (field-relative direction to target)
   * @return 3D velocity vector in field coordinates
   */
  private Translation3d calculateLaunchVelocity(
      double exitVelocity, double launchAngle, double azimuthAngle) {
    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

    double horizontalVel = exitVelocity * Math.cos(launchAngle);
    double verticalVel = exitVelocity * Math.sin(launchAngle);

    double xVel = horizontalVel * Math.cos(azimuthAngle);
    double yVel = horizontalVel * Math.sin(azimuthAngle);

    // Add robot's field velocity to the projectile
    xVel += fieldSpeeds.vxMetersPerSecond;
    yVel += fieldSpeeds.vyMetersPerSecond;

    return new Translation3d(xVel, yVel, verticalVel);
  }

  /**
   * Check if the robot can intake more fuel.
   *
   * @return True if below capacity
   */
  public boolean canIntake() {
    return fuelStored < FUEL_CAPACITY;
  }

  /** Add a fuel to the robot's inventory immediately (bypasses transit delay). */
  public void intakeFuel() {
    if (fuelStored < FUEL_CAPACITY) {
      fuelStored++;
    }
  }

  /** Queue a fuel pickup with simulated intake-to-prefeed transit delay. */
  public void queueFuel() {
    double baseDelay = transitDelay.get();
    double randomRange = transitDelayRandom.get();
    double delay = baseDelay + (Math.random() * 2.0 - 1.0) * randomRange;
    pendingFuelTimestamps.add(Timer.getFPGATimestamp() + delay);
  }

  /** Process pending fuel queue, moving fuel to stored when transit delay has elapsed. */
  private void processPendingFuel() {
    double now = Timer.getFPGATimestamp();
    while (!pendingFuelTimestamps.isEmpty() && pendingFuelTimestamps.peek() <= now) {
      pendingFuelTimestamps.poll();
      if (fuelStored < FUEL_CAPACITY) {
        fuelStored++;
      }
    }
  }

  /**
   * Get the current fuel count.
   *
   * @return Number of fuel balls stored
   */
  public int getFuelCount() {
    return fuelStored;
  }

  /**
   * Set the fuel count. Useful for testing and resetting.
   *
   * @param count Number of fuel balls to set (clamped to 0-FUEL_CAPACITY)
   */
  public void setFuelCount(int count) {
    fuelStored = Math.max(0, Math.min(FUEL_CAPACITY, count));
  }

  /**
   * Launch a fuel ball into the simulation.
   *
   * @param exitVelocity Exit velocity in m/s
   * @param launchAngle Launch angle in radians
   * @param azimuthAngle Turret azimuth angle in radians (field-relative direction to target)
   */
  public void launchFuel(double exitVelocity, double launchAngle, double azimuthAngle) {
    if (fuelStored == 0) return;
    fuelStored--;

    Translation3d launchPosition = getTurretFieldPosition();
    Translation3d launchVelocity = calculateLaunchVelocity(exitVelocity, launchAngle, azimuthAngle);

    FuelSim.getInstance().spawnFuel(launchPosition, launchVelocity);
  }

  /**
   * Creates a command that repeatedly launches fuel at the given parameters.
   *
   * @param exitVelocitySupplier Supplier for exit velocity
   * @param launchAngleSupplier Supplier for launch angle
   * @param subsystem The subsystem to use for command requirements
   * @return Command that launches fuel every 0.25 seconds
   */
  public Command repeatedlyLaunchFuel(
      Supplier<Double> exitVelocitySupplier,
      Supplier<Double> launchAngleSupplier,
      SubsystemBase subsystem) {
    return subsystem
        .runOnce(
            () ->
                launchFuel(
                    exitVelocitySupplier.get(), launchAngleSupplier.get(), currentAzimuthAngle))
        .andThen(Commands.waitSeconds(0.25))
        .repeatedly();
  }

  /**
   * Calculate trajectory points for given parameters.
   *
   * @param exitVelocity Exit velocity in m/s
   * @param launchAngle Launch angle in radians
   * @param azimuthAngle Turret azimuth angle in radians (field-relative direction to target)
   * @param trajectoryArray Array to fill with trajectory points
   */
  private void calculateTrajectoryPoints(
      double exitVelocity,
      double launchAngle,
      double azimuthAngle,
      Translation3d[] trajectoryArray) {
    Translation3d velocity = calculateLaunchVelocity(exitVelocity, launchAngle, azimuthAngle);
    Translation3d turretPos = getTurretFieldPosition();

    double startX = turretPos.getX();
    double startY = turretPos.getY();
    double startZ = turretPos.getZ();

    for (int i = 0; i < TRAJECTORY_POINTS; i++) {
      double t = i * TRAJECTORY_TIME_STEP;

      double x = startX + velocity.getX() * t;
      double y = startY + velocity.getY() * t;
      double z = startZ + velocity.getZ() * t - 0.5 * GRAVITY * t * t;

      if (z < 0) z = 0;

      trajectoryArray[i] = new Translation3d(x, y, z);
    }
  }

  /**
   * Determine the launcher status based on current and target RPM.
   *
   * @return LauncherStatus indicating ready state
   */
  private LauncherStatus getLauncherStatus() {
    double currentRPM = ShotCalculator.getCurrentLauncherRPM();
    double targetRPM = ShotCalculator.getTargetLauncherRPM();

    if (Math.abs(currentRPM) < RPM_VELOCITY_THRESHOLD
        && Math.abs(targetRPM) < RPM_VELOCITY_THRESHOLD) {
      return LauncherStatus.UNPOWERED;
    }

    if (Math.abs(currentRPM - targetRPM) < RPM_SETPOINT_TOLERANCE
        && targetRPM > RPM_VELOCITY_THRESHOLD) {
      return LauncherStatus.AT_SETPOINT;
    }

    return LauncherStatus.SPINNING_UP;
  }

  /**
   * Update the ACTUAL trajectory visualization (where we'd REALLY shoot). Uses current launcher RPM
   * and turret angle. Also logs color-coded trajectories based on launcher status.
   */
  public void updateActualTrajectory(
      double currentExitVelocity, double currentLaunchAngle, double currentAzimuthAngle) {
    calculateTrajectoryPoints(
        currentExitVelocity, currentLaunchAngle, currentAzimuthAngle, actualTrajectory);

    LauncherStatus status = getLauncherStatus();
    Logger.recordOutput("Turret/LauncherStatus", status.name());

    Logger.recordOutput(
        "Turret/Trajectory/Red",
        status == LauncherStatus.UNPOWERED ? actualTrajectory : EMPTY_TRAJECTORY);
    Logger.recordOutput(
        "Turret/Trajectory/Yellow",
        status == LauncherStatus.SPINNING_UP ? actualTrajectory : EMPTY_TRAJECTORY);
    Logger.recordOutput(
        "Turret/Trajectory/Green",
        status == LauncherStatus.AT_SETPOINT ? actualTrajectory : EMPTY_TRAJECTORY);
  }

  /** Log fuel inventory status and process pending fuel arrivals. */
  public void logFuelStatus() {
    processPendingFuel();
    Logger.recordOutput("Match/Fuel/Stored", fuelStored);
    Logger.recordOutput("Match/Fuel/InTransit", pendingFuelTimestamps.size());
    Logger.recordOutput("Match/Fuel/CanIntake", canIntake());
  }

  /**
   * Calculate and visualize the trajectory for a target. Updates the color-coded trajectory
   * (red/yellow/green based on launcher status).
   *
   * @param shotResult The calculated shot parameters
   * @param currentTurretAngleRad Current turret angle in radians (for actual trajectory)
   * @param robotHeadingRad Robot heading in radians (to convert turret angle to field-relative)
   */
  public void visualizeShot(
      ShotCalculator.ShotResult shotResult, double currentTurretAngleRad, double robotHeadingRad) {
    Translation3d turretPos = getTurretFieldPosition();
    Translation3d target = shotResult.aimTarget();

    double targetAzimuthAngle =
        Math.atan2(target.getY() - turretPos.getY(), target.getX() - turretPos.getX());

    this.currentAzimuthAngle = targetAzimuthAngle;

    updateActualTrajectory(
        shotResult.exitVelocityMps(), shotResult.launchAngleRad(), targetAzimuthAngle);

    Logger.recordOutput("Turret/Shot/CurrentRPM", ShotCalculator.getCurrentLauncherRPM());
    Logger.recordOutput(
        "Turret/Shot/CompensatedTarget", new Pose3d(shotResult.aimTarget(), new Rotation3d()));
  }

  /**
   * Get the current azimuth angle used for trajectory visualization.
   *
   * @return Current azimuth angle in radians (field-relative direction to target)
   */
  public double getCurrentAzimuthAngle() {
    return currentAzimuthAngle;
  }

  /**
   * Update the what-if trajectory visualization with user-specified parameters.
   *
   * @param exitVelocity Exit velocity in m/s
   * @param launchAngle Launch angle in radians
   * @param azimuthAngle Turret azimuth angle in radians (field-relative direction to target)
   */
  public void updateWhatIfTrajectory(double exitVelocity, double launchAngle, double azimuthAngle) {
    calculateTrajectoryPoints(exitVelocity, launchAngle, azimuthAngle, whatIfTrajectory);
    Logger.recordOutput("Turret/Trajectory/WhatIf", whatIfTrajectory);
  }

  /** Clear the what-if trajectory visualization. */
  public void clearWhatIfTrajectory() {
    Logger.recordOutput("Turret/Trajectory/WhatIf", EMPTY_TRAJECTORY);
  }

  // ========== Snapshot-driven visualization ==========

  /**
   * Main visualization entry point. Called by ShootingCoordinator.periodic() AFTER all control
   * logic. Reads from a snapshot so visualization is a passive observer of shooting state.
   *
   * @param snapshot Read-only snapshot of current shooting state
   */
  public void update(ShotSnapshot snapshot) {
    logFuelStatus();
    logSafetyMetrics(snapshot);
    updateTurretOverlay(snapshot);
    if (snapshot.currentShot() != null) {
      double currentTurretAngleRad = Math.toRadians(snapshot.currentAngleDeg());
      double robotHeadingRad = snapshot.robotPose().getRotation().getRadians();
      visualizeShot(snapshot.currentShot(), currentTurretAngleRad, robotHeadingRad);
    }
  }

  /** Log safety metrics: room to limits, position %, warning/danger zone status. */
  private void logSafetyMetrics(ShotSnapshot snapshot) {
    double currentAngle = snapshot.currentAngleDeg();
    double roomCW = snapshot.effectiveMaxAngleDeg() - currentAngle;
    double roomCCW = currentAngle - snapshot.effectiveMinAngleDeg();
    double warningZone = snapshot.warningZoneDeg();
    double dangerZone = warningZone / 2.0;

    double totalRange = snapshot.effectiveMaxAngleDeg() - snapshot.effectiveMinAngleDeg();
    double positionPercent =
        totalRange > 0
            ? ((currentAngle - snapshot.effectiveMinAngleDeg()) / totalRange) * 100.0
            : 50.0;

    boolean nearLimit = roomCW <= warningZone || roomCCW <= warningZone;
    boolean inDangerZone = roomCW <= dangerZone || roomCCW <= dangerZone;
    Logger.recordOutput("Turret/Safety/RoomCW_Deg", roomCW);
    Logger.recordOutput("Turret/Safety/RoomCCW_Deg", roomCCW);
    Logger.recordOutput("Turret/Safety/PositionPercent", positionPercent);
    Logger.recordOutput("Turret/Safety/NearLimit", nearLimit);
    Logger.recordOutput("Turret/Safety/InDangerZone", inDangerZone);
  }

  /** Update the 3D field overlay: aim line, CW/CCW limits, center nub, and target direction. */
  private void updateTurretOverlay(ShotSnapshot snapshot) {
    Pose2d robotPose = snapshot.robotPose();
    double robotX = robotPose.getX();
    double robotY = robotPose.getY();
    double robotHeadingDeg = robotPose.getRotation().getDegrees();
    double robotHeadingRad = robotPose.getRotation().getRadians();
    double currentAngle = snapshot.currentAngleDeg();

    double[] turretPos = getTurretFieldPosition2d(robotX, robotY, robotHeadingRad, snapshot);
    double turretX = turretPos[0];
    double turretY = turretPos[1];

    // Field-relative angles
    double currentFieldAngle = robotHeadingDeg + currentAngle;
    double cwLimitFieldAngle = robotHeadingDeg + snapshot.effectiveMaxAngleDeg();
    double ccwLimitFieldAngle = robotHeadingDeg + snapshot.effectiveMinAngleDeg();

    double turretHeight = snapshot.turretHeightMeters();

    // Aim line -- extends to hub distance
    double hubX =
        snapshot.isBlueAlliance()
            ? FieldConstants.Hub.innerCenterPoint.getX()
            : FieldConstants.Hub.oppInnerCenterPoint.getX();
    double hubY =
        snapshot.isBlueAlliance()
            ? FieldConstants.Hub.innerCenterPoint.getY()
            : FieldConstants.Hub.oppInnerCenterPoint.getY();
    double distanceToHub = Math.sqrt(Math.pow(hubX - turretX, 2) + Math.pow(hubY - turretY, 2));
    Logger.recordOutput(
        "Turret/Overlay/Aim",
        generateFieldLine3d(turretX, turretY, turretHeight, currentFieldAngle, distanceToHub, 5));

    // Limit visualization
    double maxRoom = snapshot.effectiveMaxAngleDeg() - snapshot.effectiveMinAngleDeg();
    double roomCW = snapshot.effectiveMaxAngleDeg() - currentAngle;
    double roomCCW = currentAngle - snapshot.effectiveMinAngleDeg();
    double center = snapshot.centerOffsetDeg();
    double centeredZoneDeg = 10.0;

    double cwProximity = Math.max(0, Math.min(1, 1.0 - (roomCW / maxRoom)));
    double ccwProximity = Math.max(0, Math.min(1, 1.0 - (roomCCW / maxRoom)));

    boolean isCentered = Math.abs(currentAngle - center) <= centeredZoneDeg;
    boolean approachingCW = !isCentered && currentAngle > center;
    boolean approachingCCW = !isCentered && currentAngle < center;

    if (approachingCW) {
      double cwLineLength = 0.4 + (cwProximity - 0.5) * 0.4;
      Logger.recordOutput(
          "Turret/Overlay/CWLimit",
          generateFieldLine3d(turretX, turretY, turretHeight, cwLimitFieldAngle, cwLineLength, 2));
      Logger.recordOutput("Turret/Overlay/CCWLimit", new Pose3d[] {});
    } else if (approachingCCW) {
      double ccwLineLength = 0.4 + (ccwProximity - 0.5) * 0.4;
      Logger.recordOutput("Turret/Overlay/CWLimit", new Pose3d[] {});
      Logger.recordOutput(
          "Turret/Overlay/CCWLimit",
          generateFieldLine3d(
              turretX, turretY, turretHeight, ccwLimitFieldAngle, ccwLineLength, 2));
    } else {
      double fixedLength = 0.4;
      Logger.recordOutput(
          "Turret/Overlay/CWLimit",
          generateFieldLine3d(turretX, turretY, turretHeight, cwLimitFieldAngle, fixedLength, 2));
      Logger.recordOutput(
          "Turret/Overlay/CCWLimit",
          generateFieldLine3d(turretX, turretY, turretHeight, ccwLimitFieldAngle, fixedLength, 2));
    }

    // Center line nub
    double centerFieldAngle = robotHeadingDeg + snapshot.centerOffsetDeg();
    Logger.recordOutput(
        "Turret/Overlay/Center",
        generateFieldLine3d(turretX, turretY, turretHeight, centerFieldAngle, 0.5, 2));

    // Target direction line
    if (snapshot.currentShot() != null) {
      double targetX = snapshot.currentShot().aimTarget().getX();
      double targetY = snapshot.currentShot().aimTarget().getY();
      double targetAngle = Math.toDegrees(Math.atan2(targetY - turretY, targetX - turretX));
      double distanceToTarget =
          Math.sqrt(Math.pow(targetX - turretX, 2) + Math.pow(targetY - turretY, 2));
      Logger.recordOutput(
          "Turret/Overlay/Target",
          generateFieldLine3d(
              turretX, turretY, turretHeight, targetAngle, Math.min(distanceToTarget, 3.0), 4));
    }
  }

  /**
   * Calculate the turret's 2D field position from robot pose and snapshot offsets.
   *
   * @return double[] {turretFieldX, turretFieldY}
   */
  private double[] getTurretFieldPosition2d(
      double robotX, double robotY, double robotHeadingRad, ShotSnapshot snapshot) {
    double tx =
        robotX
            + (snapshot.turretXOffset() * Math.cos(robotHeadingRad)
                - snapshot.turretYOffset() * Math.sin(robotHeadingRad));
    double ty =
        robotY
            + (snapshot.turretXOffset() * Math.sin(robotHeadingRad)
                + snapshot.turretYOffset() * Math.cos(robotHeadingRad));
    return new double[] {tx, ty};
  }

  /**
   * Generate a line of 3D poses for field overlay visualization.
   *
   * @param startX Start X position (meters)
   * @param startY Start Y position (meters)
   * @param height Height above ground (meters)
   * @param angleDeg Direction angle in degrees (field-relative)
   * @param length Length of line in meters
   * @param numPoints Number of poses along the line
   * @return Array of Pose3d representing the line
   */
  private Pose3d[] generateFieldLine3d(
      double startX, double startY, double height, double angleDeg, double length, int numPoints) {
    Pose3d[] poses = new Pose3d[numPoints];
    double angleRad = Math.toRadians(angleDeg);
    Rotation3d rotation = new Rotation3d(0, 0, angleRad);

    for (int i = 0; i < numPoints; i++) {
      double t = (double) i / (numPoints - 1);
      double x = startX + t * length * Math.cos(angleRad);
      double y = startY + t * length * Math.sin(angleRad);
      poses[i] = new Pose3d(x, y, height, rotation);
    }
    return poses;
  }
}
