// Adapted from FRC Team 5000 Hammerheads
// https://github.com/hammerheads5000/2026Rebuilt
// Open source under WPILib BSD license

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
 * Handles visualization of turret trajectory and fuel simulation integration. Calculates and logs
 * trajectory points for AdvantageScope visualization.
 */
public class TurretVisualizer {
  private static final int TRAJECTORY_POINTS = 50;
  private static final double TRAJECTORY_TIME_STEP = 0.04; // seconds between points
  private static final double GRAVITY = 9.81; // m/s^2

  // Trajectory for visualization (where the ball would actually go)
  private Translation3d[] actualTrajectory = new Translation3d[TRAJECTORY_POINTS];

  // Color-coded trajectory arrays (only one populated at a time based on launcher status)
  private Translation3d[] redTrajectory = new Translation3d[TRAJECTORY_POINTS];
  private Translation3d[] yellowTrajectory = new Translation3d[TRAJECTORY_POINTS];
  private Translation3d[] greenTrajectory = new Translation3d[TRAJECTORY_POINTS];
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
   * Creates a new TurretVisualizer.
   *
   * @param robotPoseSupplier Supplier for robot's 3D pose
   * @param fieldSpeedsSupplier Supplier for field-relative chassis speeds
   * @param turretHeightMeters Height of the turret in meters
   * @param turretXOffset Turret X offset from robot center (robot-relative, positive = forward)
   * @param turretYOffset Turret Y offset from robot center (robot-relative, positive = left)
   */
  public TurretVisualizer(
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
      redTrajectory[i] = new Translation3d();
      yellowTrajectory[i] = new Translation3d();
      greenTrajectory[i] = new Translation3d();
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

    // Transform turret offset from robot-relative to field-relative coordinates
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

    // Calculate velocity components
    double horizontalVel = exitVelocity * Math.cos(launchAngle);
    double verticalVel = exitVelocity * Math.sin(launchAngle);

    // Rotate horizontal velocity by turret azimuth (direction to target)
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
   * @param turret The turret subsystem (for requirements)
   * @return Command that launches fuel every 0.25 seconds
   */
  public Command repeatedlyLaunchFuel(
      Supplier<Double> exitVelocitySupplier, Supplier<Double> launchAngleSupplier, Turret turret) {
    return turret
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

      // Projectile motion equations
      double x = startX + velocity.getX() * t;
      double y = startY + velocity.getY() * t;
      double z = startZ + velocity.getZ() * t - 0.5 * GRAVITY * t * t;

      // Clamp z to ground level
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
    double currentRPM = TurretCalculator.getCurrentLauncherRPM();
    double targetRPM = TurretCalculator.getTargetLauncherRPM();

    // Unpowered: both current and target near zero
    if (Math.abs(currentRPM) < RPM_VELOCITY_THRESHOLD
        && Math.abs(targetRPM) < RPM_VELOCITY_THRESHOLD) {
      return LauncherStatus.UNPOWERED;
    }

    // At setpoint: current RPM within tolerance of target
    if (Math.abs(currentRPM - targetRPM) < RPM_SETPOINT_TOLERANCE
        && targetRPM > RPM_VELOCITY_THRESHOLD) {
      return LauncherStatus.AT_SETPOINT;
    }

    // Otherwise: spinning up
    return LauncherStatus.SPINNING_UP;
  }

  /**
   * Update the ACTUAL trajectory visualization (where we'd REALLY shoot). Uses current launcher RPM
   * and turret angle. Also logs color-coded trajectories based on launcher status.
   *
   * @param currentExitVelocity Actual exit velocity based on current launcher RPM
   * @param currentLaunchAngle Current launch angle in radians
   * @param currentAzimuthAngle Current turret azimuth angle in radians
   */
  public void updateActualTrajectory(
      double currentExitVelocity, double currentLaunchAngle, double currentAzimuthAngle) {
    calculateTrajectoryPoints(
        currentExitVelocity, currentLaunchAngle, currentAzimuthAngle, actualTrajectory);

    // Log color-coded trajectories based on launcher status
    LauncherStatus status = getLauncherStatus();
    Logger.recordOutput("Turret/LauncherStatus", status.name());

    // Populate the appropriate color trajectory, empty the others
    switch (status) {
      case UNPOWERED:
        System.arraycopy(actualTrajectory, 0, redTrajectory, 0, TRAJECTORY_POINTS);
        Logger.recordOutput("Turret/Trajectory/Red", redTrajectory);
        Logger.recordOutput("Turret/Trajectory/Yellow", EMPTY_TRAJECTORY);
        Logger.recordOutput("Turret/Trajectory/Green", EMPTY_TRAJECTORY);
        break;
      case SPINNING_UP:
        System.arraycopy(actualTrajectory, 0, yellowTrajectory, 0, TRAJECTORY_POINTS);
        Logger.recordOutput("Turret/Trajectory/Red", EMPTY_TRAJECTORY);
        Logger.recordOutput("Turret/Trajectory/Yellow", yellowTrajectory);
        Logger.recordOutput("Turret/Trajectory/Green", EMPTY_TRAJECTORY);
        break;
      case AT_SETPOINT:
        System.arraycopy(actualTrajectory, 0, greenTrajectory, 0, TRAJECTORY_POINTS);
        Logger.recordOutput("Turret/Trajectory/Red", EMPTY_TRAJECTORY);
        Logger.recordOutput("Turret/Trajectory/Yellow", EMPTY_TRAJECTORY);
        Logger.recordOutput("Turret/Trajectory/Green", greenTrajectory);
        break;
    }
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
   * @param shotData The calculated shot parameters (used for target position)
   * @param currentTurretAngleRad Current turret angle in radians (for actual trajectory)
   * @param robotHeadingRad Robot heading in radians (to convert turret angle to field-relative)
   */
  public void visualizeShot(
      TurretCalculator.ShotData shotData, double currentTurretAngleRad, double robotHeadingRad) {
    Translation3d turretPos = getTurretFieldPosition();
    Translation3d target = shotData.getTarget();

    // Calculate direction to target (field-relative)
    double targetAzimuthAngle =
        Math.atan2(target.getY() - turretPos.getY(), target.getX() - turretPos.getX());

    // Store azimuth for use by launchFuel
    this.currentAzimuthAngle = targetAzimuthAngle;

    // Update trajectory visualization (color-coded by launcher status)
    updateActualTrajectory(
        shotData.getExitVelocity(), shotData.getLaunchAngle(), targetAzimuthAngle);

    // Log current launcher RPM (Turret.java owns the other Shot/ values)
    Logger.recordOutput("Turret/Shot/CurrentRPM", TurretCalculator.getCurrentLauncherRPM());

    // Log compensated target marker (offset from hub when robot is moving)
    Logger.recordOutput(
        "Turret/Shot/CompensatedTarget", new Pose3d(shotData.getTarget(), new Rotation3d()));
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
}
