// Adapted from FRC Team 5000 Hammerheads
// https://github.com/hammerheads5000/2026Rebuilt
// Open source under WPILib BSD license

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.TrajectoryOptimizer;
import frc.robot.util.FuelSim;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Handles visualization of turret trajectory and fuel simulation integration. Calculates and logs
 * trajectory points for AdvantageScope visualization.
 */
public class TurretVisualizer {
  private static final int TRAJECTORY_POINTS = 50;
  private static final double TRAJECTORY_TIME_STEP = 0.04; // seconds between points
  private static final double GRAVITY = 9.81; // m/s^2

  // Two trajectories:
  // - Ideal: The physics-optimal arc that would actually HIT the target
  // - Actual: Where the ball would really go with current launcher RPM and turret angle
  private Translation3d[] idealTrajectory = new Translation3d[TRAJECTORY_POINTS];
  private Translation3d[] actualTrajectory = new Translation3d[TRAJECTORY_POINTS];
  private final Supplier<Pose3d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;
  private final double turretHeightMeters;

  // Turret offset from robot center (robot-relative coordinates)
  private final double turretXOffset;
  private final double turretYOffset;

  // Fuel inventory management
  private static final int FUEL_CAPACITY = 30;
  private int fuelStored = 25;

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
      idealTrajectory[i] = new Translation3d();
      actualTrajectory[i] = new Translation3d();
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

  /** Add a fuel to the robot's inventory (called when intake picks up fuel). */
  public void intakeFuel() {
    if (fuelStored < FUEL_CAPACITY) {
      fuelStored++;
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
   * Update the IDEAL trajectory visualization using the trajectory optimizer. Shows the optimal
   * RPM+hood combination that achieves desired clearance above the hub lip.
   *
   * @param target The hub center position
   * @param azimuthAngle Direction to target in radians (field-relative)
   */
  public void updateIdealTrajectory(Translation3d target, double azimuthAngle) {
    // Use trajectory optimizer with turret position (where ball actually launches from)
    Translation3d turretPos = getTurretFieldPosition();
    TrajectoryOptimizer.OptimalShot optimalShot =
        TrajectoryOptimizer.calculateOptimalShot(turretPos, target);

    double idealVelocity = optimalShot.exitVelocityMps;
    double idealAngleDeg = optimalShot.launchAngleDeg;
    double idealAngleRad = Math.toRadians(idealAngleDeg);

    calculateTrajectoryPoints(idealVelocity, idealAngleRad, azimuthAngle, idealTrajectory);
    Logger.recordOutput("Turret/IdealTrajectory", idealTrajectory);
    Logger.recordOutput("Turret/Ideal/VelocityMps", idealVelocity);
    Logger.recordOutput("Turret/Ideal/LaunchAngleDeg", idealAngleDeg);
    Logger.recordOutput("Turret/Ideal/RPM", optimalShot.rpm);
    Logger.recordOutput("Turret/Ideal/PeakHeightM", optimalShot.peakHeightM);
    Logger.recordOutput("Turret/Ideal/Achievable", optimalShot.achievable);
  }

  /**
   * Update the ACTUAL trajectory visualization (where we'd REALLY shoot). Uses current launcher RPM
   * and turret angle.
   *
   * @param currentExitVelocity Actual exit velocity based on current launcher RPM
   * @param currentLaunchAngle Current launch angle in radians
   * @param currentAzimuthAngle Current turret azimuth angle in radians
   */
  public void updateActualTrajectory(
      double currentExitVelocity, double currentLaunchAngle, double currentAzimuthAngle) {
    calculateTrajectoryPoints(
        currentExitVelocity, currentLaunchAngle, currentAzimuthAngle, actualTrajectory);
    Logger.recordOutput("Turret/ActualTrajectory", actualTrajectory);
  }

  /**
   * Legacy method - updates actual trajectory with given parameters.
   *
   * @deprecated Use updateActualTrajectory instead
   */
  @Deprecated
  public void updateTrajectory(double exitVelocity, double launchAngle, double azimuthAngle) {
    updateActualTrajectory(exitVelocity, launchAngle, azimuthAngle);
  }

  /** Log fuel inventory status. */
  public void logFuelStatus() {
    Logger.recordOutput("Intake/Fuel/Stored", fuelStored);
    Logger.recordOutput("Intake/Fuel/CanIntake", canIntake());
  }

  /**
   * Calculate and visualize trajectories for a target. Updates BOTH ideal and actual trajectories.
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

    // IDEAL trajectory: Uses trajectory optimizer for optimal RPM+hood combination
    // Uses turret position (where ball launches from), not robot center
    updateIdealTrajectory(target, targetAzimuthAngle);

    // ACTUAL trajectory: What we'll actually launch with (same as launchFuel uses)
    // Uses the shot data's ideal parameters since that's what launchFuel() uses
    double actualExitVelocity = shotData.getExitVelocity();
    double actualLaunchAngle = shotData.getLaunchAngle();

    updateActualTrajectory(actualExitVelocity, actualLaunchAngle, targetAzimuthAngle);

    // Log for debugging
    Logger.recordOutput("Turret/Shot/TargetRPM", TurretCalculator.getTargetLauncherRPM());
    Logger.recordOutput("Turret/Shot/CurrentRPM", TurretCalculator.getCurrentLauncherRPM());
    Logger.recordOutput("Turret/Shot/ExitVelocityMps", actualExitVelocity);
    Logger.recordOutput("Turret/Shot/LaunchAngleDeg", Math.toDegrees(actualLaunchAngle));
    Logger.recordOutput("Turret/Shot/AzimuthDeg", Math.toDegrees(targetAzimuthAngle));

    // Log target marker
    Logger.recordOutput(
        "Turret/TargetPosition", new Pose3d(shotData.getTarget(), new Rotation3d()));
  }

  /**
   * Calculate the optimal launch angle for a given exit velocity to hit the target. Uses higher
   * angle solution for arc shots that drop into the hub.
   *
   * @param exitVelocity Exit velocity in m/s
   * @param target Target position
   * @return Launch angle in radians
   */
  private double calculateLaunchAngleForVelocity(double exitVelocity, Translation3d target) {
    Translation3d turretPos = getTurretFieldPosition();

    double dx = target.getX() - turretPos.getX();
    double dy = target.getY() - turretPos.getY();
    double horizontalDistance = Math.sqrt(dx * dx + dy * dy);
    double heightDiff = target.getZ() - turretPos.getZ();

    double v2 = exitVelocity * exitVelocity;
    double v4 = v2 * v2;
    double g = GRAVITY;
    double x = horizontalDistance;
    double y = heightDiff;

    double discriminant = v4 - g * (g * x * x + 2 * y * v2);
    if (discriminant >= 0) {
      // Use higher angle for arc shot
      return Math.atan((v2 + Math.sqrt(discriminant)) / (g * x));
    } else {
      // Target unreachable - use 45 degrees
      return Math.PI / 4;
    }
  }

  /**
   * Legacy method - updates ideal trajectory only.
   *
   * @deprecated Use visualizeShot(ShotData, currentTurretAngleRad, robotHeadingRad) instead
   */
  @Deprecated
  public void visualizeShot(TurretCalculator.ShotData shotData) {
    Translation3d turretPos = getTurretFieldPosition();
    Translation3d target = shotData.getTarget();
    double azimuthAngle =
        Math.atan2(target.getY() - turretPos.getY(), target.getX() - turretPos.getX());

    // Store azimuth for use by launchFuel
    this.currentAzimuthAngle = azimuthAngle;

    // Uses turret position (where ball launches from), not robot center
    updateIdealTrajectory(target, azimuthAngle);

    // Log target marker
    Logger.recordOutput(
        "Turret/TargetPosition", new Pose3d(shotData.getTarget(), new Rotation3d()));
  }
}
