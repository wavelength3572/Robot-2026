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
  private static final double TURRET_HEIGHT = 0.457; // meters (18 inches)

  private Translation3d[] trajectory = new Translation3d[TRAJECTORY_POINTS];
  private final Supplier<Pose3d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;

  // Fuel inventory management
  private static final int FUEL_CAPACITY = 30;
  private int fuelStored = 8;

  /**
   * Creates a new TurretVisualizer.
   *
   * @param robotPoseSupplier Supplier for robot's 3D pose
   * @param fieldSpeedsSupplier Supplier for field-relative chassis speeds
   */
  public TurretVisualizer(
      Supplier<Pose3d> robotPoseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.fieldSpeedsSupplier = fieldSpeedsSupplier;

    // Initialize trajectory array
    for (int i = 0; i < TRAJECTORY_POINTS; i++) {
      trajectory[i] = new Translation3d();
    }
  }

  /**
   * Calculate the launch velocity vector accounting for robot rotation and movement.
   *
   * @param exitVelocity Exit velocity magnitude in m/s
   * @param launchAngle Launch angle in radians from horizontal
   * @return 3D velocity vector in field coordinates
   */
  private Translation3d calculateLaunchVelocity(double exitVelocity, double launchAngle) {
    Pose3d robot = robotPoseSupplier.get();
    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

    // Calculate velocity components
    double horizontalVel = exitVelocity * Math.cos(launchAngle);
    double verticalVel = exitVelocity * Math.sin(launchAngle);

    // Rotate horizontal velocity by robot heading
    double robotHeading = robot.getRotation().toRotation2d().getRadians();
    double xVel = horizontalVel * Math.cos(robotHeading);
    double yVel = horizontalVel * Math.sin(robotHeading);

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

  /**
   * Add a fuel to the robot's inventory (called when intake picks up fuel).
   */
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
   * Launch a fuel ball into the simulation.
   *
   * @param exitVelocity Exit velocity in m/s
   * @param launchAngle Launch angle in radians
   */
  public void launchFuel(double exitVelocity, double launchAngle) {
    if (fuelStored == 0) return;
    fuelStored--;

    Pose3d robot = robotPoseSupplier.get();
    Translation3d launchPosition =
        new Translation3d(robot.getX(), robot.getY(), robot.getZ() + TURRET_HEIGHT);
    Translation3d launchVelocity = calculateLaunchVelocity(exitVelocity, launchAngle);

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
      Supplier<Double> exitVelocitySupplier,
      Supplier<Double> launchAngleSupplier,
      Turret turret) {
    return turret
        .runOnce(() -> launchFuel(exitVelocitySupplier.get(), launchAngleSupplier.get()))
        .andThen(Commands.waitSeconds(0.25))
        .repeatedly();
  }

  /**
   * Update the trajectory visualization based on current shot parameters. Calculates a parabolic
   * arc accounting for gravity and logs it to AdvantageKit.
   *
   * @param exitVelocity Exit velocity in m/s
   * @param launchAngle Launch angle in radians
   */
  public void updateTrajectory(double exitVelocity, double launchAngle) {
    Translation3d velocity = calculateLaunchVelocity(exitVelocity, launchAngle);
    Pose3d robot = robotPoseSupplier.get();

    double startX = robot.getX();
    double startY = robot.getY();
    double startZ = robot.getZ() + TURRET_HEIGHT;

    for (int i = 0; i < TRAJECTORY_POINTS; i++) {
      double t = i * TRAJECTORY_TIME_STEP;

      // Projectile motion equations
      double x = startX + velocity.getX() * t;
      double y = startY + velocity.getY() * t;
      double z = startZ + velocity.getZ() * t - 0.5 * GRAVITY * t * t;

      // Clamp z to ground level
      if (z < 0) z = 0;

      trajectory[i] = new Translation3d(x, y, z);
    }

    Logger.recordOutput("Turret/Trajectory", trajectory);
    Logger.recordOutput("Turret/TrajectoryExitVelocity", exitVelocity);
    Logger.recordOutput("Turret/TrajectoryLaunchAngleDeg", Math.toDegrees(launchAngle));
  }

  /**
   * Update the 3D turret pose visualization.
   *
   * @param azimuthAngle Turret azimuth angle in radians
   */
  public void update3dPose(double azimuthAngle) {
    Logger.recordOutput(
        "Turret/TurretPose3d", new Pose3d(0, 0, TURRET_HEIGHT, new Rotation3d(0, 0, azimuthAngle)));
  }

  /**
   * Log fuel inventory status.
   */
  public void logFuelStatus() {
    Logger.recordOutput("Turret/FuelStored", fuelStored);
    Logger.recordOutput("Turret/CanIntake", canIntake());
  }

  /**
   * Calculate and visualize trajectory for a target.
   *
   * @param shotData The calculated shot parameters
   */
  public void visualizeShot(TurretCalculator.ShotData shotData) {
    updateTrajectory(shotData.getExitVelocity(), shotData.getLaunchAngle());

    // Log target marker
    Logger.recordOutput(
        "Turret/TargetPosition",
        new Pose3d(shotData.getTarget(), new Rotation3d()));
  }
}
