package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  /**
   * Creates a new Turret subsystem.
   *
   * @param io The IO implementation to use (real hardware or simulation)
   */
  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
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
            + (TurretConstants.TURRET_OFFSET_X * Math.cos(robotOmegaRad)
                - TurretConstants.TURRET_OFFSET_Y * Math.sin(robotOmegaRad));

    double turretFieldY =
        robotY
            + (TurretConstants.TURRET_OFFSET_X * Math.sin(robotOmegaRad)
                + TurretConstants.TURRET_OFFSET_Y * Math.cos(robotOmegaRad));

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
   * Set the turret to point at a specific angle relative to the robot's front.
   *
   * @param angleDegrees Angle in degrees (-180 to 180, positive = counter-clockwise)
   */
  public void setAngle(double angleDegrees) {
    io.setTargetAngle(new Rotation2d(Rotation2d.fromDegrees(angleDegrees).getRadians()));
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
        0,
        0,
        0.127,
        new Rotation3d(0.0, 0.0, Rotation2d.fromDegrees(getCurrentAngle()).getRadians()));
  }
}
