package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Hood subsystem for controlling the launch angle. Works with the trajectory optimizer to achieve
 * ideal arc trajectories using hybrid RPM+hood control.
 */
public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  // Physical limits (constants - these are hardware constraints)
  private static final double MIN_ANGLE_DEG = 0.0;
  private static final double MAX_ANGLE_DEG = 75.0;
  private static final double TOLERANCE_DEG = 1.0;
  private static final double HOME_ANGLE_DEG = 45.0;

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    // Log additional useful values
    Logger.recordOutput("Hood/AngleError", inputs.targetAngleDeg - inputs.currentAngleDeg);
    Logger.recordOutput("Hood/MinLimit", MIN_ANGLE_DEG);
    Logger.recordOutput("Hood/MaxLimit", MAX_ANGLE_DEG);
  }

  /**
   * Set the hood angle.
   *
   * @param angleDeg Target angle in degrees (clamped to limits)
   */
  public void setAngle(double angleDeg) {
    double clamped = clampToLimits(angleDeg);
    io.setAngle(clamped);

    if (clamped != angleDeg) {
      Logger.recordOutput("Hood/ClampedRequest", angleDeg);
    }
  }

  /** Move hood to home position. */
  public void goToHome() {
    setAngle(HOME_ANGLE_DEG);
  }

  /** Stop the hood and hold current position. */
  public void stop() {
    io.stop();
  }

  /**
   * Get the current hood angle.
   *
   * @return Current angle in degrees
   */
  @AutoLogOutput(key = "Hood/currentAngle")
  public double getCurrentAngle() {
    return inputs.currentAngleDeg;
  }

  /**
   * Get the target hood angle.
   *
   * @return Target angle in degrees
   */
  public double getTargetAngle() {
    return inputs.targetAngleDeg;
  }

  /**
   * Check if the hood is at the target angle.
   *
   * @return True if within tolerance
   */
  public boolean atTarget() {
    return inputs.atTarget;
  }

  /**
   * Check if the hood is connected.
   *
   * @return True if motor is responding
   */
  public boolean isConnected() {
    return inputs.connected;
  }

  /**
   * Get the minimum allowed angle.
   *
   * @return Min angle in degrees
   */
  public double getMinAngle() {
    return MIN_ANGLE_DEG;
  }

  /**
   * Get the maximum allowed angle.
   *
   * @return Max angle in degrees
   */
  public double getMaxAngle() {
    return MAX_ANGLE_DEG;
  }

  /**
   * Check if a given angle is within hood limits.
   *
   * @param angleDeg Angle to check
   * @return True if achievable
   */
  public boolean isAngleAchievable(double angleDeg) {
    return angleDeg >= MIN_ANGLE_DEG && angleDeg <= MAX_ANGLE_DEG;
  }

  /**
   * Clamp an angle to the hood's limits.
   *
   * @param angleDeg Angle to clamp
   * @return Clamped angle
   */
  public double clampToLimits(double angleDeg) {
    return Math.max(MIN_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, angleDeg));
  }

  // ========== Commands ==========

  /**
   * Command to set hood to a specific angle.
   *
   * @param angleDeg Target angle
   * @return Command that completes when at target
   */
  public edu.wpi.first.wpilibj2.command.Command setAngleCommand(double angleDeg) {
    return runOnce(() -> setAngle(angleDeg))
        .andThen(run(() -> {}).until(this::atTarget))
        .withName("Hood: Set to " + angleDeg + " deg");
  }

  /**
   * Command to go to home position.
   *
   * @return Command that completes when at home
   */
  public edu.wpi.first.wpilibj2.command.Command goToHomeCommand() {
    return runOnce(this::goToHome)
        .andThen(run(() -> {}).until(this::atTarget))
        .withName("Hood: Go Home");
  }
}
