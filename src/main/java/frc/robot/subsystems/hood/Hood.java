package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Hood subsystem for controlling the launch angle. Works with the trajectory optimizer to achieve
 * ideal arc trajectories using hybrid RPM+hood control.
 */
public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  // Tunable limits
  private static final LoggedTunableNumber minAngleDeg =
      new LoggedTunableNumber("Hood/MinAngleDeg", 15.0);
  private static final LoggedTunableNumber maxAngleDeg =
      new LoggedTunableNumber("Hood/MaxAngleDeg", 85.0);
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hood/ToleranceDeg", 1.0);

  // Default/home position
  private static final LoggedTunableNumber homeAngleDeg =
      new LoggedTunableNumber("Hood/HomeAngleDeg", 45.0);

  public Hood(HoodIO io) {
    this.io = io;

    System.out.println("[Hood] ========== STARTUP ==========");
    System.out.println("[Hood] Min angle: " + minAngleDeg.get() + " deg");
    System.out.println("[Hood] Max angle: " + maxAngleDeg.get() + " deg");
    System.out.println("[Hood] Home angle: " + homeAngleDeg.get() + " deg");
    System.out.println("[Hood] ==============================");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    // Log additional useful values
    Logger.recordOutput("Hood/AngleError", inputs.targetAngleDeg - inputs.currentAngleDeg);
    Logger.recordOutput("Hood/MinLimit", minAngleDeg.get());
    Logger.recordOutput("Hood/MaxLimit", maxAngleDeg.get());
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
    setAngle(homeAngleDeg.get());
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
    return minAngleDeg.get();
  }

  /**
   * Get the maximum allowed angle.
   *
   * @return Max angle in degrees
   */
  public double getMaxAngle() {
    return maxAngleDeg.get();
  }

  /**
   * Check if a given angle is within hood limits.
   *
   * @param angleDeg Angle to check
   * @return True if achievable
   */
  public boolean isAngleAchievable(double angleDeg) {
    return angleDeg >= minAngleDeg.get() && angleDeg <= maxAngleDeg.get();
  }

  /**
   * Clamp an angle to the hood's limits.
   *
   * @param angleDeg Angle to clamp
   * @return Clamped angle
   */
  public double clampToLimits(double angleDeg) {
    return Math.max(minAngleDeg.get(), Math.min(maxAngleDeg.get(), angleDeg));
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
