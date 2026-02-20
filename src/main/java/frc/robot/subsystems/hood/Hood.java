package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotConfig;
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

  private final RobotConfig config = Constants.getRobotConfig();

  // Tunable PID gains
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Tuning/Hood/kP", Constants.getRobotConfig().getHoodKp());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Tuning/Hood/kD", Constants.getRobotConfig().getHoodKd());

  // TODO: Hood uses PD-only control (no kI, no feedforward). This causes steady-state error
  // where the motor can't push through friction/gravity at small errors. Consider adding kI and/or
  // kS.

  // Tunable ready-gate tolerance for atTarget() — does NOT affect motor control
  private static final LoggedTunableNumber readyToleranceAngleDeg =
      new LoggedTunableNumber("Tuning/Hood/ReadyToleranceAngleDeg", 1.0);

  public Hood(HoodIO io) {
    this.io = io;

    // Push initial tolerance to IO
    io.setAngleTolerance(readyToleranceAngleDeg.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    // Push tunable PID changes to IO
    if (LoggedTunableNumber.hasChanged(kP, kD)) {
      io.configurePID(kP.get(), kD.get());
    }
    if (LoggedTunableNumber.hasChanged(readyToleranceAngleDeg)) {
      io.setAngleTolerance(readyToleranceAngleDeg.get());
    }

    // Log additional useful values
    Logger.recordOutput("Hood/AngleError", inputs.targetAngleDeg - inputs.currentAngleDeg);
    Logger.recordOutput("Hood/MinLimit", config.getHoodMinAngleDegrees());
    Logger.recordOutput("Hood/MaxLimit", config.getHoodMaxAngleDegrees());
  }

  /**
   * Set the hood angle.
   *
   * @param angleDeg Target angle in degrees (clamped to limits)
   */
  public void setHoodAngle(double angleDeg) {
    double clamped = clampToLimits(angleDeg);
    io.setAngle(clamped);

    if (clamped != angleDeg) {
      Logger.recordOutput("Hood/ClampedRequest", angleDeg);
    }
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
    return config.getHoodMinAngleDegrees();
  }

  /**
   * Get the maximum allowed angle.
   *
   * @return Max angle in degrees
   */
  public double getMaxAngle() {
    return config.getHoodMaxAngleDegrees();
  }

  /**
   * Check if a given angle is within hood limits.
   *
   * @param angleDeg Angle to check
   * @return True if achievable
   */
  public boolean isAngleAchievable(double angleDeg) {
    return angleDeg >= config.getHoodMinAngleDegrees()
        && angleDeg <= config.getHoodMaxAngleDegrees();
  }

  /**
   * Clamp an angle to the hood's limits.
   *
   * @param angleDeg Angle to clamp
   * @return Clamped angle
   */
  public double clampToLimits(double angleDeg) {
    return Math.max(
        config.getHoodMinAngleDegrees(), Math.min(config.getHoodMaxAngleDegrees(), angleDeg));
  }

  // ========== Commands ==========

  /**
   * Command to set hood to a specific angle.
   *
   * @param angleDeg Target angle
   * @return Command that completes when at target
   */
  public Command setAngleCommand(double angleDeg) {
    return runOnce(() -> setHoodAngle(angleDeg))
        .andThen(run(() -> {}).until(this::atTarget))
        .withName("Hood: Set to " + angleDeg + " deg");
  }
}
