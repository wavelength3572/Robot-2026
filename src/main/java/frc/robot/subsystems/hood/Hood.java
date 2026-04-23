package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Hood subsystem for controlling the launch angle. Works with the trajectory optimizer to achieve
 * ideal arc trajectories using hybrid RPM+hood control.
 */
public class Hood extends SubsystemBase {

  /** Hood operating state. */
  public enum HoodState {
    /** At target angle, actively commanded by a shooting command. */
    READY,
    /** At default (stow) position — no shooting command is driving the hood. */
    STOWED,
    /** At target, but angle is safety-clamped (e.g. trench) — not ready to fire. */
    CLAMPED,
    RAISING,
    LOWERING,
    DISCONNECTED
  }

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final RobotConfig config = Constants.getRobotConfig();

  // Tunable PID gains
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Tuning/Hood/kP", Constants.getRobotConfig().getHoodKp());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Tuning/Hood/kD", Constants.getRobotConfig().getHoodKd());

  // Note: Hood uses PD-only control (no kI, no feedforward). If steady-state error from
  // friction/gravity becomes a problem, consider adding kI and/or kS.

  // Tunable ready-gate tolerance for atTarget() — does NOT affect motor control
  private static final LoggedTunableNumber readyToleranceAngleDeg =
      new LoggedTunableNumber(
          "Tuning/Hood/ReadyToleranceAngleDeg",
          Constants.getRobotConfig().getHoodReadyToleranceAngleDeg());

  // Current state — promoted from periodic() local for external readiness checks
  private HoodState currentState = HoodState.STOWED;

  // When true, the hood is being driven by a shooting command (not the default stow).
  // Set via setActivelyCommanded() — shooting commands set true, default command sets false.
  private boolean activelyCommanded = false;

  // When true, the hood angle is being safety-clamped (e.g. trench mode).
  // Set by ShootingCoordinator. Prevents READY state so firing is blocked until unclamped.
  private boolean clamped = false;

  public Hood(HoodIO io) {
    this.io = io;

    // Push initial tolerance to IO
    io.setAngleTolerance(readyToleranceAngleDeg.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    // Compute and log state
    if (!inputs.connected) {
      currentState = HoodState.DISCONNECTED;
    } else if (inputs.atTarget) {
      if (clamped) {
        currentState = HoodState.CLAMPED;
      } else {
        currentState = activelyCommanded ? HoodState.READY : HoodState.STOWED;
      }
    } else if (inputs.targetAngleDeg > inputs.currentAngleDeg) {
      currentState = HoodState.RAISING;
    } else {
      currentState = HoodState.LOWERING;
    }
    Logger.recordOutput("Subsystems/HoodState", currentState.name());
    Logger.recordOutput("Subsystems/HoodTrenchClamped", clamped);

    // Push tunable PID changes to IO
    if (LoggedTunableNumber.hasChanged(kP, kD)) {
      io.configurePID(kP.get(), kD.get());
    }
    if (LoggedTunableNumber.hasChanged(readyToleranceAngleDeg)) {
      io.setAngleTolerance(readyToleranceAngleDeg.get());
    }
  }

  /**
   * Set the hood angle.
   *
   * @param angleDeg Target angle in degrees (clamped to limits)
   */
  public void setHoodAngle(double angleDeg) {
    double clamped = clampToLimits(angleDeg);
    io.setAngle(clamped);
  }

  /**
   * Get the current hood angle.
   *
   * @return Current angle in degrees
   */
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
   * Get the current hood operating state.
   *
   * @return Current HoodState
   */
  public HoodState getState() {
    return currentState;
  }

  /**
   * Mark whether the hood is being actively commanded by a shooting command. When false (default
   * command running), at-target reports STOWED instead of READY. Shooting commands should call
   * setActivelyCommanded(true); the default stow command should call setActivelyCommanded(false).
   */
  public void setActivelyCommanded(boolean commanded) {
    this.activelyCommanded = commanded;
  }

  /**
   * Mark whether the hood angle is being safety-clamped. When clamped, at-target reports CLAMPED
   * instead of READY so the coordinator won't gate firing on a clamped angle.
   */
  public void setClamped(boolean clamped) {
    this.clamped = clamped;
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

   public void setHoodVolts(double volts) {
    io.setHoodVolts(volts);
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
