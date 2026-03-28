package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.subsystems.shooting.ShotCalculator;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Launcher subsystem for controlling two coupled Vortex motors. Uses velocity control with hardware
 * follower mode to ensure motors run in sync.
 */
public class Launcher extends SubsystemBase {

  /** Launcher operating state. */
  public enum LauncherState {
    IDLE,
    SPINNING_UP,
    READY,
    RECOVERING,
    DISCONNECTED
  }

  private final LauncherIO io;
  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

  // Tunable PID gains
  private static final LoggedTunableNumber kP;
  private static final LoggedTunableNumber kI;
  private static final LoggedTunableNumber kD;
  private static final LoggedTunableNumber kS;
  private static final LoggedTunableNumber kV;

  private static final LoggedTunableNumber recoveryArbFFPct =
      new LoggedTunableNumber("Tuning/Launcher/RecoveryArbFFPct", 0.0);

  // IZone: integral only accumulates when error is below this threshold (motor RPM).
  // Prevents windup during spin-up while allowing kI to eliminate steady-state error.
  private static final LoggedTunableNumber iZone;

  // Tunable ready-gate tolerance for atSetpoint() — does NOT affect motor control
  private static final LoggedTunableNumber velocityToleranceRPM =
      new LoggedTunableNumber("Tuning/Launcher/ReadyToleranceRPM", 100);

  // Recovery: threshold error (wheel RPM) to activate Slot 1
  private static final LoggedTunableNumber recoveryBoostThresholdRPM =
      new LoggedTunableNumber("Tuning/Launcher/RecoveryBoostThresholdRPM", 40.0);

  static {
    RobotConfig config = Constants.getRobotConfig();
    kP = new LoggedTunableNumber("Tuning/Launcher/kP", config.getLauncherKp());
    kI = new LoggedTunableNumber("Tuning/Launcher/kI", config.getLauncherKi());
    kD = new LoggedTunableNumber("Tuning/Launcher/kD", config.getLauncherKd());
    kS = new LoggedTunableNumber("Tuning/Launcher/kS", config.getLauncherKs());
    kV = new LoggedTunableNumber("Tuning/Launcher/kV", config.getLauncherKv());
    iZone = new LoggedTunableNumber("Tuning/Launcher/IZone", config.getLauncherIZone());
  }

  // Current state — promoted from periodic() local for external readiness checks
  private LauncherState currentState = LauncherState.IDLE;

  // Feeding flag - set by ShootingCommands when prefeed starts
  private boolean feedingActive = false;

  // Tracks whether recovery mode is currently active (for hysteresis)
  private boolean recoveryActive = false;

  public Launcher(LauncherIO io) {
    this.io = io;

    // Push initial velocity tolerance to IO
    io.setVelocityTolerance(velocityToleranceRPM.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Launcher", inputs);

    // Compute recovery state before state check so they're evaluated in the same cycle,
    // avoiding a one-cycle SPINNING_UP glitch before RECOVERING after a shot.
    if (feedingActive && inputs.targetVelocityRPM > 100.0) {
      double error = inputs.targetVelocityRPM - inputs.wheelVelocityRPM;
      double threshold = recoveryBoostThresholdRPM.get();
      if (!recoveryActive && error > threshold) {
        recoveryActive = true;
      } else if (recoveryActive && error < threshold * 0.9) {
        recoveryActive = false;
      }
    } else {
      recoveryActive = false;
    }

    // Compute and log state
    if (!isConnected()) {
      currentState = LauncherState.DISCONNECTED;
    } else if (inputs.targetVelocityRPM < 100.0) {
      currentState = LauncherState.IDLE;
    } else if (inputs.atSetpoint) {
      currentState = LauncherState.READY;
    } else if (recoveryActive) {
      currentState = LauncherState.RECOVERING;
    } else {
      currentState = LauncherState.SPINNING_UP;
    }

    Logger.recordOutput("Subsystems/LauncherState", currentState.name());
    Logger.recordOutput("Subsystems/LauncherFeedingActive", feedingActive);
    Logger.recordOutput("Subsystems/LauncherRecoveryActive", recoveryActive);

    // Update ShotCalculator with current wheel RPM for trajectory calculations
    ShotCalculator.setLauncherRPM(inputs.wheelVelocityRPM);

    // Push tunable changes to IO
    if (LoggedTunableNumber.hasChanged(kP, kI, kD, iZone)) {
      io.configurePID(kP.get(), kI.get(), kD.get(), iZone.get());
    }
    if (LoggedTunableNumber.hasChanged(kS, kV)) {
      io.configureFeedforward(kS.get(), kV.get());
    }
    if (LoggedTunableNumber.hasChanged(velocityToleranceRPM)) {
      io.setVelocityTolerance(velocityToleranceRPM.get());
    }
  }

  /**
   * Set the launcher wheel velocity. Recovery state is computed in periodic() so it aligns with
   * state logging.
   *
   * @param velocityRPM Target velocity in wheel RPM
   */
  public void setVelocity(double velocityRPM) {
    // Compute recovery arbFF voltage proportional to target RPM's steady-state FF.
    // FF voltage ≈ kS + kV * motorRPM. Scale by tunable percentage.
    double gearRatio = Constants.getRobotConfig().getLauncherGearRatio();
    double motorRPM = velocityRPM / gearRatio;
    double steadyStateFF = kS.get() + (kV.get() * motorRPM);
    double recoveryArbFFVolts = recoveryActive ? recoveryArbFFPct.get() * steadyStateFF : 0.0;

    io.setVelocity(velocityRPM, recoveryActive, recoveryArbFFVolts);
    ShotCalculator.setTargetLauncherRPM(velocityRPM);
  }

  /**
   * Set whether feeding is actively pushing balls into the launcher. Used by ShootingCommands to
   * enable recovery boost only during actual shooting.
   *
   * @param active True when prefeed is running
   */
  public void setFeedingActive(boolean active) {
    this.feedingActive = active;
  }

  /** Stop the launcher. */
  public void stop() {
    feedingActive = false;
    io.stop();
    ShotCalculator.setTargetLauncherRPM(0.0);
  }

  /**
   * Signal that a ball was fired (simulation only). In sim, this triggers a recovery period where
   * atSetpoint returns false until the configured recovery time elapses. Does nothing on real
   * hardware.
   */
  public void notifyBallFired() {
    io.notifyBallFired();
  }

  /**
   * Get the current wheel velocity.
   *
   * @return Current velocity in wheel RPM
   */
  public double getVelocity() {
    return inputs.wheelVelocityRPM;
  }

  /**
   * Get the target velocity.
   *
   * @return Target velocity in wheel RPM
   */
  public double getTargetVelocity() {
    return inputs.targetVelocityRPM;
  }

  /**
   * Get the current launcher operating state.
   *
   * @return Current LauncherState
   */
  public LauncherState getState() {
    return currentState;
  }

  /**
   * Whether the launcher should be considered ready for firing. True when at setpoint (READY) or
   * actively recovering from an RPM dip after a shot (RECOVERING). Recovery dips are expected
   * during multi-ball sequences and should not gate firing.
   */
  public boolean isReady() {
    return currentState == LauncherState.READY || currentState == LauncherState.RECOVERING;
  }

  /**
   * Check if the launcher is at the target velocity. Returns false when idle (no velocity
   * commanded) to prevent downstream logic from treating an unpowered launcher as "ready".
   *
   * @return True if at setpoint within tolerance and actively spinning
   */
  // public boolean atSetpoint() {
  //   return inputs.targetVelocityRPM >= 100.0 && inputs.atSetpoint;
  // }

  /**
   * Check if both motors are connected.
   *
   * @return True if both motors are responding
   */
  public boolean isConnected() {
    return inputs.leaderConnected && inputs.followerConnected;
  }

  /** Runs the motor with the specified output. */
  public void runCharacterization(double output) {
    io.setLauncherVoltage(output);
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = io.getFFCharacterizationVelocity();
    return output;
  }

  // ========== Commands ==========

  /**
   * Command to run the launcher at a specific velocity.
   *
   * @param velocityRPM Target velocity in wheel RPM
   * @return Command that runs until interrupted
   */
  public Command runAtVelocityCommand(double velocityRPM) {
    return run(() -> setVelocity(velocityRPM))
        .finallyDo(this::stop)
        .withName("Launcher: Run at " + velocityRPM + " RPM");
  }

  /**
   * Command to run the launcher at a tunable velocity. Reads the tunable each cycle so RPM changes
   * take effect live.
   *
   * @param rpm Tunable RPM source
   * @return Command that runs until interrupted
   */
  public Command runAtTunableVelocityCommand(LoggedTunableNumber rpm) {
    return run(() -> setVelocity(rpm.get()))
        .finallyDo(this::stop)
        .withName("Launcher: Run at Tunable RPM");
  }

  /**
   * Command to stop the launcher.
   *
   * @return Instant command that stops motors
   */
  public Command stopCommand() {
    return runOnce(this::stop).withName("Launcher: Stop");
  }
}
