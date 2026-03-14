package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Spindexer subsystem for feeding balls to the spindexer. Controls one independent motor:
 *
 * <ul>
 *   <li>Motor 1 (CAN 55): spindexer wheel
 * </ul>
 */
public class Spindexer extends SubsystemBase {
  private final SpindexerIO io;
  private final MotorInputsAutoLogged spindexerInputs = new MotorInputsAutoLogged();

  // Spindexer state — tracks what the motor is actually doing, logged for observability
  private enum SpindexerState {
    STOPPED, // Motor off
    FEEDING, // Running at commanded velocity to deliver fuel
    SUPPRESSED, // Feeding suppressed by operator (motor held at 0)
    UNCLOGGING, // Reversed to clear a jam (manual)
    AUTO_UNCLOGGING, // Reversed to clear a detected stall (automatic)
    RECIPROCATING // Gentle back-and-forth jostle to keep fuel loose
  }

  private SpindexerState state = SpindexerState.STOPPED;

  // When true, setSpindexerVelocity() sends 0 instead of the requested RPM.
  // Used by the driver to temporarily suppress feeding without interrupting shooting commands.
  private boolean feedingSuppressed = false;

  // When true, setSpindexerVelocity() reverses the spindexer at unclogRPM instead of the
  // requested RPM. Used to unclog without interrupting shooting commands — on release the
  // shooting command resumes feeding instantly.
  private boolean unclogActive = false;
  private boolean wasUnclogActive = false;

  private static final LoggedTunableNumber unclogRPM =
      new LoggedTunableNumber("Tuning/Spindexer/UnclogRPM", 1000.0);

  // Auto-unclog — detects stall during FEEDING and briefly reverses to clear the jam.
  // Disabled by default; enable from dashboard when ready to test.
  private boolean autoUnclogEnabled = false;
  private boolean autoUnclogInProgress = false;
  private final Timer stallTimer = new Timer(); // How long stall condition has persisted
  private final Timer autoUnclogTimer = new Timer(); // How long the reverse burst has been running
  private int autoUnclogAttempts = 0; // Attempts this feeding session

  private static final LoggedTunableNumber autoUnclogStallCurrentThreshold =
      new LoggedTunableNumber("Tuning/Spindexer/AutoUnclog/StallCurrentAmps", 15.0);
  private static final LoggedTunableNumber autoUnclogStallVelocityThreshold =
      new LoggedTunableNumber("Tuning/Spindexer/AutoUnclog/StallVelocityRPM", 50.0);
  private static final LoggedTunableNumber autoUnclogStallDurationSec =
      new LoggedTunableNumber("Tuning/Spindexer/AutoUnclog/StallDurationSec", 0.3);
  private static final LoggedTunableNumber autoUnclogReverseDurationSec =
      new LoggedTunableNumber("Tuning/Spindexer/AutoUnclog/ReverseDurationSec", 0.25);
  private static final LoggedTunableNumber autoUnclogMaxAttempts =
      new LoggedTunableNumber("Tuning/Spindexer/AutoUnclog/MaxAttempts", 3);

  // Reciprocation — gentle back-and-forth jostle to keep fuel loose when not actively feeding.
  // Call reciprocate() each cycle to jostle; it alternates direction on a timer.
  private boolean reciprocateForward = true;
  private final Timer reciprocateTimer = new Timer();

  private static final LoggedTunableNumber reciprocateRPM =
      new LoggedTunableNumber("Tuning/Spindexer/Reciprocate/RPM", 200.0);
  private static final LoggedTunableNumber reciprocateIntervalSec =
      new LoggedTunableNumber("Tuning/Spindexer/Reciprocate/IntervalSec", 0.5);

  // Tunable PID gains
  private static final LoggedTunableNumber kP;
  private static final LoggedTunableNumber kI;
  private static final LoggedTunableNumber kD;
  private static final LoggedTunableNumber kS;
  private static final LoggedTunableNumber kV;

  static {
    RobotConfig config = Constants.getRobotConfig();
    kP = new LoggedTunableNumber("Tuning/Spindexer/kP", config.getSpindexerKp());
    kI = new LoggedTunableNumber("Tuning/Spindexer/kI", config.getSpindexerKi());
    kD = new LoggedTunableNumber("Tuning/Spindexer/kD", config.getSpindexerKd());
    kS = new LoggedTunableNumber("Tuning/Spindexer/kS", config.getSpindexerKs());
    kV = new LoggedTunableNumber("Tuning/Spindexer/kV", config.getSpindexerKv());
  }

  // Tunable ready-gate tolerance for atSetpoint() — does NOT affect motor control
  private static final LoggedTunableNumber spindexerToleranceRPM =
      new LoggedTunableNumber("Tuning/Spindexer/ReadyToleranceRPM", 100.0);

  public Spindexer(SpindexerIO io) {
    this.io = io;

    // Push initial velocity tolerances to IO
    io.setVelocityTolerance(spindexerToleranceRPM.get());

    // Default reciprocation on — toggle from Elastic dashboard
    SmartDashboard.putBoolean("Tuning/Spindexer/Reciprocate/Enabled", true);
  }

  @Override
  public void periodic() {
    io.updateInputs(spindexerInputs);
    Logger.processInputs("Spindexer", spindexerInputs);
    Logger.recordOutput("Spindexer/State", state.name());
    Logger.recordOutput(
        "Spindexer/Reciprocate/Enabled",
        SmartDashboard.getBoolean("Tuning/Spindexer/Reciprocate/Enabled", true));
    Logger.recordOutput("Spindexer/AutoUnclog/Enabled", autoUnclogEnabled);
    Logger.recordOutput("Spindexer/AutoUnclog/Attempts", autoUnclogAttempts);

    // Auto-unclog: detect stall during FEEDING and trigger a brief reverse burst.
    // Stall = high current + low velocity for a sustained period.
    if (autoUnclogEnabled && !unclogActive) {
      if (autoUnclogInProgress) {
        // Reverse burst in progress — check if duration has elapsed
        if (autoUnclogTimer.hasElapsed(autoUnclogReverseDurationSec.get())) {
          autoUnclogInProgress = false;
          autoUnclogTimer.stop();
          stallTimer.stop();
          // State will return to FEEDING on next setSpindexerVelocity() call from shooting command
        }
      } else if (state == SpindexerState.FEEDING) {
        boolean stalled =
            spindexerInputs.currentAmps > autoUnclogStallCurrentThreshold.get()
                && Math.abs(spindexerInputs.wheelRPM) < autoUnclogStallVelocityThreshold.get();
        if (stalled) {
          if (!stallTimer.isRunning()) {
            stallTimer.restart();
          }
          if (stallTimer.hasElapsed(autoUnclogStallDurationSec.get())
              && autoUnclogAttempts < (int) autoUnclogMaxAttempts.get()) {
            autoUnclogInProgress = true;
            autoUnclogAttempts++;
            autoUnclogTimer.restart();
            Logger.recordOutput("Spindexer/AutoUnclog/Triggered", true);
          }
        } else {
          stallTimer.stop();
        }
      } else {
        // Not feeding — reset stall detection
        stallTimer.stop();
      }
    }

    // Reset auto-unclog attempt counter when we leave feeding
    if (state != SpindexerState.FEEDING && state != SpindexerState.AUTO_UNCLOGGING) {
      autoUnclogAttempts = 0;
    }

    // When no command owns the spindexer, handle unclog and idle reciprocation directly
    if (getCurrentCommand() == null) {
      if (unclogActive) {
        io.setSpindexerVelocity(-Math.abs(unclogRPM.get()));
        state = SpindexerState.UNCLOGGING;
      } else if (wasUnclogActive) {
        io.stopSpindexer();
        state = SpindexerState.STOPPED;
      } else {
        reciprocate();
      }
    }
    wasUnclogActive = unclogActive;

    // Push tunable changes to IO
    if (LoggedTunableNumber.hasChanged(kP, kI, kD, kV, kS)) {
      io.configureSpindexerPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get());
    }
    if (LoggedTunableNumber.hasChanged(spindexerToleranceRPM)) {
      io.setVelocityTolerance(spindexerToleranceRPM.get());
    }
  }

  // ========== Velocity Control Methods ==========

  /**
   * Run Spindexer motor 1 at a specific velocity using closed-loop control.
   *
   * @param velocityRPM Target velocity in RPM
   */
  public void setSpindexerVelocity(double velocityRPM) {
    if (unclogActive) {
      io.setSpindexerVelocity(-Math.abs(unclogRPM.get()));
      state = SpindexerState.UNCLOGGING;
    } else if (autoUnclogInProgress) {
      io.setSpindexerVelocity(-Math.abs(unclogRPM.get()));
      state = SpindexerState.AUTO_UNCLOGGING;
    } else if (feedingSuppressed) {
      io.setSpindexerVelocity(0.0);
      state = SpindexerState.SUPPRESSED;
    } else {
      io.setSpindexerVelocity(velocityRPM);
      state = SpindexerState.FEEDING;
    }
  }

  /**
   * Run Spindexer motor in reverse at a specific velocity.
   *
   * @param velocityRPM Target velocity magnitude in RPM (will be negated)
   */
  public void reverseSpindexer(double velocityRPM) {
    io.setSpindexerVelocity(-Math.abs(velocityRPM));
    state = SpindexerState.FEEDING;
  }

  /** Stop only Spindexer motor 1. */
  public void stopSpindexer() {
    io.stopSpindexer();
    state = SpindexerState.STOPPED;
  }

  // ========== Status Methods ==========

  /**
   * Get spindexer motor 1 velocity.
   *
   * @return Motor 1 velocity in RPM
   */
  public double getSpindexerWheelVelocity() {
    return spindexerInputs.wheelRPM;
  }

  /**
   * Check if a ball is detected (future sensor).
   *
   * @return True if ball detected
   */
  public boolean isBallDetected() {
    return false; // Future: ball detection sensor
  }

  /**
   * Check if all motors are connected.
   *
   * @return True if all motors are responding
   */
  public boolean isConnected() {
    return spindexerInputs.connected;
  }

  /**
   * Check if spindexer motor 1 is at its velocity setpoint.
   *
   * @return True if motor 1 is at setpoint
   */
  public boolean isSpindexerAtSetpoint() {
    return spindexerInputs.atSetpoint;
  }

  // ========== Feeding Suppression ==========

  /** Suppress feeding — setSpindexerVelocity will send 0 while suppressed. */
  public void suppressFeeding() {
    feedingSuppressed = true;
  }

  /** Resume normal feeding. */
  public void unsuppressFeeding() {
    feedingSuppressed = false;
  }

  /**
   * @return true if feeding is currently suppressed
   */
  public boolean isFeedingSuppressed() {
    return feedingSuppressed;
  }

  // ========== Unclog Override ==========

  /** Activate unclog — setSpindexerVelocity will reverse the spindexer while active. */
  public void activateUnclog() {
    unclogActive = true;
  }

  /** Deactivate unclog — setSpindexerVelocity resumes normal behavior. */
  public void deactivateUnclog() {
    unclogActive = false;
  }

  /**
   * @return true if unclog is currently active
   */
  public boolean isUnclogActive() {
    return unclogActive;
  }

  // ========== Auto-Unclog ==========

  /** Enable auto-unclog stall detection. Off by default — enable from dashboard for testing. */
  public void enableAutoUnclog() {
    autoUnclogEnabled = true;
  }

  /** Disable auto-unclog stall detection. */
  public void disableAutoUnclog() {
    autoUnclogEnabled = false;
    autoUnclogInProgress = false;
    stallTimer.stop();
    autoUnclogTimer.stop();
  }

  /**
   * @return true if auto-unclog is enabled
   */
  public boolean isAutoUnclogEnabled() {
    return autoUnclogEnabled;
  }

  // ========== Reciprocation ==========

  /**
   * Gently jostle fuel by alternating spindexer direction at low RPM. Call this each cycle when not
   * actively feeding — it handles the timer and direction switching internally. Used by shooting
   * commands during the "waiting for systems to spin up" gap, and by periodic() when fully idle.
   */
  public void reciprocate() {
    if (!SmartDashboard.getBoolean("Tuning/Spindexer/Reciprocate/Enabled", true)) {
      // Reciprocation disabled — stop the motor and stay stopped
      if (state == SpindexerState.RECIPROCATING) {
        io.stopSpindexer();
        state = SpindexerState.STOPPED;
      }
      return;
    }
    if (!reciprocateTimer.isRunning()) {
      reciprocateTimer.restart();
    }
    if (reciprocateTimer.hasElapsed(reciprocateIntervalSec.get())) {
      reciprocateForward = !reciprocateForward;
      reciprocateTimer.restart();
    }
    double rpm = reciprocateRPM.get();
    io.setSpindexerVelocity(reciprocateForward ? rpm : -rpm);
    state = SpindexerState.RECIPROCATING;
  }

  // ========== Commands ==========

  /**
   * Command to stop all motors.
   *
   * @return Instant command that stops all motors
   */
  public Command stopSpindexerCommand() {
    return runOnce(this::stopSpindexer).withName("Spindexer: Stop");
  }

  /**
   * Command to run only the lead spindexer (motor 1) at a tunable RPM.
   *
   * @param rpm Tunable RPM source
   * @return Command that runs until interrupted
   */
  public Command runSpindexerCommand(LoggedTunableNumber rpm) {
    return run(() -> setSpindexerVelocity(rpm.get()))
        .finallyDo(this::stopSpindexer)
        .withName("Spindexer: Run Lead");
  }

  /**
   * Command to run the spindexer in reverse at a tunable RPM.
   *
   * @param rpm Tunable RPM source (magnitude; will be negated internally)
   * @return Command that runs until interrupted
   */
  public Command reverseSpindexerCommand(LoggedTunableNumber rpm) {
    return run(() -> reverseSpindexer(rpm.get()))
        .finallyDo(this::stopSpindexer)
        .withName("Spindexer: Reverse");
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    io.setSpindexerVoltage(output);
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = io.getFFCharacterizationVelocity();
    return output;
  }
}
