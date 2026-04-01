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
    JAMMED, // Jam detected but auto-unclog disabled — logged only, no motor action
    RECIPROCATING // Gentle back-and-forth jostle to keep fuel loose
  }

  private SpindexerState state = SpindexerState.STOPPED;
  private boolean stallDetected = false;

  // When true, setSpindexerVelocity() sends 0 instead of the requested RPM.
  // Used by the driver to temporarily suppress feeding without interrupting shooting commands.
  private boolean feedingSuppressed = false;

  // When true, setSpindexerVelocity() reverses the spindexer at unclogRPM instead of the
  // requested RPM. Used to unclog without interrupting shooting commands — on release the
  // shooting command resumes feeding instantly.
  private boolean unclogActive = false;
  private boolean wasUnclogActive = false;

  private static final LoggedTunableNumber unclogRPM;

  // Auto-unclog — detects stall during FEEDING and briefly reverses to clear the jam.
  // Enabled by default; can be toggled from dashboard if needed.
  private boolean autoUnclogEnabled = true;
  private boolean autoUnclogInProgress = false;
  private final Timer stallTimer = new Timer(); // How long stall condition has persisted
  private final Timer autoUnclogTimer = new Timer(); // How long the reverse burst has been running
  private int autoUnclogAttempts = 0; // Attempts this feeding session

  private static final LoggedTunableNumber autoUnclogStallCurrentThreshold;
  private static final LoggedTunableNumber autoUnclogStallVelocityThreshold;
  private static final LoggedTunableNumber autoUnclogStallDurationSec;
  private static final LoggedTunableNumber autoUnclogReverseDurationSec;
  private static final LoggedTunableNumber autoUnclogMaxAttempts;

  // Reciprocation — gentle back-and-forth jostle to keep fuel loose when not actively feeding.
  // Call reciprocate() each cycle to jostle; it alternates direction on a timer.
  private boolean reciprocateForward = true;
  private final Timer reciprocateTimer = new Timer();

  // Reverse kick — brief reverse pulse after reciprocation ends to push any partially-fed ball
  // back.
  private boolean reverseKickActive = false;
  private final Timer reverseKickTimer = new Timer();

  private static final LoggedTunableNumber reciprocateRPM;
  private static final LoggedTunableNumber reciprocateIntervalSec;

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
    unclogRPM =
        new LoggedTunableNumber("Tuning/Spindexer/UnclogRPM", config.getSpindexerUnclogRPM());
    autoUnclogStallCurrentThreshold =
        new LoggedTunableNumber(
            "Tuning/Spindexer/AutoUnclog/StallCurrentAmps",
            config.getSpindexerAutoUnclogStallCurrentAmps());
    autoUnclogStallVelocityThreshold =
        new LoggedTunableNumber(
            "Tuning/Spindexer/AutoUnclog/StallRPMError",
            config.getSpindexerAutoUnclogStallRPMError());
    autoUnclogStallDurationSec =
        new LoggedTunableNumber(
            "Tuning/Spindexer/AutoUnclog/StallDurationSec",
            config.getSpindexerAutoUnclogStallDurationSec());
    autoUnclogReverseDurationSec =
        new LoggedTunableNumber(
            "Tuning/Spindexer/AutoUnclog/ReverseDurationSec",
            config.getSpindexerAutoUnclogReverseDurationSec());
    autoUnclogMaxAttempts =
        new LoggedTunableNumber(
            "Tuning/Spindexer/AutoUnclog/MaxAttempts", config.getSpindexerAutoUnclogMaxAttempts());
    reciprocateRPM =
        new LoggedTunableNumber(
            "Tuning/Spindexer/Reciprocate/RPM", config.getSpindexerReciprocateRPM());
    reciprocateIntervalSec =
        new LoggedTunableNumber(
            "Tuning/Spindexer/Reciprocate/IntervalSec",
            config.getSpindexerReciprocateIntervalSec());
    spindexerToleranceRPM =
        new LoggedTunableNumber(
            "Tuning/Spindexer/ReadyToleranceRPM", config.getSpindexerReadyToleranceRPM());
  }

  // Tunable ready-gate tolerance for atSetpoint() — does NOT affect motor control
  private static final LoggedTunableNumber spindexerToleranceRPM;

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

    Logger.recordOutput("Subsystems/SpindexerState", state.name());

    // Stall detection: always runs during FEEDING so we can see jam events in logs.
    // Only triggers the reverse burst when autoUnclogEnabled.
    if (!unclogActive) {
      if (autoUnclogInProgress) {
        // Reverse burst in progress — check if duration has elapsed
        if (autoUnclogTimer.hasElapsed(autoUnclogReverseDurationSec.get())) {
          autoUnclogInProgress = false;
          stallDetected = false;
          autoUnclogTimer.stop();
          stallTimer.stop();
          // State will return to FEEDING on next setSpindexerVelocity() call from shooting command
        }
      } else if (state == SpindexerState.FEEDING || state == SpindexerState.JAMMED) {
        double rpmError = Math.abs(spindexerInputs.targetRPM) - Math.abs(spindexerInputs.wheelRPM);
        boolean stalled =
            Math.abs(spindexerInputs.wheelRPM) > 100.0
                && spindexerInputs.currentAmps > autoUnclogStallCurrentThreshold.get()
                && rpmError > autoUnclogStallVelocityThreshold.get();
        if (stalled) {
          if (!stallTimer.isRunning()) {
            stallTimer.restart();
          }
          if (stallTimer.hasElapsed(autoUnclogStallDurationSec.get())
              && autoUnclogAttempts < (int) autoUnclogMaxAttempts.get()) {
            stallDetected = true;
            if (autoUnclogEnabled) {
              autoUnclogInProgress = true;
              autoUnclogAttempts++;
              autoUnclogTimer.restart();
            } else {
              state = SpindexerState.JAMMED;
            }
          }
        } else {
          stallTimer.stop();
          stallDetected = false;
        }
      } else {
        // Not feeding — reset stall detection
        stallTimer.stop();
        stallDetected = false;
      }
    }

    // Reset auto-unclog attempt counter when we leave feeding
    if (state != SpindexerState.FEEDING
        && state != SpindexerState.AUTO_UNCLOGGING
        && state != SpindexerState.JAMMED) {
      autoUnclogAttempts = 0;
    }

    // When no command owns the spindexer, handle unclog and reverse kick
    if (getCurrentCommand() == null) {
      if (unclogActive) {
        io.setSpindexerVelocity(-Math.abs(unclogRPM.get()));
        state = SpindexerState.UNCLOGGING;
      } else if (wasUnclogActive) {
        io.stopSpindexer();
        state = SpindexerState.STOPPED;
      } else if (reverseKickActive) {
        // Reverse kick in progress — stop when one interval elapses
        if (reverseKickTimer.hasElapsed(reciprocateIntervalSec.get())) {
          reverseKickActive = false;
          reverseKickTimer.stop();
          io.stopSpindexer();
          state = SpindexerState.STOPPED;
        }
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
      state = stallDetected ? SpindexerState.JAMMED : SpindexerState.FEEDING;
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
   * Get spindexer target velocity.
   *
   * @return Target velocity in RPM
   */
  public double getSpindexerTargetRPM() {
    return spindexerInputs.targetRPM;
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
   * actively feeding — it handles the timer and direction switching internally.
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
    io.setSpindexerVelocity(reciprocateForward ? rpm : -rpm + -100);
    state = SpindexerState.RECIPROCATING;
  }

  /**
   * Stop reciprocation with a reverse kick — one interval in the opposite direction to push any
   * partially-fed ball back. The kick runs autonomously in periodic() and stops itself.
   */
  public void stopReciprocateWithKick() {
    double rpm = reciprocateRPM.get();
    io.setSpindexerVelocity(reciprocateForward ? -rpm : rpm);
    reverseKickActive = true;
    reverseKickTimer.restart();
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
   * Command that reciprocates the spindexer while running. On cancel, starts a reverse kick — one
   * interval in the opposite direction to push any partially-fed ball back. The kick runs
   * autonomously in periodic() and stops itself after one reciprocation interval.
   *
   * @return Command that runs until interrupted
   */
  public Command reciprocateCommand() {
    return run(this::reciprocate)
        .finallyDo(this::stopReciprocateWithKick)
        .withName("Spindexer: Reciprocate");
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
