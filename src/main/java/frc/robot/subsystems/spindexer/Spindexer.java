package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
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

  // Agitation tunables — back-and-forth oscillation to shake balls loose
  private static final LoggedTunableNumber agitateForwardRPM =
      new LoggedTunableNumber("Tuning/Spindexer/AgitateForwardRPM", 400.0);
  private static final LoggedTunableNumber agitateReverseRPM =
      new LoggedTunableNumber("Tuning/Spindexer/AgitateReverseRPM", 300.0);
  private static final LoggedTunableNumber agitateForwardTimeSec =
      new LoggedTunableNumber("Tuning/Spindexer/AgitateForwardTimeSec", 0.3);
  private static final LoggedTunableNumber agitateReverseTimeSec =
      new LoggedTunableNumber("Tuning/Spindexer/AgitateReverseTimeSec", 0.2);

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
  }

  @Override
  public void periodic() {
    io.updateInputs(spindexerInputs);
    Logger.processInputs("Spindexer", spindexerInputs);
    Logger.recordOutput("Spindexer/FeedingSuppressed", feedingSuppressed);
    Logger.recordOutput("Spindexer/UnclogActive", unclogActive);

    // When no shooting command is running, drive unclog directly from periodic
    if (getCurrentCommand() == null) {
      if (unclogActive) {
        io.setSpindexerVelocity(-Math.abs(unclogRPM.get()));
      } else if (wasUnclogActive) {
        io.stopSpindexer();
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
    } else if (feedingSuppressed) {
      io.setSpindexerVelocity(0.0);
    } else {
      io.setSpindexerVelocity(velocityRPM);
    }
  }

  /**
   * Run Spindexer motor in reverse at a specific velocity.
   *
   * @param velocityRPM Target velocity magnitude in RPM (will be negated)
   */
  public void reverseSpindexer(double velocityRPM) {
    io.setSpindexerVelocity(-Math.abs(velocityRPM));
  }

  /** Stop only Spindexer motor 1. */
  public void stopSpindexer() {
    io.stopSpindexer();
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

  // ========== Commands ==========

  /**
   * Command that agitates the spindexer back and forth to shake balls loose. Alternates between
   * forward and reverse at tunable RPM/duration. Useful during intake to prevent jamming.
   *
   * @return Command that oscillates the spindexer until cancelled
   */
  public Command agitateCommand() {
    return Commands.sequence(
            runOnce(() -> setSpindexerVelocity(agitateForwardRPM.get())),
            Commands.waitSeconds(agitateForwardTimeSec.get()),
            runOnce(() -> reverseSpindexer(agitateReverseRPM.get())),
            Commands.waitSeconds(agitateReverseTimeSec.get()))
        .repeatedly()
        .finallyDo(this::stopSpindexer)
        .withName("Spindexer: Agitate");
  }

  /**
   * Command that agitates with a net-forward bias by running forward at the supplied feeding RPM
   * and periodically reversing briefly. The forward RPM is read from the supplier each cycle so
   * dashboard tunables or launcher-ratio values take effect live.
   *
   * @param forwardRPM Supplier for the forward (feeding) RPM
   * @return Command that agitates with forward bias until cancelled
   */
  public Command agitateWithFeedCommand(DoubleSupplier forwardRPM) {
    return Commands.sequence(
            run(() -> setSpindexerVelocity(forwardRPM.getAsDouble()))
                .withTimeout(agitateForwardTimeSec.get()),
            runOnce(() -> reverseSpindexer(agitateReverseRPM.get())),
            Commands.waitSeconds(agitateReverseTimeSec.get()))
        .repeatedly()
        .finallyDo(this::stopSpindexer)
        .withName("Spindexer: Agitate+Feed");
  }

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
