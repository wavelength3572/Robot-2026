package frc.robot.subsystems.spindexer;

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

  // When true, setSpindexerVelocity() sends 0 instead of the requested RPM.
  // Used by the driver to temporarily suppress feeding without interrupting shooting commands.
  private boolean feedingSuppressed = false;

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
    io.setSpindexerVelocity(feedingSuppressed ? 0.0 : velocityRPM);
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
