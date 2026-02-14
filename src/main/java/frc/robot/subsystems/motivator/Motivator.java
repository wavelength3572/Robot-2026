package frc.robot.subsystems.motivator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Motivator subsystem for feeding balls to the launcher. Controls three independent motors:
 *
 * <ul>
 *   <li>Motor 1 (CAN 55): First motivator wheel
 * </ul>
 */
public class Motivator extends SubsystemBase {
  private final MotivatorIO io;
  private final MotorInputsAutoLogged motor1Inputs = new MotorInputsAutoLogged();

  // Tunable PID gains
  private static final LoggedTunableNumber kP;
  private static final LoggedTunableNumber kI;
  private static final LoggedTunableNumber kD;
  private static final LoggedTunableNumber kS;
  private static final LoggedTunableNumber kV;

  static {
    RobotConfig config = Constants.getRobotConfig();
    kP = new LoggedTunableNumber("Tuning/Motivator/kP", config.getMotivatorKp());
    kI = new LoggedTunableNumber("Tuning/Motivator/kI", config.getMotivatorKi());
    kD = new LoggedTunableNumber("Tuning/Motivator/kD", config.getMotivatorKd());
    kS = new LoggedTunableNumber("Tuning/Motivator/kS", config.getMotivatorKs());
    kV = new LoggedTunableNumber("Tuning/Motivator/kV", config.getMotivatorKv());
  }

  // Tunable velocity tolerances
  private static final LoggedTunableNumber motivatorToleranceRPM =
      new LoggedTunableNumber("Tuning/Motivator/ToleranceRPM", 100.0);

  public Motivator(MotivatorIO io) {
    this.io = io;

    // Push initial velocity tolerances to IO
    io.setVelocityTolerances(motivatorToleranceRPM.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(motor1Inputs);
    Logger.processInputs("Motivator/Motor1", motor1Inputs);

    // Push tunable changes to IO
    if (LoggedTunableNumber.hasChanged(kP, kI, kD, kV, kS)) {
      io.configureMotivator1PID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get());
    }
  }

  // ========== Duty Cycle Control Methods ==========

  /**
   * Run motivator motor 1 at a specific duty cycle.
   *
   * @param dutyCycle Duty cycle from -1.0 to 1.0
   */
  public void setMotivator1DutyCycle(double dutyCycle) {
    io.setMotivator1DutyCycle(dutyCycle);
  }

  // ========== Velocity Control Methods ==========

  /**
   * Run motivator motor 1 at a specific velocity using closed-loop control.
   *
   * @param velocityRPM Target velocity in RPM
   */
  public void setMotivator1Velocity(double velocityRPM) {
    io.setMotivator1Velocity(velocityRPM);
  }

  /**
   * Run both motivator motors at the same velocity.
   *
   * @param velocityRPM Target velocity in RPM for both motors
   */
  public void setMotivatorsVelocity(double velocityRPM) {
    io.setMotivator1Velocity(velocityRPM);
  }

  /**
   * Run all three motors at specific velocities using closed-loop control.
   *
   * @param motivatorRPM Target velocity for both motivator motors in RPM
   */
  public void setVelocities(double motivatorRPM) {
    io.setMotivator1Velocity(motivatorRPM);
  }

  // ========== Stop Methods ==========

  /** Stop all motors. */
  public void stop() {
    io.stop();
  }

  /** Stop only motivator motor 1. */
  public void stopMotivator1() {
    io.stopMotivator1();
  }

  /** Stop both motivator motors */
  public void stopMotivators() {
    io.stopMotivators();
  }

  // ========== Status Methods ==========

  /**
   * Get motivator motor 1 velocity.
   *
   * @return Motor 1 velocity in RPM
   */
  public double getMotivator1Velocity() {
    return motor1Inputs.velocityRPM;
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
    return motor1Inputs.connected;
  }

  /**
   * Check if motivator motor 1 is at its velocity setpoint.
   *
   * @return True if motor 1 is at setpoint
   */
  public boolean isMotivator1AtSetpoint() {
    return motor1Inputs.atSetpoint;
  }

  /**
   * Check if both motivator motors are at their velocity setpoints.
   *
   * @return True if both motors are at setpoint
   */
  public boolean areMotivatorsAtSetpoint() {
    return motor1Inputs.atSetpoint;
  }

  // ========== Commands ==========

  /**
   * Command to stop all motors.
   *
   * @return Instant command that stops all motors
   */
  public Command stopCommand() {
    return runOnce(this::stop).withName("Motivator: Stop");
  }

  /**
   * Command to run only the lead motivator (motor 1) at a tunable RPM.
   *
   * @param rpm Tunable RPM source
   * @return Command that runs until interrupted
   */
  public Command runLeadMotivatorCommand(LoggedTunableNumber rpm) {
    return run(() -> setMotivator1Velocity(rpm.get()))
        .finallyDo(this::stopMotivator1)
        .withName("Motivator: Run Lead");
  }

  /**
   * Command to run both motivator motors at a tunable RPM.
   *
   * @param rpm Tunable RPM source
   * @return Command that runs until interrupted
   */
  public Command runBothMotivatorsCommand(LoggedTunableNumber rpm) {
    return run(() -> setMotivatorsVelocity(rpm.get()))
        .finallyDo(this::stopMotivators)
        .withName("Motivator: Run Both");
  }
}
