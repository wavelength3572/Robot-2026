package frc.robot.subsystems.motivator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Motivator subsystem for feeding balls to the launcher. Controls three independent motors:
 *
 * <ul>
 *   <li>Motor 1 (CAN 55): First motivator wheel
 *   <li>Motor 2 (CAN 56): Second motivator wheel
 *   <li>Prefeed (CAN 57): Prefeed roller for ball staging
 * </ul>
 */
public class Motivator extends SubsystemBase {
  private final MotivatorIO io;
  private final MotorInputsAutoLogged motor1Inputs = new MotorInputsAutoLogged();
  private final MotorInputsAutoLogged motor2Inputs = new MotorInputsAutoLogged();
  private final MotorInputsAutoLogged prefeedInputs = new MotorInputsAutoLogged();

  // Tunable PID gains for motivator 1
  private static final LoggedTunableNumber m1kP =
      new LoggedTunableNumber("Tuning/Motivator/Motivator1/kP", 0.000056);
  private static final LoggedTunableNumber m1kI =
      new LoggedTunableNumber("Tuning/Motivator/Motivator1/kI", 0.0);
  private static final LoggedTunableNumber m1kD =
      new LoggedTunableNumber("Tuning/Motivator/Motivator1/kD", 0.00275);
  private static final LoggedTunableNumber m1kFF =
      new LoggedTunableNumber("Tuning/Motivator/Motivator1/kFF", 0.00015);

  // Tunable PID gains for motivator 2
  private static final LoggedTunableNumber m2kP =
      new LoggedTunableNumber("Tuning/Motivator/Motivator2/kP", 0.000056);
  private static final LoggedTunableNumber m2kI =
      new LoggedTunableNumber("Tuning/Motivator/Motivator2/kI", 0.0);
  private static final LoggedTunableNumber m2kD =
      new LoggedTunableNumber("Tuning/Motivator/Motivator2/kD", 0.00275);
  private static final LoggedTunableNumber m2kFF =
      new LoggedTunableNumber("Tuning/Motivator/Motivator2/kFF", 0.0001526);

  // Tunable PID gains for prefeed
  private static final LoggedTunableNumber pfkP =
      new LoggedTunableNumber("Tuning/Motivator/PreFeed/kP", 0.000101);
  private static final LoggedTunableNumber pfkI =
      new LoggedTunableNumber("Tuning/Motivator/PreFeed/kI", 0.0);
  private static final LoggedTunableNumber pfkD =
      new LoggedTunableNumber("Tuning/Motivator/PreFeed/kD", 0.005);
  private static final LoggedTunableNumber pfkFF =
      new LoggedTunableNumber("Tuning/Motivator/PreFeed/kFF", 0.00015);

  // Tunable velocity tolerances
  private static final LoggedTunableNumber motivatorToleranceRPM =
      new LoggedTunableNumber("Tuning/Motivator/Motivator1/ToleranceRPM", 100.0);
  private static final LoggedTunableNumber prefeedToleranceRPM =
      new LoggedTunableNumber("Tuning/Motivator/PreFeed/ToleranceRPM", 100.0);

  public Motivator(MotivatorIO io) {
    this.io = io;

    // Push initial velocity tolerances to IO
    io.setVelocityTolerances(motivatorToleranceRPM.get(), prefeedToleranceRPM.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(motor1Inputs, motor2Inputs, prefeedInputs);
    Logger.processInputs("Motivator/Motor1", motor1Inputs);
    Logger.processInputs("Motivator/Motor2", motor2Inputs);
    Logger.processInputs("Motivator/PreFeed", prefeedInputs);

    // Push tunable changes to IO
    if (LoggedTunableNumber.hasChanged(m1kP, m1kI, m1kD, m1kFF)) {
      io.configureMotivator1PID(m1kP.get(), m1kI.get(), m1kD.get(), m1kFF.get());
    }
    if (LoggedTunableNumber.hasChanged(m2kP, m2kI, m2kD, m2kFF)) {
      io.configureMotivator2PID(m2kP.get(), m2kI.get(), m2kD.get(), m2kFF.get());
    }
    if (LoggedTunableNumber.hasChanged(pfkP, pfkI, pfkD, pfkFF)) {
      io.configurePrefeedPID(pfkP.get(), pfkI.get(), pfkD.get(), pfkFF.get());
    }
    if (LoggedTunableNumber.hasChanged(motivatorToleranceRPM, prefeedToleranceRPM)) {
      io.setVelocityTolerances(motivatorToleranceRPM.get(), prefeedToleranceRPM.get());
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

  /**
   * Run motivator motor 2 at a specific duty cycle.
   *
   * @param dutyCycle Duty cycle from -1.0 to 1.0
   */
  public void setMotivator2DutyCycle(double dutyCycle) {
    io.setMotivator2DutyCycle(dutyCycle);
  }

  /**
   * Run the prefeed roller at a specific duty cycle.
   *
   * @param dutyCycle Duty cycle from -1.0 to 1.0
   */
  public void setPrefeedDutyCycle(double dutyCycle) {
    io.setPrefeedDutyCycle(dutyCycle);
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
   * Run motivator motor 2 at a specific velocity using closed-loop control.
   *
   * @param velocityRPM Target velocity in RPM
   */
  public void setMotivator2Velocity(double velocityRPM) {
    io.setMotivator2Velocity(velocityRPM);
  }

  /**
   * Run the prefeed roller at a specific velocity using closed-loop control.
   *
   * @param velocityRPM Target velocity in RPM
   */
  public void setPrefeedVelocity(double velocityRPM) {
    io.setPrefeedVelocity(velocityRPM);
  }

  /**
   * Run both motivator motors at the same velocity.
   *
   * @param velocityRPM Target velocity in RPM for both motors
   */
  public void setMotivatorsVelocity(double velocityRPM) {
    io.setMotivator1Velocity(velocityRPM);
    io.setMotivator2Velocity(velocityRPM);
  }

  /**
   * Run all three motors at specific velocities using closed-loop control.
   *
   * @param motivatorRPM Target velocity for both motivator motors in RPM
   * @param prefeedRPM Target velocity for prefeed in RPM
   */
  public void setVelocities(double motivatorRPM, double prefeedRPM) {
    io.setMotivator1Velocity(motivatorRPM);
    io.setMotivator2Velocity(motivatorRPM);
    io.setPrefeedVelocity(prefeedRPM);
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

  /** Stop only motivator motor 2. */
  public void stopMotivator2() {
    io.stopMotivator2();
  }

  /** Stop both motivator motors (but not prefeed). */
  public void stopMotivators() {
    io.stopMotivators();
  }

  /** Stop only the prefeed roller. */
  public void stopPrefeed() {
    io.stopPrefeed();
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
   * Get motivator motor 2 velocity.
   *
   * @return Motor 2 velocity in RPM
   */
  public double getMotivator2Velocity() {
    return motor2Inputs.velocityRPM;
  }

  /**
   * Get the prefeed roller velocity.
   *
   * @return Prefeed motor velocity in RPM
   */
  public double getPrefeedVelocity() {
    return prefeedInputs.velocityRPM;
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
    return motor1Inputs.connected && motor2Inputs.connected && prefeedInputs.connected;
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
   * Check if motivator motor 2 is at its velocity setpoint.
   *
   * @return True if motor 2 is at setpoint
   */
  public boolean isMotivator2AtSetpoint() {
    return motor2Inputs.atSetpoint;
  }

  /**
   * Check if both motivator motors are at their velocity setpoints.
   *
   * @return True if both motors are at setpoint
   */
  public boolean areMotivatorsAtSetpoint() {
    return motor1Inputs.atSetpoint && motor2Inputs.atSetpoint;
  }

  /**
   * Check if the prefeed is at its velocity setpoint.
   *
   * @return True if prefeed is at setpoint
   */
  public boolean isPrefeedAtSetpoint() {
    return prefeedInputs.atSetpoint;
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

  /**
   * Command to run only the prefeed at a tunable RPM.
   *
   * @param rpm Tunable RPM source
   * @return Command that runs until interrupted
   */
  public Command runPrefeedCommand(LoggedTunableNumber rpm) {
    return run(() -> setPrefeedVelocity(rpm.get()))
        .finallyDo(this::stopPrefeed)
        .withName("Motivator: Run Prefeed");
  }
}
