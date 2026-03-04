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

  // Tunable ready-gate tolerance for atSetpoint() — does NOT affect motor control
  private static final LoggedTunableNumber motivatorToleranceRPM =
      new LoggedTunableNumber("Tuning/Motivator/ReadyToleranceRPM", 100.0);

  public Motivator(MotivatorIO io) {
    this.io = io;

    // Push initial velocity tolerances to IO
    io.setVelocityTolerance(motivatorToleranceRPM.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(motor1Inputs);
    Logger.processInputs("Motivator", motor1Inputs);

    // Push tunable changes to IO
    if (LoggedTunableNumber.hasChanged(kP, kI, kD, kV, kS)) {
      io.configureMotivatorPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get());
    }
    if (LoggedTunableNumber.hasChanged(motivatorToleranceRPM)) {
      io.setVelocityTolerance(motivatorToleranceRPM.get());
    }
  }

  // ========== Velocity Control Methods ==========

  /**
   * Run motivator motor 1 at a specific velocity using closed-loop control.
   *
   * @param velocityRPM Target velocity in RPM
   */
  public void setMotivatorVelocity(double velocityRPM) {
    io.setMotivatorVelocity(velocityRPM);
  }

  /** Stop only motivator motor 1. */
  public void stopMotivator() {
    io.stopMotivator();
  }

  // ========== Status Methods ==========

  /**
   * Get motivator motor 1 velocity.
   *
   * @return Motor 1 velocity in RPM
   */
  public double getMotivatorWheelVelocity() {
    return motor1Inputs.wheelRPM;
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
  public boolean isMotivatorAtSetpoint() {
    return motor1Inputs.atSetpoint;
  }

  // ========== Commands ==========

  /**
   * Command to stop all motors.
   *
   * @return Instant command that stops all motors
   */
  public Command stopMotivatorCommand() {
    return runOnce(this::stopMotivator).withName("Motivator: Stop");
  }

  /**
   * Command to run only the lead motivator (motor 1) at a tunable RPM.
   *
   * @param rpm Tunable RPM source
   * @return Command that runs until interrupted
   */
  public Command runMotivatorCommand(LoggedTunableNumber rpm) {
    return run(() -> setMotivatorVelocity(rpm.get()))
        .finallyDo(this::stopMotivator)
        .withName("Motivator: Run Lead");
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    io.setMotivatorVoltage(output);
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = io.getFFCharacterizationVelocity();
    return output;
  }
}
