package frc.robot.subsystems.motivator;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

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

  // Mechanism2d visualization
  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(4, 3);
  // Motor 1 wheel - orange
  private final LoggedMechanismRoot2d motor1Root = mechanism.getRoot("motor1", 0.75, 1.5);
  private final LoggedMechanismLigament2d motor1Wheel;
  // Motor 2 wheel - yellow
  private final LoggedMechanismRoot2d motor2Root = mechanism.getRoot("motor2", 1.75, 1.5);
  private final LoggedMechanismLigament2d motor2Wheel;
  // Prefeed wheel - cyan
  private final LoggedMechanismRoot2d prefeedRoot = mechanism.getRoot("prefeed", 3.0, 1.5);
  private final LoggedMechanismLigament2d prefeedWheel;
  // Track cumulative angles for smooth rotation
  private double motor1Angle = 0.0;
  private double motor2Angle = 0.0;
  private double prefeedAngle = 0.0;

  public Motivator(MotivatorIO io) {
    this.io = io;

    // Create motor 1 wheel visualization (orange)
    motor1Wheel =
        motor1Root.append(
            new LoggedMechanismLigament2d("motor1Wheel", 0.35, 0, 8, new Color8Bit(Color.kOrange)));

    // Create motor 2 wheel visualization (yellow)
    motor2Wheel =
        motor2Root.append(
            new LoggedMechanismLigament2d("motor2Wheel", 0.35, 0, 8, new Color8Bit(Color.kYellow)));

    // Create prefeed wheel visualization (cyan)
    prefeedWheel =
        prefeedRoot.append(
            new LoggedMechanismLigament2d("prefeedWheel", 0.25, 0, 6, new Color8Bit(Color.kCyan)));
  }

  @Override
  public void periodic() {
    io.updateInputs(motor1Inputs, motor2Inputs, prefeedInputs);
    Logger.processInputs("Motivator/Motivator1", motor1Inputs);
    Logger.processInputs("Motivator/Motivator2", motor2Inputs);
    Logger.processInputs("Motivator/PreFeed", prefeedInputs);

    // Update mechanism visualization based on motor velocities
    // Convert RPM to degrees per cycle (assuming 50Hz loop = 0.02s per cycle)
    double motor1DegreesPerCycle = motor1Inputs.velocityRPM / 60.0 * 360.0 * 0.02;
    double motor2DegreesPerCycle = motor2Inputs.velocityRPM / 60.0 * 360.0 * 0.02;
    double prefeedDegreesPerCycle = prefeedInputs.velocityRPM / 60.0 * 360.0 * 0.02;

    motor1Angle += motor1DegreesPerCycle;
    motor2Angle += motor2DegreesPerCycle;
    prefeedAngle += prefeedDegreesPerCycle;

    // Keep angles in reasonable range to avoid overflow
    motor1Angle = motor1Angle % 360.0;
    motor2Angle = motor2Angle % 360.0;
    prefeedAngle = prefeedAngle % 360.0;

    motor1Wheel.setAngle(motor1Angle);
    motor2Wheel.setAngle(motor2Angle);
    prefeedWheel.setAngle(prefeedAngle);

    // Update wheel colors based on motor status
    motor1Wheel.setColor(
        getStatusColor(
            motor1Inputs.velocityRPM, motor1Inputs.targetVelocityRPM, motor1Inputs.atSetpoint));
    motor2Wheel.setColor(
        getStatusColor(
            motor2Inputs.velocityRPM, motor2Inputs.targetVelocityRPM, motor2Inputs.atSetpoint));
    prefeedWheel.setColor(
        getStatusColor(
            prefeedInputs.velocityRPM, prefeedInputs.targetVelocityRPM, prefeedInputs.atSetpoint));

    // Log the mechanism visualization
    Logger.recordOutput("Motivator/Mechanism2d", mechanism);
  }

  /**
   * Get the color for a wheel status indicator based on operational state.
   *
   * @param velocityRPM Current velocity in RPM
   * @param targetVelocityRPM Target velocity in RPM
   * @param atSetpoint Whether the motor is at setpoint
   * @return Color8Bit: Red=unpowered, Yellow=spinning up, Green=at setpoint
   */
  private Color8Bit getStatusColor(
      double velocityRPM, double targetVelocityRPM, boolean atSetpoint) {
    final double VELOCITY_THRESHOLD = 50.0; // Avoid flickering near zero

    if (Math.abs(velocityRPM) < VELOCITY_THRESHOLD
        && Math.abs(targetVelocityRPM) < VELOCITY_THRESHOLD) {
      return new Color8Bit(Color.kRed); // Unpowered
    } else if (atSetpoint) {
      return new Color8Bit(Color.kGreen); // At setpoint
    } else {
      return new Color8Bit(Color.kYellow); // Powered, spinning up
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
}
