package frc.robot.subsystems.motivator;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
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
  private final MotivatorIOInputsAutoLogged inputs = new MotivatorIOInputsAutoLogged();

  // Tunable duty cycles for different operations
  private static final LoggedTunableNumber feedDutyCycle =
      new LoggedTunableNumber("Motivator/FeedDutyCycle", 0.8);
  private static final LoggedTunableNumber prefeedDutyCycle =
      new LoggedTunableNumber("Motivator/PrefeedDutyCycle", 0.6);
  private static final LoggedTunableNumber ejectDutyCycle =
      new LoggedTunableNumber("Motivator/EjectDutyCycle", -0.5);

  // Velocity threshold for "at speed" check (RPM)
  private static final LoggedTunableNumber atSpeedThresholdRPM =
      new LoggedTunableNumber("Motivator/AtSpeedThresholdRPM", 3000.0);

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
    io.updateInputs(inputs);
    Logger.processInputs("Motivator", inputs);

    // Update mechanism visualization based on motor velocities
    // Convert RPM to degrees per cycle (assuming 50Hz loop = 0.02s per cycle)
    double motor1DegreesPerCycle = inputs.motivator1VelocityRPM / 60.0 * 360.0 * 0.02;
    double motor2DegreesPerCycle = inputs.motivator2VelocityRPM / 60.0 * 360.0 * 0.02;
    double prefeedDegreesPerCycle = inputs.prefeedVelocityRPM / 60.0 * 360.0 * 0.02;

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

    // Log the mechanism visualization
    Logger.recordOutput("Motivator/Mechanism2d", mechanism);
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

  /** Run both motivator motors at the default feed duty cycle. */
  public void feed() {
    io.setMotivator1DutyCycle(feedDutyCycle.get());
    io.setMotivator2DutyCycle(feedDutyCycle.get());
  }

  /** Run the prefeed roller at the default prefeed duty cycle. */
  public void prefeed() {
    io.setPrefeedDutyCycle(prefeedDutyCycle.get());
  }

  /** Run all three motors at their default duty cycles. */
  public void feedAndPrefeed() {
    io.setMotivator1DutyCycle(feedDutyCycle.get());
    io.setMotivator2DutyCycle(feedDutyCycle.get());
    io.setPrefeedDutyCycle(prefeedDutyCycle.get());
  }

  /** Run all motors in reverse to eject balls. */
  public void eject() {
    io.setMotivator1DutyCycle(ejectDutyCycle.get());
    io.setMotivator2DutyCycle(ejectDutyCycle.get());
    io.setPrefeedDutyCycle(ejectDutyCycle.get());
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
    return inputs.motivator1VelocityRPM;
  }

  /**
   * Get motivator motor 2 velocity.
   *
   * @return Motor 2 velocity in RPM
   */
  public double getMotivator2Velocity() {
    return inputs.motivator2VelocityRPM;
  }

  /**
   * Get the prefeed roller velocity.
   *
   * @return Prefeed motor velocity in RPM
   */
  public double getPrefeedVelocity() {
    return inputs.prefeedVelocityRPM;
  }

  /**
   * Check if a ball is detected (future sensor).
   *
   * @return True if ball detected
   */
  public boolean isBallDetected() {
    return inputs.ballDetected;
  }

  /**
   * Check if all motors are connected.
   *
   * @return True if all motors are responding
   */
  public boolean isConnected() {
    return inputs.motivator1Connected && inputs.motivator2Connected && inputs.prefeedConnected;
  }

  /**
   * Check if both motivator motors are up to speed (threshold-based check).
   *
   * @return True if both motor velocities exceed the threshold
   */
  public boolean isAtSpeed() {
    return Math.abs(inputs.motivator1VelocityRPM) >= atSpeedThresholdRPM.get()
        && Math.abs(inputs.motivator2VelocityRPM) >= atSpeedThresholdRPM.get();
  }

  /**
   * Check if motivator motor 1 is at its velocity setpoint.
   *
   * @return True if motor 1 is at setpoint
   */
  public boolean isMotivator1AtSetpoint() {
    return inputs.motivator1AtSetpoint;
  }

  /**
   * Check if motivator motor 2 is at its velocity setpoint.
   *
   * @return True if motor 2 is at setpoint
   */
  public boolean isMotivator2AtSetpoint() {
    return inputs.motivator2AtSetpoint;
  }

  /**
   * Check if both motivator motors are at their velocity setpoints.
   *
   * @return True if both motors are at setpoint
   */
  public boolean areMotivatorsAtSetpoint() {
    return inputs.motivator1AtSetpoint && inputs.motivator2AtSetpoint;
  }

  /**
   * Check if the prefeed is at its velocity setpoint.
   *
   * @return True if prefeed is at setpoint
   */
  public boolean isPrefeedAtSetpoint() {
    return inputs.prefeedAtSetpoint;
  }

  // ========== Commands ==========

  /**
   * Command to run both motivator motors while held.
   *
   * @return Command that feeds balls until interrupted
   */
  public Command feedCommand() {
    return run(this::feed).finallyDo(this::stopMotivators).withName("Motivator: Feed");
  }

  /**
   * Command to run the prefeed roller while held.
   *
   * @return Command that prefeeds balls until interrupted
   */
  public Command prefeedCommand() {
    return run(this::prefeed).finallyDo(this::stopPrefeed).withName("Motivator: Prefeed");
  }

  /**
   * Command to run all motors while held.
   *
   * @return Command that feeds and prefeeds until interrupted
   */
  public Command feedAndPrefeedCommand() {
    return run(this::feedAndPrefeed).finallyDo(this::stop).withName("Motivator: Feed+Prefeed");
  }

  /**
   * Command to eject balls (reverse) while held.
   *
   * @return Command that ejects balls until interrupted
   */
  public Command ejectCommand() {
    return run(this::eject).finallyDo(this::stop).withName("Motivator: Eject");
  }

  /**
   * Command to stop all motors.
   *
   * @return Instant command that stops all motors
   */
  public Command stopCommand() {
    return runOnce(this::stop).withName("Motivator: Stop");
  }
}
