package frc.robot.subsystems.motivator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation implementation of MotivatorIO using WPILib's FlywheelSim. All three motors are
 * simulated independently with velocity control support.
 *
 * <p>Note: Velocity tolerance is defined in MotivatorIOSparkFlex.java as the tunable source of
 * truth. This sim uses a matching constant.
 */
public class MotivatorIOSim implements MotivatorIO {
  // Simulation for motivator motor 1
  private final FlywheelSim motivator1Sim;

  // Simulation constants
  private static final double MOTIVATOR_MOI = 0.002; // kg*m^2 moment of inertia
  private static final double GEAR_RATIO = 3.0; // Direct drive assumed

  // Simple first-order response - how quickly sim reaches target (0-1, higher =
  // faster)
  private static final double SIM_RESPONSE_RATE = 0.15;

  // Current state - motor 1
  private double motivatorDutyCycle = 0.0;
  private double motivatorTargetRPM = 0.0;
  private double motivatorCurrentRPM = 0.0;
  private boolean motivatorVelocityMode = false;

  // Velocity tolerances for atSetpoint with hysteresis (set by subsystem via
  // setVelocityTolerance)
  private double motivatorEnterToleranceRPM = 100.0;
  private double motivatorExitToleranceRPM = 500.0;
  private boolean wasAtSetpoint = false;

  public MotivatorIOSim() {
    // Motivator motor 1: 1 NEO Vortex motor
    motivator1Sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), MOTIVATOR_MOI, GEAR_RATIO),
            DCMotor.getNEO(1));
  }

  @Override
  public void updateInputs(MotorInputs motor1Inputs) {
    // Update motivator 1 simulation — simple first-order response for both modes.
    // Voltage mode targets an RPM proportional to duty cycle (scaled by a nominal max RPM).
    double effectiveTarget =
        motivatorVelocityMode ? motivatorTargetRPM : motivatorDutyCycle * 6000.0;
    motivatorCurrentRPM += (effectiveTarget - motivatorCurrentRPM) * SIM_RESPONSE_RATE;

    // Motivator 1 data
    motor1Inputs.connected = true;
    double motivator1Volts =
        motivatorVelocityMode ? motivatorCurrentRPM * 0.002 : motivatorDutyCycle * 12.0;
    motor1Inputs.wheelRPM = motivatorCurrentRPM;
    motor1Inputs.appliedVolts = motivator1Volts;
    motor1Inputs.currentAmps = Math.abs(motivatorCurrentRPM) * 0.005;
    motor1Inputs.tempCelsius = 25.0;
    motor1Inputs.targetRPM = motivatorTargetRPM;
    double error = Math.abs(motivatorCurrentRPM - motivatorTargetRPM);
    double threshold = wasAtSetpoint ? motivatorExitToleranceRPM : motivatorEnterToleranceRPM;
    wasAtSetpoint = motivatorVelocityMode && error < threshold;
    motor1Inputs.atSetpoint = wasAtSetpoint;
  }

  // ========== Voltage Control ==========

  @Override
  public void setMotivatorVoltage(double volts) {
    motivatorVelocityMode = false;
    motivatorTargetRPM = 0.0;
    motivatorDutyCycle = volts / 12.0;
  }

  // ========== Velocity Control ==========

  @Override
  public void setMotivatorVelocity(double velocityRPM) {
    motivatorVelocityMode = true;
    motivatorTargetRPM = Math.abs(velocityRPM);
    motivatorDutyCycle = 0.0;
  }

  @Override
  public void stopMotivator() {
    motivatorVelocityMode = false;
    motivatorTargetRPM = 0.0;
    motivatorDutyCycle = 0.0;
    motivatorCurrentRPM = 0.0;
    motivator1Sim.setInputVoltage(0.0);
  }

  @Override
  public void setVelocityTolerance(double enterToleranceRPM, double exitToleranceRPM) {
    this.motivatorEnterToleranceRPM = enterToleranceRPM;
    this.motivatorExitToleranceRPM = exitToleranceRPM;
  }
}
