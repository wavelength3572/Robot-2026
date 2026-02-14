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
  private double motivator1DutyCycle = 0.0;
  private double motivator1TargetRPM = 0.0;
  private double motivator1CurrentRPM = 0.0;
  private boolean motivator1VelocityMode = false;

  // Velocity tolerance for atSetpoint (set by subsystem via
  // setVelocityTolerances)
  private double motivatorToleranceRPM = 100.0;

  public MotivatorIOSim() {
    // Motivator motor 1: 1 NEO Vortex motor
    motivator1Sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), MOTIVATOR_MOI, GEAR_RATIO),
            DCMotor.getNEO(1));
  }

  @Override
  public void updateInputs(MotorInputs motor1Inputs) {
    // Update motivator 1 simulation
    if (motivator1VelocityMode) {
      motivator1CurrentRPM += (motivator1TargetRPM - motivator1CurrentRPM) * SIM_RESPONSE_RATE;
    } else {
      motivator1Sim.setInputVoltage(motivator1DutyCycle * 12.0);
      motivator1Sim.update(0.02);
      motivator1CurrentRPM = motivator1Sim.getAngularVelocityRPM();
    }

    // Motivator 1 data
    motor1Inputs.connected = true;
    double motivator1Volts =
        motivator1VelocityMode ? motivator1CurrentRPM * 0.002 : motivator1DutyCycle * 12.0;
    motor1Inputs.velocityRPM = motivator1CurrentRPM;
    motor1Inputs.appliedVolts = motivator1Volts;
    motor1Inputs.currentAmps = Math.abs(motivator1CurrentRPM) * 0.005;
    motor1Inputs.tempCelsius = 25.0;
    motor1Inputs.targetVelocityRPM = motivator1TargetRPM;
    motor1Inputs.atSetpoint =
        motivator1VelocityMode
            && Math.abs(motivator1CurrentRPM - motivator1TargetRPM) < motivatorToleranceRPM;
  }

  // ========== Duty Cycle Control ==========

  @Override
  public void setMotivator1DutyCycle(double dutyCycle) {
    motivator1VelocityMode = false;
    motivator1TargetRPM = 0.0;
    motivator1DutyCycle = dutyCycle;
  }

  // ========== Velocity Control ==========

  @Override
  public void setMotivator1Velocity(double velocityRPM) {
    motivator1VelocityMode = true;
    motivator1TargetRPM = Math.abs(velocityRPM);
    motivator1DutyCycle = 0.0;
  }

  // ========== Stop Methods ==========
  @Override
  public void stop() {
    motivator1VelocityMode = false;
    motivator1TargetRPM = 0.0;
    motivator1DutyCycle = 0.0;
    motivator1CurrentRPM = 0.0;
    motivator1Sim.setInputVoltage(0.0);
  }

  @Override
  public void stopMotivator1() {
    motivator1VelocityMode = false;
    motivator1TargetRPM = 0.0;
    motivator1DutyCycle = 0.0;
    motivator1CurrentRPM = 0.0;
    motivator1Sim.setInputVoltage(0.0);
  }

  @Override
  public void stopMotivators() {
    stopMotivator1();
  }

  @Override
  public void setVelocityTolerances(double motivatorToleranceRPM) {
    this.motivatorToleranceRPM = motivatorToleranceRPM;
  }
}
