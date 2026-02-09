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

  // Simulation for motivator motor 2
  private final FlywheelSim motivator2Sim;

  // Simulation for prefeed
  private final FlywheelSim prefeedSim;

  // Simulation constants
  private static final double MOTIVATOR_MOI = 0.002; // kg*m^2 moment of inertia
  private static final double PREFEED_MOI = 0.001; // kg*m^2 moment of inertia
  private static final double GEAR_RATIO = 1.0; // Direct drive assumed

  // Simple first-order response - how quickly sim reaches target (0-1, higher = faster)
  private static final double SIM_RESPONSE_RATE = 0.15;

  // Current state - motor 1
  private double motivator1DutyCycle = 0.0;
  private double motivator1TargetRPM = 0.0;
  private double motivator1CurrentRPM = 0.0;
  private boolean motivator1VelocityMode = false;

  // Current state - motor 2
  private double motivator2DutyCycle = 0.0;
  private double motivator2TargetRPM = 0.0;
  private double motivator2CurrentRPM = 0.0;
  private boolean motivator2VelocityMode = false;

  // Current state - prefeed
  private double prefeedDutyCycle = 0.0;
  private double prefeedTargetRPM = 0.0;
  private double prefeedCurrentRPM = 0.0;
  private boolean prefeedVelocityMode = false;

  // Velocity tolerance for atSetpoint (set by subsystem via setVelocityTolerances)
  private double motivatorToleranceRPM = 100.0;
  private double prefeedToleranceRPM = 100.0;

  public MotivatorIOSim() {
    // Motivator motor 1: 1 NEO Vortex motor
    motivator1Sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), MOTIVATOR_MOI, GEAR_RATIO),
            DCMotor.getNEO(1));

    // Motivator motor 2: 1 NEO Vortex motor
    motivator2Sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), MOTIVATOR_MOI, GEAR_RATIO),
            DCMotor.getNEO(1));

    // Prefeed: 1 NEO Vortex motor
    prefeedSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), PREFEED_MOI, GEAR_RATIO),
            DCMotor.getNEO(1));
  }

  @Override
  public void updateInputs(
      MotorInputs motor1Inputs, MotorInputs motor2Inputs, MotorInputs prefeedInputs) {
    // Update motivator 1 simulation
    if (motivator1VelocityMode) {
      motivator1CurrentRPM += (motivator1TargetRPM - motivator1CurrentRPM) * SIM_RESPONSE_RATE;
    } else {
      motivator1Sim.setInputVoltage(motivator1DutyCycle * 12.0);
      motivator1Sim.update(0.02);
      motivator1CurrentRPM = motivator1Sim.getAngularVelocityRPM();
    }

    // Update motivator 2 simulation
    if (motivator2VelocityMode) {
      motivator2CurrentRPM += (motivator2TargetRPM - motivator2CurrentRPM) * SIM_RESPONSE_RATE;
    } else {
      motivator2Sim.setInputVoltage(motivator2DutyCycle * 12.0);
      motivator2Sim.update(0.02);
      motivator2CurrentRPM = motivator2Sim.getAngularVelocityRPM();
    }

    // Update prefeed simulation
    if (prefeedVelocityMode) {
      prefeedCurrentRPM += (prefeedTargetRPM - prefeedCurrentRPM) * SIM_RESPONSE_RATE;
    } else {
      prefeedSim.setInputVoltage(prefeedDutyCycle * 12.0);
      prefeedSim.update(0.02);
      prefeedCurrentRPM = prefeedSim.getAngularVelocityRPM();
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

    // Motivator 2 data
    motor2Inputs.connected = true;
    double motivator2Volts =
        motivator2VelocityMode ? motivator2CurrentRPM * 0.002 : motivator2DutyCycle * 12.0;
    motor2Inputs.velocityRPM = motivator2CurrentRPM;
    motor2Inputs.appliedVolts = motivator2Volts;
    motor2Inputs.currentAmps = Math.abs(motivator2CurrentRPM) * 0.005;
    motor2Inputs.tempCelsius = 25.0;
    motor2Inputs.targetVelocityRPM = motivator2TargetRPM;
    motor2Inputs.atSetpoint =
        motivator2VelocityMode
            && Math.abs(motivator2CurrentRPM - motivator2TargetRPM) < motivatorToleranceRPM;

    // Prefeed data
    prefeedInputs.connected = true;
    double prefeedVolts = prefeedVelocityMode ? prefeedCurrentRPM * 0.002 : prefeedDutyCycle * 12.0;
    prefeedInputs.velocityRPM = prefeedCurrentRPM;
    prefeedInputs.appliedVolts = prefeedVolts;
    prefeedInputs.currentAmps = Math.abs(prefeedCurrentRPM) * 0.005;
    prefeedInputs.tempCelsius = 25.0;
    prefeedInputs.targetVelocityRPM = prefeedTargetRPM;
    prefeedInputs.atSetpoint =
        prefeedVelocityMode && Math.abs(prefeedCurrentRPM - prefeedTargetRPM) < prefeedToleranceRPM;
  }

  // ========== Duty Cycle Control ==========

  @Override
  public void setMotivator1DutyCycle(double dutyCycle) {
    motivator1VelocityMode = false;
    motivator1TargetRPM = 0.0;
    motivator1DutyCycle = dutyCycle;
  }

  @Override
  public void setMotivator2DutyCycle(double dutyCycle) {
    motivator2VelocityMode = false;
    motivator2TargetRPM = 0.0;
    motivator2DutyCycle = dutyCycle;
  }

  @Override
  public void setPrefeedDutyCycle(double dutyCycle) {
    prefeedVelocityMode = false;
    prefeedTargetRPM = 0.0;
    prefeedDutyCycle = dutyCycle;
  }

  // ========== Velocity Control ==========

  @Override
  public void setMotivator1Velocity(double velocityRPM) {
    motivator1VelocityMode = true;
    motivator1TargetRPM = Math.abs(velocityRPM);
    motivator1DutyCycle = 0.0;
  }

  @Override
  public void setMotivator2Velocity(double velocityRPM) {
    motivator2VelocityMode = true;
    motivator2TargetRPM = Math.abs(velocityRPM);
    motivator2DutyCycle = 0.0;
  }

  @Override
  public void setPrefeedVelocity(double velocityRPM) {
    prefeedVelocityMode = true;
    prefeedTargetRPM = Math.abs(velocityRPM);
    prefeedDutyCycle = 0.0;
  }

  // ========== Stop Methods ==========

  @Override
  public void stop() {
    motivator1VelocityMode = false;
    motivator2VelocityMode = false;
    prefeedVelocityMode = false;
    motivator1TargetRPM = 0.0;
    motivator2TargetRPM = 0.0;
    prefeedTargetRPM = 0.0;
    motivator1DutyCycle = 0.0;
    motivator2DutyCycle = 0.0;
    prefeedDutyCycle = 0.0;
    motivator1CurrentRPM = 0.0;
    motivator2CurrentRPM = 0.0;
    prefeedCurrentRPM = 0.0;
    motivator1Sim.setInputVoltage(0.0);
    motivator2Sim.setInputVoltage(0.0);
    prefeedSim.setInputVoltage(0.0);
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
  public void stopMotivator2() {
    motivator2VelocityMode = false;
    motivator2TargetRPM = 0.0;
    motivator2DutyCycle = 0.0;
    motivator2CurrentRPM = 0.0;
    motivator2Sim.setInputVoltage(0.0);
  }

  @Override
  public void stopMotivators() {
    stopMotivator1();
    stopMotivator2();
  }

  @Override
  public void stopPrefeed() {
    prefeedVelocityMode = false;
    prefeedTargetRPM = 0.0;
    prefeedDutyCycle = 0.0;
    prefeedCurrentRPM = 0.0;
    prefeedSim.setInputVoltage(0.0);
  }

  @Override
  public void setVelocityTolerances(double motivatorToleranceRPM, double prefeedToleranceRPM) {
    this.motivatorToleranceRPM = motivatorToleranceRPM;
    this.prefeedToleranceRPM = prefeedToleranceRPM;
  }
}
