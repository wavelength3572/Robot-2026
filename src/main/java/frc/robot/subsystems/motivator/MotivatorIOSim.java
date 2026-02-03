package frc.robot.subsystems.motivator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation implementation of MotivatorIO using WPILib's FlywheelSim. Simulates the motivator and
 * prefeed rollers as simple flywheels.
 */
public class MotivatorIOSim implements MotivatorIO {
  // Simulation for main motivator (2 motors coupled)
  private final FlywheelSim motivatorSim;

  // Simulation for prefeed (1 motor)
  private final FlywheelSim prefeedSim;

  // Simulation constants
  private static final double MOTIVATOR_MOI = 0.002; // kg*m^2 moment of inertia
  private static final double PREFEED_MOI = 0.001; // kg*m^2 moment of inertia
  private static final double GEAR_RATIO = 1.0; // Direct drive assumed

  // Current duty cycle commands
  private double motivatorDutyCycle = 0.0;
  private double prefeedDutyCycle = 0.0;

  public MotivatorIOSim() {
    // Main motivator: 2 NEO Vortex motors
    motivatorSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), MOTIVATOR_MOI, GEAR_RATIO),
            DCMotor.getNEO(2));

    // Prefeed: 1 NEO Vortex motor
    prefeedSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), PREFEED_MOI, GEAR_RATIO),
            DCMotor.getNEO(1));
  }

  @Override
  public void updateInputs(MotivatorIOInputs inputs) {
    // Update simulations
    motivatorSim.setInputVoltage(motivatorDutyCycle * 12.0);
    prefeedSim.setInputVoltage(prefeedDutyCycle * 12.0);

    motivatorSim.update(0.02);
    prefeedSim.update(0.02);

    // All motors connected in sim
    inputs.motivatorLeaderConnected = true;
    inputs.motivatorFollowerConnected = true;
    inputs.prefeedConnected = true;

    // Main motivator data (both motors report same in sim)
    double motivatorRPM = motivatorSim.getAngularVelocityRPM();
    double motivatorVolts = motivatorDutyCycle * 12.0;
    double motivatorCurrent = Math.abs(motivatorRPM) * 0.005; // Rough estimate

    inputs.motivatorLeaderVelocityRPM = motivatorRPM;
    inputs.motivatorLeaderAppliedVolts = motivatorVolts;
    inputs.motivatorLeaderCurrentAmps = motivatorCurrent;
    inputs.motivatorLeaderTempCelsius = 25.0;

    inputs.motivatorFollowerVelocityRPM = motivatorRPM;
    inputs.motivatorFollowerAppliedVolts = motivatorVolts;
    inputs.motivatorFollowerCurrentAmps = motivatorCurrent;
    inputs.motivatorFollowerTempCelsius = 25.0;

    // Prefeed data
    double prefeedRPM = prefeedSim.getAngularVelocityRPM();
    double prefeedVolts = prefeedDutyCycle * 12.0;
    double prefeedCurrent = Math.abs(prefeedRPM) * 0.005;

    inputs.prefeedVelocityRPM = prefeedRPM;
    inputs.prefeedAppliedVolts = prefeedVolts;
    inputs.prefeedCurrentAmps = prefeedCurrent;
    inputs.prefeedTempCelsius = 25.0;

    // Future: ball detection
    inputs.ballDetected = false;
  }

  @Override
  public void setMotivatorDutyCycle(double dutyCycle) {
    motivatorDutyCycle = dutyCycle;
  }

  @Override
  public void setPrefeedDutyCycle(double dutyCycle) {
    prefeedDutyCycle = dutyCycle;
  }

  @Override
  public void stop() {
    motivatorDutyCycle = 0.0;
    prefeedDutyCycle = 0.0;
    motivatorSim.setInputVoltage(0.0);
    prefeedSim.setInputVoltage(0.0);
  }

  @Override
  public void stopMotivator() {
    motivatorDutyCycle = 0.0;
    motivatorSim.setInputVoltage(0.0);
  }

  @Override
  public void stopPrefeed() {
    prefeedDutyCycle = 0.0;
    prefeedSim.setInputVoltage(0.0);
  }
}
