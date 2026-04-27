package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** FlywheelSim-backed sim for the Climber pre-feed wheel, mirroring the Spindexer sim. */
public class ClimberIOSim implements ClimberIO {
  private final FlywheelSim sim;

  private static final double MOI = 0.002;
  private static final double SIM_RESPONSE_RATE = 0.15;

  private double targetRPM = 0.0;
  private double currentRPM = 0.0;
  private double dutyCycle = 0.0;
  private boolean velocityMode = false;
  private double toleranceRPM = 100.0;

  public ClimberIOSim() {
    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), MOI, 1.0),
            DCMotor.getNeo550(1));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    if (velocityMode) {
      currentRPM += (targetRPM - currentRPM) * SIM_RESPONSE_RATE;
    } else {
      sim.setInputVoltage(dutyCycle * 12.0);
      sim.update(0.02);
      currentRPM = sim.getAngularVelocityRPM();
    }

    inputs.connected = true;
    inputs.wheelRPM = currentRPM;
    inputs.appliedVolts = velocityMode ? currentRPM * 0.002 : dutyCycle * 12.0;
    inputs.currentAmps = Math.abs(currentRPM) * 0.005;
    inputs.tempCelsius = 25.0;
    inputs.targetRPM = targetRPM;
    inputs.atSetpoint = velocityMode && Math.abs(currentRPM - targetRPM) < toleranceRPM;
  }

  @Override
  public void setClimberVelocity(double wheelVelocityRPM) {
    velocityMode = true;
    targetRPM = wheelVelocityRPM;
    dutyCycle = 0.0;
  }

  @Override
  public void setClimberVoltage(double volts) {
    velocityMode = false;
    targetRPM = 0.0;
    dutyCycle = volts / 12.0;
  }

  @Override
  public void stopClimber() {
    velocityMode = false;
    targetRPM = 0.0;
    dutyCycle = 0.0;
    currentRPM = 0.0;
    sim.setInputVoltage(0.0);
  }

  @Override
  public void setVelocityTolerance(double toleranceRPM) {
    this.toleranceRPM = toleranceRPM;
  }
}
