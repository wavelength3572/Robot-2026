package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation implementation of SpindexerIO using WPILib's FlywheelSim with velocity control
 * support.
 */
public class SpindexerIOSim implements SpindexerIO {
  // Simulation for spindexer motor
  private final FlywheelSim spindexer1Sim;

  // Simulation constants
  private static final double SPINDEXER_MOI = 0.002; // kg*m^2 moment of inertia
  private static final double GEAR_RATIO = 3.0; // Direct drive assumed

  // Simple first-order response - how quickly sim reaches target (0-1, higher =
  // faster)
  private static final double SIM_RESPONSE_RATE = 0.15;

  // Current state - motor 1
  private double spindexerDutyCycle = 0.0;
  private double spindexerTargetRPM = 0.0;
  private double spindexerCurrentRPM = 0.0;
  private boolean spindexerVelocityMode = false;

  // Velocity tolerance for atSetpoint (set by subsystem via
  // setVelocityTolerances)
  private double spindexerToleranceRPM = 100.0;

  public SpindexerIOSim() {
    // Spindexer motor 1: 1 NEO Vortex motor
    spindexer1Sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), SPINDEXER_MOI, GEAR_RATIO),
            DCMotor.getNEO(1));
  }

  @Override
  public void updateInputs(MotorInputs motor1Inputs) {
    // Update spindexer 1 simulation
    if (spindexerVelocityMode) {
      spindexerCurrentRPM += (spindexerTargetRPM - spindexerCurrentRPM) * SIM_RESPONSE_RATE;
    } else {
      spindexer1Sim.setInputVoltage(spindexerDutyCycle * 12.0);
      spindexer1Sim.update(0.02);
      spindexerCurrentRPM = spindexer1Sim.getAngularVelocityRPM();
    }

    // Spindexer 1 data
    motor1Inputs.connected = true;
    double spindexer1Volts =
        spindexerVelocityMode ? spindexerCurrentRPM * 0.002 : spindexerDutyCycle * 12.0;
    motor1Inputs.wheelRPM = spindexerCurrentRPM;
    motor1Inputs.appliedVolts = spindexer1Volts;
    motor1Inputs.currentAmps = Math.abs(spindexerCurrentRPM) * 0.005;
    motor1Inputs.tempCelsius = 25.0;
    motor1Inputs.targetRPM = spindexerTargetRPM;
    motor1Inputs.atSetpoint =
        spindexerVelocityMode
            && Math.abs(spindexerCurrentRPM - spindexerTargetRPM) < spindexerToleranceRPM;
  }

  // ========== Velocity Control ==========

  @Override
  public void setSpindexerVelocity(double velocityRPM) {
    spindexerVelocityMode = true;
    spindexerTargetRPM = Math.abs(velocityRPM);
    spindexerDutyCycle = 0.0;
  }

  @Override
  public void stopSpindexer() {
    spindexerVelocityMode = false;
    spindexerTargetRPM = 0.0;
    spindexerDutyCycle = 0.0;
    spindexerCurrentRPM = 0.0;
    spindexer1Sim.setInputVoltage(0.0);
  }

  @Override
  public void setVelocityTolerance(double spindexerToleranceRPM) {
    this.spindexerToleranceRPM = spindexerToleranceRPM;
  }
}
