package frc.robot.subsystems.launcher;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.RobotConfig;

/**
 * Simulation implementation of LauncherIO using WPILib's FlywheelSim. Simulates two NEO Vortex
 * motors driving a flywheel.
 */
public class LauncherIOSim implements LauncherIO {
  private final FlywheelSim sim;
  private final double gearRatio;

  private double targetWheelRPM = 0.0;
  private double appliedVolts = 0.0;
  private boolean voltageMode = false; // True when using direct voltage control (SysId)

  // Simulation constants
  private static final double FLYWHEEL_MOI = 0.005; // kg*m^2 moment of inertia
  private static final double VELOCITY_TOLERANCE_RPM = 50.0;

  public LauncherIOSim() {
    RobotConfig config = Constants.getRobotConfig();
    gearRatio = config.getLauncherGearRatio();

    // NEO Vortex motors (2 motors coupled together)
    // Note: DCMotor.getNeoVortex() may not exist in all WPILib versions
    // Using NEO as approximation - Vortex has similar characteristics
    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(2), // Two motors
                FLYWHEEL_MOI,
                1.0 / gearRatio), // Reduction from motor to wheel
            DCMotor.getNEO(2));
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    // Only apply P-control when in velocity mode (not voltage mode for SysId)
    if (!voltageMode) {
      // Simple P control for simulation
      double currentWheelRPM = sim.getAngularVelocityRPM();
      double error = targetWheelRPM - currentWheelRPM;
      appliedVolts = error * 0.01; // Simple proportional gain
      appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
      sim.setInputVoltage(appliedVolts);
    }
    // In voltage mode, appliedVolts is set directly by setVoltage()

    sim.update(0.02); // 20ms update period

    double wheelRPM = sim.getAngularVelocityRPM();
    double motorRPM = wheelRPM / gearRatio;

    // Both motors report same values in simulation
    inputs.leaderConnected = true;
    inputs.followerConnected = true;
    inputs.leaderVelocityRPM = motorRPM;
    inputs.followerVelocityRPM = motorRPM;
    inputs.wheelVelocityRPM = wheelRPM;
    inputs.leaderAppliedVolts = appliedVolts;
    inputs.followerAppliedVolts = appliedVolts;
    inputs.leaderCurrentAmps = sim.getCurrentDrawAmps() / 2.0;
    inputs.followerCurrentAmps = sim.getCurrentDrawAmps() / 2.0;
    inputs.leaderTempCelsius = 25.0;
    inputs.followerTempCelsius = 25.0;
    inputs.targetVelocityRPM = targetWheelRPM;
    inputs.atSetpoint = Math.abs(wheelRPM - targetWheelRPM) < VELOCITY_TOLERANCE_RPM;
  }

  @Override
  public void setVelocity(double velocityRPM) {
    voltageMode = false; // Exit voltage mode, use P-control
    targetWheelRPM = Math.abs(velocityRPM);
  }

  @Override
  public void setVoltage(double volts) {
    voltageMode = true; // Enter voltage mode for SysId
    targetWheelRPM = 0.0; // Clear velocity target
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    voltageMode = false;
    targetWheelRPM = 0.0;
    appliedVolts = 0.0;
    sim.setInputVoltage(0.0);
  }
}
