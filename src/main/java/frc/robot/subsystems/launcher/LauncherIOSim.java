package frc.robot.subsystems.launcher;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;

/**
 * Simulation implementation of LauncherIO using WPILib's FlywheelSim. Simulates two NEO Vortex
 * motors driving a flywheel.
 */
public class LauncherIOSim implements LauncherIO {
  private final FlywheelSim sim;
  private final double gearRatio;

  private double targetWheelRPM = 0.0;
  private double currentWheelRPM = 0.0; // Simulated current velocity
  private double appliedVolts = 0.0;
  private boolean voltageMode = false; // True when using direct voltage control (SysId)

  // Simulation constants
  private static final double FLYWHEEL_MOI = 0.005; // kg*m^2 moment of inertia

  // Shared tunable tolerance (same as LauncherIOSparkFlex for consistency)
  private static final LoggedTunableNumber velocityToleranceRPM =
      new LoggedTunableNumber("Launcher/VelocityToleranceRPM", 50.0);

  // Simple first-order response - how quickly sim reaches target (0-1, higher = faster)
  // At 0.15, reaches ~95% of target in about 0.2 seconds (10 cycles)
  private static final double SIM_RESPONSE_RATE = 0.15;

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
    // Simple first-order response for velocity mode - just track the target smoothly
    if (!voltageMode) {
      // Exponential approach to target (simple and reliable)
      currentWheelRPM += (targetWheelRPM - currentWheelRPM) * SIM_RESPONSE_RATE;

      // Calculate approximate voltage for logging (not used for control)
      appliedVolts = currentWheelRPM * 0.004; // Rough approximation
    } else {
      // Voltage mode for SysId - use physics sim
      sim.update(0.02);
      currentWheelRPM = sim.getAngularVelocityRPM();
    }

    double motorRPM = currentWheelRPM / gearRatio;

    // Both motors report same values in simulation
    inputs.leaderConnected = true;
    inputs.followerConnected = true;
    inputs.leaderVelocityRPM = motorRPM;
    inputs.followerVelocityRPM = motorRPM;
    inputs.wheelVelocityRPM = currentWheelRPM;
    inputs.leaderAppliedVolts = appliedVolts;
    inputs.followerAppliedVolts = appliedVolts;
    inputs.leaderCurrentAmps = Math.abs(currentWheelRPM) * 0.01; // Rough current estimate
    inputs.followerCurrentAmps = Math.abs(currentWheelRPM) * 0.01;
    inputs.leaderTempCelsius = 25.0;
    inputs.followerTempCelsius = 25.0;
    inputs.targetVelocityRPM = targetWheelRPM;
    inputs.atSetpoint = Math.abs(currentWheelRPM - targetWheelRPM) < velocityToleranceRPM.get();
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

    // Safety: limit voltage when approaching max velocity to prevent overspeed during SysId
    double maxVelocityRPM = 3500.0;
    if (Math.abs(currentWheelRPM) >= maxVelocityRPM * 0.95) {
      volts = 0.0;
    } else if (Math.abs(currentWheelRPM) >= maxVelocityRPM * 0.85) {
      double scale = (maxVelocityRPM * 0.95 - Math.abs(currentWheelRPM)) / (maxVelocityRPM * 0.1);
      volts = volts * scale;
    }

    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    voltageMode = false;
    targetWheelRPM = 0.0;
    currentWheelRPM = 0.0; // Instant stop in sim
    appliedVolts = 0.0;
    sim.setInputVoltage(0.0);
  }
}
