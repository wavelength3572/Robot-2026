package frc.robot.subsystems.climber;

/**
 * Simulation implementation of ClimberIO. Uses a simple first-order response to track the target
 * position, matching real robot behavior for testing in sim.
 */
public class ClimberIOSim implements ClimberIO {
  private double currentPosition = 0.0;
  private double targetPosition = 0.0;

  // How fast the sim climber moves toward target (reaches target in ~0.5s)
  private static final double SIM_RESPONSE_RATE = 0.1;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // Simple first-order response — position moves toward target each cycle
    currentPosition += (targetPosition - currentPosition) * SIM_RESPONSE_RATE;

    inputs.positionRotations = currentPosition;
    inputs.appliedVolts = (targetPosition - currentPosition) * 0.5;
    inputs.currentAmps = Math.abs(inputs.appliedVolts) * 0.5;
  }

  @Override
  public void setPosition(double motorRotations) {
    targetPosition = motorRotations;
  }

  @Override
  public void setPosition(double motorRotations, double arbFFVolts) {
    targetPosition = motorRotations;
  }

  @Override
  public void stop() {
    targetPosition = currentPosition;
  }

  @Override
  public void configurePID(double kP) {
    // No-op in sim
  }

  @Override
  public void zeroEncoder() {
    currentPosition = 0.0;
    targetPosition = 0.0;
  }
}
