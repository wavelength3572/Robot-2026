package frc.robot.subsystems.hood;

/**
 * Simulation implementation of HoodIO. Uses simple first-order response to track target angle.
 *
 * <p>Note: Angle limits and tolerance are defined in Hood.java (the subsystem). This sim just
 * tracks whatever angle the subsystem requests.
 */
public class HoodIOSim implements HoodIO {
  private double currentAngleDeg = 45.0; // Start at middle position
  private double targetAngleDeg = 45.0;

  // Sim response rate (how fast hood moves to target)
  private static final double SIM_RESPONSE_RATE = 0.2; // Reaches target in ~0.25 seconds

  // Tolerance for atTarget (matches Hood.java default)
  private static final double TOLERANCE_DEG = 1.0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Simple first-order response - hood moves toward target
    currentAngleDeg += (targetAngleDeg - currentAngleDeg) * SIM_RESPONSE_RATE;

    inputs.connected = true;
    inputs.currentAngleDeg = currentAngleDeg;
    inputs.targetAngleDeg = targetAngleDeg;
    inputs.appliedVolts = (targetAngleDeg - currentAngleDeg) * 0.5; // Rough estimate
    inputs.currentAmps = Math.abs(inputs.appliedVolts) * 0.5;
    inputs.tempCelsius = 25.0;
    inputs.atTarget = Math.abs(currentAngleDeg - targetAngleDeg) < TOLERANCE_DEG;
  }

  @Override
  public void setAngle(double angleDeg) {
    // Trust that Hood.java already clamped the angle
    targetAngleDeg = angleDeg;
  }

  @Override
  public void stop() {
    // Hold current position
    targetAngleDeg = currentAngleDeg;
  }
}
