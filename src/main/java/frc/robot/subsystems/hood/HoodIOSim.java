package frc.robot.subsystems.hood;

import frc.robot.util.LoggedTunableNumber;

/** Simulation implementation of HoodIO. Uses simple first-order response to track target angle. */
public class HoodIOSim implements HoodIO {
  private double currentAngleDeg = 45.0; // Start at middle position
  private double targetAngleDeg = 45.0;

  // Tunable limits (same defaults as hardware would have)
  private static final LoggedTunableNumber minAngleDeg =
      new LoggedTunableNumber("Hood/MinAngleDeg", 15.0);
  private static final LoggedTunableNumber maxAngleDeg =
      new LoggedTunableNumber("Hood/MaxAngleDeg", 85.0);
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hood/ToleranceDeg", 1.0);

  // Sim response rate (how fast hood moves to target)
  private static final double SIM_RESPONSE_RATE = 0.2; // Reaches target in ~0.25 seconds

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
    inputs.atTarget = Math.abs(currentAngleDeg - targetAngleDeg) < toleranceDeg.get();
  }

  @Override
  public void setAngle(double angleDeg) {
    // Clamp to limits
    targetAngleDeg = Math.max(minAngleDeg.get(), Math.min(maxAngleDeg.get(), angleDeg));
  }

  @Override
  public void stop() {
    // Hold current position
    targetAngleDeg = currentAngleDeg;
  }
}
