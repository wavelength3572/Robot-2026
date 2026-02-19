package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.RobotConfig;

/**
 * Simulation implementation of TurretIO using a simple motion profile.
 *
 * <p>This simulation moves the turret smoothly toward the target using a trapezoidal velocity
 * profile. No PID - just follows the profile directly for guaranteed stable motion.
 */
public class TurretIOSim implements TurretIO {
  private final RobotConfig config;

  // Configuration values
  private final double maxAngleDegrees;
  private final double minAngleDegrees;

  // Motion profile for smooth movement
  private final TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State currentState = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State goalState = new TrapezoidProfile.State(0, 0);

  // Target tracking
  private Rotation2d targetRotation = new Rotation2d();

  // Tunable speed - increase if turret can't keep up
  private static final double MAX_VELOCITY_DEG_PER_SEC = 540.0; // 1.5 rotations per second
  private static final double MAX_ACCEL_DEG_PER_SEC_SQ = 1080.0; // Fast acceleration

  public TurretIOSim() {
    config = Constants.getRobotConfig();

    // Get config values
    maxAngleDegrees = config.getTurretOutsideMaxAngleDeg();
    minAngleDegrees = config.getTurretOutsideMinAngleDeg();

    // Create motion profile constraints
    constraints =
        new TrapezoidProfile.Constraints(MAX_VELOCITY_DEG_PER_SEC, MAX_ACCEL_DEG_PER_SEC_SQ);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Create a new profile from current state to goal
    TrapezoidProfile profile = new TrapezoidProfile(constraints);

    // Step the profile forward by one timestep (20ms)
    currentState = profile.calculate(0.02, currentState, goalState);

    // Clamp to soft limits
    double clampedPosition =
        MathUtil.clamp(currentState.position, minAngleDegrees, maxAngleDegrees);

    // If we hit a limit, stop velocity in that direction
    double clampedVelocity = currentState.velocity;
    if (clampedPosition >= maxAngleDegrees && clampedVelocity > 0) {
      clampedVelocity = 0;
    } else if (clampedPosition <= minAngleDegrees && clampedVelocity < 0) {
      clampedVelocity = 0;
    }

    // Update current state with clamped values
    currentState = new TrapezoidProfile.State(clampedPosition, clampedVelocity);

    // Populate inputs
    inputs.targetInsideAngleDeg = targetRotation.getDegrees();
    inputs.currentInsideAngleDeg = currentState.position;
    inputs.velocityDegreesPerSec = currentState.velocity;
    inputs.appliedVolts = 0.0;
    inputs.currentAmps = 0.0;
  }

  @Override
  public void setOutsideTurretAngle(Rotation2d rotation) {
    // Clamp the target angle to valid range
    double clampedDegrees = MathUtil.clamp(rotation.getDegrees(), minAngleDegrees, maxAngleDegrees);
    targetRotation = Rotation2d.fromDegrees(clampedDegrees);

    // Set the goal state (target position, zero velocity at goal)
    goalState = new TrapezoidProfile.State(clampedDegrees, 0);
  }
}
