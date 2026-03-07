package frc.robot.subsystems.climber;

import frc.robot.subsystems.climber.ClimberConstants.CLIMB_STATE;

public class ClimberIOVirtualSim implements ClimberIO {

  private CLIMB_STATE currentClimberState = CLIMB_STATE.STOWED;
  private double targetEncoderRotations = 0;
  private double virtualEncoderRotations = 0;

  public ClimberIOVirtualSim() {}

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    inputs.currentClimbState = currentClimberState;
    inputs.targetPosition = targetEncoderRotations;

    // Simulate gradual movement toward the target (smooth virtual motion)
    if (virtualEncoderRotations < targetEncoderRotations) {
      virtualEncoderRotations = Math.min(virtualEncoderRotations + 5, targetEncoderRotations);
    } else if (virtualEncoderRotations > targetEncoderRotations) {
      virtualEncoderRotations = Math.max(virtualEncoderRotations - 5, targetEncoderRotations);
    }

    inputs.currentPosition = virtualEncoderRotations;
  }

  @Override
  public void deployClimber() {
    currentClimberState = CLIMB_STATE.DEPLOY;
    targetEncoderRotations = ClimberConstants.DEPLOY_POSITION;
  }

  @Override
  public void climb() {
    if (currentClimberState == CLIMB_STATE.DEPLOY) {
      currentClimberState = CLIMB_STATE.CLIMB;
      targetEncoderRotations = ClimberConstants.CLIMBED_POSITION;
    }
  }

  @Override
  public boolean isClimberDeployed() {
    return (this.currentClimberState != CLIMB_STATE.STOWED);
  }

  @Override
  public boolean isClimbingFinished() {
    if (currentClimberState == CLIMB_STATE.CLIMB) {
      double difference = Math.abs(ClimberConstants.CLIMBED_POSITION - virtualEncoderRotations);
      return (difference < ClimberConstants.CLIMBING_TOLERANCE) ? true : false;
    } else return false;
  }

  public double angleToRotations(double angle) {
    return (angle / 360.0) * ClimberConstants.kClimberGearing;
  }

  public double rotationsToAngle(double rotations) {
    return (rotations / ClimberConstants.kClimberGearing) * 360.0;
  }
}
