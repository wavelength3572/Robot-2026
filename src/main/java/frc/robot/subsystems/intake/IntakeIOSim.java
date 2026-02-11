package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  // Simulation motors
  private final DCMotorSim deploySim;
  private final DCMotorSim rollerSim;

  // Deploy control
  private final PIDController deployController =
      new PIDController(
          IntakeConstants.DEPLOY_KP, IntakeConstants.DEPLOY_KI, IntakeConstants.DEPLOY_KD);
  private double deployTargetPosition = 0.0;
  private double deployAppliedVolts = 0.0;

  // Roller control
  private final PIDController rollerVelocityController =
      new PIDController(
          IntakeConstants.ROLLER_KP, IntakeConstants.ROLLER_KI, IntakeConstants.ROLLER_KD);
  private double rollerKFF = IntakeConstants.ROLLER_KFF;
  private double rollerAppliedVolts = 0.0;
  private double rollerTargetSpeed = 0.0;
  private boolean rollerVelocityMode = false;
  private double rollerTargetRPM = 0.0;

  // Motor models
  private final DCMotor deployGearbox = DCMotor.getNEO(1);
  private final DCMotor rollerGearbox = DCMotor.getNEO(1);

  public IntakeIOSim() {
    // Create deploy motor simulation
    deploySim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                deployGearbox, IntakeConstants.DEPLOY_SIM_MOI, IntakeConstants.DEPLOY_GEAR_RATIO),
            deployGearbox);

    // Create roller motor simulation
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                rollerGearbox, IntakeConstants.ROLLER_SIM_MOI, IntakeConstants.ROLLER_GEAR_RATIO),
            rollerGearbox);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Run deploy closed-loop control
    deployAppliedVolts =
        deployController.calculate(deploySim.getAngularPositionRotations(), deployTargetPosition);

    // Simulate soft limits for deploy
    double currentPosition = deploySim.getAngularPositionRotations();
    if ((currentPosition >= IntakeConstants.DEPLOY_EXTENDED_POSITION && deployAppliedVolts > 0)
        || (currentPosition <= IntakeConstants.DEPLOY_RETRACTED_POSITION
            && deployAppliedVolts < 0)) {
      deployAppliedVolts = 0.0;
    }

    // Run roller velocity closed-loop if in velocity mode
    if (rollerVelocityMode) {
      double ffVolts = rollerKFF * rollerTargetRPM;
      double pidVolts =
          rollerVelocityController.calculate(rollerSim.getAngularVelocityRPM(), rollerTargetRPM);
      rollerAppliedVolts = ffVolts + pidVolts;
    }

    // Update simulations
    deploySim.setInputVoltage(MathUtil.clamp(deployAppliedVolts, -12.0, 12.0));
    rollerSim.setInputVoltage(MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0));
    deploySim.update(0.02);
    rollerSim.update(0.02);

    // Clamp deploy position to soft limits
    double simDeployPosition =
        MathUtil.clamp(
            deploySim.getAngularPositionRotations(),
            IntakeConstants.DEPLOY_RETRACTED_POSITION,
            IntakeConstants.DEPLOY_EXTENDED_POSITION);

    // Update deploy inputs
    inputs.deployConnected = true;
    inputs.deployPositionRotations = simDeployPosition;
    inputs.deployVelocityRPM = deploySim.getAngularVelocityRPM();
    inputs.deployAppliedVolts = deployAppliedVolts;
    inputs.deployCurrentAmps = Math.abs(deploySim.getCurrentDrawAmps());
    inputs.deployTargetPosition = deployTargetPosition;

    // Update roller inputs
    inputs.rollerConnected = true;
    inputs.rollerVelocityRPM = rollerSim.getAngularVelocityRPM();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());
    inputs.rollerTargetSpeed = rollerTargetSpeed;
  }

  @Override
  public void setDeployPosition(double positionRotations) {
    // Clamp to valid range
    deployTargetPosition =
        Math.max(
            IntakeConstants.DEPLOY_RETRACTED_POSITION,
            Math.min(IntakeConstants.DEPLOY_EXTENDED_POSITION, positionRotations));
    deployController.setSetpoint(deployTargetPosition);
  }

  @Override
  public void setRollerDutyCycle(double dutyCycle) {
    rollerVelocityMode = false;
    rollerTargetSpeed = dutyCycle;
    rollerAppliedVolts = dutyCycle * 12.0; // Convert duty cycle to voltage
  }

  @Override
  public void setRollerVelocity(double rpm) {
    rollerVelocityMode = true;
    rollerTargetSpeed = rpm;
    rollerTargetRPM = rpm;
  }

  @Override
  public void stop() {
    deployTargetPosition = deploySim.getAngularPositionRotations();
    deployAppliedVolts = 0.0;
    rollerAppliedVolts = 0.0;
    rollerTargetSpeed = 0.0;
    rollerVelocityMode = false;
    rollerTargetRPM = 0.0;
  }

  @Override
  public void configureDeployPID(double kP, double kI, double kD) {
    deployController.setPID(kP, kI, kD);
  }

  @Override
  public void configureRollerPID(double kP, double kI, double kD, double kFF) {
    rollerVelocityController.setPID(kP, kI, kD);
    rollerKFF = kFF;
  }
}
