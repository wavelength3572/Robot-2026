package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.RobotConfig;

public class IntakeIOSim implements IntakeIO {
  // Sim-only moment of inertia constants
  private static final double DEPLOY_SIM_MOI = 0.01; // kg*m^2
  private static final double ROLLER_SIM_MOI = 0.001; // kg*m^2

  // Deploy position limits (from config)
  private final double deployStowedPosition;
  private final double deployRetractedPosition;
  private final double deployExtendedPosition;

  // Simulation motors
  private final DCMotorSim deploySim;
  private final DCMotorSim rollerSim;

  // Deploy control
  private final PIDController deployController;
  private double deployTargetPosition = 0.0;
  private double deployAppliedVolts = 0.0;

  // MAXMotion simulation (trapezoidal profiling)
  private TrapezoidProfile deployProfile;
  private TrapezoidProfile.State deployProfileState = new TrapezoidProfile.State(0.0, 0.0);

  // Roller control
  private final PIDController rollerVelocityController;
  private double rollerKFF;
  private double rollerAppliedVolts = 0.0;
  private double rollerTargetSpeed = 0.0;
  private boolean rollerVelocityMode = false;
  private double rollerTargetRPM = 0.0;

  // Motor models
  private final DCMotor deployGearbox = DCMotor.getNEO(1);
  private final DCMotor rollerGearbox = DCMotor.getNEO(1);

  public IntakeIOSim() {
    RobotConfig config = Constants.getRobotConfig();

    deployStowedPosition = config.getIntakeDeployStowedPosition();
    deployRetractedPosition = config.getIntakeDeployRetractedPosition();
    deployExtendedPosition = config.getIntakeDeployExtendedPosition();

    deployController =
        new PIDController(
            config.getIntakeDeployKp(), config.getIntakeDeployKi(), config.getIntakeDeployKd());

    rollerVelocityController =
        new PIDController(
            config.getIntakeRollerKp(), config.getIntakeRollerKi(), config.getIntakeRollerKd());
    rollerKFF = config.getIntakeRollerKff();

    // MAXMotion constraints: convert RPM to rotations/s for TrapezoidProfile
    double maxVelRotPerSec = config.getIntakeDeployMaxVelocity() / 60.0;
    double maxAccelRotPerSecSq = config.getIntakeDeployMaxAcceleration() / 60.0;
    deployProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelRotPerSec, maxAccelRotPerSecSq));

    // Create deploy motor simulation
    deploySim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                deployGearbox, DEPLOY_SIM_MOI, config.getIntakeDeployGearRatio()),
            deployGearbox);

    // Create roller motor simulation
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                rollerGearbox, ROLLER_SIM_MOI, config.getIntakeRollerGearRatio()),
            rollerGearbox);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Run deploy through trapezoidal profile then PID
    TrapezoidProfile.State goal = new TrapezoidProfile.State(deployTargetPosition, 0.0);
    deployProfileState = deployProfile.calculate(0.02, deployProfileState, goal);
    deployAppliedVolts =
        deployController.calculate(
            deploySim.getAngularPositionRotations(), deployProfileState.position);

    // Simulate soft limits for deploy
    double currentPosition = deploySim.getAngularPositionRotations();
    double maxPos =
        Math.max(deployStowedPosition, Math.max(deployExtendedPosition, deployRetractedPosition));
    double minPos =
        Math.min(deployStowedPosition, Math.min(deployExtendedPosition, deployRetractedPosition));
    if ((currentPosition >= maxPos && deployAppliedVolts > 0)
        || (currentPosition <= minPos && deployAppliedVolts < 0)) {
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
        MathUtil.clamp(deploySim.getAngularPositionRotations(), minPos, maxPos);

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
    // Clamp to valid range (includes stowed, retracted, and extended positions)
    double minPos =
        Math.min(deployStowedPosition, Math.min(deployRetractedPosition, deployExtendedPosition));
    double maxPos =
        Math.max(deployStowedPosition, Math.max(deployRetractedPosition, deployExtendedPosition));
    deployTargetPosition = Math.max(minPos, Math.min(maxPos, positionRotations));
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
    deployProfileState = new TrapezoidProfile.State(deploySim.getAngularPositionRotations(), 0.0);
    rollerAppliedVolts = 0.0;
    rollerTargetSpeed = 0.0;
    rollerVelocityMode = false;
    rollerTargetRPM = 0.0;
  }

  @Override
  public void stopDeploy() {
    deployTargetPosition = deploySim.getAngularPositionRotations();
    deployAppliedVolts = 0.0;
    deployProfileState = new TrapezoidProfile.State(deploySim.getAngularPositionRotations(), 0.0);
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

  @Override
  public void configureDeployMaxMotion(
      double maxVelocity, double maxAcceleration, double allowedError) {
    double maxVelRotPerSec = maxVelocity / 60.0;
    double maxAccelRotPerSecSq = maxAcceleration / 60.0;
    deployProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelRotPerSec, maxAccelRotPerSecSq));
  }
}
