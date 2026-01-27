package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.RobotConfig;

public class TurretIOSim implements TurretIO {
  private final RobotConfig config;
  private final DCMotorSim turretSim;
  private final PIDController turretController;

  // Configuration values from RobotConfig
  private final double maxAngleDegrees;
  private final double minAngleDegrees;

  // Simulation
  private Rotation2d targetRotation = new Rotation2d();
  private DCMotor turretGearBox = DCMotor.getNEO(1);
  private double turretFFVolts = 0.0;
  private double turretAppliedVolts = 0.0;

  public TurretIOSim() {
    config = Constants.getRobotConfig();

    // Get config values
    maxAngleDegrees = config.getTurretMaxAngleDegrees();
    minAngleDegrees = config.getTurretMinAngleDegrees();

    // Create PID controller from config
    turretController = new PIDController(config.getTurretKp(), 0, config.getTurretKd());

    turretSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turretGearBox, 0.001, config.getTurretGearRatio()),
            turretGearBox);

    // Do NOT enable continuous input - turret has limited travel
    // turretController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Get the voltage being applied
    turretAppliedVolts =
        turretFFVolts + turretController.calculate(turretSim.getAngularPositionRad());

    // Simulate soft limits - stop motor if at limit and trying to go further
    double currentDegrees = Math.toDegrees(turretSim.getAngularPositionRad());
    if ((currentDegrees >= maxAngleDegrees && turretAppliedVolts > 0)
        || (currentDegrees <= minAngleDegrees && turretAppliedVolts < 0)) {
      turretAppliedVolts = 0.0; // Soft limit reached, stop motor
    }

    // Update motor simulation
    turretSim.setInputVoltage(MathUtil.clamp(turretAppliedVolts, -12.0, 12.0));
    turretSim.update(0.02);

    // Clamp simulated position to soft limits (in case of overshoot)
    double simPositionDegrees =
        MathUtil.clamp(
            Math.toDegrees(turretSim.getAngularPositionRad()), minAngleDegrees, maxAngleDegrees);

    // Get simulated position in degrees
    inputs.targetAngleDegrees = targetRotation.getDegrees();
    inputs.targetAngleRadians = targetRotation.getRadians();
    inputs.currentAngleDegrees = simPositionDegrees;
    inputs.currentAngleRadians = Math.toRadians(simPositionDegrees);
    inputs.velocityDegreesPerSec = Math.toDegrees(turretSim.getAngularVelocityRadPerSec());
    inputs.appliedVolts = turretAppliedVolts;
    inputs.currentAmps = Math.abs(turretSim.getCurrentDrawAmps());
  }

  @Override
  public void setTargetAngle(Rotation2d rotation) {
    // Clamp the target angle to valid range (matching real robot soft limits)
    double clampedDegrees =
        Math.max(minAngleDegrees, Math.min(maxAngleDegrees, rotation.getDegrees()));
    targetRotation = Rotation2d.fromDegrees(clampedDegrees);
    turretController.setSetpoint(targetRotation.getRadians());
  }
}
