package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
  private final DCMotorSim turretSim;
  private PIDController turretController =
      new PIDController(TurretConstants.kP, 0, TurretConstants.kD);

  // Simulation
  private Rotation2d targetRotation = new Rotation2d();
  private DCMotor turretGearBox = DCMotor.getNEO(1);
  private double turretFFVolts = 0.0;
  private double turretAppliedVolts = 0.0;

  public TurretIOSim() {
    turretSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turretGearBox, 0.001, TurretConstants.GEAR_RATIO),
            turretGearBox);

    // Enable wrapping for turn PID
    turretController.enableContinuousInput(-Math.PI - 0.349066, Math.PI + 0.349066);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Get the voltage being applied
    turretAppliedVolts =
        turretFFVolts + turretController.calculate(turretSim.getAngularPositionRad());

    // Update motor simulation
    turretSim.setInputVoltage(MathUtil.clamp(turretAppliedVolts, -12.0, 12.0));
    turretSim.update(0.02);

    // Get simulated position in degrees
    inputs.targetAngleDegrees = targetRotation.getDegrees();
    inputs.targetAngleRadians = targetRotation.getRadians();
    inputs.currentAngleDegrees = Math.toDegrees(turretSim.getAngularPositionRad());
    inputs.currentAngleRadians = turretSim.getAngularPositionRad();
    inputs.velocityDegreesPerSec = Math.toDegrees(turretSim.getAngularVelocityRadPerSec());
    inputs.appliedVolts = turretAppliedVolts;
    inputs.currentAmps = Math.abs(turretSim.getCurrentDrawAmps());
  }

  @Override
  public void setTargetAngle(Rotation2d rotation) {
    targetRotation = rotation;
    turretController.setSetpoint(targetRotation.getRadians());
  }
}
