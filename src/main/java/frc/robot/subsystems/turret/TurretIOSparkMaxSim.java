package frc.robot.subsystems.turret;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSparkMaxSim implements TurretIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController closedLoopController;

  // Simulation
  private final DCMotorSim motorSim;
  private double targetAngleDegrees = 0.0;
  private double appliedVolts = 0.0;
  private DCMotor turretGearBox = DCMotor.getNEO(1);

  public TurretIOSparkMaxSim() {
    motor = new SparkMax(TurretConstants.TURRET_MOTOR_CAN_ID, MotorType.kBrushless);

    // Create configuration object
    SparkMaxConfig config = new SparkMaxConfig();

    // Configure basic settings
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(TurretConstants.CURRENT_LIMIT_AMPS)
        .secondaryCurrentLimit(TurretConstants.CURRENT_THRESHOLD_AMPS)
        .voltageCompensation(12.0);

    // Configure encoder conversion factors
    // Position conversion: motor rotations -> turret degrees
    config
        .encoder
        .positionConversionFactor(360.0 / TurretConstants.GEAR_RATIO)
        .velocityConversionFactor(360.0 / TurretConstants.GEAR_RATIO / 60.0);

    // Configure closed loop (PID)
    config
        .closedLoop
        .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
        .pid(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD)
        .outputRange(-1.0, 1.0)
        .maxMotion
        .cruiseVelocity(TurretConstants.MAX_VELOCITY_DEG_PER_SEC)
        .maxAcceleration(TurretConstants.MAX_ACCELERATION_DEG_PER_SEC_SQUARED)
        .allowedProfileError(TurretConstants.ANGLE_TOLERANCE_DEGREES);

    // Apply configuration
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Get encoder and closed loop controller references
    encoder = motor.getEncoder();
    closedLoopController = motor.getClosedLoopController();

    // Initialize simulation
    // Using NEO motor (1 motor with gear ratio)
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turretGearBox, 0.001, TurretConstants.GEAR_RATIO),
            turretGearBox);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // In simulation, update the motor sim
    if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
      // Get the voltage being applied
      appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();

      // Update motor simulation
      motorSim.setInputVoltage(appliedVolts);
      motorSim.update(0.02); // 20ms update period

      // Get simulated position in degrees
      double simPositionDegrees = Math.toDegrees(motorSim.getAngularPositionRad());
      double simVelocityDegreesPerSec = Math.toDegrees(motorSim.getAngularVelocityRadPerSec());

      inputs.currentAngleDegrees = simPositionDegrees;
      inputs.velocityDegreesPerSec = simVelocityDegreesPerSec;
      inputs.appliedVolts = appliedVolts;
      inputs.currentAmps = motorSim.getCurrentDrawAmps();
    } else {
      // Real hardware
      inputs.currentAngleDegrees = encoder.getPosition();
      inputs.velocityDegreesPerSec = encoder.getVelocity();
      inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
      inputs.currentAmps = motor.getOutputCurrent();
      inputs.temperatureCelsius = motor.getMotorTemperature();
    }

    inputs.targetAngleDegrees = targetAngleDegrees;
  }

  @Override
  public void setTargetAngle(double angleDegrees) {
    // Clamp angle to valid range
    targetAngleDegrees =
        MathUtil.clamp(
            angleDegrees, TurretConstants.MIN_ANGLE_DEGREES, TurretConstants.MAX_ANGLE_DEGREES);

    // Use MAXMotion for smoother motion profiling
    closedLoopController.setReference(
        targetAngleDegrees,
        SparkMax.ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,
        0.0, // Arbitrary feedforward
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    motor.stopMotor();
    targetAngleDegrees = encoder.getPosition();
  }

  @Override
  public void resetEncoder(double angleDegrees) {
    encoder.setPosition(angleDegrees);

    if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
      motorSim.setState(Math.toRadians(angleDegrees), 0.0);
    }
  }

  @Override
  public void setBrakeMode(boolean enable) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
