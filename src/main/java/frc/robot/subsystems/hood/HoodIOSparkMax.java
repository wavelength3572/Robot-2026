package frc.robot.subsystems.hood;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.RobotConfig;

public class HoodIOSparkMax implements HoodIO {
  private final RobotConfig config;

  // Hardware
  private final SparkMax motorSpark;
  private final RelativeEncoder motorEncoder;
  private final SparkClosedLoopController motorController;
  private final boolean motorInverted;
  private final double maxAngleDegrees;
  private final double minAngleDegrees;

  private double targetAngle;
  private double toleranceDeg = 1.0;

  public HoodIOSparkMax() {
    config = Constants.getRobotConfig();

    // Create Spark Max controller
    motorSpark = new SparkMax(config.getHoodCanId(), MotorType.kBrushless);

    // Get encoders and controller
    motorEncoder = motorSpark.getEncoder();

    motorController = motorSpark.getClosedLoopController();

    motorInverted = config.getHoodMotorInverted();

    maxAngleDegrees = config.getHoodMaxAngleDegrees();

    minAngleDegrees = config.getHoodMinAngleDegrees();

    var motorConfig = new SparkMaxConfig();
    motorConfig
        .smartCurrentLimit(config.getHoodCurrentLimitAmps())
        .voltageCompensation(12.0)
        .inverted(motorInverted)
        .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast)
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit((float) degreesToMotorRotations(maxAngleDegrees))
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit((float) degreesToMotorRotations(minAngleDegrees));

    // PID control using motor's relative encoder (units: motor rotations)
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(config.getHoodKp(), 0.0, config.getHoodKd());

    tryUntilOk(
        motorSpark,
        5,
        () ->
            motorSpark.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(HoodIOInputs inputs) {

    // Connection status
    inputs.connected = true;

    // Position data
    inputs.currentMotorRotations = motorEncoder.getPosition();
    inputs.currentAngleDeg = motorRotationsToDegrees(motorEncoder.getPosition());
    inputs.targetAngleDeg = targetAngle;
    inputs.targetMotorRotations = degreesToMotorRotations(targetAngle);
    inputs.appliedVolts = motorSpark.getAppliedOutput() * motorSpark.getBusVoltage();
    inputs.currentAmps = motorSpark.getOutputCurrent();
    inputs.tempCelsius = motorSpark.getMotorTemperature();

    // Control state
    inputs.atTarget = Math.abs(inputs.currentAngleDeg - targetAngle) < toleranceDeg;
  }

  /** Convert hood degrees to motor rotations (applies software inversion if configured). */
  private double degreesToMotorRotations(double degrees) {
    double rotations = (degrees * 3.0) - 38.0;
    return rotations;
  }

  /** Convert motor rotations to hood degrees (applies software inversion if configured). */
  private double motorRotationsToDegrees(double rotations) {
    double degrees = (rotations + 38) / 3.0;
    return degrees;
  }

  @Override
  public void configurePID(double kP, double kD) {
    var pidConfig = new SparkMaxConfig();
    pidConfig.closedLoop.pid(kP, 0.0, kD);
    motorSpark.configure(
        pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setAngleTolerance(double toleranceDeg) {
    this.toleranceDeg = toleranceDeg;
  }

  @Override
  public void setAngle(double angleDeg) {
    // Clamp the target angle to valid range
    // double clampedDegrees =
    // Math.max(minAngleDegrees, Math.min(maxAngleDegrees, rotation.getDegrees()));
    // targetAngle = Rotation2d.fromDegrees(clampedDegrees);

    // Convert degrees to motor rotations for the PID controller
    targetAngle = angleDeg;
    motorController.setSetpoint(degreesToMotorRotations(targetAngle), ControlType.kPosition);
  }
}
