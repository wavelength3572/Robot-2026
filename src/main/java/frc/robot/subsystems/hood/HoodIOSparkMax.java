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
import frc.robot.util.LoggedTunableNumber;

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

  // Tunable PID gains
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber hoodAngle;

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

    // Initialize tunable numbers from config
    kP = new LoggedTunableNumber("Tuning/Hood/kP", config.getHoodKp());
    kD = new LoggedTunableNumber("Tuning/Hood/kD", config.getHoodKd());

    hoodAngle = new LoggedTunableNumber("Tuning/Hood/Hood Angle", 15);

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
        .pid(kP.get(), 0.0, kD.get());

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

    if (LoggedTunableNumber.hasChanged(kP, kD)) {
      var pidConfig = new SparkMaxConfig();
      pidConfig.closedLoop.pid(kP.get(), 0.0, kD.get());
      motorSpark.configure(
          pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    if (LoggedTunableNumber.hasChanged(hoodAngle)) {
      setAngle(hoodAngle.get());
    }

    // Connection status
    inputs.connected = false;

    // Position data
    inputs.currentAngleDeg = motorEncoder.getPosition();
    inputs.targetAngleDeg = targetAngle;
    inputs.appliedVolts = motorSpark.getAppliedOutput() * motorSpark.getBusVoltage();
    inputs.currentAmps = motorSpark.getOutputCurrent();
    inputs.tempCelsius = motorSpark.getMotorTemperature();

    // Control state
    inputs.atTarget = false;
  }

  /** Convert hood degrees to motor rotations (applies software inversion if configured). */
  private double degreesToMotorRotations(double degrees) {
    double rotations = degrees * 1;
    return rotations;
  }

  /** Convert motor rotations to hood degrees (applies software inversion if configured). */
  private double motorRotationsToDegrees(double rotations) {
    double degrees = rotations * 1;
    return degrees;
  }

  @Override
  public void setAngle(double angleDeg) {
    // Clamp the target angle to valid range
    // double clampedDegrees =
    // Math.max(minAngleDegrees, Math.min(maxAngleDegrees, rotation.getDegrees()));
    // targetAngle = Rotation2d.fromDegrees(clampedDegrees);

    // Convert degrees to motor rotations for the PID controller
    targetAngle = angleDeg;
    motorController.setSetpoint(angleDeg, ControlType.kPosition);
  }

  @Override
  public void stop() {}
}
