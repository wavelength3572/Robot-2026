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
import frc.robot.util.SparkConnection;

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
  private int periodicCounter = 0;

  // Skip CAN reads when motor is disconnected to prevent loop overruns
  private final SparkConnection hoodConnection = new SparkConnection();

  // Closed-loop output limit (fraction of full output, ±).
  private static double constCurrentOutputLimit = 1.0;

  private static final LoggedTunableNumber hoodOutputLimit =
      new LoggedTunableNumber("Tuning/Hood/outputLimit", constCurrentOutputLimit);

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

    // PID + kS static-friction feedforward (native REV 2026 onboard).
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(config.getHoodKp(), 0.0, config.getHoodKd())
        .outputRange(-constCurrentOutputLimit, constCurrentOutputLimit);
    motorConfig.closedLoop.feedForward.kS(config.getHoodKs());

    // Signal update rates - slow down diagnostic frames to reduce CAN bus load
    motorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .appliedOutputPeriodMs(100)
        .busVoltagePeriodMs(100)
        .outputCurrentPeriodMs(100);

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
    if (LoggedTunableNumber.hasChanged(hoodOutputLimit)) {
      changeLimits(hoodOutputLimit.get());
    }

    inputs.targetAngleDeg = targetAngle;
    inputs.targetMotorRotations = degreesToMotorRotations(targetAngle);

    // Skip CAN reads if motor is disconnected to prevent timeout delays
    if (!hoodConnection.isSkipping()) {
      inputs.currentMotorRotations = motorEncoder.getPosition();
      inputs.currentAngleDeg = motorRotationsToDegrees(inputs.currentMotorRotations);
      inputs.appliedVolts = motorSpark.getAppliedOutput() * motorSpark.getBusVoltage();
      inputs.currentAmps = motorSpark.getOutputCurrent();
      inputs.connected = motorSpark.getLastError() == com.revrobotics.REVLibError.kOk;
      hoodConnection.update(inputs.connected);

      // Throttle temperature reads to every ~1 second (50 cycles at 20ms)
      if (++periodicCounter >= 50) {
        periodicCounter = 0;
        inputs.tempCelsius = motorSpark.getMotorTemperature();
      }
    } else {
      inputs.connected = false;
    }

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
  public void setKs(double ks) {
    var ksConfig = new SparkMaxConfig();
    ksConfig.closedLoop.feedForward.kS(ks);
    motorSpark.configure(
        ksConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void changeLimits(double outputLimit) {
    var limitConfig = new SparkMaxConfig();
    limitConfig.closedLoop.outputRange(-outputLimit, outputLimit);
    tryUntilOk(
        motorSpark,
        5,
        () ->
            motorSpark.configure(
                limitConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void setAngleTolerance(double toleranceDeg) {
    this.toleranceDeg = toleranceDeg;
  }

  @Override
  public void setAngle(double angleDeg) {
    targetAngle = angleDeg;
    motorController.setSetpoint(degreesToMotorRotations(targetAngle), ControlType.kPosition);
  }

  @Override
  public void setHoodVolts(double volts) {
    if (volts > .5) volts = .5;
    motorController.setSetpoint(volts, ControlType.kVoltage);
  }
}
