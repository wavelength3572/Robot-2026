package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import java.util.function.DoubleSupplier;

public class IntakeIOSparkMax implements IntakeIO {
  // Hardware
  private final SparkMax deployMotor;
  private final SparkMax rollerMotor;
  private final RelativeEncoder deployEncoder;
  private final RelativeEncoder rollerEncoder;
  private final SparkClosedLoopController deployController;
  private final SparkClosedLoopController rollerController;

  // Connection debouncers
  private final Debouncer deployConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer rollerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Target tracking
  private double deployTargetPosition = 0.0;
  private double rollerTargetSpeed = 0.0;

  // Deploy position limits (from config)
  private final double deployRetractedPosition;
  private final double deployExtendedPosition;

  public IntakeIOSparkMax() {
    RobotConfig config = Constants.getRobotConfig();

    deployRetractedPosition = config.getIntakeDeployRetractedPosition();
    deployExtendedPosition = config.getIntakeDeployExtendedPosition();

    // Create motors
    deployMotor = new SparkMax(config.getIntakeDeployMotorCanId(), MotorType.kBrushless);
    rollerMotor = new SparkMax(config.getIntakeRollerMotorCanId(), MotorType.kBrushless);

    // Get encoders and controllers
    deployEncoder = deployMotor.getEncoder();
    rollerEncoder = rollerMotor.getEncoder();
    deployController = deployMotor.getClosedLoopController();
    rollerController = rollerMotor.getClosedLoopController();

    // Configure deploy motor
    var deployConfig = new SparkMaxConfig();
    deployConfig
        .inverted(config.getIntakeDeployMotorInverted())
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(config.getIntakeDeployCurrentLimit())
        .voltageCompensation(12.0);
    deployConfig
        .encoder
        .positionConversionFactor(1.0 / config.getIntakeDeployGearRatio()) // Output rotations
        .velocityConversionFactor(1.0 / config.getIntakeDeployGearRatio()); // Output RPM
    deployConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(config.getIntakeDeployKp(), config.getIntakeDeployKi(), config.getIntakeDeployKd());
    deployConfig
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit((float) Math.max(deployExtendedPosition, deployRetractedPosition))
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit((float) Math.min(deployExtendedPosition, deployRetractedPosition));
    deployConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        deployMotor,
        5,
        () ->
            deployMotor.configure(
                deployConfig,
                com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters));
    tryUntilOk(deployMotor, 5, () -> deployEncoder.setPosition(0.0));

    // Configure roller motor
    var rollerConfig = new SparkMaxConfig();
    rollerConfig
        .inverted(config.getIntakeRollerMotorInverted())
        .idleMode(IdleMode.kCoast) // Coast for rollers
        .smartCurrentLimit(config.getIntakeRollerCurrentLimit())
        .voltageCompensation(12.0);
    rollerConfig
        .encoder
        .positionConversionFactor(1.0 / config.getIntakeRollerGearRatio())
        .velocityConversionFactor(1.0 / config.getIntakeRollerGearRatio());
    rollerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(config.getIntakeRollerKp(), config.getIntakeRollerKi(), config.getIntakeRollerKd())
        .feedForward
        .kV(config.getIntakeRollerKff());
    rollerConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        rollerMotor,
        5,
        () ->
            rollerMotor.configure(
                rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Update deploy motor inputs
    sparkStickyFault = false;
    ifOk(
        deployMotor, deployEncoder::getPosition, (value) -> inputs.deployPositionRotations = value);
    ifOk(deployMotor, deployEncoder::getVelocity, (value) -> inputs.deployVelocityRPM = value);
    ifOk(
        deployMotor,
        new DoubleSupplier[] {deployMotor::getAppliedOutput, deployMotor::getBusVoltage},
        (values) -> inputs.deployAppliedVolts = values[0] * values[1]);
    ifOk(deployMotor, deployMotor::getOutputCurrent, (value) -> inputs.deployCurrentAmps = value);
    inputs.deployConnected = deployConnectedDebounce.calculate(!sparkStickyFault);
    inputs.deployTargetPosition = deployTargetPosition;

    // Update roller motor inputs
    sparkStickyFault = false;
    ifOk(rollerMotor, rollerEncoder::getVelocity, (value) -> inputs.rollerVelocityRPM = value);
    ifOk(
        rollerMotor,
        new DoubleSupplier[] {rollerMotor::getAppliedOutput, rollerMotor::getBusVoltage},
        (values) -> inputs.rollerAppliedVolts = values[0] * values[1]);
    ifOk(rollerMotor, rollerMotor::getOutputCurrent, (value) -> inputs.rollerCurrentAmps = value);
    inputs.rollerConnected = rollerConnectedDebounce.calculate(!sparkStickyFault);
    inputs.rollerTargetSpeed = rollerTargetSpeed;
  }

  @Override
  public void setDeployPosition(double positionRotations) {
    // Clamp to valid range
    double minPos = Math.min(deployRetractedPosition, deployExtendedPosition);
    double maxPos = Math.max(deployRetractedPosition, deployExtendedPosition);
    deployTargetPosition = Math.max(minPos, Math.min(maxPos, positionRotations));
    deployController.setSetpoint(deployTargetPosition, ControlType.kPosition);
  }

  @Override
  public void setRollerDutyCycle(double dutyCycle) {
    rollerTargetSpeed = dutyCycle;
    rollerMotor.set(dutyCycle);
  }

  @Override
  public void setRollerVelocity(double rpm) {
    rollerTargetSpeed = rpm;
    rollerController.setSetpoint(rpm, ControlType.kVelocity);
  }

  @Override
  public void stop() {
    deployMotor.stopMotor();
    rollerMotor.stopMotor();
    deployTargetPosition = deployEncoder.getPosition();
    rollerTargetSpeed = 0.0;
  }

  @Override
  public void setDeployBrakeMode(boolean brake) {
    var config = new SparkMaxConfig();
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    deployMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void configureDeployPID(double kP, double kI, double kD) {
    var config = new SparkMaxConfig();
    config.closedLoop.pid(kP, kI, kD);
    deployMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void configureRollerPID(double kP, double kI, double kD, double kFF) {
    var config = new SparkMaxConfig();
    config.closedLoop.pid(kP, kI, kD).feedForward.kV(kFF);
    rollerMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
