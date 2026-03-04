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
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import java.util.function.DoubleSupplier;

/**
 * Roller-only intake IO for robots without a deploy mechanism (e.g., SQUAREBOT). Only drives the
 * roller SparkMax; deploy methods are no-ops. Reports deploy position as fully extended so
 * isDeployed() always returns true.
 */
public class IntakeIOSparkMaxRollerOnly implements IntakeIO {
  private final SparkMax rollerMotor;
  private final RelativeEncoder rollerEncoder;
  private final SparkClosedLoopController rollerController;
  private final PowerDistribution pdh;

  private final Debouncer rollerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private double rollerTargetSpeed = 0.0;

  // Deploy extended position (from config, for reporting as always deployed)
  private final double deployExtendedPosition;

  public IntakeIOSparkMaxRollerOnly() {
    RobotConfig config = Constants.getRobotConfig();

    deployExtendedPosition = config.getIntakeDeployExtendedPosition();

    rollerMotor = new SparkMax(config.getIntakeRollerMotorCanId(), MotorType.kBrushless);
    rollerEncoder = rollerMotor.getEncoder();
    rollerController = rollerMotor.getClosedLoopController();

    var rollerConfig = new SparkMaxConfig();
    rollerConfig
        .inverted(config.getIntakeRollerMotorInverted())
        .idleMode(IdleMode.kCoast)
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

    // Create PDH for independent current monitoring
    pdh = new PowerDistribution();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // No deploy motor — report as always fully extended
    inputs.deployConnected = true;
    inputs.deployPositionRotations = deployExtendedPosition;
    inputs.deployTargetPosition = deployExtendedPosition;

    // Roller motor inputs
    sparkStickyFault = false;
    ifOk(rollerMotor, rollerEncoder::getVelocity, (value) -> inputs.rollerVelocityRPM = value);
    ifOk(
        rollerMotor,
        new DoubleSupplier[] {rollerMotor::getAppliedOutput, rollerMotor::getBusVoltage},
        (values) -> inputs.rollerAppliedVolts = values[0] * values[1]);
    ifOk(rollerMotor, rollerMotor::getOutputCurrent, (value) -> inputs.rollerCurrentAmps = value);
    inputs.rollerPdhCurrentAmps = pdh.getCurrent(12);
    inputs.rollerPdhVoltage = pdh.getVoltage();
    inputs.rollerConnected = rollerConnectedDebounce.calculate(!sparkStickyFault);
    inputs.rollerTargetSpeed = rollerTargetSpeed;
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
    rollerMotor.stopMotor();
    rollerTargetSpeed = 0.0;
  }

  @Override
  public void configureRollerPID(double kP, double kI, double kD, double kFF) {
    var config = new SparkMaxConfig();
    config.closedLoop.pid(kP, kI, kD).feedForward.kV(kFF);
    rollerMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
