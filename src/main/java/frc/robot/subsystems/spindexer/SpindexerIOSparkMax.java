package frc.robot.subsystems.spindexer;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import java.util.function.DoubleSupplier;

/**
 * SpindexerIO implementation using a SparkMax controller with a NEO motor.
 *
 * <p>Hardware configuration:
 *
 * <ul>
 *   <li>Motor 55: Spindexer motor (independent)
 * </ul>
 */
public class SpindexerIOSparkMax implements SpindexerIO {
  private final RobotConfig config;

  // private final PowerDistribution pdh;

  // Hardware - Spindexer motor 1 (CAN ID 55)
  private final SparkMax spindexer;
  private final RelativeEncoder spindexerEncoder;
  private final SparkClosedLoopController spindexerController;
  private double spindexerMotorRPM = 0.0;

  // Connection debouncers
  private final Debouncer spindexerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Velocity tolerance for at-setpoint check (set by subsystem via
  // setVelocityTolerances)
  private double spindexerToleranceRPM = 20.0;

  // Feedforward controller (rebuilt when gains change via configureFeedforward)
  private SimpleMotorFeedforward feedforward;

  // Target tracking
  private double wheelTargetRPM = 0.0;
  private boolean spindexerVelocityMode = false;

  public SpindexerIOSparkMax() {
    config = Constants.getRobotConfig();

    // Create PDH for independent current monitoring
    // pdh = new PowerDistribution();

    // Create SparkFlex controllers
    spindexer = new SparkMax(config.getSpindexerCanId(), MotorType.kBrushless);

    // Get encoders and closed-loop controllers
    spindexerEncoder = spindexer.getEncoder();
    spindexerController = spindexer.getClosedLoopController();

    // Configure spindexer 1 (CAN ID 55)
    double initKp = config.getSpindexerKp();
    double initKi = config.getSpindexerKi();
    double initKd = config.getSpindexerKd();
    double initKs = config.getSpindexerKs();
    double initKv = config.getSpindexerKv();

    feedforward = new SimpleMotorFeedforward(initKs, initKv);

    var motor1Config = new SparkMaxConfig();
    motor1Config
        .inverted(false) // TODO: Make robot-specific when MainBot is ready
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(config.getSpindexerCurrentLimit())
        .voltageCompensation(12.0);

    motor1Config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(initKp, initKi, initKd);

    motor1Config
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        spindexer,
        5,
        () ->
            spindexer.configure(
                motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Startup diagnostics
    System.out.println("[SpindexerIOSparkFlex] ========== STARTUP ==========");
    System.out.println("[SpindexerIOSparkFlex] Spindexer 1 CAN ID: " + config.getSpindexerCanId());
    System.out.println(
        "[SpindexerIOSparkFlex] Spindexer Current Limit: " + config.getSpindexerCurrentLimit());
    System.out.println("[SpindexerIOSparkFlex] ==============================");
  }

  /** Convert motor RPM to wheel RPM */
  private double motorToWheelRPM(double motorRPM) {
    return motorRPM * config.getSpindexerGearRatio();
  }

  /** Convert wheel RPM to motor RPM */
  private double wheelToMotorRPM(double wheelRPM) {
    return wheelRPM / config.getSpindexerGearRatio();
  }

  @Override
  public void updateInputs(MotorInputs motor1Inputs) {
    // Update spindexer 1 inputs
    sparkStickyFault = false;
    ifOk(spindexer, spindexerEncoder::getVelocity, (value) -> spindexerMotorRPM = value);
    motor1Inputs.wheelRPM = motorToWheelRPM(spindexerMotorRPM);
    ifOk(
        spindexer,
        new DoubleSupplier[] {spindexer::getAppliedOutput, spindexer::getBusVoltage},
        (values) -> motor1Inputs.appliedVolts = values[0] * values[1]);
    ifOk(spindexer, spindexer::getOutputCurrent, (value) -> motor1Inputs.currentAmps = value);
    ifOk(spindexer, spindexer::getMotorTemperature, (value) -> motor1Inputs.tempCelsius = value);
    // motor1Inputs.PdhCurrentAmps = pdh.getCurrent(2);
    motor1Inputs.connected = spindexerConnectedDebounce.calculate(!sparkStickyFault);

    // Velocity control status
    motor1Inputs.targetRPM = wheelTargetRPM;

    motor1Inputs.atSetpoint =
        spindexerVelocityMode
            && Math.abs(motor1Inputs.wheelRPM - wheelTargetRPM) < this.spindexerToleranceRPM;
  }

  @Override
  public void setSpindexerVoltage(double volts) {
    // Clear velocity target when in voltage mode (for SysId characterization)
    wheelTargetRPM = 0.0;
    spindexerController.setSetpoint(volts, ControlType.kVoltage);
  }

  // ========== Velocity Control ==========

  @Override
  public void setSpindexerVelocity(double wheelVelocityRPM) {
    spindexerVelocityMode = true;
    wheelTargetRPM = wheelVelocityRPM;
    // ks & kv were calculated in motor RPM thus the conversion in the parameter.
    // Use Math.abs for feedforward magnitude, then apply sign to match direction.
    double motorRPM = wheelToMotorRPM(wheelTargetRPM);
    double arbFFVolts = Math.copySign(feedforward.calculate(Math.abs(motorRPM)), motorRPM);
    // PID control is also in motor rotations
    spindexerController.setSetpoint(
        motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0, arbFFVolts);
  }

  @Override
  public void stopSpindexer() {
    spindexerVelocityMode = false;
    wheelTargetRPM = 0.0;
    spindexer.stopMotor();
  }

  // ========== Configuration Methods ==========

  @Override
  public void configureSpindexerPID(double kP, double kI, double kD, double kS, double kV) {
    var pidConfig = new SparkMaxConfig();
    pidConfig.closedLoop.pid(kP, kI, kD);
    spindexer.configure(
        pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    feedforward.setKs(kS);
    feedforward.setKv(kV);
  }

  @Override
  public void setVelocityTolerance(double spindexerToleranceRPM) {
    this.spindexerToleranceRPM = spindexerToleranceRPM;
  }

  @Override
  public double getFFCharacterizationVelocity() {
    return spindexerMotorRPM;
  }
}
