package frc.robot.subsystems.motivator;

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
 * MotivatorIO implementation using three SparkFlex controllers with NEO Vortex motors. All three
 * motors are independently controlled with their own PID loops.
 *
 * <p>Hardware configuration:
 *
 * <ul>
 *   <li>Motor 55: Motivator motor 1 (independent)
 * </ul>
 */
public class MotivatorIOSparkMax implements MotivatorIO {
  private final RobotConfig config;

  // private final PowerDistribution pdh;

  // Hardware - Motivator motor 1 (CAN ID 55)
  private final SparkMax motivator;
  private final RelativeEncoder motivatorEncoder;
  private final SparkClosedLoopController motivatorController;
  private double motivatorMotorRPM = 0.0;

  // Connection debouncers
  private final Debouncer motivatorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Velocity tolerance for at-setpoint check (set by subsystem via
  // setVelocityTolerances)
  private double motivatorToleranceRPM = 20.0;

  // Feedforward controller (rebuilt when gains change via configureFeedforward)
  private SimpleMotorFeedforward feedforward;

  // Target tracking
  private double wheelTargetRPM = 0.0;
  private boolean motivatorVelocityMode = false;

  public MotivatorIOSparkMax() {
    config = Constants.getRobotConfig();

    // Create PDH for independent current monitoring
    // pdh = new PowerDistribution();

    // Create SparkFlex controllers
    motivator = new SparkMax(config.getMotivatorCanId(), MotorType.kBrushless);

    // Get encoders and closed-loop controllers
    motivatorEncoder = motivator.getEncoder();
    motivatorController = motivator.getClosedLoopController();

    // Configure motivator 1 (CAN ID 55)
    double initKp = config.getMotivatorKp();
    double initKi = config.getMotivatorKi();
    double initKd = config.getMotivatorKd();
    double initKs = config.getMotivatorKs();
    double initKv = config.getMotivatorKv();

    feedforward = new SimpleMotorFeedforward(initKs, initKv);

    var motor1Config = new SparkMaxConfig();
    motor1Config
        .inverted(true) // TODO: Make robot-specific when MainBot is ready
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(config.getMotivatorCurrentLimit())
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
        motivator,
        5,
        () ->
            motivator.configure(
                motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Startup diagnostics
    System.out.println("[MotivatorIOSparkFlex] ========== STARTUP ==========");
    System.out.println("[MotivatorIOSparkFlex] Motivator 1 CAN ID: " + config.getMotivatorCanId());
    System.out.println(
        "[MotivatorIOSparkFlex] Motivator Current Limit: " + config.getMotivatorCurrentLimit());
    System.out.println("[MotivatorIOSparkFlex] ==============================");
  }

  /** Convert motor RPM to wheel RPM */
  private double motorToWheelRPM(double motorRPM) {
    return motorRPM * config.getMotivatorGearRatio();
  }

  /** Convert wheel RPM to motor RPM */
  private double wheelToMotorRPM(double wheelRPM) {
    return wheelRPM / config.getMotivatorGearRatio();
  }

  @Override
  public void updateInputs(MotorInputs motor1Inputs) {
    // Update motivator 1 inputs
    sparkStickyFault = false;
    ifOk(motivator, motivatorEncoder::getVelocity, (value) -> motivatorMotorRPM = value);
    motor1Inputs.wheelRPM = motorToWheelRPM(motivatorMotorRPM);
    ifOk(
        motivator,
        new DoubleSupplier[] {motivator::getAppliedOutput, motivator::getBusVoltage},
        (values) -> motor1Inputs.appliedVolts = values[0] * values[1]);
    ifOk(motivator, motivator::getOutputCurrent, (value) -> motor1Inputs.currentAmps = value);
    ifOk(motivator, motivator::getMotorTemperature, (value) -> motor1Inputs.tempCelsius = value);
    // motor1Inputs.PdhCurrentAmps = pdh.getCurrent(2);
    motor1Inputs.connected = motivatorConnectedDebounce.calculate(!sparkStickyFault);

    // Velocity control status
    motor1Inputs.targetRPM = wheelTargetRPM;

    motor1Inputs.atSetpoint =
        motivatorVelocityMode
            && Math.abs(motor1Inputs.wheelRPM - wheelTargetRPM) < this.motivatorToleranceRPM;
  }

  @Override
  public void setMotivatorVoltage(double volts) {
    // Clear velocity target when in voltage mode (for SysId characterization)
    wheelTargetRPM = 0.0;
    motivatorController.setSetpoint(volts, ControlType.kVoltage);
  }

  // ========== Velocity Control ==========

  @Override
  public void setMotivatorVelocity(double wheelVelocityRPM) {
    motivatorVelocityMode = true;
    wheelTargetRPM = Math.abs(wheelVelocityRPM);
    // ks & kv were calculated in motor RPM thus the conversion in the parameter
    double arbFFVolts = feedforward.calculate(wheelToMotorRPM(wheelTargetRPM));
    // PID control is also in motor rotations
    motivatorController.setSetpoint(
        wheelToMotorRPM(wheelTargetRPM), ControlType.kVelocity, ClosedLoopSlot.kSlot0, arbFFVolts);
  }

  @Override
  public void stopMotivator() {
    motivatorVelocityMode = false;
    wheelTargetRPM = 0.0;
    motivator.stopMotor();
  }

  // ========== Configuration Methods ==========

  @Override
  public void configureMotivatorPID(double kP, double kI, double kD, double kS, double kV) {
    var pidConfig = new SparkMaxConfig();
    pidConfig.closedLoop.pid(kP, kI, kD);
    motivator.configure(
        pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    feedforward.setKs(kS);
    feedforward.setKv(kV);
  }

  @Override
  public void setVelocityTolerance(double motivatorToleranceRPM) {
    this.motivatorToleranceRPM = motivatorToleranceRPM;
  }

  @Override
  public double getFFCharacterizationVelocity() {
    return motivatorMotorRPM;
  }
}
