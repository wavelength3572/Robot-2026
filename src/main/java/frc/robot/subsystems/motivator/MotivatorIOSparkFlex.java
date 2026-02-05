package frc.robot.subsystems.motivator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

/**
 * MotivatorIO implementation using three SparkFlex controllers with NEO Vortex motors. All three
 * motors are independently controlled with their own PID loops.
 *
 * <p>Hardware configuration:
 *
 * <ul>
 *   <li>Motor 55: Motivator motor 1 (independent)
 *   <li>Motor 56: Motivator motor 2 (independent)
 *   <li>Motor 57: Prefeed roller (independent)
 * </ul>
 */
public class MotivatorIOSparkFlex implements MotivatorIO {
  private final RobotConfig config;

  // Hardware - Motivator motor 1 (CAN ID 55)
  private final SparkFlex motivator1;
  private final RelativeEncoder motivator1Encoder;
  private final SparkClosedLoopController motivator1Controller;

  // Hardware - Motivator motor 2 (CAN ID 56)
  private final SparkFlex motivator2;
  private final RelativeEncoder motivator2Encoder;
  private final SparkClosedLoopController motivator2Controller;

  // Hardware - Prefeed roller (CAN ID 57)
  private final SparkFlex prefeedMotor;
  private final RelativeEncoder prefeedEncoder;
  private final SparkClosedLoopController prefeedController;

  // Connection debouncers
  private final Debouncer motivator1ConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer motivator2ConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer prefeedConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Tunable PID gains for motivator 1
  private final LoggedTunableNumber motivator1Kp =
      new LoggedTunableNumber("Tuning/Motivator/Motor1_kP", 0.0001);
  private final LoggedTunableNumber motivator1Ki =
      new LoggedTunableNumber("Tuning/Motivator/Motor1_kI", 0.0);
  private final LoggedTunableNumber motivator1Kd =
      new LoggedTunableNumber("Tuning/Motivator/Motor1_kD", 0.0);
  private final LoggedTunableNumber motivator1Kff =
      new LoggedTunableNumber("Tuning/Motivator/Motor1_kFF", 0.00018);

  // Tunable PID gains for motivator 2
  private final LoggedTunableNumber motivator2Kp =
      new LoggedTunableNumber("Tuning/Motivator/Motor2_kP", 0.0001);
  private final LoggedTunableNumber motivator2Ki =
      new LoggedTunableNumber("Tuning/Motivator/Motor2_kI", 0.0);
  private final LoggedTunableNumber motivator2Kd =
      new LoggedTunableNumber("Tuning/Motivator/Motor2_kD", 0.0);
  private final LoggedTunableNumber motivator2Kff =
      new LoggedTunableNumber("Tuning/Motivator/Motor2_kFF", 0.00018);

  // Tunable PID gains for prefeed
  private final LoggedTunableNumber prefeedKp =
      new LoggedTunableNumber("Tuning/Motivator/Prefeed_kP", 0.0001);
  private final LoggedTunableNumber prefeedKi =
      new LoggedTunableNumber("Tuning/Motivator/Prefeed_kI", 0.0);
  private final LoggedTunableNumber prefeedKd =
      new LoggedTunableNumber("Tuning/Motivator/Prefeed_kD", 0.0);
  private final LoggedTunableNumber prefeedKff =
      new LoggedTunableNumber("Tuning/Motivator/Prefeed_kFF", 0.00018);

  // Velocity tolerance for at-setpoint check
  private static final LoggedTunableNumber velocityToleranceRPM =
      new LoggedTunableNumber("Tuning/Motivator/VelocityToleranceRPM", 100.0);

  // Target tracking
  private double motivator1TargetRPM = 0.0;
  private double motivator2TargetRPM = 0.0;
  private double prefeedTargetRPM = 0.0;
  private boolean motivator1VelocityMode = false;
  private boolean motivator2VelocityMode = false;
  private boolean prefeedVelocityMode = false;

  public MotivatorIOSparkFlex() {
    config = Constants.getRobotConfig();

    // Create SparkFlex controllers
    motivator1 = new SparkFlex(config.getMotivatorLeaderCanId(), MotorType.kBrushless);
    motivator2 = new SparkFlex(config.getMotivatorFollowerCanId(), MotorType.kBrushless);
    prefeedMotor = new SparkFlex(config.getPrefeedCanId(), MotorType.kBrushless);

    // Get encoders and closed-loop controllers
    motivator1Encoder = motivator1.getEncoder();
    motivator2Encoder = motivator2.getEncoder();
    prefeedEncoder = prefeedMotor.getEncoder();
    motivator1Controller = motivator1.getClosedLoopController();
    motivator2Controller = motivator2.getClosedLoopController();
    prefeedController = prefeedMotor.getClosedLoopController();

    // Configure motivator 1 (CAN ID 55)
    var motor1Config = new SparkFlexConfig();
    motor1Config
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(config.getMotivatorCurrentLimit())
        .voltageCompensation(12.0);

    motor1Config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(motivator1Kp.get(), motivator1Ki.get(), motivator1Kd.get(), motivator1Kff.get());

    motor1Config
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        motivator1,
        5,
        () ->
            motivator1.configure(
                motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Configure motivator 2 (CAN ID 56) - independent, not a follower
    var motor2Config = new SparkFlexConfig();
    motor2Config
        .inverted(false) // Inverted since it faces opposite direction
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(config.getMotivatorCurrentLimit())
        .voltageCompensation(12.0);

    motor2Config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(motivator2Kp.get(), motivator2Ki.get(), motivator2Kd.get(), motivator2Kff.get());

    motor2Config
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        motivator2,
        5,
        () ->
            motivator2.configure(
                motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Configure prefeed motor (CAN ID 57)
    var prefeedConfig = new SparkFlexConfig();
    prefeedConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(config.getPrefeedCurrentLimit())
        .voltageCompensation(12.0);

    prefeedConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(prefeedKp.get(), prefeedKi.get(), prefeedKd.get(), prefeedKff.get());

    prefeedConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        prefeedMotor,
        5,
        () ->
            prefeedMotor.configure(
                prefeedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Startup diagnostics
    System.out.println("[MotivatorIOSparkFlex] ========== STARTUP ==========");
    System.out.println(
        "[MotivatorIOSparkFlex] Motivator 1 CAN ID: " + config.getMotivatorLeaderCanId());
    System.out.println(
        "[MotivatorIOSparkFlex] Motivator 2 CAN ID: " + config.getMotivatorFollowerCanId());
    System.out.println("[MotivatorIOSparkFlex] Prefeed CAN ID: " + config.getPrefeedCanId());
    System.out.println(
        "[MotivatorIOSparkFlex] Motivator Current Limit: " + config.getMotivatorCurrentLimit());
    System.out.println(
        "[MotivatorIOSparkFlex] Prefeed Current Limit: " + config.getPrefeedCurrentLimit());
    System.out.println("[MotivatorIOSparkFlex] All motors configured as INDEPENDENT (no follower)");
    System.out.println("[MotivatorIOSparkFlex] ==============================");
  }

  @Override
  public void updateInputs(MotivatorIOInputs inputs) {
    // Check for tunable PID changes and apply to motivator 1
    if (LoggedTunableNumber.hasChanged(motivator1Kp, motivator1Ki, motivator1Kd, motivator1Kff)) {
      var pidConfig = new SparkFlexConfig();
      pidConfig.closedLoop.pidf(
          motivator1Kp.get(), motivator1Ki.get(), motivator1Kd.get(), motivator1Kff.get());
      motivator1.configure(
          pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Check for tunable PID changes and apply to motivator 2
    if (LoggedTunableNumber.hasChanged(motivator2Kp, motivator2Ki, motivator2Kd, motivator2Kff)) {
      var pidConfig = new SparkFlexConfig();
      pidConfig.closedLoop.pidf(
          motivator2Kp.get(), motivator2Ki.get(), motivator2Kd.get(), motivator2Kff.get());
      motivator2.configure(
          pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Check for tunable PID changes and apply to prefeed
    if (LoggedTunableNumber.hasChanged(prefeedKp, prefeedKi, prefeedKd, prefeedKff)) {
      var pidConfig = new SparkFlexConfig();
      pidConfig.closedLoop.pidf(
          prefeedKp.get(), prefeedKi.get(), prefeedKd.get(), prefeedKff.get());
      prefeedMotor.configure(
          pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Update motivator 1 inputs
    sparkStickyFault = false;
    ifOk(
        motivator1,
        motivator1Encoder::getVelocity,
        (value) -> inputs.motivator1VelocityRPM = value);
    ifOk(
        motivator1,
        new DoubleSupplier[] {motivator1::getAppliedOutput, motivator1::getBusVoltage},
        (values) -> inputs.motivator1AppliedVolts = values[0] * values[1]);
    ifOk(motivator1, motivator1::getOutputCurrent, (value) -> inputs.motivator1CurrentAmps = value);
    ifOk(
        motivator1,
        motivator1::getMotorTemperature,
        (value) -> inputs.motivator1TempCelsius = value);
    inputs.motivator1Connected = motivator1ConnectedDebounce.calculate(!sparkStickyFault);

    // Update motivator 2 inputs
    sparkStickyFault = false;
    ifOk(
        motivator2,
        motivator2Encoder::getVelocity,
        (value) -> inputs.motivator2VelocityRPM = value);
    ifOk(
        motivator2,
        new DoubleSupplier[] {motivator2::getAppliedOutput, motivator2::getBusVoltage},
        (values) -> inputs.motivator2AppliedVolts = values[0] * values[1]);
    ifOk(motivator2, motivator2::getOutputCurrent, (value) -> inputs.motivator2CurrentAmps = value);
    ifOk(
        motivator2,
        motivator2::getMotorTemperature,
        (value) -> inputs.motivator2TempCelsius = value);
    inputs.motivator2Connected = motivator2ConnectedDebounce.calculate(!sparkStickyFault);

    // Update prefeed inputs
    sparkStickyFault = false;
    ifOk(prefeedMotor, prefeedEncoder::getVelocity, (value) -> inputs.prefeedVelocityRPM = value);
    ifOk(
        prefeedMotor,
        new DoubleSupplier[] {prefeedMotor::getAppliedOutput, prefeedMotor::getBusVoltage},
        (values) -> inputs.prefeedAppliedVolts = values[0] * values[1]);
    ifOk(
        prefeedMotor, prefeedMotor::getOutputCurrent, (value) -> inputs.prefeedCurrentAmps = value);
    ifOk(
        prefeedMotor,
        prefeedMotor::getMotorTemperature,
        (value) -> inputs.prefeedTempCelsius = value);
    inputs.prefeedConnected = prefeedConnectedDebounce.calculate(!sparkStickyFault);

    // Velocity control status
    inputs.motivator1TargetVelocityRPM = motivator1TargetRPM;
    inputs.motivator2TargetVelocityRPM = motivator2TargetRPM;
    inputs.prefeedTargetVelocityRPM = prefeedTargetRPM;

    inputs.motivator1AtSetpoint =
        motivator1VelocityMode
            && Math.abs(inputs.motivator1VelocityRPM - motivator1TargetRPM)
                < velocityToleranceRPM.get();
    inputs.motivator2AtSetpoint =
        motivator2VelocityMode
            && Math.abs(inputs.motivator2VelocityRPM - motivator2TargetRPM)
                < velocityToleranceRPM.get();
    inputs.prefeedAtSetpoint =
        prefeedVelocityMode
            && Math.abs(inputs.prefeedVelocityRPM - prefeedTargetRPM) < velocityToleranceRPM.get();

    // Future: ball detection sensor
    inputs.ballDetected = false;
  }

  // ========== Duty Cycle Control ==========

  @Override
  public void setMotivator1DutyCycle(double dutyCycle) {
    motivator1VelocityMode = false;
    motivator1TargetRPM = 0.0;
    motivator1.set(dutyCycle);
  }

  @Override
  public void setMotivator2DutyCycle(double dutyCycle) {
    motivator2VelocityMode = false;
    motivator2TargetRPM = 0.0;
    motivator2.set(dutyCycle);
  }

  @Override
  public void setPrefeedDutyCycle(double dutyCycle) {
    prefeedVelocityMode = false;
    prefeedTargetRPM = 0.0;
    prefeedMotor.set(dutyCycle);
  }

  // ========== Velocity Control ==========

  @Override
  public void setMotivator1Velocity(double velocityRPM) {
    motivator1VelocityMode = true;
    motivator1TargetRPM = Math.abs(velocityRPM);
    motivator1Controller.setReference(
        motivator1TargetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setMotivator2Velocity(double velocityRPM) {
    motivator2VelocityMode = true;
    motivator2TargetRPM = Math.abs(velocityRPM);
    motivator2Controller.setReference(
        motivator2TargetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setPrefeedVelocity(double velocityRPM) {
    prefeedVelocityMode = true;
    prefeedTargetRPM = Math.abs(velocityRPM);
    prefeedController.setReference(prefeedTargetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  // ========== Stop Methods ==========

  @Override
  public void stop() {
    motivator1VelocityMode = false;
    motivator2VelocityMode = false;
    prefeedVelocityMode = false;
    motivator1TargetRPM = 0.0;
    motivator2TargetRPM = 0.0;
    prefeedTargetRPM = 0.0;
    motivator1.stopMotor();
    motivator2.stopMotor();
    prefeedMotor.stopMotor();
  }

  @Override
  public void stopMotivator1() {
    motivator1VelocityMode = false;
    motivator1TargetRPM = 0.0;
    motivator1.stopMotor();
  }

  @Override
  public void stopMotivator2() {
    motivator2VelocityMode = false;
    motivator2TargetRPM = 0.0;
    motivator2.stopMotor();
  }

  @Override
  public void stopMotivators() {
    motivator1VelocityMode = false;
    motivator2VelocityMode = false;
    motivator1TargetRPM = 0.0;
    motivator2TargetRPM = 0.0;
    motivator1.stopMotor();
    motivator2.stopMotor();
  }

  @Override
  public void stopPrefeed() {
    prefeedVelocityMode = false;
    prefeedTargetRPM = 0.0;
    prefeedMotor.stopMotor();
  }
}
