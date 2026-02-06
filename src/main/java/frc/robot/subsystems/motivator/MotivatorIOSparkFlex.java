package frc.robot.subsystems.motivator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
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
      new LoggedTunableNumber("Tuning/Motivator/Motivator1/kP", 0.000056);
  private final LoggedTunableNumber motivator1Ki =
      new LoggedTunableNumber("Tuning/Motivator/Motivator1/kI", 0.0);
  private final LoggedTunableNumber motivator1Kd =
      new LoggedTunableNumber("Tuning/Motivator/Motivator1/kD", 0.00275);
  private final LoggedTunableNumber motivator1Kff =
      new LoggedTunableNumber("Tuning/Motivator/Motivator1/kFF", 0.00015);

  // Tunable PID gains for motivator 2
  private final LoggedTunableNumber motivator2Kp =
      new LoggedTunableNumber("Tuning/Motivator/Motivator2/kP", 0.000056);
  private final LoggedTunableNumber motivator2Ki =
      new LoggedTunableNumber("Tuning/Motivator/Motivator2/kI", 0.0);
  private final LoggedTunableNumber motivator2Kd =
      new LoggedTunableNumber("Tuning/Motivator/Motivator2/kD", 0.00275);
  private final LoggedTunableNumber motivator2Kff =
      new LoggedTunableNumber("Tuning/Motivator/Motivator2/kFF", 0.0001526);

  // Tunable PID gains for prefeed
  private final LoggedTunableNumber prefeedKp =
      new LoggedTunableNumber("Tuning/Motivator/PreFeed/kP", 0.000101);
  private final LoggedTunableNumber prefeedKi =
      new LoggedTunableNumber("Tuning/Motivator/PreFeed/kI", 0.0);
  private final LoggedTunableNumber prefeedKd =
      new LoggedTunableNumber("Tuning/Motivator/PreFeed/kD", 0.005);
  private final LoggedTunableNumber prefeedKff =
      new LoggedTunableNumber("Tuning/Motivator/PreFeed/kFF", 0.00015);

  // Velocity tolerance for at-setpoint check
  private static final LoggedTunableNumber motivatorToleranceRPM =
      new LoggedTunableNumber("Tuning/Motivator/Motivator1/ToleranceRPM", 100.0);
  private static final LoggedTunableNumber prefeedToleranceRPM =
      new LoggedTunableNumber("Tuning/Motivator/PreFeed/ToleranceRPM", 100.0);

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
        .inverted(false) // TODO: Make robot-specific when MainBot is ready
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(config.getMotivatorCurrentLimit())
        .voltageCompensation(12.0);

    motor1Config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(motivator1Kp.get(), motivator1Ki.get(), motivator1Kd.get())
        .feedForward
        .kV(motivator1Kff.get());

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
        .inverted(true) // TODO: Make robot-specific when MainBot is ready
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(config.getMotivatorCurrentLimit())
        .voltageCompensation(12.0);

    motor2Config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(motivator2Kp.get(), motivator2Ki.get(), motivator2Kd.get())
        .feedForward
        .kV(motivator2Kff.get());

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
        .inverted(false) // TODO: Make robot-specific when MainBot is ready
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(config.getPrefeedCurrentLimit())
        .voltageCompensation(12.0);

    prefeedConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(prefeedKp.get(), prefeedKi.get(), prefeedKd.get())
        .feedForward
        .kV(prefeedKff.get());

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
  public void updateInputs(
      MotorInputs motor1Inputs, MotorInputs motor2Inputs, MotorInputs prefeedInputs) {
    // Check for tunable PID changes and apply to motivator 1
    if (LoggedTunableNumber.hasChanged(motivator1Kp, motivator1Ki, motivator1Kd, motivator1Kff)) {
      var pidConfig = new SparkFlexConfig();
      pidConfig
          .closedLoop
          .pid(motivator1Kp.get(), motivator1Ki.get(), motivator1Kd.get())
          .feedForward
          .kV(motivator1Kff.get());
      motivator1.configure(
          pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Check for tunable PID changes and apply to motivator 2
    if (LoggedTunableNumber.hasChanged(motivator2Kp, motivator2Ki, motivator2Kd, motivator2Kff)) {
      var pidConfig = new SparkFlexConfig();
      pidConfig
          .closedLoop
          .pid(motivator2Kp.get(), motivator2Ki.get(), motivator2Kd.get())
          .feedForward
          .kV(motivator2Kff.get());
      motivator2.configure(
          pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Check for tunable PID changes and apply to prefeed
    if (LoggedTunableNumber.hasChanged(prefeedKp, prefeedKi, prefeedKd, prefeedKff)) {
      var pidConfig = new SparkFlexConfig();
      pidConfig
          .closedLoop
          .pid(prefeedKp.get(), prefeedKi.get(), prefeedKd.get())
          .feedForward
          .kV(prefeedKff.get());
      prefeedMotor.configure(
          pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Update motivator 1 inputs
    sparkStickyFault = false;
    ifOk(motivator1, motivator1Encoder::getVelocity, (value) -> motor1Inputs.velocityRPM = value);
    ifOk(
        motivator1,
        new DoubleSupplier[] {motivator1::getAppliedOutput, motivator1::getBusVoltage},
        (values) -> motor1Inputs.appliedVolts = values[0] * values[1]);
    ifOk(motivator1, motivator1::getOutputCurrent, (value) -> motor1Inputs.currentAmps = value);
    ifOk(motivator1, motivator1::getMotorTemperature, (value) -> motor1Inputs.tempCelsius = value);
    motor1Inputs.connected = motivator1ConnectedDebounce.calculate(!sparkStickyFault);

    // Update motivator 2 inputs
    sparkStickyFault = false;
    ifOk(motivator2, motivator2Encoder::getVelocity, (value) -> motor2Inputs.velocityRPM = value);
    ifOk(
        motivator2,
        new DoubleSupplier[] {motivator2::getAppliedOutput, motivator2::getBusVoltage},
        (values) -> motor2Inputs.appliedVolts = values[0] * values[1]);
    ifOk(motivator2, motivator2::getOutputCurrent, (value) -> motor2Inputs.currentAmps = value);
    ifOk(motivator2, motivator2::getMotorTemperature, (value) -> motor2Inputs.tempCelsius = value);
    motor2Inputs.connected = motivator2ConnectedDebounce.calculate(!sparkStickyFault);

    // Update prefeed inputs
    sparkStickyFault = false;
    ifOk(prefeedMotor, prefeedEncoder::getVelocity, (value) -> prefeedInputs.velocityRPM = value);
    ifOk(
        prefeedMotor,
        new DoubleSupplier[] {prefeedMotor::getAppliedOutput, prefeedMotor::getBusVoltage},
        (values) -> prefeedInputs.appliedVolts = values[0] * values[1]);
    ifOk(
        prefeedMotor, prefeedMotor::getOutputCurrent, (value) -> prefeedInputs.currentAmps = value);
    ifOk(
        prefeedMotor,
        prefeedMotor::getMotorTemperature,
        (value) -> prefeedInputs.tempCelsius = value);
    prefeedInputs.connected = prefeedConnectedDebounce.calculate(!sparkStickyFault);

    // Velocity control status
    motor1Inputs.targetVelocityRPM = motivator1TargetRPM;
    motor2Inputs.targetVelocityRPM = motivator2TargetRPM;
    prefeedInputs.targetVelocityRPM = prefeedTargetRPM;

    motor1Inputs.atSetpoint =
        motivator1VelocityMode
            && Math.abs(motor1Inputs.velocityRPM - motivator1TargetRPM)
                < motivatorToleranceRPM.get();
    motor2Inputs.atSetpoint =
        motivator2VelocityMode
            && Math.abs(motor2Inputs.velocityRPM - motivator2TargetRPM)
                < motivatorToleranceRPM.get();
    prefeedInputs.atSetpoint =
        prefeedVelocityMode
            && Math.abs(prefeedInputs.velocityRPM - prefeedTargetRPM) < prefeedToleranceRPM.get();
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
    motivator1Controller.setSetpoint(
        motivator1TargetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setMotivator2Velocity(double velocityRPM) {
    motivator2VelocityMode = true;
    motivator2TargetRPM = Math.abs(velocityRPM);
    motivator2Controller.setSetpoint(
        motivator2TargetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setPrefeedVelocity(double velocityRPM) {
    prefeedVelocityMode = true;
    prefeedTargetRPM = Math.abs(velocityRPM);
    prefeedController.setSetpoint(prefeedTargetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
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
