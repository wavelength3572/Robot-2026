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
  private final SparkMax motivator1;
  private final RelativeEncoder motivator1Encoder;
  private final SparkClosedLoopController motivator1Controller;

  // Connection debouncers
  private final Debouncer motivator1ConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Velocity tolerance for at-setpoint check (set by subsystem via setVelocityTolerances)
  private double motivatorToleranceRPM = 100.0;

  // Feedforward controller (rebuilt when gains change via configureFeedforward)
  private SimpleMotorFeedforward feedforward;

  // Target tracking
  private double motivator1TargetRPM = 0.0;
  private boolean motivator1VelocityMode = false;

  public MotivatorIOSparkMax() {
    config = Constants.getRobotConfig();

    // Create PDH for independent current monitoring
    // pdh = new PowerDistribution();

    // Create SparkFlex controllers
    motivator1 = new SparkMax(config.getMotivatorCanId(), MotorType.kBrushless);

    // Get encoders and closed-loop controllers
    motivator1Encoder = motivator1.getEncoder();
    motivator1Controller = motivator1.getClosedLoopController();

    // Configure motivator 1 (CAN ID 55)
    double initKp = config.getMotivatorKp();
    double initKi = config.getMotivatorKi();
    double initKd = config.getMotivatorKd();
    double initKs = config.getMotivatorKs();
    double initKv = config.getMotivatorKv();

    feedforward = new SimpleMotorFeedforward(initKs, initKv);

    var motor1Config = new SparkMaxConfig();
    motor1Config
        .inverted(false) // TODO: Make robot-specific when MainBot is ready
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
        motivator1,
        5,
        () ->
            motivator1.configure(
                motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Startup diagnostics
    System.out.println("[MotivatorIOSparkFlex] ========== STARTUP ==========");
    System.out.println("[MotivatorIOSparkFlex] Motivator 1 CAN ID: " + config.getMotivatorCanId());
    System.out.println(
        "[MotivatorIOSparkFlex] Motivator Current Limit: " + config.getMotivatorCurrentLimit());
    System.out.println("[MotivatorIOSparkFlex] ==============================");
  }

  @Override
  public void updateInputs(MotorInputs motor1Inputs) {
    // Update motivator 1 inputs
    sparkStickyFault = false;
    ifOk(motivator1, motivator1Encoder::getVelocity, (value) -> motor1Inputs.velocityRPM = value);
    ifOk(
        motivator1,
        new DoubleSupplier[] {motivator1::getAppliedOutput, motivator1::getBusVoltage},
        (values) -> motor1Inputs.appliedVolts = values[0] * values[1]);
    ifOk(motivator1, motivator1::getOutputCurrent, (value) -> motor1Inputs.currentAmps = value);
    ifOk(motivator1, motivator1::getMotorTemperature, (value) -> motor1Inputs.tempCelsius = value);
    // motor1Inputs.PdhCurrentAmps = pdh.getCurrent(2);
    motor1Inputs.connected = motivator1ConnectedDebounce.calculate(!sparkStickyFault);

    // Velocity control status
    motor1Inputs.targetVelocityRPM = motivator1TargetRPM;

    motor1Inputs.atSetpoint =
        motivator1VelocityMode
            && Math.abs(motor1Inputs.velocityRPM - motivator1TargetRPM)
                < this.motivatorToleranceRPM;
  }

  // ========== Duty Cycle Control ==========

  @Override
  public void setMotivator1DutyCycle(double dutyCycle) {
    motivator1VelocityMode = false;
    motivator1TargetRPM = 0.0;
    motivator1.set(dutyCycle);
  }

  // ========== Velocity Control ==========

  @Override
  public void setMotivator1Velocity(double velocityRPM) {
    motivator1VelocityMode = true;
    motivator1TargetRPM = Math.abs(velocityRPM);
    double arbFFVolts = feedforward.calculate(velocityRPM);
    motivator1Controller.setSetpoint(
        motivator1TargetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0, arbFFVolts);
  }

  // ========== Stop Methods ==========

  @Override
  public void stop() {
    motivator1VelocityMode = false;
    motivator1TargetRPM = 0.0;
    motivator1.stopMotor();
  }

  @Override
  public void stopMotivator1() {
    motivator1VelocityMode = false;
    motivator1TargetRPM = 0.0;
    motivator1.stopMotor();
  }

  @Override
  public void stopMotivators() {
    motivator1VelocityMode = false;
    motivator1TargetRPM = 0.0;
    motivator1.stopMotor();
  }

  // ========== Configuration Methods ==========

  @Override
  public void configureMotivator1PID(double kP, double kI, double kD, double kS, double kV) {
    var pidConfig = new SparkMaxConfig();
    pidConfig.closedLoop.pid(kP, kI, kD);
    motivator1.configure(
        pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    feedforward.setKs(kS);
    feedforward.setKv(kV);
  }

  @Override
  public void setVelocityTolerances(double motivatorToleranceRPM) {
    this.motivatorToleranceRPM = motivatorToleranceRPM;
  }
}
