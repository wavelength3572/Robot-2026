package frc.robot.subsystems.climber;

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
import frc.robot.util.SparkConnection;
import java.util.function.DoubleSupplier;

/** SparkMax + NEO550 implementation of the Climber pre-feed wheel. */
public class ClimberIOSpark implements ClimberIO {
  private final RobotConfig config;
  private final SparkMax climber;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final SparkConnection connection = new SparkConnection();

  private SimpleMotorFeedforward feedforward;
  private double motorRPM = 0.0;
  private double wheelTargetRPM = 0.0;
  private boolean velocityMode = false;
  private double toleranceRPM = 100.0;
  private int periodicCounter = 0;

  public ClimberIOSpark() {
    config = Constants.getRobotConfig();

    climber = new SparkMax(config.getClimberCanId(), MotorType.kBrushless);
    encoder = climber.getEncoder();
    controller = climber.getClosedLoopController();

    feedforward = new SimpleMotorFeedforward(config.getClimberKs(), config.getClimberKv());

    var motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(config.getClimberCurrentLimit())
        .voltageCompensation(12.0);

    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(config.getClimberKp(), config.getClimberKi(), config.getClimberKd());

    motorConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(100)
        .busVoltagePeriodMs(100)
        .outputCurrentPeriodMs(100);

    tryUntilOk(
        climber,
        5,
        () ->
            climber.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    System.out.println("[ClimberIOSpark] ========== STARTUP ==========");
    System.out.println("[ClimberIOSpark] Climber CAN ID: " + config.getClimberCanId());
    System.out.println("[ClimberIOSpark] Current Limit: " + config.getClimberCurrentLimit());
    System.out.println("[ClimberIOSpark] ==============================");
  }

  private double motorToWheelRPM(double motorRPM) {
    return motorRPM * config.getClimberGearRatio();
  }

  private double wheelToMotorRPM(double wheelRPM) {
    return wheelRPM / config.getClimberGearRatio();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    if (!connection.isSkipping()) {
      sparkStickyFault = false;
      ifOk(climber, encoder::getVelocity, (value) -> motorRPM = value);
      inputs.wheelRPM = motorToWheelRPM(motorRPM);
      ifOk(
          climber,
          new DoubleSupplier[] {climber::getAppliedOutput, climber::getBusVoltage},
          (values) -> inputs.appliedVolts = values[0] * values[1]);
      ifOk(climber, climber::getOutputCurrent, (value) -> inputs.currentAmps = value);

      if (++periodicCounter >= 50) {
        periodicCounter = 0;
        ifOk(climber, climber::getMotorTemperature, (value) -> inputs.tempCelsius = value);
      }
      inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
      connection.update(inputs.connected);
    } else {
      inputs.connected = false;
    }

    inputs.targetRPM = wheelTargetRPM;
    inputs.atSetpoint = velocityMode && Math.abs(inputs.wheelRPM - wheelTargetRPM) < toleranceRPM;
  }

  @Override
  public void setClimberVelocity(double wheelVelocityRPM) {
    velocityMode = true;
    wheelTargetRPM = wheelVelocityRPM;
    double motorRPM = wheelToMotorRPM(wheelTargetRPM);
    double arbFFVolts = Math.copySign(feedforward.calculate(Math.abs(motorRPM)), motorRPM);
    controller.setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0, arbFFVolts);
  }

  @Override
  public void setClimberVoltage(double volts) {
    velocityMode = false;
    wheelTargetRPM = 0.0;
    controller.setSetpoint(volts, ControlType.kVoltage);
  }

  @Override
  public void stopClimber() {
    velocityMode = false;
    wheelTargetRPM = 0.0;
    climber.stopMotor();
  }

  @Override
  public void configureClimberPID(double kP, double kI, double kD, double kS, double kV) {
    var pidConfig = new SparkMaxConfig();
    pidConfig.closedLoop.pid(kP, kI, kD);
    climber.configure(
        pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    feedforward.setKs(kS);
    feedforward.setKv(kV);
  }

  @Override
  public void setVelocityTolerance(double toleranceRPM) {
    this.toleranceRPM = toleranceRPM;
  }

  @Override
  public double getFFCharacterizationVelocity() {
    return motorRPM;
  }
}
