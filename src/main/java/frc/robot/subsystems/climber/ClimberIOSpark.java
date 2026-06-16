package frc.robot.subsystems.climber;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.RobotConfig;

public class ClimberIOSpark implements ClimberIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;

  public ClimberIOSpark() {
    RobotConfig config = Constants.getRobotConfig();

    motor = new SparkMax(config.getClimberCanId(), MotorType.kBrushless);
    encoder = motor.getEncoder();
    controller = motor.getClosedLoopController();

    var motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(config.getClimberCurrentLimit())
        .voltageCompensation(12.0);

    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(config.getClimberKp(), 0.0, 0.0);

    // Hardware soft limits — prevent over-travel even if software fails
    motorConfig
        .softLimit
        .forwardSoftLimit((float) config.getClimberExtendPosition())
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(0.0f)
        .reverseSoftLimitEnabled(true);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig,
                com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters));

    // Zero encoder at startup (robot must start with climber stowed)
    tryUntilOk(motor, 5, () -> encoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.positionRotations = encoder.getPosition();
    inputs.appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
  }

  @Override
  public void setPosition(double motorRotations) {
    controller.setSetpoint(motorRotations, ControlType.kPosition);
  }

  @Override
  public void stop() {
    motor.setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP) {
    var pidConfig = new SparkMaxConfig();
    pidConfig.closedLoop.pid(kP, 0.0, 0.0);
    motor.configure(
        pidConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }

  @Override
  public void zeroEncoder() {
    tryUntilOk(motor, 5, () -> encoder.setPosition(0.0));
  }
}
