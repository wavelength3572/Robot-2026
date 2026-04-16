package frc.robot.subsystems.climber;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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

    // Slot 0: extend/stow — full output range. Slot 1: climb — output capped to
    // climbMaxOutput so the robot lifts slowly and predictably under load.
    double climbMax = config.getClimberClimbMaxOutput();
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(config.getClimberKp(), 0.0, 0.0, ClosedLoopSlot.kSlot0)
        .pid(config.getClimberKp(), 0.0, 0.0, ClosedLoopSlot.kSlot1)
        .outputRange(-climbMax, climbMax, ClosedLoopSlot.kSlot1);

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
    // Slot 0 — normal extend/stow motion, full output range.
    controller.setSetpoint(
        motorRotations,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        0.0,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setPosition(double motorRotations, double arbFFVolts) {
    // Slot 1 — climb motion, output range capped to climbMaxOutput. arbFF adds on top.
    controller.setSetpoint(
        motorRotations,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot1,
        arbFFVolts,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setSoftLimitsEnabled(boolean enabled) {
    var config = new SparkMaxConfig();
    config.softLimit.forwardSoftLimitEnabled(enabled).reverseSoftLimitEnabled(enabled);
    motor.configure(
        config,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }

  @Override
  public void stop() {
    motor.setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double climbMaxOutput) {
    var pidConfig = new SparkMaxConfig();
    pidConfig
        .closedLoop
        .pid(kP, 0.0, 0.0, ClosedLoopSlot.kSlot0)
        .pid(kP, 0.0, 0.0, ClosedLoopSlot.kSlot1)
        .outputRange(-climbMaxOutput, climbMaxOutput, ClosedLoopSlot.kSlot1);
    motor.configure(
        pidConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }

  @Override
  public void zeroEncoder() {
    tryUntilOk(motor, 5, () -> encoder.setPosition(0.0));
  }

  @Override
  public void setEncoderPosition(double rotations) {
    tryUntilOk(motor, 5, () -> encoder.setPosition(rotations));
  }
}
