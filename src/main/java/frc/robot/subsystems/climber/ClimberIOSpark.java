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
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;

public class ClimberIOSpark implements ClimberIO {

  private final SparkMax climberMotor;
  private final RelativeEncoder climberEncoder;
  private final SparkClosedLoopController climberController;

  private double targetPosition = 0;

  private static final LoggedTunableNumber climbServoPosition = new LoggedTunableNumber("Climber/servo", 0.0);

  private Servo servo = new Servo(1);

  public ClimberIOSpark() {
    RobotConfig config = Constants.getRobotConfig();

    climberMotor = new SparkMax(config.getClimberCanId(), MotorType.kBrushless);
    climberEncoder = climberMotor.getEncoder();
    climberController = climberMotor.getClosedLoopController();

    // Configure climber motor
    var climberConfig = new SparkMaxConfig();
    climberConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .voltageCompensation(12.0);

    climberConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.0, 0.0, 0.0);

    tryUntilOk(
        climberMotor,
        5,
        () -> climberMotor.configure(
            climberConfig,
            com.revrobotics.ResetMode.kResetSafeParameters,
            com.revrobotics.PersistMode.kPersistParameters));

    climberMotor.set(0.0);
    tryUntilOk(climberMotor, 5, () -> climberEncoder.setPosition(0.0));
    servo.set(0.0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    if (LoggedTunableNumber.hasChanged(climbServoPosition)) {
      setServoPosition(climbServoPosition.get());
    }
    inputs.appliedVolts = climberMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = climberMotor.getOutputCurrent();
    inputs.currentPosition = climberEncoder.getPosition();
    inputs.targetPosition = this.targetPosition;
  }

  @Override
  public void setClimberVoltage(double volts) {
    climberMotor.setVoltage(volts);
  }

  @Override
  public void setServoPosition(double position) {
    // Position is 0 to 1.0
    // where 0.0 is one extreme and 1.0 is the other
    servo.set(position);
  }

  @Override
  public void deployClimber() {

  }

  @Override
  public void stopClimber() {
    climberMotor.setVoltage(0.0);
  }

  public void climb() {
  }

  @Override
  public boolean isClimberDeployed() {
    return false;
  }

  public double drumToEncoder(double drumRotations) {
    return drumRotations;
  }

  public double encoderToDrum(double encoderRotations) {
    return encoderRotations;
  }

  @Override
  public boolean isClimbingFinished() {
    return false;
  }
}
