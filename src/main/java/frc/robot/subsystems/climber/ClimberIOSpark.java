package frc.robot.subsystems.climber;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
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

  private static final LoggedTunableNumber climbServoPosition =
      new LoggedTunableNumber("Climber/servo", 0.0);

  private Servo servo = new Servo(1);
  private double servoDelay = 0;

  public ClimberIOSpark() {
    RobotConfig config = Constants.getRobotConfig();

    climberMotor = new SparkMax(99, MotorType.kBrushless);
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
        () ->
            climberMotor.configure(
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
      servo.set(climbServoPosition.get());
    }

    inputs.appliedVolts = climberMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = climberMotor.getOutputCurrent();
    inputs.currentPosition = climberEncoder.getPosition();
    inputs.targetPosition = this.targetPosition;
  }

  public void deployClimber() {
    servoDelay = 0;
  }

  // public void stopClimber() {
  // targetPosition = climberEncoder.getPosition();
  // currentClimberState = CLIMB_STATE.FINAL;
  // }

  public void climb() {}

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
