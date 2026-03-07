package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.subsystems.climber.ClimberConstants.CLIMB_STATE;
import frc.robot.util.LoggedTunableNumber;

public class ClimberIOSpark implements ClimberIO {

  private SparkMax climberMotor = new SparkMax(ClimberConstants.canId, MotorType.kBrushless);
  private RelativeEncoder climberEncoder = climberMotor.getEncoder();
  private SparkClosedLoopController climberController = climberMotor.getClosedLoopController();

  //FIXME: I don't think we will have a relay...This was for the foot last year. 
  // private Relay spike = new Relay(0);

  private CLIMB_STATE currentClimberState = CLIMB_STATE.STOWED;

  private double targetPosition = 0;

  private static final LoggedTunableNumber ClimberkP =
      new LoggedTunableNumber("Climber/ClimberKP", ClimberConstants.climbKp);

  private static final LoggedTunableNumber climbServoPosition =
      new LoggedTunableNumber("Climber/servo", 0.0);

  private Servo servo = new Servo(1);
  private double servoDelay = 0;

  //FIXME:Footcode needs to go
  private double footDeployTimer = 0;

  public ClimberIOSpark() {
    climberMotor.configure(
        ClimberConfigs.ClimberSubsystem.climberConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    climberMotor.set(0.0);
    climberEncoder.setPosition(0.0);
    // spike.setDirection(Relay.Direction.kBoth);
    servo.set(0.0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {

    inputs.climbingFinished = isClimbingFinished();

    // if (ClimberkP.hasChanged(hashCode())) {
    // final SparkMaxConfig config = new SparkMaxConfig();
    // config.closedLoop.pidf(ClimberkP.get(), 0.0, 0.0, 0.0,ClosedLoopSlot.kSlot1);
    // climberMotor.configure(
    // config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    // }

    if (climbServoPosition.hasChanged()) {
      servo.set(climbServoPosition.get());
    }

    switch (currentClimberState) {
      case STOWED:
        climberMotor.set(0.0);
        servoDelay = 0;
        footDeployTimer = 0;
        break;
      case SERVO:
        setRelayState(Relay.Value.kReverse); // Foot longer
        servo.set(0.2);
        footDeployTimer++;
        if (servoDelay > 25) { // About .5 seconds
          currentClimberState = CLIMB_STATE.FAST_DEPLOY;
        } else {
          servoDelay++;
        }
        break;
      case FAST_DEPLOY:
        if (footDeployTimer >= ClimberConstants.footDeployDuration / 0.02) {
          setRelayState(Relay.Value.kOff); // Foot stop
        } else {
          footDeployTimer++;
        }
        targetPosition = ClimberConstants.FAST_DEPLOY_POSITION;
        climberMotor.set(ClimberConstants.climberMaxDeploySpeed); // Deploy as fast as we can
        if (climberEncoder.getPosition() < targetPosition) {
          currentClimberState = CLIMB_STATE.DEPLOY;
        }
        break;
      case DEPLOY:
        targetPosition = ClimberConstants.DEPLOY_POSITION;
        climberController.setReference(
            targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        break;
      case CLIMB:
        setRelayState(Relay.Value.kReverse); // Foot longer
        targetPosition = ClimberConstants.CLIMBED_POSITION;
        climberMotor.set(ClimberConstants.climberMaxClimbSpeed);
        // climberController.setReference(
        // targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        if (climberEncoder.getPosition() > ClimberConstants.CLIMBED_SERVO_RELEASE_POSITION) {
          servo.set(0.0); // Release the servo so climber locks
        }
        break;
      case FINAL:
        // Used for testing
        // if (RobotStatus.algaeArmIsSafeForClimbing()) {
        // climberController.setReference(targetPosition, ControlType.kPosition);
        // }
        break;
      default:
        break;
    }

    inputs.appliedVolts = climberMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = climberMotor.getOutputCurrent();
    inputs.currentPosition = climberEncoder.getPosition();
    inputs.targetPosition = this.targetPosition;
    inputs.currentClimbState = this.currentClimberState;
    // inputs.relayState = spike.get();
  }

  public void deployClimber() {
    servoDelay = 0;
    currentClimberState = CLIMB_STATE.SERVO;
  }

  // public void stopClimber() {
  // targetPosition = climberEncoder.getPosition();
  // currentClimberState = CLIMB_STATE.FINAL;
  // }

  public void climb() {
    if (currentClimberState == CLIMB_STATE.DEPLOY) {
      currentClimberState = CLIMB_STATE.CLIMB;
    }
  }

  // public void setRelayState(Relay.Value newState) {
  //   spike.set(newState);
  // }

  @Override
  public boolean isClimberDeployed() {
    return (this.currentClimberState != CLIMB_STATE.STOWED);
  }

  public double drumToEncoder(double drumRotations) {
    return drumRotations * ClimberConstants.kClimberGearing;
  }

  public double encoderToDrum(double encoderRotations) {
    return encoderRotations / ClimberConstants.kClimberGearing;
  }

  @Override
  public boolean isClimbingFinished() {
    if (currentClimberState == CLIMB_STATE.CLIMB) {
      double difference =
          Math.abs(ClimberConstants.CLIMBED_POSITION - climberEncoder.getPosition());
      return (difference < ClimberConstants.CLIMBING_TOLERANCE) ? true : false;
    } else return false;
  }
}
