package frc.robot.subsystems.climber;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class ClimberConfigs {

  public static final class ClimberSubsystem {
    public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the elevator motor
      climberConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(ClimberConstants.climberCurrentLimit)
          .openLoopRampRate(0.0)
          .closedLoopRampRate(0.1)
          .voltageCompensation(12);
      climberConfig
          .softLimit
          .forwardSoftLimit(ClimberConstants.CLIMBED_POSITION)
          .forwardSoftLimitEnabled(true)
          .reverseSoftLimitEnabled(false);
      climberConfig
          .closedLoop
          // Set PID values for position control
          // Slot 0 is for going out
          .p(ClimberConstants.deployKp, ClosedLoopSlot.kSlot0)
          .d(ClimberConstants.deployKd, ClosedLoopSlot.kSlot0)
          .outputRange(ClimberConstants.climberMaxDeploySpeed, 1.0, ClosedLoopSlot.kSlot0)
          // Slot 1 is for climbing
          .p(ClimberConstants.climbKp, ClosedLoopSlot.kSlot1)
          .d(ClimberConstants.climbKd, ClosedLoopSlot.kSlot1)
          .outputRange(0.0, 1.0, ClosedLoopSlot.kSlot1)
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    }
  }
}
