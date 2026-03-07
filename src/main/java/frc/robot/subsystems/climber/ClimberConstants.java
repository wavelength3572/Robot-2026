package frc.robot.subsystems.climber;

public final class ClimberConstants {
  public static final int canId = 0; //FIXME: Update with actual CAN ID
  public static final int climberCurrentLimit = 50;
  public static final double deployPower = -1.0; //FIXME: Update with actual deploy power
public static final double climbPower = 1.0; //FIXME: Update with actual climb power


//FIXME: Update with actual encoder positions for each stage of the climb. These values are placeholders and should be determined through testing and tuning.
  // 60:1 Encoder Positions
  // public static final double DEPLOY_POSITION = -500;
  // public static final double FAST_DEPLOY_POSITION = -400;
  // public static final double CLIMBED_POSITION = -140;

  // 25:1 Encoder Positions
  // public static final double DEPLOY_POSITION = -208.3333333333;
  // public static final double FAST_DEPLOY_POSITION = -166.666666666;
  // public static final double CLIMBED_POSITION = -58.33333333333;

  // 20:1 Encoder Positions
  public static final double DEPLOY_POSITION = -155.0;
  public static final double FAST_DEPLOY_POSITION = -133.333333333;
  public static final double CLIMBED_POSITION = -52.4285; // -46.666666666;
  public static final double CLIMBED_SERVO_RELEASE_POSITION = -80;

  // public static final double DEPLOY_POSITION = -8.33333333; // Drum Position
  // public static final double FAST_DEPLOY_POSITION = -6.6666666; // Drum Position
  // public static final double CLIMBED_POSITION = -2.33333333; // Drum Position

  //FIXME: Update with actual PID values for each stage of the climb. These values are placeholders and should be determined through testing and tuning.
  public static final double deployKp = 0.03;
  public static final double deployKd = 0.0;

  public static final double climbKp = 0.06;
  public static final double climbKd = 0.0;

  public static final double CLIMBING_TOLERANCE = 4.0;

  public static final double kClimberGearing = 60.0;

  public static final double climberMaxDeploySpeed = -0.8;
  public static final double climberMaxClimbSpeed = 1.0;

  public static final double footDeployDuration = 1.0; // Seconds

  public static enum CLIMB_STATE {
    STOWED,
    SERVO,
    FAST_DEPLOY,
    DEPLOY,
    CLIMB,
    FINAL
  }
}
