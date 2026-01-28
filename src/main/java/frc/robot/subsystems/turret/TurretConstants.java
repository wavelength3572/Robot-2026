package frc.robot.subsystems.turret;

public final class TurretConstants {
  // Hardware IDs
  public static final int TURRET_MOTOR_CAN_ID = 10; // Adjust to your CAN ID

  // Physical constraints
  public static final double GEAR_RATIO = 100.0; // Motor rotations per turret rotation
  public static final double MIN_ANGLE_DEGREES = -180.0; // Minimum turret angle
  public static final double MAX_ANGLE_DEGREES = 180.0; // Maximum turret angle

  // Turret offset from robot center (in robot-relative coordinates)
  // Positive X = forward from robot center
  // Positive Y = left from robot center
  public static final double TURRET_X_OFFSET = -0.1819604184; // meters
  public static final double TURRET_Y_OFFSET = -0.085211539; // meters
  public static final double TURRET_HEIGHT_METERS = 0.3597275; // Meters

  // PID Constants (tune these for your robot)
  public static final double kP = 15.0;
  public static final double kD = 0.0;
  public static final double kFF = 0.0;

  // Motion constraints
  public static final double MAX_VELOCITY_DEG_PER_SEC = 360.0;
  public static final double MAX_ACCELERATION_DEG_PER_SEC_SQUARED = 720.0;

  // Tolerances
  public static final double ANGLE_TOLERANCE_DEGREES = 2.0;

  // Current limits
  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final int CURRENT_THRESHOLD_AMPS = 50;
  public static final double CURRENT_THRESHOLD_TIME_SEC = 0.1;
}
