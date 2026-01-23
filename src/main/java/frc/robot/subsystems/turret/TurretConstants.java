package frc.robot.subsystems.turret;

public final class TurretConstants {
  // Hardware IDs
  public static final int TURRET_MOTOR_CAN_ID = 50;

  // Physical dimensions
  /** Height of the turret barrel above the ground in meters */
  public static final double TURRET_HEIGHT_METERS = 0.4826; // 19 inches

  // Physical constraints
  public static final double GEAR_RATIO = 1.0; // Motor rotations per turret rotation

  // Turret travel limits (400 degrees total travel due to wiring constraints)
  // This means ±200 degrees from the center/home position
  public static final double MAX_ANGLE_DEGREES = 200.0; // Forward soft limit
  public static final double MIN_ANGLE_DEGREES = -200.0; // Reverse soft limit

  // Convert to rotations for TalonFX soft limits (mechanism rotations after gear ratio)
  public static final double FORWARD_SOFT_LIMIT_ROTATIONS = MAX_ANGLE_DEGREES / 360.0;
  public static final double REVERSE_SOFT_LIMIT_ROTATIONS = MIN_ANGLE_DEGREES / 360.0;

  // PID Constants (tune these for your robot)
  public static final double kP = 5.0;
  public static final double kD = 0.1;
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
