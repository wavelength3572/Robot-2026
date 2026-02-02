package frc.robot.subsystems.turret;

/**
 * Universal turret constants that apply to all robots. Robot-specific turret configuration (CAN
 * IDs, gear ratios, PID gains, etc.) should be in the respective RobotConfig implementation
 * (SquareBotConfig, TurretBotConfig, etc.)
 */
public final class TurretConstants {

  // Physical dimensions
  // Turret offset from robot center (in robot-relative coordinates)
  // Positive X = forward from robot center
  // Positive Y = left from robot center
  public static final double TURRET_X_OFFSET = -0.085211539; // meters
  public static final double TURRET_Y_OFFSET = 0.1819604184; // meters
  public static final double TURRET_HEIGHT_METERS = 0.3597275; // Meters

  // ========== Universal Constants ==========

  // Hard limits (mechanical stops - never exceed these)
  public static final double HARD_LIMIT_MIN_DEG = -200.0;
  public static final double HARD_LIMIT_MAX_DEG = 200.0;

  // Tolerances
  public static final double ANGLE_TOLERANCE_DEGREES = 2.0;

  // Motion constraints (used for trajectory generation if needed)
  public static final double MAX_VELOCITY_DEG_PER_SEC = 360.0;
  public static final double MAX_ACCELERATION_DEG_PER_SEC_SQUARED = 720.0;
}
