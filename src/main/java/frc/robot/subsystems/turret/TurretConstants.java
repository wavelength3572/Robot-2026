package frc.robot.subsystems.turret;

/**
 * Universal turret constants that apply to all robots. Robot-specific turret configuration (CAN
 * IDs, gear ratios, PID gains, etc.) should be in the respective RobotConfig implementation
 * (SquareBotConfig, TurretBotConfig, etc.)
 */
public final class TurretConstants {

  private TurretConstants() {} // Prevent instantiation

  // ========== Universal Constants ==========

  // Tolerances
  public static final double ANGLE_TOLERANCE_DEGREES = 2.0;

  // Motion constraints (used for trajectory generation if needed)
  public static final double MAX_VELOCITY_DEG_PER_SEC = 360.0;
  public static final double MAX_ACCELERATION_DEG_PER_SEC_SQUARED = 720.0;
}
