package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Turret subsystem — physical rotation control only. Handles motor IO, angle commands, limit
 * clamping, and dual-mode range (tracking vs launch). All shot calculation, auto-shoot, and
 * visualization live in {@link frc.robot.subsystems.shooting.ShootingCoordinator}.
 */
public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final double turretHeightMeters;
  private final double turretXOffset;
  private final double turretYOffset;

  // ========== Tunable Limits (adjust in AdvantageScope without recompiling!) ==========
  // FlipAngleDeg: How far can the turret rotate before it must "flip" (go the other way)
  // CenterOffsetDeg: Shifts the whole range (0 = symmetric around robot forward)
  // Effective limits = [CenterOffset - FlipAngle, CenterOffset + FlipAngle]
  private final LoggedTunableNumber flipAngleDeg;
  private final LoggedTunableNumber centerOffsetDeg;
  private final LoggedTunableNumber warningZoneDeg;

  // Dual-mode range: narrower range when tracking (idle) to reduce cable wear,
  // full range only when actively launching.
  private final LoggedTunableNumber trackingFlipAngleDeg;
  private boolean launchModeActive = false;

  // Cached effective limits (updated when tunables change or mode switches)
  private double effectiveMinAngleDeg;
  private double effectiveMaxAngleDeg;

  /**
   * Creates a new Turret subsystem.
   *
   * @param io The IO implementation to use (real hardware or simulation)
   */
  public Turret(TurretIO io) {
    this.io = io;
    var config = Constants.getRobotConfig();
    this.turretHeightMeters = config.getTurretHeightMeters();
    this.turretXOffset = config.getTurretOffsetX();
    this.turretYOffset = config.getTurretOffsetY();

    // Log turret configuration (logged once at startup)
    Logger.recordOutput("Turret/Config/hardLimitMinDeg", TurretConstants.HARD_LIMIT_MIN_DEG);
    Logger.recordOutput("Turret/Config/hardLimitMaxDeg", TurretConstants.HARD_LIMIT_MAX_DEG);
    Logger.recordOutput("Turret/Config/heightMeters", turretHeightMeters);
    Logger.recordOutput("Turret/Config/gearRatio", config.getTurretGearRatio());
    Logger.recordOutput("Turret/Config/kP", config.getTurretKp());
    Logger.recordOutput("Turret/Config/kD", config.getTurretKd());
    Logger.recordOutput("Turret/Config/currentLimitAmps", config.getTurretCurrentLimitAmps());

    // Initialize tunable limits (adjustable via NetworkTables at runtime)
    double configMin = config.getTurretMinAngleDegrees();
    double configMax = config.getTurretMaxAngleDegrees();
    double defaultCenter = (configMax + configMin) / 2.0;
    double defaultFlipAngle = (configMax - configMin) / 2.0;

    flipAngleDeg = new LoggedTunableNumber("Tuning/Turret/FlipAngleDeg", defaultFlipAngle);
    centerOffsetDeg = new LoggedTunableNumber("Tuning/Turret/CenterOffsetDeg", defaultCenter);
    warningZoneDeg = new LoggedTunableNumber("Tuning/Turret/WarningZoneDeg", 20.0);
    trackingFlipAngleDeg =
        new LoggedTunableNumber("Tuning/Turret/TrackingFlipAngleDeg", defaultFlipAngle - 10.0);

    // Calculate initial effective limits
    updateEffectiveLimits();
  }

  /** Update effective min/max limits from tunable flip angle, center offset, and active mode. */
  private void updateEffectiveLimits() {
    double center = centerOffsetDeg.get();
    double flipAngle = launchModeActive ? flipAngleDeg.get() : trackingFlipAngleDeg.get();
    effectiveMinAngleDeg = Math.max(TurretConstants.HARD_LIMIT_MIN_DEG, center - flipAngle);
    effectiveMaxAngleDeg = Math.min(TurretConstants.HARD_LIMIT_MAX_DEG, center + flipAngle);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    if (LoggedTunableNumber.hasChanged(flipAngleDeg, centerOffsetDeg, trackingFlipAngleDeg)) {
      updateEffectiveLimits();
    }
    Logger.recordOutput("Turret/LaunchModeActive", launchModeActive);
    Logger.recordOutput(
        "Turret/AngleError", inputs.targetAngleDegrees - inputs.currentAngleDegrees);
    Logger.recordOutput("Turret/AtTarget", atTarget());
  }

  // ========== Angle Control ==========

  /**
   * Set the turret to point at a specific angle relative to the robot's front. The angle will be
   * clamped to the effective limits.
   *
   * @param angleDegrees Angle in degrees (positive = counter-clockwise when viewed from above)
   */
  public void setTurretAngle(double angleDegrees) {
    double clampedAngle =
        Math.max(effectiveMinAngleDeg, Math.min(effectiveMaxAngleDeg, angleDegrees));

    if (clampedAngle != angleDegrees) {
      Logger.recordOutput("Turret/Safety/ClampedRequestDeg", angleDegrees);
    }

    io.setTurretAngle(Rotation2d.fromDegrees(clampedAngle));
  }

  /**
   * Calculate and set the turret angle to point at a field position.
   *
   * @param robotX Robot's X position on field (meters)
   * @param robotY Robot's Y position on field (meters)
   * @param robotOmega Robot's heading in degrees
   * @param targetX Target X position on field (meters)
   * @param targetY Target Y position on field (meters)
   */
  public void aimAtFieldPosition(
      double robotX, double robotY, double robotOmega, double targetX, double targetY) {
    double turretAngle = calculateTurretAngle(robotX, robotY, robotOmega, targetX, targetY);
    setTurretAngle(turretAngle);
  }

  /**
   * Calculate robot-relative turret angle to point at a field target. Accounts for turret offset,
   * wraps to minimize rotation from current position, and clamps to effective limits.
   */
  private double calculateTurretAngle(
      double robotX, double robotY, double robotOmega, double targetX, double targetY) {
    double robotOmegaRad = Math.toRadians(robotOmega);

    double turretFieldX =
        robotX
            + (turretXOffset * Math.cos(robotOmegaRad) - turretYOffset * Math.sin(robotOmegaRad));
    double turretFieldY =
        robotY
            + (turretXOffset * Math.sin(robotOmegaRad) + turretYOffset * Math.cos(robotOmegaRad));

    double deltaX = targetX - turretFieldX;
    double deltaY = targetY - turretFieldY;
    double absoluteAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

    double baseAngle = absoluteAngle - robotOmega;
    double wrapped =
        MathUtil.inputModulus(
            baseAngle, inputs.currentAngleDegrees - 180, inputs.currentAngleDegrees + 180);

    return Math.max(effectiveMinAngleDeg, Math.min(effectiveMaxAngleDeg, wrapped));
  }

  // ========== Dual-Mode Range Methods ==========

  /** Enable launch mode (full turret range). */
  public void enableLaunchMode() {
    if (!launchModeActive) {
      launchModeActive = true;
      updateEffectiveLimits();
    }
  }

  /** Disable launch mode (narrower tracking range). */
  public void disableLaunchMode() {
    if (launchModeActive) {
      launchModeActive = false;
      updateEffectiveLimits();
    }
  }

  /**
   * Check if launch mode is active.
   *
   * @return True if turret is using full launch range
   */
  public boolean isLaunchModeActive() {
    return launchModeActive;
  }

  // ========== State Queries ==========

  /**
   * Get the current turret angle.
   *
   * @return Current angle in degrees
   */
  public double getCurrentAngle() {
    return inputs.currentAngleDegrees;
  }

  /**
   * Get the target turret angle.
   *
   * @return Target angle in degrees
   */
  public double getTargetAngle() {
    return inputs.targetAngleDegrees;
  }

  /**
   * Check if the turret is at the target angle within tolerance.
   *
   * @return True if at target
   */
  public boolean atTarget() {
    return Math.abs(inputs.currentAngleDegrees - inputs.targetAngleDegrees)
        <= TurretConstants.ANGLE_TOLERANCE_DEGREES;
  }

  /**
   * Get the effective minimum angle (CCW limit).
   *
   * @return Minimum angle in degrees
   */
  public double getEffectiveMinAngle() {
    return effectiveMinAngleDeg;
  }

  /**
   * Get the effective maximum angle (CW limit).
   *
   * @return Maximum angle in degrees
   */
  public double getEffectiveMaxAngle() {
    return effectiveMaxAngleDeg;
  }

  /**
   * Get the center offset tunable value (for snapshot creation by coordinator).
   *
   * @return Center offset in degrees
   */
  public double getCenterOffsetDeg() {
    return centerOffsetDeg.get();
  }

  /**
   * Get the warning zone tunable value (for snapshot creation by coordinator).
   *
   * @return Warning zone in degrees
   */
  public double getWarningZoneDeg() {
    return warningZoneDeg.get();
  }

  /**
   * Get the room available to rotate clockwise (positive direction).
   *
   * @return Degrees of room until CW limit
   */
  public double getRoomCW() {
    return effectiveMaxAngleDeg - inputs.currentAngleDegrees;
  }

  /**
   * Get the room available to rotate counter-clockwise (negative direction).
   *
   * @return Degrees of room until CCW limit
   */
  public double getRoomCCW() {
    return inputs.currentAngleDegrees - effectiveMinAngleDeg;
  }

  /**
   * Check if the turret is near either limit.
   *
   * @return True if within warning zone of either limit
   */
  public boolean isNearLimit() {
    double warningZone = warningZoneDeg.get();
    return getRoomCW() <= warningZone || getRoomCCW() <= warningZone;
  }

  // ========== 3D Pose & Config ==========

  /** Returns the current turret 3D pose for component visualization. */
  @AutoLogOutput(key = "Odometry/Turret")
  public Pose3d getPose() {
    return new Pose3d(
        turretXOffset,
        turretYOffset,
        turretHeightMeters,
        new Rotation3d(0.0, 0.0, Rotation2d.fromDegrees(getCurrentAngle()).getRadians()));
  }

  /**
   * Get the turret height from configuration.
   *
   * @return Turret height in meters
   */
  public double getTurretHeightMeters() {
    return turretHeightMeters;
  }
}
