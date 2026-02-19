package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotConfig;
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

  private final RobotConfig config;

  private final double turretHeightMeters;
  private final double turretXOffset;
  private final double turretYOffset;

  private final double outsideAngleMin;
  private final double outsideAngleMax;
  private final double outsideCenterDeg;
  private final double flipAngleDeg;

  // ========== Tunable Limits (adjust in AdvantageScope without recompiling!)
  // ==========
  // FlipAngleDeg: How far can the turret rotate before it must "flip" (go the
  // other way)
  // Effective limits = [CenterOffset - FlipAngle, CenterOffset + FlipAngle]
  private final LoggedTunableNumber warningZoneDeg;

  // Dual-mode range: narrower range when tracking (idle) to reduce cable wear,
  // full range only when actively launching.
  private final LoggedTunableNumber trackingFlipAngleDeg;
  private boolean launchModeActive = false;

  // Cached effective limits (updated when tunables change or mode switches)
  private double effectiveOutsideMinAngleDeg;
  private double effectiveOutsideMaxAngleDeg;

  /**
   * Creates a new Turret subsystem.
   *
   * @param io The IO implementation to use (real hardware or simulation)
   */
  public Turret(TurretIO io) {
    this.io = io;
    config = Constants.getRobotConfig();
    this.turretHeightMeters = config.getTurretHeightMeters();
    this.turretXOffset = config.getTurretOffsetX();
    this.turretYOffset = config.getTurretOffsetY();

    // Initialize tunable limits (adjustable via NetworkTables at runtime)
    outsideAngleMin = config.getTurretOutsideMinAngleDeg(); // example -116.127
    outsideAngleMax = config.getTurretOutsideMaxAngleDeg(); // example 243.873
    outsideCenterDeg = (outsideAngleMax + outsideAngleMin) / 2.0; // Example 63.873 - Same as offset
    flipAngleDeg =
        (outsideAngleMax - outsideAngleMin) / 2.0; // Example 180 - Not sure how this helps

    warningZoneDeg = new LoggedTunableNumber("Tuning/Turret/WarningZoneDeg", 20.0);
    trackingFlipAngleDeg =
        new LoggedTunableNumber("Tuning/Turret/TrackingFlipAngleDeg", flipAngleDeg);

    // Calculate initial effective limits
    updateEffectiveLimits();

    // Log turret configuration (logged once at startup)
    Logger.recordOutput("Turret/Config/outsideAngleMin", outsideAngleMin);
    Logger.recordOutput("Turret/Config/outsideAngleMax", outsideAngleMax);
    Logger.recordOutput("Turret/Config/heightMeters", turretHeightMeters);
    Logger.recordOutput("Turret/Config/gearRatio", config.getTurretGearRatio());
    Logger.recordOutput("Turret/Config/kP", config.getTurretKp());
    Logger.recordOutput("Turret/Config/kD", config.getTurretKd());
    Logger.recordOutput("Turret/Config/currentLimitAmps", config.getTurretCurrentLimitAmps());
  }

  /** Update effective min/max limits from tunable flip angle, center offset, and active mode. */
  private void updateEffectiveLimits() {
    double flipAngle = launchModeActive ? flipAngleDeg : trackingFlipAngleDeg.get();
    effectiveOutsideMinAngleDeg =
        Math.max(outsideAngleMin, outsideCenterDeg - flipAngle); // -116.127
    effectiveOutsideMaxAngleDeg =
        Math.min(outsideAngleMax, outsideCenterDeg + flipAngle); // 243.873
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    if (LoggedTunableNumber.hasChanged(trackingFlipAngleDeg)) {
      updateEffectiveLimits();
    }
    Logger.recordOutput("Turret/LaunchModeActive", launchModeActive);
    Logger.recordOutput("Turret/AngleError", getOutsideTargetAngle() - getOutsideCurrentAngle());
    Logger.recordOutput("Turret/AtTarget", atTarget());
  }

  // ========== Angle Control ==========

  /**
   * Set the turret to point at a specific angle relative to the robot's front. The angle will be
   * clamped to the effective limits.
   *
   * @param angleDegrees Angle in degrees (positive = counter-clockwise when viewed from above)
   */
  public void setOutsideTurretAngle(double angleDegrees) {
    double clampedAngle =
        Math.max(effectiveOutsideMinAngleDeg, Math.min(effectiveOutsideMaxAngleDeg, angleDegrees));

    if (clampedAngle != angleDegrees) {
      Logger.recordOutput("Turret/Safety/ClampedRequestDeg", clampedAngle);
    }

    io.setOutsideTurretAngle(Rotation2d.fromDegrees(clampedAngle));
  }

  /**
   * Set the turret to point at a specific angle relative to the robot's front. The angle will be
   * clamped to the effective limits.
   *
   * @param angleDegrees Angle in degrees (positive = counter-clockwise when viewed from above)
   */
  public void setInsideTurretAngle_ONLY_FOR_TESTING(double angleDegrees) {
    io.setInsideTurretAngle_ONLY_FOR_TESTING(Rotation2d.fromDegrees(angleDegrees));
  }

  public void setTurretVolts(double volts) {
    io.setTurretVolts(volts);
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
    double turretAngle =
        calculateOutsideTurretAngleFromTurret(robotX, robotY, robotOmega, targetX, targetY);
    setOutsideTurretAngle(turretAngle);
  }

  /**
   * Calculate the turret angle needed to point at a target location on the field, accounting for
   * the turret's offset from the robot's center.
   *
   * @param robotX Robot center's X position on field (meters)
   * @param robotY Robot center's Y position on field (meters)
   * @param robotOmega Robot's heading in degrees (0-360, counter-clockwise from +X axis)
   * @param targetX Target X position on field (meters)
   * @param targetY Target Y position on field (meters)
   * @return Angle in degrees the turret needs to rotate relative to robot heading Range: -180 to
   *     +180 degrees
   */
  private double calculateOutsideTurretAngleFromTurret(
      double robotX, double robotY, double robotOmega, double targetX, double targetY) {

    // Normalize robotOmega to -180 to +180 range
    // This code actually probably doesn't do anything
    // Since our robot omega is always -180 to +180
    double robotOmegaNormalized = robotOmega % 360;
    if (robotOmegaNormalized > 180) {
      robotOmegaNormalized -= 360;
    } else if (robotOmegaNormalized <= -180) {
      robotOmegaNormalized += 360;
    }
    // Convert robot heading to radians for rotation calculations
    double robotOmegaRad = Math.toRadians(robotOmega);

    // Calculate turret's actual position on the field
    // The turret offset is in robot-relative coordinates, so we need to rotate it
    // to field coordinates based on the robot's heading
    double turretFieldX =
        robotX
            + (config.getTurretOffsetX() * Math.cos(robotOmegaRad)
                - config.getTurretOffsetY() * Math.sin(robotOmegaRad));

    double turretFieldY =
        robotY
            + (config.getTurretOffsetX() * Math.sin(robotOmegaRad)
                + config.getTurretOffsetY() * Math.cos(robotOmegaRad));

    // Calculate vector from turret position to target
    double deltaX = targetX - turretFieldX;
    double deltaY = targetY - turretFieldY;

    // Calculate absolute angle to target from field coordinates
    double absoluteAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

    // Calculate relative (desired) angle (turret angle relative to robot heading)
    double relativeAngle = absoluteAngle - robotOmega;

    // Log calculation values
    Logger.recordOutput("Turret/RobotOmegaNormalized", robotOmegaNormalized);
    Logger.recordOutput("Turret/AbsoluteAngle", absoluteAngle);
    Logger.recordOutput("Turret/RelativeAngle", relativeAngle);

    double bestOutsideAngle = flipOutsideAngle(relativeAngle);

    return bestOutsideAngle;
  }

  public double flipOutsideAngle(double unflippedOutsideAngle) {

    // Get current turret angle (could be outside -180 to +180 range)
    double currentAngle = getOutsideCurrentAngle();

    // Find the equivalent angle closest to current position
    // Check desiredAngle and its ±360° versions
    double[] candidates = {
      unflippedOutsideAngle, unflippedOutsideAngle + 360.0, unflippedOutsideAngle - 360.0
    };

    double bestOutsideAngle =
        unflippedOutsideAngle; // doesn't matter what we set this to, it's just for initalization
    double smallestMove = 1000000.0; // Set this high do first viable candidate becomes the best.

    for (double candidate : candidates) {
      // Check if this candidate is within physical limits
      if (candidate >= outsideAngleMin && candidate <= outsideAngleMax) {

        double moveDistance = Math.abs(candidate - currentAngle);
        if (moveDistance < smallestMove) {
          smallestMove = moveDistance;
          bestOutsideAngle = candidate;
        }
      }
    }

    // If bestAngle is still out of range, clamp to nearest limit
    // This should actually never come into play since one of the candidates
    // should always work and be within range.
    if (bestOutsideAngle < outsideAngleMin) {
      bestOutsideAngle = outsideAngleMin;
    } else if (bestOutsideAngle > outsideAngleMax) {
      bestOutsideAngle = outsideAngleMax;
    }

    Logger.recordOutput("Turret/bestOutsideAngle", bestOutsideAngle);

    return bestOutsideAngle;
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
  public double getOutsideCurrentAngle() {
    return io.getOutsideCurrentAngle().getDegrees();
  }

  /**
   * Get the target turret angle.
   *
   * @return Target angle in degrees
   */
  public double getOutsideTargetAngle() {
    return io.getOutsideTargetAngle().getDegrees();
  }

  /**
   * Check if the turret is at the target angle within tolerance.
   *
   * @return True if at target
   */
  public boolean atTarget() {
    return Math.abs(getOutsideCurrentAngle() - getOutsideTargetAngle())
        <= TurretConstants.ANGLE_TOLERANCE_DEGREES;
  }

  /**
   * Get the effective minimum angle (CCW limit).
   *
   * @return Minimum angle in degrees
   */
  public double getEffectiveMinAngle() {
    return effectiveOutsideMinAngleDeg;
  }

  /**
   * Get the effective maximum angle (CW limit).
   *
   * @return Maximum angle in degrees
   */
  public double getEffectiveMaxAngle() {
    return effectiveOutsideMaxAngleDeg;
  }

  /**
   * Get the center offset tunable value (for snapshot creation by coordinator).
   *
   * @return Center offset in degrees
   */
  public double getOutsideCenterDeg() {
    return outsideCenterDeg;
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
    return effectiveOutsideMaxAngleDeg - getOutsideCurrentAngle();
  }

  /**
   * Get the room available to rotate counter-clockwise (negative direction).
   *
   * @return Degrees of room until CCW limit
   */
  public double getRoomCCW() {
    return getOutsideCurrentAngle() - effectiveOutsideMinAngleDeg;
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
        new Rotation3d(0.0, 0.0, Rotation2d.fromDegrees(getOutsideCurrentAngle()).getRadians()));
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
