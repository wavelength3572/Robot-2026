package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  // Tunable PID gains (visible in all modes including sim)
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Tuning/Turret/kP", Constants.getRobotConfig().getTurretKp());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Tuning/Turret/kD", Constants.getRobotConfig().getTurretKd());

  // TODO: Turret uses PD-only control (no kI, no feedforward). This causes steady-state error
  // where the motor can't push through friction at small errors. Consider adding kI and/or kS.

  // Tunable ready-gate tolerance for atTarget() — does NOT affect motor control
  private static final LoggedTunableNumber readyToleranceAngleDeg =
      new LoggedTunableNumber("Tuning/Turret/ReadyToleranceAngleDeg", 2.0);

  // How close to a limit (degrees) before safety indicators fire
  private static final double WARNING_ZONE_DEG = 20.0;

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

    // Physical angle limits from config
    outsideAngleMin = config.getTurretOutsideMinAngleDeg();
    outsideAngleMax = config.getTurretOutsideMaxAngleDeg();
    outsideCenterDeg = (outsideAngleMax + outsideAngleMin) / 2.0;

    // Log turret configuration (logged once at startup)
    Logger.recordOutput("Turret/Config/outsideAngleMin", outsideAngleMin);
    Logger.recordOutput("Turret/Config/outsideAngleMax", outsideAngleMax);
    Logger.recordOutput("Turret/Config/heightMeters", turretHeightMeters);
    Logger.recordOutput("Turret/Config/gearRatio", config.getTurretGearRatio());
    Logger.recordOutput("Turret/Config/kP", config.getTurretKp());
    Logger.recordOutput("Turret/Config/kD", config.getTurretKd());
    Logger.recordOutput("Turret/Config/currentLimitAmps", config.getTurretCurrentLimitAmps());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // Push tunable PID changes to IO
    if (LoggedTunableNumber.hasChanged(kP, kD)) {
      io.configurePID(kP.get(), kD.get());
    }

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
    double clampedAngle = Math.max(outsideAngleMin, Math.min(outsideAngleMax, angleDegrees));

    if (clampedAngle != angleDegrees) {
      Logger.recordOutput("Turret/Safety/ClampedRequestDeg", clampedAngle);
    }

    io.setOutsideTurretAngle(clampedAngle);
  }

  /**
   * Set the turret to point at a specific angle relative to the robot's front. And maintain that
   * angle reletive to the robot position The angle will be clamped to the effective limits.
   *
   * @param angleDegrees Angle in degrees (positive = counter-clockwise when viewed from above)
   */
  public void holdOutsideTurretAngle(double angleDegrees, double robotOmega) {

    // Normalize robotOmega to -180 to +180 range
    // This code actually probably doesn't do anything
    // Since our robot omega is always -180 to +180
    double robotOmegaNormalized = robotOmega % 360;
    if (robotOmegaNormalized > 180) {
      robotOmegaNormalized -= 360;
    } else if (robotOmegaNormalized <= -180) {
      robotOmegaNormalized += 360;
    }

    // Normalize desired field angle to -180 to +180 range
    double desiredFieldAngleNormalized = angleDegrees % 360;
    if (desiredFieldAngleNormalized > 180) {
      desiredFieldAngleNormalized -= 360;
    } else if (desiredFieldAngleNormalized <= -180) {
      desiredFieldAngleNormalized += 360;
    }

    // Calculate the relative angle needed
    // If turret points at 0° relative to robot, it points at robotOmega in field
    // coords
    // To point at desiredFieldAngle, turret needs: desiredFieldAngle - robotOmega
    double relativeAngle = desiredFieldAngleNormalized - robotOmegaNormalized;

    // Find the equivalent angle closest to current position to maintain continuity
    // Check relativeAngle and its ±360° versions
    double[] candidates = {relativeAngle, relativeAngle + 360.0, relativeAngle - 360.0};

    double bestAngle = relativeAngle;
    double smallestMove =
        Double.MAX_VALUE; // Set this high do first viable candidate becomes the best.

    for (double candidate : candidates) {
      // Check if this candidate is within physical limits
      if (candidate >= outsideAngleMin && candidate <= outsideAngleMax) {

        double moveDistance = Math.abs(candidate - getOutsideCurrentAngle());
        if (moveDistance < smallestMove) {
          smallestMove = moveDistance;
          bestAngle = candidate;
        }
      }
    }

    io.setOutsideTurretAngle(bestAngle);
  }

  /**
   * Set the turret to point at a specific angle relative to the robot's front. The angle will be
   * clamped to the effective limits.
   *
   * @param angleDegrees Angle in degrees (positive = counter-clockwise when viewed from above)
   */
  public void setInsideTurretAngle_ONLY_FOR_TESTING(double angleDegrees) {
    io.setInsideTurretAngle_ONLY_FOR_TESTING(angleDegrees);
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
    double smallestMove =
        Double.MAX_VALUE; // Set this high do first viable candidate becomes the best.

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

  // ========== State Queries ==========

  /**
   * Get the current turret angle.
   *
   * @return Current angle in degrees
   */
  public double getOutsideCurrentAngle() {
    return io.getOutsideCurrentAngle();
  }

  /**
   * Get the target turret angle.
   *
   * @return Target angle in degrees
   */
  public double getOutsideTargetAngle() {
    return io.getOutsideTargetAngle();
  }

  /**
   * Check if the turret is at the target angle within tolerance.
   *
   * @return True if at target
   */
  public boolean atTarget() {
    return Math.abs(getOutsideCurrentAngle() - getOutsideTargetAngle())
        <= readyToleranceAngleDeg.get();
  }

  /**
   * Get the minimum angle (CCW limit).
   *
   * @return Minimum angle in degrees
   */
  public double getMinAngle() {
    return outsideAngleMin;
  }

  /**
   * Get the maximum angle (CW limit).
   *
   * @return Maximum angle in degrees
   */
  public double getMaxAngle() {
    return outsideAngleMax;
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
    return WARNING_ZONE_DEG;
  }

  /**
   * Get the room available to rotate clockwise (positive direction).
   *
   * @return Degrees of room until CW limit
   */
  public double getRoomCW() {
    return outsideAngleMax - getOutsideCurrentAngle();
  }

  /**
   * Get the room available to rotate counter-clockwise (negative direction).
   *
   * @return Degrees of room until CCW limit
   */
  public double getRoomCCW() {
    return getOutsideCurrentAngle() - outsideAngleMin;
  }

  /**
   * Check if the turret is near either limit.
   *
   * @return True if within warning zone of either limit
   */
  public boolean isNearLimit() {
    return getRoomCW() <= WARNING_ZONE_DEG || getRoomCCW() <= WARNING_ZONE_DEG;
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
