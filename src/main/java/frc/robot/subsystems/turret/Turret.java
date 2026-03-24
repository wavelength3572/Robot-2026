package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.subsystems.led.IndicatorLight.TurretEncoderStatus;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Turret subsystem — physical rotation control only. Handles motor IO, angle commands, limit
 * clamping, and dual-mode range (tracking vs launch). All shot calculation, auto-shoot, and
 * visualization live in {@link frc.robot.subsystems.shooting.ShootingCoordinator}.
 */
public class Turret extends SubsystemBase {

  /** Turret operating state. */
  public enum TurretState {
    LOCKED,
    ROTATING_CW,
    ROTATING_CCW,
    READY,
    FLIPPING,
    STALLED,
    DISCONNECTED
  }

  private final TurretIO io;
  private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

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

  // Tolerance for atTarget() — does NOT affect motor control
  private final double readyToleranceAngleDeg;

  private final Timer stallTimer = new Timer(); // How long stall condition has persisted
  private boolean stallDetected = false;

  // How close to a limit (degrees) before safety indicators fire
  private static final double WARNING_ZONE_DEG = 20.0;

  // Startup encoder validation thresholds (degrees from expected zero position)
  private static final double ENCODER_WARNING_THRESHOLD_DEG = 5.0;
  private static final double ENCODER_ERROR_THRESHOLD_DEG = 15.0;

  // Current state — promoted from periodic() local for external readiness checks
  private TurretState currentState = TurretState.LOCKED;

  // Turret lock — when true, all movement commands are blocked and motor is in brake hold
  private boolean locked = false;

  // Startup encoder validation
  private boolean startupValidationDone = false;
  private TurretEncoderStatus encoderValidationStatus = TurretEncoderStatus.VALID;

  private final Alert encoderWarningAlert =
      new Alert(
          "Turret absolute encoder offset from expected position (5-15 deg). Check encoder mount.",
          AlertType.kWarning);
  private final Alert encoderErrorAlert =
      new Alert(
          "TURRET LOCKED — Absolute encoder >15 deg from expected position! Encoder may have shifted. Do NOT enable until inspected.",
          AlertType.kError);

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

    readyToleranceAngleDeg = config.getTurretToleranceAngleDeg();
  }

  @Override
  public void periodic() {
    io.updateInputs(turretInputs);
    Logger.processInputs("Turret", turretInputs);

    // One-time startup encoder validation: check if the inside angle is near 0°
    // (the expected position when the team places the turret at the known setup position).
    // If the absolute encoder offset has physically shifted, this angle will be wrong.
    if (!startupValidationDone && turretInputs.connected) {
      startupValidationDone = true;
      double startupAngleError = Math.abs(turretInputs.currentInsideAngleDeg);
      Logger.recordOutput("Turret/StartupAngleError", startupAngleError);

      if (startupAngleError >= ENCODER_ERROR_THRESHOLD_DEG) {
        encoderValidationStatus = TurretEncoderStatus.ERROR;
        encoderErrorAlert.set(true);
        lock();
        System.err.println(
            "[Turret] CRITICAL: Startup angle is "
                + String.format("%.1f", startupAngleError)
                + " deg from expected. Encoder offset may have shifted! Turret LOCKED.");
      } else if (startupAngleError >= ENCODER_WARNING_THRESHOLD_DEG) {
        encoderValidationStatus = TurretEncoderStatus.WARNING;
        encoderWarningAlert.set(true);
        System.err.println(
            "[Turret] WARNING: Startup angle is "
                + String.format("%.1f", startupAngleError)
                + " deg from expected. Check encoder mount.");
      }
    }

    // Compute and log state
    if (!turretInputs.connected) {
      currentState = TurretState.DISCONNECTED;
    } else if (locked) {
      currentState = TurretState.LOCKED;
    } else if (atTarget()) {
      currentState = TurretState.READY;
    } else if (Math.abs(getOutsideTargetAngle() - getOutsideCurrentAngle()) > 100) {
      currentState = TurretState.FLIPPING;
    } else if (getOutsideTargetAngle() > getOutsideCurrentAngle()) {
      currentState = TurretState.ROTATING_CW;
    } else {
      currentState = TurretState.ROTATING_CCW;
    }

    // See if we can detect a Jam
    if (currentState == TurretState.READY
        || currentState == TurretState.FLIPPING
        || currentState == TurretState.ROTATING_CW
        || currentState == TurretState.ROTATING_CCW) {
      double positionError =
          Math.abs(turretInputs.targetOutsideAngleDeg - turretInputs.currentOutsideAngleDeg);
      boolean stalled = (turretInputs.currentAmps > 14.0 && positionError > 3.0);
      if (stalled) {
        if (!stallTimer.isRunning()) {
          stallTimer.restart();
        }
        if (stallTimer.hasElapsed(.5)) {
          stallDetected = true;
          currentState = TurretState.STALLED;
        }
      } else {
        stallTimer.stop();
        stallDetected = false;
      }

    } else {
      stallTimer.stop();
      stallDetected = false;
    }

    Logger.recordOutput("Subsystems/TurretState", currentState.name());

    // Push tunable PID changes to IO
    if (LoggedTunableNumber.hasChanged(kP, kD)) {
      io.configurePID(kP.get(), kD.get());
    }
  }

  // ========== Turret Lock ==========

  /** Lock the turret — cuts motor output and blocks all movement commands. */
  public void lock() {
    locked = true;
    io.stop();
  }

  /** Unlock the turret — allows movement commands again. */
  public void unlock() {
    locked = false;
  }

  /**
   * @return true if the turret is locked
   */
  public boolean isLocked() {
    return locked;
  }

  /**
   * Get the startup encoder validation status.
   *
   * @return VALID, WARNING, or ERROR based on startup angle deviation
   */
  public TurretEncoderStatus getEncoderValidationStatus() {
    return encoderValidationStatus;
  }

  // ========== Angle Control ==========

  /**
   * Set the turret to point at a specific angle relative to the robot's front. The angle will be
   * clamped to the effective limits.
   *
   * @param angleDegrees Angle in degrees (positive = counter-clockwise when viewed from above)
   */
  public void setOutsideTurretAngle(double angleDegrees) {
    if (locked) return;
    double clampedAngle = Math.max(outsideAngleMin, Math.min(outsideAngleMax, angleDegrees));
    io.setOutsideTurretAngle(clampedAngle);
  }

  /**
   * Set the turret to point at a specific angle relative to the robot's front. And maintain that
   * angle reletive to the robot position The angle will be clamped to the effective limits.
   *
   * @param angleDegrees Angle in degrees (positive = counter-clockwise when viewed from above)
   */
  public void holdOutsideTurretAngle(double angleDegrees, double robotOmega) {
    if (locked) return;

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
    if (locked) return;
    io.setInsideTurretAngle_ONLY_FOR_TESTING(angleDegrees);
  }

  public void setTurretVolts(double volts) {
    if (locked) return;
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
    if (locked) return;
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

    return bestOutsideAngle;
  }

  // ========== State Queries ==========

  /**
   * Get the current turret angle.
   *
   * @return Current angle in degrees
   */
  public double getOutsideCurrentAngle() {
    return turretInputs.currentOutsideAngleDeg;
  }

  /**
   * Get the target turret angle.
   *
   * @return Target angle in degrees
   */
  public double getOutsideTargetAngle() {
    return turretInputs.targetOutsideAngleDeg;
  }

  /**
   * Get the current turret operating state.
   *
   * @return Current TurretState
   */
  public TurretState getState() {
    return currentState;
  }

  /**
   * Check if the turret is at the target angle within tolerance.
   *
   * @return True if at target
   */
  public boolean atTarget() {
    if (locked) return true;
    return Math.abs(getOutsideCurrentAngle() - getOutsideTargetAngle()) <= readyToleranceAngleDeg;
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
  @AutoLogOutput(key = "Visualizations/Turret")
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
