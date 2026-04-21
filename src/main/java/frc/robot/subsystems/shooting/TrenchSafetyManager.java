package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.hood.Hood;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TurretAimingHelper;
import frc.robot.util.ZoneDetector;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Manages hood and drive safety when operating in or near the trench structure.
 *
 * <p>Three safety layers, in increasing severity:
 *
 * <ol>
 *   <li><b>Hood clamping</b> — forces hood to min angle in neutral/danger trench zones while
 *       moving, preventing the hood from hitting the trench ceiling.
 *   <li><b>Trench hood safety</b> — limits drive speed when the hood is physically above the safe
 *       angle while moving in a trench zone.
 *   <li><b>Danger trench safety</b> — near-stops the robot when the hood is above the safe angle
 *       in the narrow danger zone at the alliance/neutral boundary. Escalates to a fallback speed
 *       after a timeout so the driver can escape a broken hood.
 * </ol>
 *
 * <p>Also provides <b>predictive imminent detection</b>: when in ALLIANCE_TRENCH and heading toward
 * DANGER_TRENCH, starts applying danger-level safety early so the hood has lead time to lower.
 */
public class TrenchSafetyManager {

  private final Hood hood;
  private final ShotCalculator.TurretConfig turretConfig;
  private Supplier<ChassisSpeeds> fieldSpeedsSupplier;
  private Supplier<Pose2d> robotPoseSupplier;

  // ===== Hood Clamping Tunables =====
  // Hysteresis thresholds: clamp is triggered at a lower speed than unclamp so the
  // hood doesn't oscillate when the robot is right at the threshold speed.
  private final LoggedTunableNumber trenchHoodClampSpeedMps =
      new LoggedTunableNumber(
          "Shots/TrenchMode/HoodClampSpeedMps",
          Constants.getRobotConfig().getTrenchHoodClampSpeedMps());
  private final LoggedTunableNumber trenchHoodUnclampSpeedMps =
      new LoggedTunableNumber(
          "Shots/TrenchMode/HoodUnclampSpeedMps",
          Constants.getRobotConfig().getTrenchHoodUnclampSpeedMps());

  // ===== Trench Hood Safety Tunables =====
  private final LoggedTunableNumber trenchHoodMaxDeg =
      new LoggedTunableNumber(
          "Shots/TrenchMode/HoodMaxDeg", Constants.getRobotConfig().getTrenchHoodMaxDeg());
  private final LoggedTunableNumber trenchSafetySpeedLimitMps =
      new LoggedTunableNumber(
          "Shots/TrenchMode/SafetySpeedLimitMps",
          Constants.getRobotConfig().getTrenchSafetySpeedLimitMps());
  private final LoggedTunableNumber trenchMovingThresholdMps =
      new LoggedTunableNumber(
          "Shots/TrenchMode/MovingThresholdMps",
          Constants.getRobotConfig().getTrenchMovingThresholdMps());

  // ===== Danger Trench Safety Tunables =====
  private final LoggedTunableNumber dangerTrenchSpeedLimitMps =
      new LoggedTunableNumber("Shots/TrenchMode/DangerSpeedLimitMps", 0.3);
  // After this timeout, relax to fallback speed so the driver can escape a broken/jammed hood
  private final LoggedTunableNumber dangerTrenchTimeoutSec =
      new LoggedTunableNumber("Shots/TrenchMode/DangerTimeoutSec", 2.0);
  private final LoggedTunableNumber dangerTrenchFallbackSpeedMps =
      new LoggedTunableNumber("Shots/TrenchMode/DangerFallbackSpeedMps", 2.0);

  // ===== Predictive Imminent Detection Tunables =====
  // Project turret position forward by lookaheadSec using field velocity. If the projected
  // point lands in DANGER_TRENCH, treat it as already-dangerous so the hood has lead time.
  private final LoggedTunableNumber trenchLookaheadSec =
      new LoggedTunableNumber("Shots/TrenchMode/LookaheadSec", 0.3);
  // If predictedInDanger stays continuously false for this long while latched, release.
  // Lets a driver who reverses out of the approach recover hood-up shooting.
  private final LoggedTunableNumber trenchReleaseDwellSec =
      new LoggedTunableNumber("Shots/TrenchMode/ReleaseDwellSec", 0.5);

  // ===== State =====
  private boolean movingInTrench = false; // hysteresis flag for hood clamp threshold
  private boolean trenchHoodSafetyActive = false;
  private boolean dangerTrenchActive = false;
  private double dangerTrenchEntryTimestamp = 0.0;
  private boolean dangerTrenchImminent = false;
  private double trenchImminentClearSinceSec = 0.0;

  public TrenchSafetyManager(Hood hood, ShotCalculator.TurretConfig turretConfig) {
    this.hood = hood;
    this.turretConfig = turretConfig;
  }

  /** Provide pose and speed suppliers after the drive subsystem is initialized. */
  public void init(
      Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.fieldSpeedsSupplier = fieldSpeedsSupplier;
  }

  // ===== Per-cycle update methods (called from ShootingCoordinator.periodic()) =====

  /**
   * Update hood clamping. Called from within shot calculation when zone and speed are known.
   * Forces hood to min in neutral/danger trench; leaves it free in alliance trench.
   */
  public void updateHoodClamping(
      ZoneDetector.Zone zone, double robotSpeedMps) {
    boolean inTrenchZone =
        zone == ZoneDetector.Zone.ALLIANCE_TRENCH
            || zone == ZoneDetector.Zone.NEUTRAL_TRENCH
            || zone == ZoneDetector.Zone.DANGER_TRENCH;

    if (inTrenchZone) {
      double threshold =
          movingInTrench ? trenchHoodUnclampSpeedMps.get() : trenchHoodClampSpeedMps.get();
      movingInTrench = robotSpeedMps > threshold;
      // Only neutral and danger zones physically clamp the hood; alliance trench does not
      boolean shouldClamp =
          zone == ZoneDetector.Zone.NEUTRAL_TRENCH || zone == ZoneDetector.Zone.DANGER_TRENCH;
      if (hood != null) {
        hood.setClamped(shouldClamp);
      }
    } else {
      movingInTrench = false;
      if (hood != null) {
        hood.setClamped(false);
      }
    }
  }

  /**
   * Update predictive danger-trench detection. Projects the turret position forward using current
   * field velocity; if the projected point falls in DANGER_TRENCH, sets the imminent flag so the
   * coordinator can apply NO_FIRE_ZONE and danger-speed limits early.
   *
   * <p>The flag is latched once set and only released after the robot has been continuously clear
   * of DANGER_TRENCH for {@code trenchReleaseDwellSec} seconds, preventing oscillation at the
   * boundary.
   */
  public void updateDangerTrenchImminent(
      TurretAimingHelper.AimResult aimResult, DriverStation.Alliance alliance, boolean shouldLog) {
    double lookaheadSec = trenchLookaheadSec.get();
    ZoneDetector.Zone zone = aimResult != null ? aimResult.zone() : null;
    boolean inAllianceTrench = zone == ZoneDetector.Zone.ALLIANCE_TRENCH;
    boolean inAnyTrench =
        zone == ZoneDetector.Zone.ALLIANCE_TRENCH
            || zone == ZoneDetector.Zone.DANGER_TRENCH
            || zone == ZoneDetector.Zone.NEUTRAL_TRENCH;

    if (!inAnyTrench
        || lookaheadSec <= 0.0
        || robotPoseSupplier == null
        || fieldSpeedsSupplier == null) {
      if (dangerTrenchImminent) {
        dangerTrenchImminent = false;
        trenchImminentClearSinceSec = 0.0;
        if (shouldLog) {
          Logger.recordOutput("SmartLaunch/TrenchHoodSafety/Imminent", false);
        }
      }
      return;
    }

    boolean predictedInDanger = false;
    double predX = 0.0;
    double predY = 0.0;
    Pose2d robotPose = null;

    if (inAllianceTrench) {
      robotPose = robotPoseSupplier.get();
      ChassisSpeeds speeds = fieldSpeedsSupplier.get();
      double[] turretPos =
          ShotCalculator.getTurretFieldPosition(
              robotPose.getX(),
              robotPose.getY(),
              robotPose.getRotation().getRadians(),
              turretConfig);
      predX = turretPos[0] + speeds.vxMetersPerSecond * lookaheadSec;
      predY = turretPos[1] + speeds.vyMetersPerSecond * lookaheadSec;
      predictedInDanger = FieldConstants.TrenchZones.isInDangerTrenchZone(predX, predY, alliance);

      if (!dangerTrenchImminent) {
        dangerTrenchImminent = predictedInDanger;
        trenchImminentClearSinceSec = 0.0;
      } else {
        // Latched — release only after predictedInDanger has been false for the full dwell time
        double releaseDwellSec = trenchReleaseDwellSec.get();
        if (releaseDwellSec <= 0.0 || predictedInDanger) {
          trenchImminentClearSinceSec = 0.0;
        } else {
          double now = Timer.getFPGATimestamp();
          if (trenchImminentClearSinceSec == 0.0) {
            trenchImminentClearSinceSec = now;
          } else if (now - trenchImminentClearSinceSec >= releaseDwellSec) {
            dangerTrenchImminent = false;
            trenchImminentClearSinceSec = 0.0;
          }
        }
      }
    }

    if (shouldLog) {
      Logger.recordOutput("SmartLaunch/TrenchHoodSafety/Imminent", dangerTrenchImminent);
      Logger.recordOutput("SmartLaunch/TrenchHoodSafety/PredictedInDanger", predictedInDanger);
      double clearElapsed =
          (dangerTrenchImminent && trenchImminentClearSinceSec > 0.0)
              ? Timer.getFPGATimestamp() - trenchImminentClearSinceSec
              : 0.0;
      Logger.recordOutput("SmartLaunch/TrenchHoodSafety/ClearDwellSec", clearElapsed);
      if (robotPose != null) {
        Logger.recordOutput(
            "SmartLaunch/TrenchHoodSafety/PredictedTurret",
            new Pose2d(predX, predY, robotPose.getRotation()));
      }
    }
  }

  /**
   * Update trench hood safety speed limiting. When moving in a trench zone with the hood
   * physically above the safe angle, limits drive speed so the hood has time to lower before the
   * robot enters a tighter section.
   */
  public void updateTrenchHoodSafety(
      TurretAimingHelper.AimResult aimResult, boolean shouldLog) {
    boolean wasActive = trenchHoodSafetyActive;

    boolean inTrenchZone =
        aimResult != null
            && (aimResult.zone() == ZoneDetector.Zone.ALLIANCE_TRENCH
                || aimResult.zone() == ZoneDetector.Zone.NEUTRAL_TRENCH
                || aimResult.zone() == ZoneDetector.Zone.DANGER_TRENCH);

    if (inTrenchZone && hood != null && fieldSpeedsSupplier != null) {
      ChassisSpeeds speeds = fieldSpeedsSupplier.get();
      double robotSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      boolean isMoving = robotSpeed > trenchMovingThresholdMps.get();
      boolean hoodAboveSafe = hood.getCurrentAngle() > trenchHoodMaxDeg.get();
      trenchHoodSafetyActive = isMoving && hoodAboveSafe;
    } else {
      trenchHoodSafetyActive = false;
    }

    // Apply or release drive speed limit on state transitions.
    // Defer to danger trench safety when it's active — it uses a more aggressive limit.
    if (trenchHoodSafetyActive && !dangerTrenchActive) {
      DriveCommands.setSpeedLimit(trenchSafetySpeedLimitMps.get());
    } else if (wasActive && !dangerTrenchActive) {
      DriveCommands.clearSpeedLimit();
    }

    if (shouldLog) {
      Logger.recordOutput("SmartLaunch/TrenchHoodSafety/Active", trenchHoodSafetyActive);
    }
  }

  /**
   * Update danger trench safety. When in the DANGER_TRENCH zone (or imminently approaching it)
   * with the hood above the safe angle, near-stops the robot. After a timeout, relaxes to a
   * fallback speed so the driver can escape a broken or jammed hood.
   */
  public void updateDangerTrenchSafety(
      TurretAimingHelper.AimResult aimResult, boolean shouldLog) {
    boolean wasActive = dangerTrenchActive;
    boolean inDangerZone =
        (aimResult != null && aimResult.zone() == ZoneDetector.Zone.DANGER_TRENCH)
            || dangerTrenchImminent;

    if (inDangerZone && hood != null) {
      boolean hoodAboveSafe = hood.getCurrentAngle() > trenchHoodMaxDeg.get();
      if (hoodAboveSafe) {
        if (!wasActive) {
          dangerTrenchEntryTimestamp = Timer.getFPGATimestamp();
        }
        dangerTrenchActive = true;
        double elapsed = Timer.getFPGATimestamp() - dangerTrenchEntryTimestamp;
        if (elapsed > dangerTrenchTimeoutSec.get()) {
          DriveCommands.setSpeedLimit(dangerTrenchFallbackSpeedMps.get());
        } else {
          DriveCommands.setSpeedLimit(dangerTrenchSpeedLimitMps.get());
        }
      } else {
        dangerTrenchActive = false;
      }
    } else {
      dangerTrenchActive = false;
    }

    if (wasActive && !dangerTrenchActive) {
      DriveCommands.clearSpeedLimit();
    }

    if (shouldLog) {
      Logger.recordOutput("SmartLaunch/DangerTrenchSafety/Active", dangerTrenchActive);
    }
  }

  // ===== Getters =====

  /** True when hood is above safe angle while moving in a trench zone. */
  public boolean isTrenchHoodSafetyActive() {
    return trenchHoodSafetyActive;
  }

  /**
   * True when the robot is in ALLIANCE_TRENCH and projected to enter DANGER_TRENCH within {@code
   * trenchLookaheadSec} seconds. Latched until cleared by dwell timer.
   */
  public boolean isDangerTrenchImminent() {
    return dangerTrenchImminent;
  }
}
