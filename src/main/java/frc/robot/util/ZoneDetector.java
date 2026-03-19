package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.FieldConstants;

/**
 * Detects which field zone the robot is in for auto-shoot purposes.
 *
 * <p>Six zones, checked in priority order:
 *
 * <ol>
 *   <li><b>BUMP</b> — over the bump ramps flanking the hub. No shooting allowed.
 *   <li><b>TRENCH_NEAR</b> — under trench on alliance side of hub. Hood clamped, can shoot hub.
 *   <li><b>TRENCH_FAR</b> — under trench on neutral side of hub. Hood clamped, no shooting.
 *   <li><b>ALLIANCE</b> — our scoring zone. Aim at hub, speed-gated.
 *   <li><b>NEUTRAL</b> — mid-field. Pass shots allowed (speed-gated by tunable).
 *   <li><b>OPPONENT</b> — their side. Long pass back to alliance zone.
 * </ol>
 *
 * <p>BUMP zones use the robot's physical dimensions as margin (any part of the chassis on the
 * ramp). TRENCH zones use the turret's field position with a small radius margin (~7 inches), since
 * what matters is whether the turret/hood is under the trench structure. ALLIANCE/NEUTRAL/OPPONENT
 * are X-axis-only with a small hysteresis buffer.
 */
public class ZoneDetector {

  /** Field zones for auto-shoot behavior. */
  public enum Zone {
    /** Over a bump — suppress all shooting. */
    BUMP,
    /** Under trench on alliance side — hood clamped, can shoot hub (low angle). */
    TRENCH_NEAR,
    /** Under trench on neutral side — hood clamped, suppress shooting (just transiting). */
    TRENCH_FAR,
    /** Alliance scoring zone — aim at hub, fire when slow. */
    ALLIANCE,
    /** Neutral zone — pass shots, speed-gated. */
    NEUTRAL,
    /** Opponent side — long pass back to alliance zone. */
    OPPONENT
  }

  /**
   * Turret radius margin for obstacle zone detection (~7 inches). Zone checks use turret field
   * position rather than robot center, since what matters is whether the turret/hood is in or near
   * the obstacle. This radius accounts for the turret opening where balls pass through.
   */
  private static final double TURRET_RADIUS = 0.178; // 7 inches

  /** Small hysteresis buffer for X-based zone transitions (ALLIANCE↔NEUTRAL). */
  private static final double X_HYSTERESIS = 0.3;

  /** Tracks the current X-based zone to apply hysteresis. */
  private static Zone currentXZone = Zone.ALLIANCE;

  /** Minimum gyro pitch (degrees) to confirm the robot is actually on a bump. */
  private static final double BUMP_PITCH_THRESHOLD_DEG = 5.0;

  /**
   * Determine the robot's current zone (without turret position — uses robot center for trench).
   *
   * @param robotX Robot X position (field coords)
   * @param robotY Robot Y position (field coords)
   * @param alliance Current alliance
   * @return The active zone
   */
  public static Zone getCurrentZone(double robotX, double robotY, Alliance alliance) {
    return getCurrentZone(robotX, robotY, alliance, 0.0, robotX, robotY);
  }

  /**
   * Determine the robot's current zone, with gyro pitch for bump confirmation. Uses robot center
   * for trench detection (prefer the turret-aware overload).
   *
   * @param robotX Robot X position (field coords)
   * @param robotY Robot Y position (field coords)
   * @param alliance Current alliance
   * @param robotPitchDeg Absolute gyro pitch in degrees (positive = nose up)
   * @return The active zone
   */
  public static Zone getCurrentZone(
      double robotX, double robotY, Alliance alliance, double robotPitchDeg) {
    return getCurrentZone(robotX, robotY, alliance, robotPitchDeg, robotX, robotY);
  }

  /**
   * Determine the robot's current zone using turret field position for obstacle detection.
   *
   * <p>Both bump and trench zones use the turret's field position with a turret-radius margin,
   * since what matters is where the turret/hood is relative to the obstacle. Bump additionally
   * requires gyro tilt confirmation to avoid false positives on flat ground.
   *
   * @param robotX Robot X position (field coords, used for X-based zone logic)
   * @param robotY Robot Y position (field coords)
   * @param alliance Current alliance
   * @param robotPitchDeg Absolute gyro pitch in degrees (positive = nose up)
   * @param turretX Turret X position in field coords
   * @param turretY Turret Y position in field coords
   * @return The active zone
   */
  public static Zone getCurrentZone(
      double robotX,
      double robotY,
      Alliance alliance,
      double robotPitchDeg,
      double turretX,
      double turretY) {
    // --- Priority 1: 2D obstacle zones (turret position + turret radius) ---
    // BUMP: turret over a bump ramp AND robot is tilted
    if (FieldConstants.BumpZones.isInAnyBumpZone(turretX, turretY, TURRET_RADIUS)
        && Math.abs(robotPitchDeg) >= BUMP_PITCH_THRESHOLD_DEG) {
      return Zone.BUMP;
    }
    // TRENCH: turret under the overhead trench structure
    if (FieldConstants.TrenchZones.isInAnyTrenchZone(turretX, turretY, TURRET_RADIUS)) {
      boolean onAllianceSide = isOnAllianceSideOfTrench(robotX, alliance);
      return onAllianceSide ? Zone.TRENCH_NEAR : Zone.TRENCH_FAR;
    }

    // --- Priority 2: X-based field zones with hysteresis ---
    currentXZone = calculateXZone(robotX, alliance);
    return currentXZone;
  }

  /**
   * Check if the robot is on the alliance (near) side of the trench physical barrier.
   *
   * <p>The barrier sits at the hub center line. TRENCH_NEAR means the robot is between the alliance
   * wall and the barrier; TRENCH_FAR means the robot is past the barrier toward neutral/opponent
   * territory. The opponent's trench is always FAR.
   */
  private static boolean isOnAllianceSideOfTrench(double robotX, Alliance alliance) {
    double blueHubCenter = FieldConstants.LinesVertical.hubCenter;
    double redHubCenter = FieldConstants.LinesVertical.oppHubCenter;

    if (alliance == Alliance.Blue) {
      // If we're in the red-side trench, that's always FAR (opponent territory)
      double distToBlue = Math.abs(robotX - blueHubCenter);
      double distToRed = Math.abs(robotX - redHubCenter);
      if (distToRed < distToBlue) return false;
      // NEAR if robot is on the alliance side of the hub center (physical barrier)
      return robotX <= blueHubCenter;
    } else {
      double distToBlue = Math.abs(robotX - blueHubCenter);
      double distToRed = Math.abs(robotX - redHubCenter);
      if (distToBlue < distToRed) return false;
      // NEAR if robot is on the alliance side of the hub center (physical barrier)
      return robotX >= redHubCenter;
    }
  }

  /**
   * X-based zone calculation with hysteresis on the ALLIANCE↔NEUTRAL boundary.
   *
   * <p>Hysteresis prevents flickering when driving along the boundary.
   */
  private static Zone calculateXZone(double robotX, Alliance alliance) {
    double allianceZoneEnd = FieldConstants.LinesVertical.allianceZone;
    double opponentZoneStart = FieldConstants.fieldLength - allianceZoneEnd;

    if (alliance == Alliance.Blue) {
      // Blue: alliance is low-X, opponent is high-X
      if (robotX > opponentZoneStart) return Zone.OPPONENT;
      return applyHysteresis(robotX, allianceZoneEnd, true);
    } else {
      // Red: alliance is high-X, opponent is low-X
      if (robotX < allianceZoneEnd) return Zone.OPPONENT;
      return applyHysteresis(robotX, opponentZoneStart, false);
    }
  }

  /**
   * Apply hysteresis at the alliance/neutral boundary.
   *
   * @param robotX Robot X position
   * @param boundary The alliance zone edge X coordinate
   * @param allianceIsLowX True for blue (alliance at low X), false for red (alliance at high X)
   */
  private static Zone applyHysteresis(double robotX, double boundary, boolean allianceIsLowX) {
    if (allianceIsLowX) {
      // Blue: X < boundary = ALLIANCE, X > boundary = NEUTRAL
      if (currentXZone == Zone.ALLIANCE) {
        return (robotX > boundary + X_HYSTERESIS) ? Zone.NEUTRAL : Zone.ALLIANCE;
      } else {
        return (robotX < boundary - X_HYSTERESIS) ? Zone.ALLIANCE : Zone.NEUTRAL;
      }
    } else {
      // Red: X > boundary = ALLIANCE, X < boundary = NEUTRAL
      if (currentXZone == Zone.ALLIANCE) {
        return (robotX < boundary - X_HYSTERESIS) ? Zone.NEUTRAL : Zone.ALLIANCE;
      } else {
        return (robotX > boundary + X_HYSTERESIS) ? Zone.ALLIANCE : Zone.NEUTRAL;
      }
    }
  }

  /** Get the appropriate pass target Y based on robot's Y position. */
  public static double getPassTargetY(double robotY) {
    double fieldCenter = FieldConstants.fieldWidth / 2.0;
    if (robotY < fieldCenter) {
      return Constants.StrategyConstants.RIGHT_PASS_TARGET_Y;
    } else {
      return Constants.StrategyConstants.LEFT_PASS_TARGET_Y;
    }
  }
}
