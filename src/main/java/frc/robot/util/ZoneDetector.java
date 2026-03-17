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
 * <p>BUMP and TRENCH zones are 2D rectangle checks that use the robot's physical dimensions as
 * margins, so the zone activates when ANY part of the robot overlaps the field element.
 * ALLIANCE/NEUTRAL/OPPONENT are X-axis-only with a small hysteresis buffer.
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
   * Half the longest robot dimension (bumper-to-bumper). Used as margin so that zone activates when
   * any part of the robot body overlaps the field element, regardless of orientation.
   */
  private static final double ROBOT_HALF_EXTENT = 0.49; // ~max(0.787, 0.978) / 2

  /** Small hysteresis buffer for X-based zone transitions (ALLIANCE↔NEUTRAL). */
  private static final double X_HYSTERESIS = 0.3;

  /** Tracks the current X-based zone to apply hysteresis. */
  private static Zone currentXZone = Zone.ALLIANCE;

  /** Minimum gyro pitch (degrees) to confirm the robot is actually on a bump. */
  private static final double BUMP_PITCH_THRESHOLD_DEG = 5.0;

  /**
   * Determine the robot's current zone.
   *
   * <p>2D zones (BUMP, TRENCH) take priority over X-based zones. Margins are set to the robot's
   * half-extent so the zone triggers when any part of the robot overlaps the physical element.
   *
   * <p>BUMP requires BOTH position overlap AND gyro tilt above threshold to avoid false positives.
   * If the robot is in the bump rectangle but level, it falls through to the X-based zone.
   *
   * @param robotX Robot X position (field coords)
   * @param robotY Robot Y position (field coords)
   * @param alliance Current alliance
   * @return The active zone
   */
  public static Zone getCurrentZone(double robotX, double robotY, Alliance alliance) {
    return getCurrentZone(robotX, robotY, alliance, 0.0);
  }

  /**
   * Determine the robot's current zone, with gyro pitch for bump confirmation.
   *
   * @param robotX Robot X position (field coords)
   * @param robotY Robot Y position (field coords)
   * @param alliance Current alliance
   * @param robotPitchDeg Absolute gyro pitch in degrees (positive = nose up)
   * @return The active zone
   */
  public static Zone getCurrentZone(
      double robotX, double robotY, Alliance alliance, double robotPitchDeg) {
    // --- Priority 1: 2D obstacle zones (robot-size margins) ---
    // BUMP requires both position AND tilt — if flat, fall through to X-based zone
    if (FieldConstants.BumpZones.isInAnyBumpZone(robotX, robotY, ROBOT_HALF_EXTENT)
        && Math.abs(robotPitchDeg) >= BUMP_PITCH_THRESHOLD_DEG) {
      return Zone.BUMP;
    }
    if (FieldConstants.TrenchZones.isInAnyTrenchZone(robotX, robotY, ROBOT_HALF_EXTENT)) {
      // Determine if we're on the alliance side (NEAR) or neutral side (FAR) of this trench.
      // The trench straddles the hub center line. For blue, alliance is low-X; for red, high-X.
      boolean onAllianceSide = isOnAllianceSideOfTrench(robotX, alliance);
      return onAllianceSide ? Zone.TRENCH_NEAR : Zone.TRENCH_FAR;
    }

    // --- Priority 2: X-based field zones with hysteresis ---
    currentXZone = calculateXZone(robotX, alliance);
    return currentXZone;
  }

  /**
   * Check if the robot's bumper is still touching the alliance-neutral boundary tape.
   *
   * <p>Game rule: shots only count if any part of the robot crosses the alliance zone boundary
   * line. TRENCH_NEAR means our bumper still reaches that tape; TRENCH_FAR means we're entirely
   * past it into neutral territory. The opponent's trench is always FAR.
   */
  private static boolean isOnAllianceSideOfTrench(double robotX, Alliance alliance) {
    double blueHubCenter = FieldConstants.LinesVertical.hubCenter;
    double redHubCenter = FieldConstants.LinesVertical.oppHubCenter;
    double allianceLine = FieldConstants.LinesVertical.allianceZone;
    double oppAllianceLine = FieldConstants.LinesVertical.oppAllianceZone;

    if (alliance == Alliance.Blue) {
      // If we're in the red-side trench, that's always FAR (opponent territory)
      double distToBlue = Math.abs(robotX - blueHubCenter);
      double distToRed = Math.abs(robotX - redHubCenter);
      if (distToRed < distToBlue) return false;
      // NEAR if our bumper still touches the alliance zone tape
      return robotX <= allianceLine + ROBOT_HALF_EXTENT;
    } else {
      double distToBlue = Math.abs(robotX - blueHubCenter);
      double distToRed = Math.abs(robotX - redHubCenter);
      if (distToBlue < distToRed) return false;
      // NEAR if our bumper still touches the alliance zone tape
      return robotX >= oppAllianceLine - ROBOT_HALF_EXTENT;
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
