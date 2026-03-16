package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Detects which field zone the robot is in for auto-shoot purposes.
 *
 * <p>Five zones, checked in priority order:
 *
 * <ol>
 *   <li><b>BUMP</b> — over the bump ramps flanking the hub. No shooting allowed.
 *   <li><b>TRENCH</b> — under the trench hood. Hood must be clamped, only low-angle shots.
 *   <li><b>ALLIANCE</b> — our scoring zone. Aim at hub, speed-gated.
 *   <li><b>NEUTRAL</b> — mid-field. Pass shots allowed (speed-gated by tunable).
 *   <li><b>OPPONENT</b> — their side. No shooting.
 * </ol>
 *
 * <p>BUMP and TRENCH are 2D rectangle checks that use the robot's physical dimensions as margins,
 * so the zone activates when ANY part of the robot overlaps the field element. ALLIANCE/NEUTRAL/
 * OPPONENT are X-axis-only with a small hysteresis buffer to prevent aim-target flickering.
 */
public class ZoneDetector {

  /** Field zones for auto-shoot behavior. */
  public enum Zone {
    /** Over a bump — suppress all shooting. */
    BUMP,
    /** Under a trench hood — clamp hood angle, low shots only. */
    TRENCH,
    /** Alliance scoring zone — aim at hub, fire when slow. */
    ALLIANCE,
    /** Neutral zone — pass shots, speed-gated. */
    NEUTRAL,
    /** Opponent side — suppress shooting. */
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

  /**
   * Determine the robot's current zone.
   *
   * <p>2D zones (BUMP, TRENCH) take priority over X-based zones. Margins are set to the robot's
   * half-extent so the zone triggers when any part of the robot overlaps the physical element.
   *
   * @param robotX Robot X position (field coords)
   * @param robotY Robot Y position (field coords)
   * @param alliance Current alliance
   * @return The active zone
   */
  public static Zone getCurrentZone(double robotX, double robotY, Alliance alliance) {
    // --- Priority 1: 2D obstacle zones (robot-size margins) ---
    if (FieldConstants.BumpZones.isInAnyBumpZone(robotX, robotY, ROBOT_HALF_EXTENT)) {
      Logger.recordOutput("ZoneDetector/Active", "BUMP");
      return Zone.BUMP;
    }
    if (FieldConstants.TrenchZones.isInAnyTrenchZone(robotX, robotY, ROBOT_HALF_EXTENT)) {
      Logger.recordOutput("ZoneDetector/Active", "TRENCH");
      return Zone.TRENCH;
    }

    // --- Priority 2: X-based field zones with hysteresis ---
    currentXZone = calculateXZone(robotX, alliance);
    Logger.recordOutput("ZoneDetector/Active", currentXZone.name());
    return currentXZone;
  }

  /**
   * X-based zone calculation with hysteresis on the ALLIANCE↔NEUTRAL boundary.
   *
   * <p>The shoot zone extends past the alliance line by {@code shootZoneExtension} meters (tuned in
   * TurretAimingHelper). Hysteresis prevents flickering when driving along the boundary.
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
        // Stay ALLIANCE until we're clearly past the boundary
        return (robotX > boundary + X_HYSTERESIS) ? Zone.NEUTRAL : Zone.ALLIANCE;
      } else {
        // Stay NEUTRAL until we're clearly inside the alliance zone
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
