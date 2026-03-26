package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.FieldConstants;

/**
 * Detects which field zone the robot is in for auto-shoot purposes. Zone is purely a function of
 * robot/turret position — single source of truth.
 *
 * <p>Seven zones, checked in priority order:
 *
 * <ol>
 *   <li><b>BUMP</b> — over the bump ramps flanking the hub. No shooting allowed.
 *   <li><b>ALLIANCE_TRENCH</b> — under trench in alliance zone. Hood clamped, can shoot hub.
 *   <li><b>NEUTRAL_TRENCH</b> — under trench in neutral zone. Hood clamped, no shooting.
 *   <li><b>ALLIANCE_CLOSE</b> — alliance zone, within close-distance boundary of hub
 *   <li><b>ALLIANCE_MID</b> — alliance zone, between close and far distance boundaries
 *   <li><b>ALLIANCE_FAR</b> — alliance zone, beyond far-distance boundary
 *   <li><b>NEUTRAL</b> — mid-field. Pass shots allowed (speed-gated by tunable).
 *   <li><b>OPPONENT</b> — their side. Long pass back to alliance zone.
 * </ol>
 *
 * <p>BUMP zones use the physical bump geometry and require gyro pitch confirmation. TRENCH zone
 * bounds include hood lead distance (~0.5m) so the hood starts lowering before reaching the
 * physical structure. Alliance sub-zones are classified by distance from turret to hub.
 * NEUTRAL/OPPONENT are X-axis-only with a small hysteresis buffer.
 */
public class ZoneDetector {

  /** Field zones for auto-shoot behavior. */
  public enum Zone {
    /** Over a bump — suppress all shooting. */
    BUMP,
    /** Under trench in alliance zone — hood clamped, can shoot hub (low angle). */
    ALLIANCE_TRENCH,
    /** Under trench in neutral zone — hood clamped, suppress shooting (just transiting). */
    NEUTRAL_TRENCH,
    /** Alliance zone, close to hub (within close-distance boundary). */
    ALLIANCE_CLOSE,
    /** Alliance zone, mid range (between close and far distance boundaries). */
    ALLIANCE_MID,
    /** Alliance zone, far from hub (beyond far-distance boundary). */
    ALLIANCE_FAR,
    /** Neutral zone — pass shots, speed-gated. */
    NEUTRAL,
    /** Opponent side — long pass back to alliance zone. */
    OPPONENT
  }

  // ========== Alliance Sub-Zone Distance Boundaries ==========
  // These tunables define the distance-to-hub boundaries between ALLIANCE_CLOSE, ALLIANCE_MID,
  // and ALLIANCE_FAR. They are the single source of truth — ShotCalculator's efficiency
  // interpolation also uses these boundaries so zone edges and efficiency breakpoints always agree.
  // Tunable via NetworkTables under Shots/Zones/.
  private static final LoggedTunableNumber zoneBoundaryClose =
      new LoggedTunableNumber("Shots/Zones/CloseDist", 2.0);
  private static final LoggedTunableNumber zoneBoundaryMid =
      new LoggedTunableNumber("Shots/Zones/MidDist", 3.5);
  private static final LoggedTunableNumber zoneBoundaryFar =
      new LoggedTunableNumber("Shots/Zones/FarDist", 4.1);
  private static final LoggedTunableNumber zoneBoundaryCorner =
      new LoggedTunableNumber("Shots/Zones/CornerDist", 5.1);

  /** Get the close zone boundary distance (meters). */
  public static double getZoneBoundaryClose() {
    return zoneBoundaryClose.get();
  }

  /** Get the mid zone boundary distance (meters). */
  public static double getZoneBoundaryMid() {
    return zoneBoundaryMid.get();
  }

  /** Get the far zone boundary distance (meters). */
  public static double getZoneBoundaryFar() {
    return zoneBoundaryFar.get();
  }

  /** Get the corner zone boundary distance (meters). */
  public static double getZoneBoundaryCorner() {
    return zoneBoundaryCorner.get();
  }

  /** Small hysteresis buffer for X-based zone transitions (ALLIANCE↔NEUTRAL). */
  private static final double X_HYSTERESIS = 0.3;

  /** Tracks whether we're in alliance area for hysteresis (true = alliance side). */
  private static boolean inAllianceArea = true;

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
   * <p>Alliance sub-zone (CLOSE/MID/FAR) is determined by 2D distance from turret to the alliance
   * hub center, using the zone boundary tunables under Shots/Zones/.
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
    // Clamp all positions to field bounds — vision can occasionally report positions outside
    // the field walls, which would cause nonsensical zone assignments.
    double fL = FieldConstants.fieldLength;
    double fW = FieldConstants.fieldWidth;
    robotX = Math.max(0, Math.min(fL, robotX));
    robotY = Math.max(0, Math.min(fW, robotY));
    turretX = Math.max(0, Math.min(fL, turretX));
    turretY = Math.max(0, Math.min(fW, turretY));

    // --- Priority 1: 2D obstacle zones (bounds include lead distance where needed) ---
    // BUMP: turret over a bump ramp AND robot is tilted
    if (FieldConstants.BumpZones.isInAnyBumpZone(turretX, turretY)
        && Math.abs(robotPitchDeg) >= BUMP_PITCH_THRESHOLD_DEG) {
      return Zone.BUMP;
    }
    // TRENCH: turret in trench zone (bounds already include hood lead distance)
    if (FieldConstants.TrenchZones.isInAnyTrenchZone(turretX, turretY)) {
      boolean onAllianceSide = isOnAllianceSideOfTrench(robotX, alliance);
      return onAllianceSide ? Zone.ALLIANCE_TRENCH : Zone.NEUTRAL_TRENCH;
    }

    // --- Priority 2: X-based field zones with hysteresis ---
    boolean isAlliance = calculateIsAllianceArea(robotX, alliance);
    inAllianceArea = isAlliance;

    if (!isAlliance) {
      return isInOpponentZone(robotX, alliance) ? Zone.OPPONENT : Zone.NEUTRAL;
    }

    // Alliance area — classify by distance from turret to hub
    double hubX =
        (alliance == Alliance.Blue)
            ? FieldConstants.Hub.innerCenterPoint.getX()
            : FieldConstants.Hub.oppInnerCenterPoint.getX();
    double hubY =
        (alliance == Alliance.Blue)
            ? FieldConstants.Hub.innerCenterPoint.getY()
            : FieldConstants.Hub.oppInnerCenterPoint.getY();
    double distToHub = Math.hypot(turretX - hubX, turretY - hubY);
    return classifyAllianceDistance(distToHub);
  }

  /**
   * Classify alliance sub-zone by distance. Returns ALLIANCE_CLOSE, ALLIANCE_MID, or ALLIANCE_FAR.
   */
  private static Zone classifyAllianceDistance(double distanceM) {
    double closeDist = zoneBoundaryClose.get();
    double midDist = zoneBoundaryMid.get();
    if (distanceM <= closeDist) return Zone.ALLIANCE_CLOSE;
    if (distanceM >= midDist) return Zone.ALLIANCE_FAR;
    return Zone.ALLIANCE_MID;
  }

  /**
   * Check if the robot is on the alliance (near) side of the trench physical barrier.
   *
   * <p>The barrier sits at the hub center line. ALLIANCE_TRENCH means the robot is between the
   * alliance wall and the barrier; NEUTRAL_TRENCH means the robot is past the barrier toward
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

  /** Check if robot X is in the opponent zone (far end of field). */
  private static boolean isInOpponentZone(double robotX, Alliance alliance) {
    double allianceZoneEnd = FieldConstants.LinesVertical.allianceZone;
    double opponentZoneStart = FieldConstants.fieldLength - allianceZoneEnd;
    if (alliance == Alliance.Blue) {
      return robotX > opponentZoneStart;
    } else {
      return robotX < allianceZoneEnd;
    }
  }

  /**
   * X-based alliance area check with hysteresis on the ALLIANCE↔NEUTRAL boundary.
   *
   * <p>Hysteresis prevents flickering when driving along the boundary.
   *
   * @return true if in alliance area, false if neutral/opponent
   */
  private static boolean calculateIsAllianceArea(double robotX, Alliance alliance) {
    double allianceZoneEnd = FieldConstants.LinesVertical.allianceZone;
    double opponentZoneStart = FieldConstants.fieldLength - allianceZoneEnd;

    double boundary;
    boolean allianceIsLowX;
    if (alliance == Alliance.Blue) {
      boundary = allianceZoneEnd;
      allianceIsLowX = true;
    } else {
      boundary = opponentZoneStart;
      allianceIsLowX = false;
    }

    if (allianceIsLowX) {
      // Blue: X < boundary = alliance, X > boundary = neutral/opponent
      if (inAllianceArea) {
        return robotX <= boundary + X_HYSTERESIS;
      } else {
        return robotX < boundary - X_HYSTERESIS;
      }
    } else {
      // Red: X > boundary = alliance, X < boundary = neutral/opponent
      if (inAllianceArea) {
        return robotX >= boundary - X_HYSTERESIS;
      } else {
        return robotX > boundary + X_HYSTERESIS;
      }
    }
  }

  /**
   * Check if a zone is any alliance-side zone. Useful for checks that apply to the entire alliance
   * area regardless of distance.
   *
   * @param zone The zone to check
   * @return true if ALLIANCE_CLOSE, ALLIANCE_MID, or ALLIANCE_FAR
   */
  public static boolean isAllianceZone(Zone zone) {
    return zone == Zone.ALLIANCE_CLOSE || zone == Zone.ALLIANCE_MID || zone == Zone.ALLIANCE_FAR;
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
