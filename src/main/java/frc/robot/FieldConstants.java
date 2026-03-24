// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

/**
 * Contains information for location of field element and other useful reference points.
 *
 * <p>NOTE: All constants are defined relative to the field coordinate system, and from the
 * perspective of the blue alliance station
 */
public class FieldConstants {

  // AprilTag related constants
  public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  // Field dimensions
  public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
  public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

  /**
   * Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
   */
  public static class LinesVertical {
    public static final double center = fieldLength / 2.0;
    public static final double starting =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX();
    public static final double allianceZone = starting;
    public static final double hubCenter =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + Hub.width / 2.0;
    public static final double neutralZoneNear = center - Units.inchesToMeters(120);
    public static final double neutralZoneFar = center + Units.inchesToMeters(120);
    public static final double oppHubCenter =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + Hub.width / 2.0;
    public static final double oppAllianceZone =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(10).get().getX();
  }

  /**
   * Officially defined and relevant horizontal lines found on the field (defined by Y-axis offset)
   *
   * <p>NOTE: The field element start and end are always left to right from the perspective of the
   * alliance station
   */
  public static class LinesHorizontal {

    public static final double center = fieldWidth / 2.0;

    // Right of hub
    public static final double rightBumpStart = Hub.nearRightCorner.getY();
    public static final double rightBumpEnd = rightBumpStart - RightBump.width;
    public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
    public static final double rightTrenchOpenEnd = 0;

    // Left of hub
    public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
    public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
    public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
    public static final double leftTrenchOpenStart = fieldWidth;
  }

  /** Hub related constants */
  public static class Hub {

    // Dimensions
    public static final double width = Units.inchesToMeters(47.0);
    public static final double height =
        Units.inchesToMeters(72.0); // includes the catcher at the top
    public static final double innerWidth = Units.inchesToMeters(41.7);
    public static final double innerHeight = Units.inchesToMeters(56.5);

    // Relevant reference points on alliance side
    public static final Translation3d topCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            height);
    public static final Translation3d innerCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            innerHeight);

    public static final Translation2d nearLeftCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d nearRightCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d farLeftCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d farRightCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

    // Relevant reference points on the opposite side
    public static final Translation3d oppTopCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            height);
    public static final Translation3d oppInnerCenterPoint =
        new Translation3d(oppTopCenterPoint.getX(), oppTopCenterPoint.getY(), innerHeight);
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d oppNearRightCorner =
        new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d oppFarRightCorner =
        new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

    // Hub faces
    public static final Pose2d nearFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().toPose2d();
    public static final Pose2d farFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(20).get().toPose2d();
    public static final Pose2d rightFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(18).get().toPose2d();
    public static final Pose2d leftFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(21).get().toPose2d();
  }

  /** Left Bump related constants */
  public static class LeftBump {

    // Dimensions
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    // Relevant reference points on alliance side
    public static final Translation2d nearLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
    public static final Translation2d farLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d farRightCorner = Hub.farLeftCorner;

    // Relevant reference points on opposing side
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
  }

  /** Right Bump related constants */
  public static class RightBump {
    // Dimensions
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    // Relevant reference points on alliance side
    public static final Translation2d nearLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
    public static final Translation2d farLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d farRightCorner = Hub.farLeftCorner;

    // Relevant reference points on opposing side
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
  }

  /** Left Trench related constants */
  public static class LeftTrench {
    // Dimensions
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    // Relevant reference points on alliance side
    public static final Translation3d openingTopLeft =
        new Translation3d(LinesVertical.hubCenter, fieldWidth, openingHeight);
    public static final Translation3d openingTopRight =
        new Translation3d(LinesVertical.hubCenter, fieldWidth - openingWidth, openingHeight);

    // Relevant reference points on opposing side
    public static final Translation3d oppOpeningTopLeft =
        new Translation3d(LinesVertical.oppHubCenter, fieldWidth, openingHeight);
    public static final Translation3d oppOpeningTopRight =
        new Translation3d(LinesVertical.oppHubCenter, fieldWidth - openingWidth, openingHeight);
  }

  public static class RightTrench {

    // Dimensions
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    // Relevant reference points on alliance side
    public static final Translation3d openingTopLeft =
        new Translation3d(LinesVertical.hubCenter, openingWidth, openingHeight);
    public static final Translation3d openingTopRight =
        new Translation3d(LinesVertical.hubCenter, 0, openingHeight);

    // Relevant reference points on opposing side
    public static final Translation3d oppOpeningTopLeft =
        new Translation3d(LinesVertical.oppHubCenter, openingWidth, openingHeight);
    public static final Translation3d oppOpeningTopRight =
        new Translation3d(LinesVertical.oppHubCenter, 0, openingHeight);
  }

  /** Tower related constants */
  public static class Tower {
    // Dimensions
    public static final double width = Units.inchesToMeters(49.25);
    public static final double depth = Units.inchesToMeters(45.0);
    public static final double height = Units.inchesToMeters(78.25);
    public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
    public static final double frontFaceX = Units.inchesToMeters(43.51);

    public static final double uprightHeight = Units.inchesToMeters(72.1);

    // Rung heights from the floor
    public static final double lowRungHeight = Units.inchesToMeters(27.0);
    public static final double midRungHeight = Units.inchesToMeters(45.0);
    public static final double highRungHeight = Units.inchesToMeters(63.0);

    // Relevant reference points on alliance side
    public static final Translation2d centerPoint =
        new Translation2d(
            frontFaceX, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY());
    public static final Translation2d leftUpright =
        new Translation2d(
            frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                + innerOpeningWidth / 2
                + Units.inchesToMeters(0.75));
    public static final Translation2d rightUpright =
        new Translation2d(
            frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                - innerOpeningWidth / 2
                - Units.inchesToMeters(0.75));

    // Relevant reference points on opposing side
    public static final Translation2d oppCenterPoint =
        new Translation2d(
            fieldLength - frontFaceX,
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY());
    public static final Translation2d oppLeftUpright =
        new Translation2d(
            fieldLength - frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                + innerOpeningWidth / 2
                + Units.inchesToMeters(0.75));
    public static final Translation2d oppRightUpright =
        new Translation2d(
            fieldLength - frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                - innerOpeningWidth / 2
                - Units.inchesToMeters(0.75));
  }

  public static class Depot {
    // Dimensions
    public static final double width = Units.inchesToMeters(42.0);
    public static final double depth = Units.inchesToMeters(27.0);
    public static final double height = Units.inchesToMeters(1.125);
    public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

    // Relevant reference points on alliance side
    public static final Translation3d depotCenter =
        new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY, height);
    public static final Translation3d leftCorner =
        new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
    public static final Translation3d rightCorner =
        new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
  }

  public static class Outpost {
    // Dimensions
    public static final double width = Units.inchesToMeters(31.8);
    public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
    public static final double height = Units.inchesToMeters(7.0);

    // Relevant reference points on alliance side
    public static final Translation2d centerPoint =
        new Translation2d(0, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(29).get().getY());
  }

  /**
   * Trench avoidance zones — rectangular regions where the robot must keep the hood low to fit
   * under the trench structure. All 4 trenches (both sides, both alliances) are defined here.
   *
   * <p>Each zone is defined by X and Y bounds. The robot's pose is checked against all 4 zones
   * every cycle; if inside any of them, the hood max angle is clamped.
   *
   * <p>The margin is added to all edges to account for robot size and pose uncertainty.
   */
  public static class TrenchZones {
    /** Safety margin added to each edge of the trench zone (meters). */
    public static final double DEFAULT_MARGIN_METERS = 0.4;

    // X extent: trench depth centered on hub center line
    private static final double halfDepth = LeftTrench.depth / 2.0;

    // ---- Blue-side trenches (centered on hubCenter) ----
    public static final double BLUE_LEFT_MIN_X = LinesVertical.hubCenter - halfDepth;
    public static final double BLUE_LEFT_MAX_X = LinesVertical.hubCenter + halfDepth;
    public static final double BLUE_LEFT_MIN_Y = fieldWidth - LeftTrench.openingWidth;
    public static final double BLUE_LEFT_MAX_Y = fieldWidth;

    public static final double BLUE_RIGHT_MIN_X = LinesVertical.hubCenter - halfDepth;
    public static final double BLUE_RIGHT_MAX_X = LinesVertical.hubCenter + halfDepth;
    public static final double BLUE_RIGHT_MIN_Y = 0;
    public static final double BLUE_RIGHT_MAX_Y = RightTrench.openingWidth;

    // ---- Red-side trenches (centered on oppHubCenter) ----
    public static final double RED_LEFT_MIN_X = LinesVertical.oppHubCenter - halfDepth;
    public static final double RED_LEFT_MAX_X = LinesVertical.oppHubCenter + halfDepth;
    public static final double RED_LEFT_MIN_Y = fieldWidth - LeftTrench.openingWidth;
    public static final double RED_LEFT_MAX_Y = fieldWidth;

    public static final double RED_RIGHT_MIN_X = LinesVertical.oppHubCenter - halfDepth;
    public static final double RED_RIGHT_MAX_X = LinesVertical.oppHubCenter + halfDepth;
    public static final double RED_RIGHT_MIN_Y = 0;
    public static final double RED_RIGHT_MAX_Y = RightTrench.openingWidth;

    /**
     * Check if a point (robot pose) is inside any of the 4 trench zones, with margin applied.
     *
     * @param x Robot X position (field coordinates)
     * @param y Robot Y position (field coordinates)
     * @param margin Extra margin in meters added to each edge
     * @return true if inside any trench zone
     */
    public static boolean isInAnyTrenchZone(double x, double y, double margin) {
      return isInZone(
              x, y, BLUE_LEFT_MIN_X, BLUE_LEFT_MAX_X, BLUE_LEFT_MIN_Y, BLUE_LEFT_MAX_Y, margin)
          || isInZone(
              x, y, BLUE_RIGHT_MIN_X, BLUE_RIGHT_MAX_X, BLUE_RIGHT_MIN_Y, BLUE_RIGHT_MAX_Y, margin)
          || isInZone(x, y, RED_LEFT_MIN_X, RED_LEFT_MAX_X, RED_LEFT_MIN_Y, RED_LEFT_MAX_Y, margin)
          || isInZone(
              x, y, RED_RIGHT_MIN_X, RED_RIGHT_MAX_X, RED_RIGHT_MIN_Y, RED_RIGHT_MAX_Y, margin);
    }

    /**
     * Get the name of the trench zone the point is in (for logging), or empty string if none.
     *
     * @param x Robot X position (field coordinates)
     * @param y Robot Y position (field coordinates)
     * @param margin Extra margin in meters added to each edge
     * @return Name of the trench zone, or "" if not in any
     */
    public static String getActiveTrenchZone(double x, double y, double margin) {
      if (isInZone(
          x, y, BLUE_LEFT_MIN_X, BLUE_LEFT_MAX_X, BLUE_LEFT_MIN_Y, BLUE_LEFT_MAX_Y, margin))
        return "BLUE_LEFT";
      if (isInZone(
          x, y, BLUE_RIGHT_MIN_X, BLUE_RIGHT_MAX_X, BLUE_RIGHT_MIN_Y, BLUE_RIGHT_MAX_Y, margin))
        return "BLUE_RIGHT";
      if (isInZone(x, y, RED_LEFT_MIN_X, RED_LEFT_MAX_X, RED_LEFT_MIN_Y, RED_LEFT_MAX_Y, margin))
        return "RED_LEFT";
      if (isInZone(
          x, y, RED_RIGHT_MIN_X, RED_RIGHT_MAX_X, RED_RIGHT_MIN_Y, RED_RIGHT_MAX_Y, margin))
        return "RED_RIGHT";
      return "";
    }

    static boolean isInZone(
        double x, double y, double minX, double maxX, double minY, double maxY, double margin) {
      // Clamp effective bounds to field walls — zones should never extend outside the field.
      // This prevents vision glitches (robot reported at negative X/Y) from triggering zones.
      double effMinX = Math.max(0, minX - margin);
      double effMaxX = Math.min(fieldLength, maxX + margin);
      double effMinY = Math.max(0, minY - margin);
      double effMaxY = Math.min(fieldWidth, maxY + margin);
      return x >= effMinX && x <= effMaxX && y >= effMinY && y <= effMaxY;
    }
  }

  /**
   * Bump zones — rectangular areas over each bump on both sides of the field.
   *
   * <p>Bumps run along the Y-axis between the hub and the trenches. The X extent uses the same
   * depth as trench zones (centered on hub center line). The Y extent spans from the hub corner to
   * the end of the bump.
   */
  public static class BumpZones {
    /** Safety margin added to each edge of the bump zone (meters). */
    public static final double DEFAULT_MARGIN_METERS = 0.4;

    // X extent: same as trench zones (centered on hub center line)
    private static final double halfDepth = LeftTrench.depth / 2.0;

    // ---- Blue-side bumps ----
    public static final double BLUE_LEFT_MIN_X = LinesVertical.hubCenter - halfDepth;
    public static final double BLUE_LEFT_MAX_X = LinesVertical.hubCenter + halfDepth;
    public static final double BLUE_LEFT_MIN_Y = LinesHorizontal.leftBumpEnd; // hub corner
    public static final double BLUE_LEFT_MAX_Y = LinesHorizontal.leftBumpStart; // bump outer edge

    public static final double BLUE_RIGHT_MIN_X = LinesVertical.hubCenter - halfDepth;
    public static final double BLUE_RIGHT_MAX_X = LinesVertical.hubCenter + halfDepth;
    public static final double BLUE_RIGHT_MIN_Y = LinesHorizontal.rightBumpEnd; // bump outer edge
    public static final double BLUE_RIGHT_MAX_Y = LinesHorizontal.rightBumpStart; // hub corner

    // ---- Red-side bumps ----
    public static final double RED_LEFT_MIN_X = LinesVertical.oppHubCenter - halfDepth;
    public static final double RED_LEFT_MAX_X = LinesVertical.oppHubCenter + halfDepth;
    public static final double RED_LEFT_MIN_Y = LinesHorizontal.leftBumpEnd;
    public static final double RED_LEFT_MAX_Y = LinesHorizontal.leftBumpStart;

    public static final double RED_RIGHT_MIN_X = LinesVertical.oppHubCenter - halfDepth;
    public static final double RED_RIGHT_MAX_X = LinesVertical.oppHubCenter + halfDepth;
    public static final double RED_RIGHT_MIN_Y = LinesHorizontal.rightBumpEnd;
    public static final double RED_RIGHT_MAX_Y = LinesHorizontal.rightBumpStart;

    /** Check if a point is inside any of the 4 bump zones, with margin applied. */
    public static boolean isInAnyBumpZone(double x, double y, double margin) {
      return TrenchZones.isInZone(
              x, y, BLUE_LEFT_MIN_X, BLUE_LEFT_MAX_X, BLUE_LEFT_MIN_Y, BLUE_LEFT_MAX_Y, margin)
          || TrenchZones.isInZone(
              x, y, BLUE_RIGHT_MIN_X, BLUE_RIGHT_MAX_X, BLUE_RIGHT_MIN_Y, BLUE_RIGHT_MAX_Y, margin)
          || TrenchZones.isInZone(
              x, y, RED_LEFT_MIN_X, RED_LEFT_MAX_X, RED_LEFT_MIN_Y, RED_LEFT_MAX_Y, margin)
          || TrenchZones.isInZone(
              x, y, RED_RIGHT_MIN_X, RED_RIGHT_MAX_X, RED_RIGHT_MIN_Y, RED_RIGHT_MAX_Y, margin);
    }
  }

  /**
   * Log blue-side zone boundaries as Pose2d trajectory arrays for AdvantageScope's 2D field view.
   * Call once at startup after Logger.start(). Uses the actual detection margin from ZoneDetector
   * so visualized boundaries match what the code checks. Set each entry's display type to
   * "Trajectory" in AdvantageScope to see connected outlines.
   */
  public static void logZoneBoundaries() {
    // Use the actual margins from ZoneDetector so visualization matches detection
    double tm = frc.robot.util.ZoneDetector.TRENCH_MARGIN; // trench detection margin
    double bm = frc.robot.util.ZoneDetector.BUMP_MARGIN; // bump detection margin
    double hub = LinesVertical.hubCenter;
    double allianceX = LinesVertical.allianceZone;
    double neutralEndX = fieldLength - allianceX; // opponent zone starts here

    // ALLIANCE zone — rectangle from field origin to allianceZone line, full width
    logRect("Visualizations/Zones/Alliance", 0, allianceX, 0, fieldWidth, 0);

    // NEUTRAL zone — rectangle between alliance and opponent boundaries
    logRect("Visualizations/Zones/Neutral", allianceX, neutralEndX, 0, fieldWidth, 0);

    // TRENCH_NEAR — alliance-side half of each blue trench (X <= hubCenter)
    logRect(
        "Visualizations/Zones/TrenchNear_Left",
        TrenchZones.BLUE_LEFT_MIN_X,
        hub,
        TrenchZones.BLUE_LEFT_MIN_Y,
        TrenchZones.BLUE_LEFT_MAX_Y,
        tm);
    logRect(
        "Visualizations/Zones/TrenchNear_Right",
        TrenchZones.BLUE_RIGHT_MIN_X,
        hub,
        TrenchZones.BLUE_RIGHT_MIN_Y,
        TrenchZones.BLUE_RIGHT_MAX_Y,
        tm);

    // TRENCH_FAR — neutral-side half of each blue trench (X > hubCenter)
    logRect(
        "Visualizations/Zones/TrenchFar_Left",
        hub,
        TrenchZones.BLUE_LEFT_MAX_X,
        TrenchZones.BLUE_LEFT_MIN_Y,
        TrenchZones.BLUE_LEFT_MAX_Y,
        tm);
    logRect(
        "Visualizations/Zones/TrenchFar_Right",
        hub,
        TrenchZones.BLUE_RIGHT_MAX_X,
        TrenchZones.BLUE_RIGHT_MIN_Y,
        TrenchZones.BLUE_RIGHT_MAX_Y,
        tm);

    // BUMP — blue-side bump zones (smaller margin — pitch confirms)
    logRect(
        "Visualizations/Zones/Bump_Left",
        BumpZones.BLUE_LEFT_MIN_X,
        BumpZones.BLUE_LEFT_MAX_X,
        BumpZones.BLUE_LEFT_MIN_Y,
        BumpZones.BLUE_LEFT_MAX_Y,
        bm);
    logRect(
        "Visualizations/Zones/Bump_Right",
        BumpZones.BLUE_RIGHT_MIN_X,
        BumpZones.BLUE_RIGHT_MAX_X,
        BumpZones.BLUE_RIGHT_MIN_Y,
        BumpZones.BLUE_RIGHT_MAX_Y,
        bm);
  }

  /**
   * Log a closed rectangle outline as a Pose2d trajectory (5 points). Bounds are clamped to field
   * walls so visualized zones match what isInZone() actually checks.
   */
  private static void logRect(
      String key, double minX, double maxX, double minY, double maxY, double margin) {
    double x0 = Math.max(0, minX - margin);
    double x1 = Math.min(fieldLength, maxX + margin);
    double y0 = Math.max(0, minY - margin);
    double y1 = Math.min(fieldWidth, maxY + margin);
    Rotation2d r = new Rotation2d();
    Logger.recordOutput(
        key,
        new Pose2d[] {
          new Pose2d(x0, y0, r),
          new Pose2d(x1, y0, r),
          new Pose2d(x1, y1, r),
          new Pose2d(x0, y1, r),
          new Pose2d(x0, y0, r),
        });
  }

  public enum AprilTagLayoutType {
    OFFICIAL(AprilTagFields.k2026RebuiltWelded);

    private final AprilTagFields field;
    private AprilTagFieldLayout layout;

    AprilTagLayoutType(AprilTagFields field) {
      this.field = field;
    }

    public AprilTagFieldLayout getLayout() {
      if (layout == null) {
        layout = AprilTagFieldLayout.loadField(field);
      }
      return layout;
    }
  }
}
