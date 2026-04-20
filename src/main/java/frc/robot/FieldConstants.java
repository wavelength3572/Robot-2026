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
import edu.wpi.first.wpilibj.DriverStation;
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
   * Safe landing zone for geo-fenced passing. Defined from blue alliance perspective; red alliance
   * mirrors X bounds across field center. The geo-fence approves a pass shot iff the predicted
   * landing point (after shrinking by a tunable margin) falls inside this rectangle.
   */
  public static class PassSafeZone {
    /** Blue alliance zone X bounds: ball must land between alliance wall and zone line. */
    public static final double blueMinX = 0.0;

    public static final double blueMaxX = LinesVertical.allianceZone;

    /** Y bounds: full field width (ball must not leave the sidelines). */
    public static final double minY = 0.0;

    public static final double maxY = fieldWidth;
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

    // Climbing pole positions (alliance side) — all at same X, different Y.
    public static final Translation2d[] poles = {leftUpright, rightUpright};

    // Measured robot climb poses (blue alliance) — robot position + heading when latched on pole.
    public static final Pose2d leftClimbPose =
        new Pose2d(1.013, 4.577, Rotation2d.fromDegrees(90.0));
    public static final Pose2d rightClimbPose =
        new Pose2d(1.109, 2.916, Rotation2d.fromDegrees(-90.0));
    public static final Pose2d[] climbPoses = {leftClimbPose, rightClimbPose};

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

    // Climbing pole positions (opposing side)
    public static final Translation2d[] oppPoles = {oppLeftUpright, oppRightUpright};

    // Measured robot climb poses (red alliance) — 180° rotation of blue poses around field center.
    public static final Pose2d oppLeftClimbPose =
        new Pose2d(fieldLength - 1.013, fieldWidth - 4.577, Rotation2d.fromDegrees(-90.0));
    public static final Pose2d oppRightClimbPose =
        new Pose2d(fieldLength - 1.109, fieldWidth - 2.916, Rotation2d.fromDegrees(90.0));
    public static final Pose2d[] oppClimbPoses = {oppLeftClimbPose, oppRightClimbPose};
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
   * <p>Each zone is defined by X and Y bounds. The bounds include hood lead distance (~0.5m) so the
   * robot enters the zone early enough for the hood to lower at max speed. The zone boundaries ARE
   * the detection boundaries — no separate margin is applied at runtime.
   *
   * <p>Bounds are clamped to field walls so vision glitches can't trigger false zone entries.
   */
  public static class TrenchZones {
    /**
     * Hood lead distance baked into zone bounds (~20 inches). At ~4 m/s this gives the hood ~125ms
     * to lower from max angle to the trench-safe 18 degrees before reaching the physical structure.
     */
    public static final double HOOD_LEAD_METERS = 0.5;

    /**
     * Extra X-direction extension beyond HOOD_LEAD_METERS. Widens each trench zone 0.5m further on
     * each end so the alliance trench reaches deeper into the alliance zone and the neutral side
     * reaches further into neutral/opponent territory.
     */
    public static final double X_EXTRA_LEAD_METERS = 0.5;

    // X extent: trench depth centered on hub center line, expanded by lead + extra X lead
    private static final double halfDepth = LeftTrench.depth / 2.0;

    // ---- Blue-side trenches (centered on hubCenter, expanded by lead distances) ----
    public static final double BLUE_LEFT_MIN_X =
        LinesVertical.hubCenter - halfDepth - HOOD_LEAD_METERS - X_EXTRA_LEAD_METERS;
    public static final double BLUE_LEFT_MAX_X =
        LinesVertical.hubCenter + halfDepth + HOOD_LEAD_METERS + X_EXTRA_LEAD_METERS;
    public static final double BLUE_LEFT_MIN_Y =
        fieldWidth - LeftTrench.openingWidth - HOOD_LEAD_METERS;
    public static final double BLUE_LEFT_MAX_Y = fieldWidth; // field wall, no expansion needed

    public static final double BLUE_RIGHT_MIN_X =
        LinesVertical.hubCenter - halfDepth - HOOD_LEAD_METERS - X_EXTRA_LEAD_METERS;
    public static final double BLUE_RIGHT_MAX_X =
        LinesVertical.hubCenter + halfDepth + HOOD_LEAD_METERS + X_EXTRA_LEAD_METERS;
    public static final double BLUE_RIGHT_MIN_Y = 0; // field wall, no expansion needed
    public static final double BLUE_RIGHT_MAX_Y = RightTrench.openingWidth + HOOD_LEAD_METERS;

    // ---- Red-side trenches (centered on oppHubCenter, expanded by lead distances) ----
    public static final double RED_LEFT_MIN_X =
        LinesVertical.oppHubCenter - halfDepth - HOOD_LEAD_METERS - X_EXTRA_LEAD_METERS;
    public static final double RED_LEFT_MAX_X =
        LinesVertical.oppHubCenter + halfDepth + HOOD_LEAD_METERS + X_EXTRA_LEAD_METERS;
    public static final double RED_LEFT_MIN_Y =
        fieldWidth - LeftTrench.openingWidth - HOOD_LEAD_METERS;
    public static final double RED_LEFT_MAX_Y = fieldWidth; // field wall

    public static final double RED_RIGHT_MIN_X =
        LinesVertical.oppHubCenter - halfDepth - HOOD_LEAD_METERS - X_EXTRA_LEAD_METERS;
    public static final double RED_RIGHT_MAX_X =
        LinesVertical.oppHubCenter + halfDepth + HOOD_LEAD_METERS + X_EXTRA_LEAD_METERS;
    public static final double RED_RIGHT_MIN_Y = 0; // field wall
    public static final double RED_RIGHT_MAX_Y = RightTrench.openingWidth + HOOD_LEAD_METERS;

    /**
     * Check if a point is inside any of the 4 trench zones (bounds already include lead distance).
     */
    public static boolean isInAnyTrenchZone(double x, double y) {
      return isInZone(x, y, BLUE_LEFT_MIN_X, BLUE_LEFT_MAX_X, BLUE_LEFT_MIN_Y, BLUE_LEFT_MAX_Y)
          || isInZone(x, y, BLUE_RIGHT_MIN_X, BLUE_RIGHT_MAX_X, BLUE_RIGHT_MIN_Y, BLUE_RIGHT_MAX_Y)
          || isInZone(x, y, RED_LEFT_MIN_X, RED_LEFT_MAX_X, RED_LEFT_MIN_Y, RED_LEFT_MAX_Y)
          || isInZone(x, y, RED_RIGHT_MIN_X, RED_RIGHT_MAX_X, RED_RIGHT_MIN_Y, RED_RIGHT_MAX_Y);
    }

    /**
     * Check if a point is inside only the alliance's own trench zones. The opponent's trench is
     * irrelevant for hood safety when the robot is in neutral/opponent territory.
     */
    public static boolean isInAllianceTrenchZone(
        double x, double y, DriverStation.Alliance alliance) {
      if (alliance == DriverStation.Alliance.Blue) {
        return isInZone(x, y, BLUE_LEFT_MIN_X, BLUE_LEFT_MAX_X, BLUE_LEFT_MIN_Y, BLUE_LEFT_MAX_Y)
            || isInZone(
                x, y, BLUE_RIGHT_MIN_X, BLUE_RIGHT_MAX_X, BLUE_RIGHT_MIN_Y, BLUE_RIGHT_MAX_Y);
      } else {
        return isInZone(x, y, RED_LEFT_MIN_X, RED_LEFT_MAX_X, RED_LEFT_MIN_Y, RED_LEFT_MAX_Y)
            || isInZone(x, y, RED_RIGHT_MIN_X, RED_RIGHT_MAX_X, RED_RIGHT_MIN_Y, RED_RIGHT_MAX_Y);
      }
    }

    /**
     * Check if a point is inside the opponent's trench zones using bump-aligned X bounds (no lead
     * distance). Used for hood safety in opponent territory without the expanded bounds that would
     * overlap neutral zone passing lanes near the walls.
     */
    public static boolean isInOpponentTrenchZoneTight(
        double x, double y, DriverStation.Alliance alliance) {
      // X bounds match bump zones (hub center ± halfDepth, no lead).
      // Y bounds use physical trench opening width only (no hood lead).
      if (alliance == DriverStation.Alliance.Blue) {
        // Opponent is red — use red bump X extents
        return isInZone(
                x,
                y,
                BumpZones.RED_LEFT_MIN_X,
                BumpZones.RED_LEFT_MAX_X,
                fieldWidth - LeftTrench.openingWidth,
                fieldWidth)
            || isInZone(
                x,
                y,
                BumpZones.RED_RIGHT_MIN_X,
                BumpZones.RED_RIGHT_MAX_X,
                0,
                RightTrench.openingWidth);
      } else {
        // Opponent is blue — use blue bump X extents
        return isInZone(
                x,
                y,
                BumpZones.BLUE_LEFT_MIN_X,
                BumpZones.BLUE_LEFT_MAX_X,
                fieldWidth - LeftTrench.openingWidth,
                fieldWidth)
            || isInZone(
                x,
                y,
                BumpZones.BLUE_RIGHT_MIN_X,
                BumpZones.BLUE_RIGHT_MAX_X,
                0,
                RightTrench.openingWidth);
      }
    }

    /** Get the name of the trench zone the point is in (for logging), or empty string if none. */
    public static String getActiveTrenchZone(double x, double y) {
      if (isInZone(x, y, BLUE_LEFT_MIN_X, BLUE_LEFT_MAX_X, BLUE_LEFT_MIN_Y, BLUE_LEFT_MAX_Y))
        return "BLUE_LEFT";
      if (isInZone(x, y, BLUE_RIGHT_MIN_X, BLUE_RIGHT_MAX_X, BLUE_RIGHT_MIN_Y, BLUE_RIGHT_MAX_Y))
        return "BLUE_RIGHT";
      if (isInZone(x, y, RED_LEFT_MIN_X, RED_LEFT_MAX_X, RED_LEFT_MIN_Y, RED_LEFT_MAX_Y))
        return "RED_LEFT";
      if (isInZone(x, y, RED_RIGHT_MIN_X, RED_RIGHT_MAX_X, RED_RIGHT_MIN_Y, RED_RIGHT_MAX_Y))
        return "RED_RIGHT";
      return "";
    }

    /**
     * Half-width of the DANGER_TRENCH zone (meters). The zone extends this far on each side of the
     * alliance/neutral trench boundary (hubCenter for blue, oppHubCenter for red). Total width is
     * 2x this value (~24 inches).
     */
    public static final double DANGER_ZONE_HALF_WIDTH_METERS = 0.3;

    /**
     * Check if a point is inside the danger zone at the alliance/neutral trench boundary. Only
     * checks the robot's own alliance trenches (not opponent). The danger zone uses alliance trench
     * Y-bounds (robot must be laterally under the trench) and a narrow X-band centered on the hub
     * center line.
     */
    public static boolean isInDangerTrenchZone(
        double x, double y, DriverStation.Alliance alliance) {
      double dangerHalf = DANGER_ZONE_HALF_WIDTH_METERS;
      if (alliance == DriverStation.Alliance.Blue) {
        double centerX = LinesVertical.hubCenter;
        // Check both blue trenches (left and right) with danger X bounds
        return isInZone(
                x, y, centerX - dangerHalf, centerX + dangerHalf, BLUE_LEFT_MIN_Y, BLUE_LEFT_MAX_Y)
            || isInZone(
                x,
                y,
                centerX - dangerHalf,
                centerX + dangerHalf,
                BLUE_RIGHT_MIN_Y,
                BLUE_RIGHT_MAX_Y);
      } else {
        double centerX = LinesVertical.oppHubCenter;
        // Check both red trenches (left and right) with danger X bounds
        return isInZone(
                x, y, centerX - dangerHalf, centerX + dangerHalf, RED_LEFT_MIN_Y, RED_LEFT_MAX_Y)
            || isInZone(
                x, y, centerX - dangerHalf, centerX + dangerHalf, RED_RIGHT_MIN_Y, RED_RIGHT_MAX_Y);
      }
    }

    /** Check if point is inside a rectangle, clamped to field walls. */
    static boolean isInZone(
        double x, double y, double minX, double maxX, double minY, double maxY) {
      double effMinX = Math.max(0, minX);
      double effMaxX = Math.min(fieldLength, maxX);
      double effMinY = Math.max(0, minY);
      double effMaxY = Math.min(fieldWidth, maxY);
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

    /** Check if a point is inside any of the 4 bump zones. */
    public static boolean isInAnyBumpZone(double x, double y) {
      return TrenchZones.isInZone(
              x, y, BLUE_LEFT_MIN_X, BLUE_LEFT_MAX_X, BLUE_LEFT_MIN_Y, BLUE_LEFT_MAX_Y)
          || TrenchZones.isInZone(
              x, y, BLUE_RIGHT_MIN_X, BLUE_RIGHT_MAX_X, BLUE_RIGHT_MIN_Y, BLUE_RIGHT_MAX_Y)
          || TrenchZones.isInZone(
              x, y, RED_LEFT_MIN_X, RED_LEFT_MAX_X, RED_LEFT_MIN_Y, RED_LEFT_MAX_Y)
          || TrenchZones.isInZone(
              x, y, RED_RIGHT_MIN_X, RED_RIGHT_MAX_X, RED_RIGHT_MIN_Y, RED_RIGHT_MAX_Y);
    }
  }

  /**
   * Log blue-side zone boundaries as Pose2d trajectory arrays for AdvantageScope's 2D field view.
   * Call once at startup after Logger.start(). Uses the actual detection margins from ZoneDetector
   * so visualized boundaries match what the code checks. Set each entry's display type to
   * "Trajectory" in AdvantageScope to see connected outlines.
   */
  public static void logZoneBoundaries() {
    double hub = LinesVertical.hubCenter;
    double allianceX = hub; // Alliance/neutral boundary aligns with hub center
    double neutralEndX = fieldLength - allianceX; // opponent zone starts here

    // ALLIANCE zone — rectangle from field origin to hub center line, full width
    logRect("Visualizations/Zones/Alliance", 0, allianceX, 0, fieldWidth);

    // NEUTRAL zone — rectangle between alliance and opponent boundaries
    logRect("Visualizations/Zones/Neutral", allianceX, neutralEndX, 0, fieldWidth);

    // OPPONENT zone — far end of field
    logRect("Visualizations/Zones/Opponent", neutralEndX, fieldLength, 0, fieldWidth);

    // ALLIANCE_TRENCH — alliance-side half of each blue trench (X <= hubCenter)
    // Bounds already include hood lead distance
    logRect(
        "Visualizations/Zones/AllianceTrench_Left",
        TrenchZones.BLUE_LEFT_MIN_X,
        hub,
        TrenchZones.BLUE_LEFT_MIN_Y,
        TrenchZones.BLUE_LEFT_MAX_Y);
    logRect(
        "Visualizations/Zones/AllianceTrench_Right",
        TrenchZones.BLUE_RIGHT_MIN_X,
        hub,
        TrenchZones.BLUE_RIGHT_MIN_Y,
        TrenchZones.BLUE_RIGHT_MAX_Y);

    // DANGER_TRENCH — narrow band straddling hubCenter in each blue trench
    double dangerHalf = TrenchZones.DANGER_ZONE_HALF_WIDTH_METERS;
    logRect(
        "Visualizations/Zones/DangerTrench_Left",
        hub - dangerHalf,
        hub + dangerHalf,
        TrenchZones.BLUE_LEFT_MIN_Y,
        TrenchZones.BLUE_LEFT_MAX_Y);
    logRect(
        "Visualizations/Zones/DangerTrench_Right",
        hub - dangerHalf,
        hub + dangerHalf,
        TrenchZones.BLUE_RIGHT_MIN_Y,
        TrenchZones.BLUE_RIGHT_MAX_Y);

    // NEUTRAL_TRENCH — neutral-side half of each blue trench (X > hubCenter)
    logRect(
        "Visualizations/Zones/NeutralTrench_Left",
        hub,
        TrenchZones.BLUE_LEFT_MAX_X,
        TrenchZones.BLUE_LEFT_MIN_Y,
        TrenchZones.BLUE_LEFT_MAX_Y);
    logRect(
        "Visualizations/Zones/NeutralTrench_Right",
        hub,
        TrenchZones.BLUE_RIGHT_MAX_X,
        TrenchZones.BLUE_RIGHT_MIN_Y,
        TrenchZones.BLUE_RIGHT_MAX_Y);

    // OPPONENT_TRENCH — bump-aligned X bounds (no lead), matches actual detection
    logRect(
        "Visualizations/Zones/OpponentTrench_Left",
        BumpZones.RED_LEFT_MIN_X,
        BumpZones.RED_LEFT_MAX_X,
        fieldWidth - LeftTrench.openingWidth,
        fieldWidth);
    logRect(
        "Visualizations/Zones/OpponentTrench_Right",
        BumpZones.RED_RIGHT_MIN_X,
        BumpZones.RED_RIGHT_MAX_X,
        0,
        RightTrench.openingWidth);

    // BUMP — blue-side bump zones (no lead distance — pitch confirms)
    logRect(
        "Visualizations/Zones/Bump_Left",
        BumpZones.BLUE_LEFT_MIN_X,
        BumpZones.BLUE_LEFT_MAX_X,
        BumpZones.BLUE_LEFT_MIN_Y,
        BumpZones.BLUE_LEFT_MAX_Y);
    logRect(
        "Visualizations/Zones/Bump_Right",
        BumpZones.BLUE_RIGHT_MIN_X,
        BumpZones.BLUE_RIGHT_MAX_X,
        BumpZones.BLUE_RIGHT_MIN_Y,
        BumpZones.BLUE_RIGHT_MAX_Y);

    // BUMP — red-side bump zones
    logRect(
        "Visualizations/Zones/OpponentBump_Left",
        BumpZones.RED_LEFT_MIN_X,
        BumpZones.RED_LEFT_MAX_X,
        BumpZones.RED_LEFT_MIN_Y,
        BumpZones.RED_LEFT_MAX_Y);
    logRect(
        "Visualizations/Zones/OpponentBump_Right",
        BumpZones.RED_RIGHT_MIN_X,
        BumpZones.RED_RIGHT_MAX_X,
        BumpZones.RED_RIGHT_MIN_Y,
        BumpZones.RED_RIGHT_MAX_Y);

    // ALLIANCE sub-zone arcs (CLOSE/MID/FAR distance boundaries from hub)
    double hubX = Hub.innerCenterPoint.getX();
    double hubY = Hub.innerCenterPoint.getY();
    double closeDist = frc.robot.util.ZoneDetector.getZoneBoundaryClose();
    double midDist = frc.robot.util.ZoneDetector.getZoneBoundaryMid();
    logArc(
        "Visualizations/Zones/AllianceClose_Boundary",
        hubX,
        hubY,
        closeDist,
        0,
        allianceX,
        0,
        fieldWidth);
    logArc(
        "Visualizations/Zones/AllianceMid_Boundary",
        hubX,
        hubY,
        midDist,
        0,
        allianceX,
        0,
        fieldWidth);
  }

  /**
   * Log a closed rectangle outline as a Pose2d trajectory (5 points). Bounds are clamped to field
   * walls so visualized zones match what isInZone() actually checks.
   */
  private static void logRect(String key, double minX, double maxX, double minY, double maxY) {
    double x0 = Math.max(0, minX);
    double x1 = Math.min(fieldLength, maxX);
    double y0 = Math.max(0, minY);
    double y1 = Math.min(fieldWidth, maxY);
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

  /**
   * Log an arc at a given radius from a center point, clipped to a bounding box. The arc is
   * approximated as a polyline with 5-degree steps. Used to visualize the CLOSE/MID/FAR distance
   * boundaries which are circles around the hub, not rectangles.
   */
  private static void logArc(
      String key,
      double centerX,
      double centerY,
      double radius,
      double clipMinX,
      double clipMaxX,
      double clipMinY,
      double clipMaxY) {
    java.util.List<Pose2d> points = new java.util.ArrayList<>();
    Rotation2d r = new Rotation2d();
    // Sweep full circle in 5-degree steps, keep points inside clip bounds
    for (int deg = 0; deg <= 360; deg += 5) {
      double rad = Math.toRadians(deg);
      double x = centerX + radius * Math.cos(rad);
      double y = centerY + radius * Math.sin(rad);
      if (x >= clipMinX && x <= clipMaxX && y >= clipMinY && y <= clipMaxY) {
        points.add(new Pose2d(x, y, r));
      }
    }
    if (!points.isEmpty()) {
      Logger.recordOutput(key, points.toArray(new Pose2d[0]));
    }
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
