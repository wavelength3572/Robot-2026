// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.FieldConstants;

/** Helper class for turret aiming calculations. */
public class TurretAimingHelper {

  /** Aiming mode based on robot position. */
  public enum AimMode {
    /** In alliance zone — aim at hub and shoot. */
    SHOOT,
    /** In pass zone — aim at pass target and fire. */
    PASS,
    /** Near bumps/trenches — pre-aim but hold fire. */
    HOLDFIRE
  }

  /** Result of aim target calculation. */
  public record AimResult(Translation2d target, AimMode mode) {}

  /** Hysteresis buffer to prevent rapid mode switching at zone boundaries (meters). */
  private static final double HYSTERESIS_BUFFER = 0.5;

  /**
   * How far on either side of a hub center X to extend the holdfire zone (meters). The
   * bumps/trenches are physical obstacles near each hub, so we suppress firing in this band.
   */
  private static final double HOLDFIRE_RADIUS = 2.0;

  /** Current aim mode with hysteresis applied. */
  private static AimMode currentMode = AimMode.SHOOT;

  /**
   * Get the aim target based on robot position and alliance with hysteresis.
   *
   * @param robotX Robot X position in meters
   * @param robotY Robot Y position in meters
   * @param alliance Current alliance
   * @return AimResult containing target coordinates and aim mode
   */
  public static AimResult getAimTarget(double robotX, double robotY, Alliance alliance) {
    // Update mode with hysteresis
    currentMode = calculateModeWithHysteresis(robotX, alliance);

    if (currentMode == AimMode.SHOOT) {
      double targetX =
          (alliance == Alliance.Blue)
              ? FieldConstants.Hub.innerCenterPoint.getX()
              : FieldConstants.Hub.oppInnerCenterPoint.getX();
      double targetY =
          (alliance == Alliance.Blue)
              ? FieldConstants.Hub.innerCenterPoint.getY()
              : FieldConstants.Hub.oppInnerCenterPoint.getY();
      return new AimResult(new Translation2d(targetX, targetY), AimMode.SHOOT);
    } else if (currentMode == AimMode.PASS) {
      double targetX =
          (alliance == Alliance.Blue)
              ? Constants.StrategyConstants.BLUE_PASS_TARGET_X
              : Constants.StrategyConstants.RED_PASS_TARGET_X;
      double targetY = ZoneDetector.getPassTargetY(robotY);
      return new AimResult(new Translation2d(targetX, targetY), AimMode.PASS);
    } else {
      // HOLDFIRE — pre-aim at pass target but don't fire
      double targetX =
          (alliance == Alliance.Blue)
              ? Constants.StrategyConstants.BLUE_PASS_TARGET_X
              : Constants.StrategyConstants.RED_PASS_TARGET_X;
      double targetY = ZoneDetector.getPassTargetY(robotY);
      return new AimResult(new Translation2d(targetX, targetY), AimMode.HOLDFIRE);
    }
  }

  /**
   * Check if the robot is inside a holdfire zone (near bumps/trenches at either hub).
   *
   * @param robotX Robot X position in meters
   * @return true if the robot is within HOLDFIRE_RADIUS of either hub center
   */
  private static boolean isInHoldfireZone(double robotX) {
    double allianceHub = FieldConstants.LinesVertical.hubCenter;
    double opponentHub = FieldConstants.LinesVertical.oppHubCenter;
    return Math.abs(robotX - allianceHub) < HOLDFIRE_RADIUS
        || Math.abs(robotX - opponentHub) < HOLDFIRE_RADIUS;
  }

  /**
   * Calculate the aim mode with hysteresis to prevent rapid switching.
   *
   * <p>Field layout for Blue (mirrored for Red):
   *
   * <pre>
   * SHOOT | HOLDFIRE (near hub) | PASS | HOLDFIRE (near opp hub) | no firing
   * </pre>
   *
   * @param robotX Robot X position in meters
   * @param alliance Current alliance
   * @return The aim mode after applying hysteresis
   */
  private static AimMode calculateModeWithHysteresis(double robotX, Alliance alliance) {
    double allianceZoneEnd = FieldConstants.LinesVertical.allianceZone;
    double fieldLength = FieldConstants.fieldLength;
    boolean inHoldfire = isInHoldfireZone(robotX);

    if (alliance == Alliance.Blue) {
      if (robotX < allianceZoneEnd - HYSTERESIS_BUFFER && currentMode != AimMode.SHOOT) {
        return AimMode.SHOOT;
      } else if (robotX > allianceZoneEnd + HYSTERESIS_BUFFER && currentMode == AimMode.SHOOT) {
        // Left alliance zone — go to HOLDFIRE or PASS depending on bump proximity
        return inHoldfire ? AimMode.HOLDFIRE : AimMode.PASS;
      }
      // Outside alliance zone: toggle between PASS and HOLDFIRE based on bump proximity
      if (currentMode == AimMode.HOLDFIRE && !inHoldfire) {
        return AimMode.PASS;
      } else if (currentMode == AimMode.PASS && inHoldfire) {
        return AimMode.HOLDFIRE;
      }
    } else {
      double redAllianceZoneStart = fieldLength - allianceZoneEnd;

      if (robotX > redAllianceZoneStart + HYSTERESIS_BUFFER && currentMode != AimMode.SHOOT) {
        return AimMode.SHOOT;
      } else if (robotX < redAllianceZoneStart - HYSTERESIS_BUFFER
          && currentMode == AimMode.SHOOT) {
        return inHoldfire ? AimMode.HOLDFIRE : AimMode.PASS;
      }
      // Outside alliance zone: toggle between PASS and HOLDFIRE based on bump proximity
      if (currentMode == AimMode.HOLDFIRE && !inHoldfire) {
        return AimMode.PASS;
      } else if (currentMode == AimMode.PASS && inHoldfire) {
        return AimMode.HOLDFIRE;
      }
    }

    return currentMode;
  }
}
