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

/**
 * Helper class for turret aiming calculations.
 *
 * <p>Delegates zone detection to {@link ZoneDetector} and maps zones to aim modes:
 *
 * <ul>
 *   <li>ALLIANCE → SHOOT_ON_THE_MOVE (aim at hub, fire while driving)
 *   <li>ALLIANCE_TRENCH → SHOOT_ON_THE_MOVE (aim at hub, fire while driving)
 *   <li>NEUTRAL → PASS (aim at pass target)
 *   <li>OPPONENT → LONG_PASS (aggressive pass back to alliance zone)
 *   <li>NEUTRAL_TRENCH / BUMP → NONE (suppress shooting, keep last aim target)
 * </ul>
 */
public class TurretAimingHelper {

  /** Aiming mode based on robot position. */
  public enum AimMode {
    /** Aim at hub and shoot while moving (alliance zone, open field, trench). */
    SHOOT_ON_THE_MOVE,
    /** Aim at pass target and fire (neutral zone). */
    PASS,
    /** Aggressive pass from opponent zone — longer distance, steeper trajectory. */
    LONG_PASS,
    /** Suppress shooting — over bump or transiting through far trench. */
    NONE
  }

  /** Result of aim target calculation. */
  public record AimResult(Translation2d target, AimMode mode, ZoneDetector.Zone zone) {}

  /** Last computed aim result, used to keep a stable aim target during NONE mode. */
  private static AimResult lastResult = null;

  /**
   * Get the aim target based on robot position and alliance.
   *
   * <p>Zone detection (including robot-size margins and hysteresis) is handled by {@link
   * ZoneDetector}. This method maps the zone to an aim mode and target.
   *
   * @param robotX Robot X position in meters
   * @param robotY Robot Y position in meters
   * @param alliance Current alliance
   * @return AimResult containing target coordinates, aim mode, and zone
   */
  public static AimResult getAimTarget(double robotX, double robotY, Alliance alliance) {
    return getAimTarget(robotX, robotY, alliance, 0.0);
  }

  /**
   * Get the aim target based on robot position, alliance, and gyro pitch.
   *
   * @param robotX Robot X position in meters
   * @param robotY Robot Y position in meters
   * @param alliance Current alliance
   * @param robotPitchDeg Gyro pitch in degrees for bump confirmation
   * @return AimResult containing target coordinates, aim mode, and zone
   */
  public static AimResult getAimTarget(
      double robotX, double robotY, Alliance alliance, double robotPitchDeg) {
    return getAimTarget(robotX, robotY, alliance, robotPitchDeg, robotX, robotY);
  }

  /**
   * Get the aim target using turret field position for trench detection.
   *
   * @param robotX Robot X position in meters
   * @param robotY Robot Y position in meters
   * @param alliance Current alliance
   * @param robotPitchDeg Gyro pitch in degrees for bump confirmation
   * @param turretX Turret X position in field coords
   * @param turretY Turret Y position in field coords
   * @return AimResult containing target coordinates, aim mode, and zone
   */
  public static AimResult getAimTarget(
      double robotX,
      double robotY,
      Alliance alliance,
      double robotPitchDeg,
      double turretX,
      double turretY) {
    ZoneDetector.Zone zone =
        ZoneDetector.getCurrentZone(robotX, robotY, alliance, robotPitchDeg, turretX, turretY);

    AimResult result =
        switch (zone) {
          case ALLIANCE_CLOSE, ALLIANCE_MID, ALLIANCE_FAR -> {
            Translation2d hubTarget =
                (alliance == Alliance.Blue)
                    ? FieldConstants.Hub.innerCenterPoint.toTranslation2d()
                    : FieldConstants.Hub.oppInnerCenterPoint.toTranslation2d();
            yield new AimResult(hubTarget, AimMode.SHOOT_ON_THE_MOVE, zone);
          }
          case ALLIANCE_TRENCH -> {
            Translation2d hubTarget =
                (alliance == Alliance.Blue)
                    ? FieldConstants.Hub.innerCenterPoint.toTranslation2d()
                    : FieldConstants.Hub.oppInnerCenterPoint.toTranslation2d();
            yield new AimResult(hubTarget, AimMode.SHOOT_ON_THE_MOVE, zone);
          }
          case NEUTRAL -> {
            double targetX =
                (alliance == Alliance.Blue)
                    ? Constants.StrategyConstants.BLUE_PASS_TARGET_X
                    : Constants.StrategyConstants.RED_PASS_TARGET_X;
            double targetY = ZoneDetector.getPassTargetY(robotY);
            yield new AimResult(new Translation2d(targetX, targetY), AimMode.PASS, zone);
          }
          case OPPONENT -> {
            // Long pass — same target area but will use more aggressive shot parameters
            double targetX =
                (alliance == Alliance.Blue)
                    ? Constants.StrategyConstants.BLUE_PASS_TARGET_X
                    : Constants.StrategyConstants.RED_PASS_TARGET_X;
            double targetY = ZoneDetector.getPassTargetY(robotY);
            yield new AimResult(new Translation2d(targetX, targetY), AimMode.LONG_PASS, zone);
          }
          case NEUTRAL_TRENCH, BUMP -> {
            // Keep last aim target for smooth turret motion, but suppress firing
            if (lastResult != null) {
              yield new AimResult(lastResult.target(), AimMode.NONE, zone);
            }
            // Fallback: aim at hub
            Translation2d fallback =
                (alliance == Alliance.Blue)
                    ? FieldConstants.Hub.innerCenterPoint.toTranslation2d()
                    : FieldConstants.Hub.oppInnerCenterPoint.toTranslation2d();
            yield new AimResult(fallback, AimMode.NONE, zone);
          }
        };

    lastResult = result;
    return result;
  }
}
