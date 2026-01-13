// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldPositions;

/**
 * Helper class for determining turret aiming targets based on robot position and alliance. Supports
 * both shooting (at hub) and passing (to teammates) modes based on field zones.
 */
public class TurretAimingHelper {

  /** The current aiming mode - either shooting at hub or passing to a teammate. */
  public enum AimMode {
    SHOOT,
    PASS
  }

  /** Target position and aiming mode for the turret. */
  public record AimTarget(double x, double y, AimMode mode) {}

  /**
   * Determines the appropriate aiming target based on robot position and alliance.
   *
   * <p>When in the shooting zone (near our alliance wall), aims at our hub. When outside the
   * shooting zone (neutral/opponent area), aims to pass to the center of whichever half of the
   * field the robot is on.
   *
   * @param robotX Robot X position in meters (field coordinates)
   * @param robotY Robot Y position in meters (field coordinates)
   * @param alliance Current alliance color
   * @return AimTarget containing target coordinates and aim mode
   */
  public static AimTarget getTargetForPosition(double robotX, double robotY, Alliance alliance) {
    // Determine if in shooting zone based on alliance
    boolean inShootingZone =
        (alliance == Alliance.Blue)
            ? robotX < FieldPositions.BLUE_SHOOTING_ZONE_MAX_X
            : robotX > FieldPositions.RED_SHOOTING_ZONE_MIN_X;

    if (inShootingZone) {
      // Aim at hub
      double hubX =
          (alliance == Alliance.Blue) ? FieldPositions.BLUE_HUB_X : FieldPositions.RED_HUB_X;
      double hubY =
          (alliance == Alliance.Blue) ? FieldPositions.BLUE_HUB_Y : FieldPositions.RED_HUB_Y;
      return new AimTarget(hubX, hubY, AimMode.SHOOT);
    } else {
      // Aim to pass - target back into our own alliance zone
      // X: aim toward our alliance zone (Blue = low X, Red = high X)
      double passX =
          (alliance == Alliance.Blue)
              ? FieldPositions.BLUE_PASS_TARGET_X
              : FieldPositions.RED_PASS_TARGET_X;
      // Y: target 1/3 from wall (closer to walls to avoid hub net structure)
      double passY =
          (robotY < FieldPositions.FIELD_WIDTH / 2.0)
              ? FieldPositions.LEFT_HALF_PASS_Y
              : FieldPositions.RIGHT_HALF_PASS_Y;
      return new AimTarget(passX, passY, AimMode.PASS);
    }
  }

  private TurretAimingHelper() {
    // Utility class - no instantiation
  }
}
