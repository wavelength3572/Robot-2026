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
    SHOOT,
    PASS
  }

  /** Result of aim target calculation. */
  public record AimResult(Translation2d target, AimMode mode) {}

  /** Hysteresis buffer to prevent rapid mode switching at zone boundaries (meters). */
  private static final double HYSTERESIS_BUFFER = 0.5;

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
      // In our zone - aim at hub to shoot
      double targetX =
          (alliance == Alliance.Blue)
              ? FieldConstants.Hub.innerCenterPoint.getX()
              : FieldConstants.Hub.oppInnerCenterPoint.getX();
      double targetY =
          (alliance == Alliance.Blue)
              ? FieldConstants.Hub.innerCenterPoint.getY()
              : FieldConstants.Hub.oppInnerCenterPoint.getY();
      return new AimResult(new Translation2d(targetX, targetY), AimMode.SHOOT);
    } else {
      // In neutral/opponent zone - aim at pass target
      double targetX =
          (alliance == Alliance.Blue)
              ? Constants.StrategyConstants.BLUE_PASS_TARGET_X
              : Constants.StrategyConstants.RED_PASS_TARGET_X;
      double targetY = ZoneDetector.getPassTargetY(robotY);
      return new AimResult(new Translation2d(targetX, targetY), AimMode.PASS);
    }
  }

  /**
   * Calculate the aim mode with hysteresis to prevent rapid switching.
   *
   * @param robotX Robot X position in meters
   * @param alliance Current alliance
   * @return The aim mode after applying hysteresis
   */
  private static AimMode calculateModeWithHysteresis(double robotX, Alliance alliance) {
    double allianceZoneEnd = FieldConstants.LinesVertical.allianceZone;
    double fieldLength = FieldConstants.fieldLength;

    if (alliance == Alliance.Blue) {
      // Blue alliance zone is at low X values
      if (currentMode == AimMode.SHOOT) {
        // Only switch to PASS when clearly outside alliance zone
        if (robotX > allianceZoneEnd + HYSTERESIS_BUFFER) {
          return AimMode.PASS;
        }
      } else {
        // Only switch to SHOOT when clearly inside alliance zone
        if (robotX < allianceZoneEnd - HYSTERESIS_BUFFER) {
          return AimMode.SHOOT;
        }
      }
    } else {
      // Red alliance zone is at high X values
      double redAllianceZoneStart = fieldLength - allianceZoneEnd;
      if (currentMode == AimMode.SHOOT) {
        // Only switch to PASS when clearly outside alliance zone
        if (robotX < redAllianceZoneStart - HYSTERESIS_BUFFER) {
          return AimMode.PASS;
        }
      } else {
        // Only switch to SHOOT when clearly inside alliance zone
        if (robotX > redAllianceZoneStart + HYSTERESIS_BUFFER) {
          return AimMode.SHOOT;
        }
      }
    }

    return currentMode;
  }
}
