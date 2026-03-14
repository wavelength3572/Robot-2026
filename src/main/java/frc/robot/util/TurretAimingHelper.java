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
    PASS
  }

  /** Result of aim target calculation. */
  public record AimResult(Translation2d target, AimMode mode) {}

  /** Hysteresis buffer to prevent rapid mode switching at zone boundaries (meters). */
  private static final double HYSTERESIS_BUFFER = 0.5;

  /**
   * How far past the alliance zone line (into neutral zone) the SHOOT zone extends (meters). Larger
   * values mean the robot aims at the hub longer when leaving and switches to hub aiming sooner
   * when returning. Tunable at Tuning/Turret/ShootZoneExtensionMeters.
   */
  private static final LoggedTunableNumber shootZoneExtension =
      new LoggedTunableNumber("Tuning/Turret/ShootZoneExtensionMeters", 1.5);

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
    } else {
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
   * <p>Field layout for Blue (mirrored for Red):
   *
   * <pre>
   * SHOOT | PASS
   * </pre>
   *
   * @param robotX Robot X position in meters
   * @param alliance Current alliance
   * @return The aim mode after applying hysteresis
   */
  private static AimMode calculateModeWithHysteresis(double robotX, Alliance alliance) {
    double allianceZoneEnd = FieldConstants.LinesVertical.allianceZone;
    double fieldLength = FieldConstants.fieldLength;

    double extension = shootZoneExtension.get();

    if (alliance == Alliance.Blue) {
      double shootBoundary = allianceZoneEnd + extension;
      if (robotX < shootBoundary - HYSTERESIS_BUFFER && currentMode != AimMode.SHOOT) {
        return AimMode.SHOOT;
      } else if (robotX > shootBoundary + HYSTERESIS_BUFFER && currentMode == AimMode.SHOOT) {
        return AimMode.PASS;
      }
    } else {
      double redShootBoundary = fieldLength - allianceZoneEnd - extension;

      if (robotX > redShootBoundary + HYSTERESIS_BUFFER && currentMode != AimMode.SHOOT) {
        return AimMode.SHOOT;
      } else if (robotX < redShootBoundary - HYSTERESIS_BUFFER && currentMode == AimMode.SHOOT) {
        return AimMode.PASS;
      }
    }

    return currentMode;
  }
}
