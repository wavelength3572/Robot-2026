package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/** Utility class for detecting which field zone the robot is in. */
public class ZoneDetector {
  /** Field zones relative to robot's alliance. */
  public enum Zone {
    ALLIANCE,
    NEUTRAL,
    OPPONENT
  }

  /** Determine which zone the robot is in based on X position and alliance. */
  public static Zone getCurrentZone(double robotX, Alliance alliance) {
    double allianceZoneEnd = Constants.FieldPositions.ALLIANCE_ZONE_DEPTH;
    double opponentZoneStart = Constants.FieldPositions.FIELD_LENGTH - allianceZoneEnd;

    if (alliance == Alliance.Blue) {
      if (robotX < allianceZoneEnd) return Zone.ALLIANCE;
      if (robotX > opponentZoneStart) return Zone.OPPONENT;
      return Zone.NEUTRAL;
    } else {
      if (robotX > opponentZoneStart) return Zone.ALLIANCE;
      if (robotX < allianceZoneEnd) return Zone.OPPONENT;
      return Zone.NEUTRAL;
    }
  }

  /** Get the appropriate pass target Y based on robot's Y position. */
  public static double getPassTargetY(double robotY) {
    double fieldCenter = Constants.FieldPositions.FIELD_WIDTH / 2.0;
    if (robotY < fieldCenter) {
      return Constants.FieldPositions.LEFT_TRENCH_Y;
    } else {
      return Constants.FieldPositions.RIGHT_TRENCH_Y;
    }
  }
}
