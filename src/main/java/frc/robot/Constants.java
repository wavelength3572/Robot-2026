// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final RobotType currentRobot = RobotType.MAINBOT;

  private static RobotConfig robotConfig = null;

  public static RobotConfig getRobotConfig() {
    if (robotConfig == null) {
      switch (currentRobot) {
        case MAINBOT:
          robotConfig = new MainBotConfig();
          break;
        case MINIBOT:
          robotConfig = new MiniBotConfig();
          break;
      }
    }
    return robotConfig;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum RobotType {
    /** MainBot2026 - 21.25" chassis with NEO drive motors */
    MAINBOT,

    /** MiniBot2026 - 14" chassis with NEO Vortex drive motors */
    MINIBOT
  }

  /** Field positions for targeting (in meters). */
  public static final class FieldPositions {
    /** Blue alliance speaker position. */
    public static final double BLUE_HUB_X = 4.6; // TODO: Update with actual field coordinates

    public static final double BLUE_HUB_Y = 4.0; // TODO: Update with actual field coordinates

    /** Red alliance speaker position. */
    public static final double RED_HUB_X = 4.6; // TODO: Update with actual field coordinates

    public static final double RED_HUB_Y = 4.0; // TODO: Update with actual field coordinates
  }
}
