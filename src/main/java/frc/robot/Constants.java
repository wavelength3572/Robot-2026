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

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** Field positions for targeting (in meters). */
  public static final class FieldPositions {
    /** Blue alliance speaker position. */
    public static final double BLUE_HUB_X = 4.575;

    public static final double BLUE_HUB_Y = 4.115;
    /** Red alliance speaker position. */
    public static final double RED_HUB_X = 11.9865;

    public static final double RED_HUB_Y = 4.115;
  }
}
