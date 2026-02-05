package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;

/**
 * A tunable number that can be adjusted via NetworkTables during tuning mode. Values are published
 * under /SmartDashboard/{key} and can be modified from Shuffleboard, AdvantageScope, or other NT
 * clients. Using SmartDashboard path allows tunables to appear alongside command buttons in the
 * same hierarchy.
 *
 * <p>In competition mode (non-tuning), values are fixed at their defaults for performance.
 */
public class LoggedTunableNumber {
  private static final String TABLE_KEY = "/SmartDashboard";

  private final String key;
  private final double defaultValue;
  private final DoubleEntry entry;
  private double lastValue;

  private static final Map<String, LoggedTunableNumber> instances = new HashMap<>();

  /**
   * Creates a new LoggedTunableNumber.
   *
   * @param key The NetworkTables key (will be prefixed with /Tuning/)
   * @param defaultValue The default value
   */
  public LoggedTunableNumber(String key, double defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;
    this.lastValue = defaultValue;

    // Only create NT entry in tuning mode
    if (isTuningMode()) {
      entry =
          NetworkTableInstance.getDefault()
              .getDoubleTopic(TABLE_KEY + "/" + key)
              .getEntry(defaultValue);
      entry.set(defaultValue);
    } else {
      entry = null;
    }

    instances.put(key, this);
  }

  /**
   * Gets the current value. In tuning mode, reads from NetworkTables. Otherwise returns the default
   * value.
   *
   * @return The current value
   */
  public double get() {
    if (isTuningMode() && entry != null) {
      lastValue = entry.get(defaultValue);
    }
    return lastValue;
  }

  /**
   * Checks if the value has changed since the last call to hasChanged().
   *
   * @return True if the value has changed
   */
  public boolean hasChanged() {
    if (!isTuningMode() || entry == null) {
      return false;
    }
    double currentValue = entry.get(defaultValue);
    if (currentValue != lastValue) {
      lastValue = currentValue;
      return true;
    }
    return false;
  }

  /**
   * Checks if any of the provided tunable numbers have changed.
   *
   * @param tunables The tunable numbers to check
   * @return True if any value has changed
   */
  public static boolean hasChanged(LoggedTunableNumber... tunables) {
    boolean changed = false;
    for (LoggedTunableNumber tunable : tunables) {
      if (tunable.entry != null) {
        double currentNTValue = tunable.entry.get(tunable.defaultValue);
        if (currentNTValue != tunable.lastValue) {
          tunable.lastValue = currentNTValue;
          changed = true;
        }
      }
    }
    return changed;
  }

  /** Returns true if tuning mode is enabled (non-competition, real or sim mode). */
  private static boolean isTuningMode() {
    // Enable tuning in REAL and SIM modes, but could be disabled for competition
    return Constants.currentMode == Constants.Mode.REAL
        || Constants.currentMode == Constants.Mode.SIM;
  }

  /** Gets the key for this tunable number. */
  public String getKey() {
    return key;
  }

  /** Gets the default value. */
  public double getDefault() {
    return defaultValue;
  }
}
