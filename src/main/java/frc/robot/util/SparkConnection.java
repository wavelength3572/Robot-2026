package frc.robot.util;

/**
 * Tracks SparkMax/SparkFlex connection status and skips CAN reads when a motor is disconnected.
 *
 * <p>When a motor disconnects, every CAN read blocks until the timeout expires (~5-10ms each). With
 * multiple reads per motor per cycle, a single dead motor can cause loop overruns for the entire
 * robot. This class prevents that by skipping reads on disconnected motors, retrying periodically
 * to detect reconnection.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * private final SparkConnection connection = new SparkConnection();
 *
 * public void updateInputs(Inputs inputs) {
 *   if (connection.isSkipping()) {
 *     inputs.connected = false;
 *     return;
 *   }
 *   sparkStickyFault = false;
 *   ifOk(spark, encoder::getPosition, (v) -> inputs.position = v);
 *   // ... more reads ...
 *   connection.update(connectedDebounce.calculate(!sparkStickyFault));
 * }
 * }</pre>
 */
public class SparkConnection {
  private boolean connected = true;
  private int skipCounter = 0;

  /** How many cycles to skip before retrying a disconnected motor (~1 second at 20ms). */
  private static final int RETRY_INTERVAL = 50;

  /**
   * Returns true if CAN reads should be skipped this cycle. When the motor is disconnected, reads
   * are skipped for RETRY_INTERVAL cycles, then one retry is allowed to check if the motor is back.
   */
  public boolean isSkipping() {
    if (connected) {
      return false;
    }
    if (++skipCounter >= RETRY_INTERVAL) {
      skipCounter = 0;
      return false; // Allow a retry read
    }
    return true; // Skip this cycle
  }

  /**
   * Update the connection status after reads. Call this with the result of your debouncer or
   * connection check at the end of updateInputs().
   */
  public void update(boolean isConnected) {
    this.connected = isConnected;
    if (isConnected) {
      skipCounter = 0;
    }
  }

  /** Returns the current connection status. */
  public boolean isConnected() {
    return connected;
  }
}
