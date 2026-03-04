package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/**
 * Singleton utility for tracking bench test metrics. Logs to AdvantageKit under
 * "BenchTest/Shooting/Metrics/*" for real-time monitoring in AdvantageScope.
 *
 * <p>Tracks:
 *
 * <ul>
 *   <li>Cumulative shot count (resettable)
 *   <li>Rolling shots-per-second (last 10 shots)
 *   <li>RPM recovery time (ms from shot to RPM returning within tolerance)
 *   <li>Burst statistics (auto-detected groupings of shots)
 *   <li>Motor temperature tracking (max temps during session)
 * </ul>
 */
public class BenchTestMetrics {
  private static BenchTestMetrics instance;

  // Shot tracking
  private int cumulativeShotCount = 0;
  private static final int ROLLING_WINDOW_SIZE = 10;
  private final double[] shotTimestamps = new double[ROLLING_WINDOW_SIZE];
  private int shotTimestampIndex = 0;
  private int shotTimestampCount = 0;

  // Recovery tracking
  private static final double RECOVERY_TOLERANCE_RPM = 50.0;
  private boolean waitingForRecovery = false;
  private double recoveryStartTime = 0.0;
  private double lastRecoveryTimeMs = 0.0;

  // Burst detection
  private static final double BURST_GAP_THRESHOLD_SEC = 1.0; // shots >1s apart = new burst
  private double burstStartTime = 0.0;
  private int burstShotCount = 0;
  private double burstMinRPM = Double.MAX_VALUE;
  private double burstMaxRPM = 0.0;
  private boolean burstActive = false;
  private double lastShotTime = 0.0;

  // Temperature tracking
  private double maxLeaderTempC = 0.0;
  private double maxFollowerTempC = 0.0;

  // Last known state for periodic
  private double currentRPM = 0.0;
  private double targetRPM = 0.0;

  private BenchTestMetrics() {}

  public static BenchTestMetrics getInstance() {
    if (instance == null) {
      instance = new BenchTestMetrics();
    }
    return instance;
  }

  /**
   * Called each robot cycle with current launcher state. Handles recovery detection and continuous
   * logging.
   *
   * @param currentRPM Current wheel RPM
   * @param targetRPM Target wheel RPM
   * @param leaderTempC Leader motor temperature in Celsius
   * @param followerTempC Follower motor temperature in Celsius
   */
  public void periodic(
      double currentRPM, double targetRPM, double leaderTempC, double followerTempC) {
    this.currentRPM = currentRPM;
    this.targetRPM = targetRPM;

    // Temperature tracking
    if (leaderTempC > maxLeaderTempC) {
      maxLeaderTempC = leaderTempC;
    }
    if (followerTempC > maxFollowerTempC) {
      maxFollowerTempC = followerTempC;
    }

    // Recovery detection
    if (waitingForRecovery && targetRPM > 100.0) {
      double error = Math.abs(targetRPM - currentRPM);
      if (error <= RECOVERY_TOLERANCE_RPM) {
        lastRecoveryTimeMs = (Timer.getFPGATimestamp() - recoveryStartTime) * 1000.0;
        waitingForRecovery = false;
        Logger.recordOutput("BenchTest/Shooting/Metrics/LastRecoveryTimeMs", lastRecoveryTimeMs);
      }
    }

    // Burst timeout detection
    if (burstActive && (Timer.getFPGATimestamp() - lastShotTime) > BURST_GAP_THRESHOLD_SEC) {
      finalizeBurst();
    }

    // Log continuous metrics
    Logger.recordOutput("BenchTest/Shooting/Metrics/CumulativeShotCount", cumulativeShotCount);
    Logger.recordOutput("BenchTest/Shooting/Metrics/ShotsPerSecond", getShotsPerSecond());
    Logger.recordOutput("BenchTest/Shooting/Metrics/LastRecoveryTimeMs", lastRecoveryTimeMs);
    Logger.recordOutput("BenchTest/Shooting/Metrics/MaxLeaderTempC", maxLeaderTempC);
    Logger.recordOutput("BenchTest/Shooting/Metrics/MaxFollowerTempC", maxFollowerTempC);
    Logger.recordOutput("BenchTest/Shooting/Metrics/LeaderTempC", leaderTempC);
    Logger.recordOutput("BenchTest/Shooting/Metrics/FollowerTempC", followerTempC);
  }

  /** Record that a shot was fired. Call this each time a ball leaves the launcher. */
  public void recordShot() {
    double now = Timer.getFPGATimestamp();

    cumulativeShotCount++;

    // Store timestamp in rolling window
    shotTimestamps[shotTimestampIndex] = now;
    shotTimestampIndex = (shotTimestampIndex + 1) % ROLLING_WINDOW_SIZE;
    if (shotTimestampCount < ROLLING_WINDOW_SIZE) {
      shotTimestampCount++;
    }

    // Start recovery tracking
    waitingForRecovery = true;
    recoveryStartTime = now;

    // Burst tracking
    if (!burstActive || (now - lastShotTime) > BURST_GAP_THRESHOLD_SEC) {
      // Start new burst
      if (burstActive) {
        finalizeBurst();
      }
      burstActive = true;
      burstStartTime = now;
      burstShotCount = 1;
      burstMinRPM = currentRPM;
      burstMaxRPM = currentRPM;
    } else {
      burstShotCount++;
      if (currentRPM < burstMinRPM) burstMinRPM = currentRPM;
      if (currentRPM > burstMaxRPM) burstMaxRPM = currentRPM;
    }
    lastShotTime = now;

    Logger.recordOutput("BenchTest/Shooting/Metrics/LastShotTimestamp", now);
    Logger.recordOutput("BenchTest/Shooting/Metrics/CumulativeShotCount", cumulativeShotCount);
  }

  /** Calculate shots per second from the rolling window of recent shots. */
  public double getShotsPerSecond() {
    if (shotTimestampCount < 2) return 0.0;

    // Find oldest and newest timestamps in the window
    double oldest = Double.MAX_VALUE;
    double newest = 0.0;
    for (int i = 0; i < shotTimestampCount; i++) {
      if (shotTimestamps[i] < oldest) oldest = shotTimestamps[i];
      if (shotTimestamps[i] > newest) newest = shotTimestamps[i];
    }

    double timeSpan = newest - oldest;
    if (timeSpan <= 0.0) return 0.0;

    // Only report rate if the most recent shot was within the burst gap threshold
    double now = Timer.getFPGATimestamp();
    if ((now - newest) > BURST_GAP_THRESHOLD_SEC) return 0.0;

    return (shotTimestampCount - 1) / timeSpan;
  }

  /** Finalize the current burst and log its statistics. */
  private void finalizeBurst() {
    if (!burstActive || burstShotCount < 2) {
      burstActive = false;
      return;
    }

    double burstDuration = lastShotTime - burstStartTime;
    double avgRate = burstDuration > 0 ? (burstShotCount - 1) / burstDuration : 0.0;

    Logger.recordOutput("BenchTest/Shooting/Metrics/LastBurst/ShotCount", burstShotCount);
    Logger.recordOutput("BenchTest/Shooting/Metrics/LastBurst/DurationSec", burstDuration);
    Logger.recordOutput("BenchTest/Shooting/Metrics/LastBurst/AvgShotsPerSec", avgRate);
    Logger.recordOutput("BenchTest/Shooting/Metrics/LastBurst/MinRPM", burstMinRPM);
    Logger.recordOutput("BenchTest/Shooting/Metrics/LastBurst/MaxRPM", burstMaxRPM);

    System.out.println(
        "[BenchTest] Burst complete: "
            + burstShotCount
            + " shots in "
            + String.format("%.2f", burstDuration)
            + "s ("
            + String.format("%.1f", avgRate)
            + " shots/sec), RPM range: "
            + String.format("%.0f", burstMinRPM)
            + "-"
            + String.format("%.0f", burstMaxRPM));

    burstActive = false;
  }

  /** Reset all metrics. */
  public void reset() {
    cumulativeShotCount = 0;
    shotTimestampIndex = 0;
    shotTimestampCount = 0;
    waitingForRecovery = false;
    lastRecoveryTimeMs = 0.0;
    burstActive = false;
    burstShotCount = 0;
    lastShotTime = 0.0;
    maxLeaderTempC = 0.0;
    maxFollowerTempC = 0.0;

    Logger.recordOutput("BenchTest/Shooting/Metrics/CumulativeShotCount", 0);
    Logger.recordOutput("BenchTest/Shooting/Metrics/ShotsPerSecond", 0.0);
    Logger.recordOutput("BenchTest/Shooting/Metrics/LastRecoveryTimeMs", 0.0);
    Logger.recordOutput("BenchTest/Shooting/Metrics/MaxLeaderTempC", 0.0);
    Logger.recordOutput("BenchTest/Shooting/Metrics/MaxFollowerTempC", 0.0);

    System.out.println("[BenchTest] Metrics reset");
  }
}
