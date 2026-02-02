package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks match phases and determines when our alliance's hub is active for scoring. The 2026 game
 * has alternating scoring windows where only one alliance's hub is active at a time during teleop.
 *
 * <p>Match timing:
 *
 * <ul>
 *   <li>AUTO (0-20s): Both hubs active
 *   <li>TRANSITION (20-30s): Both hubs active
 *   <li>SHIFT_1 (30-55s): Loser of AUTO scores
 *   <li>SHIFT_2 (55-80s): Winner of AUTO scores
 *   <li>SHIFT_3 (80-105s): Loser of AUTO scores
 *   <li>SHIFT_4 (105-130s): Winner of AUTO scores
 *   <li>END_GAME (130-160s): Both hubs active
 * </ul>
 */
public class MatchPhaseTracker {

  /** Match phases with their timing boundaries. */
  public enum MatchPhase {
    AUTO(0, 20),
    TRANSITION(20, 30),
    SHIFT_1(30, 55),
    SHIFT_2(55, 80),
    SHIFT_3(80, 105),
    SHIFT_4(105, 130),
    END_GAME(130, 160);

    private final double startTime;
    private final double endTime;

    MatchPhase(double startTime, double endTime) {
      this.startTime = startTime;
      this.endTime = endTime;
    }

    public double getStartTime() {
      return startTime;
    }

    public double getEndTime() {
      return endTime;
    }
  }

  /** Hub override options for testing. */
  public enum HubOverride {
    AUTO, // Use match phase logic
    ON, // Force hub active
    OFF // Force hub inactive
  }

  private static MatchPhaseTracker instance;

  // Configuration - dashboard tunable
  private static final LoggedTunableNumber weWonAutoNumber =
      new LoggedTunableNumber("Shooting/WeWonAuto", 1.0); // 1.0 = true, 0.0 = false

  // Manual override for testing (null = use match logic)
  private HubOverride hubOverride = HubOverride.AUTO;

  // Cache for efficiency
  private MatchPhase cachedPhase = MatchPhase.AUTO;
  private double lastUpdateTime = 0;

  private MatchPhaseTracker() {}

  /** Get the singleton instance. */
  public static MatchPhaseTracker getInstance() {
    if (instance == null) {
      instance = new MatchPhaseTracker();
    }
    return instance;
  }

  /**
   * Check if we won auto (scored more points in autonomous).
   *
   * @return True if we won auto
   */
  public boolean getWeWonAuto() {
    return weWonAutoNumber.get() > 0.5;
  }

  /**
   * Set whether we won auto. This affects which shifts we score during.
   *
   * @param won True if we scored more in auto
   */
  public void setWeWonAuto(boolean won) {
    // Update via dashboard since tunable numbers are read-only at runtime
    // The dashboard value can be changed by the operator
    Logger.recordOutput("Shooting/WeWonAutoSetting", won);
  }

  /**
   * Set manual hub override for testing.
   *
   * @param override The override setting (AUTO, ON, or OFF)
   */
  public void setHubOverride(HubOverride override) {
    this.hubOverride = override;
    Logger.recordOutput("Shooting/HubOverride", override.toString());
  }

  /**
   * Get the current hub override setting.
   *
   * @return Current override
   */
  public HubOverride getHubOverride() {
    return hubOverride;
  }

  /**
   * Get the current match phase based on elapsed time.
   *
   * @return Current MatchPhase
   */
  public MatchPhase getCurrentPhase() {
    double matchTime = getMatchTime();

    for (MatchPhase phase : MatchPhase.values()) {
      if (matchTime >= phase.startTime && matchTime < phase.endTime) {
        return phase;
      }
    }

    // After match or before start
    return MatchPhase.END_GAME;
  }

  /**
   * Check if our alliance's hub is currently active for scoring.
   *
   * @param isBlueAlliance True if we are on blue alliance
   * @return True if our hub is active and we should shoot
   */
  public boolean isOurHubActive(boolean isBlueAlliance) {
    // Check manual override first
    if (hubOverride == HubOverride.ON) {
      return true;
    }
    if (hubOverride == HubOverride.OFF) {
      return false;
    }

    // Use match phase logic
    MatchPhase phase = getCurrentPhase();

    // Both hubs active during AUTO, TRANSITION, and END_GAME
    if (phase == MatchPhase.AUTO
        || phase == MatchPhase.TRANSITION
        || phase == MatchPhase.END_GAME) {
      return true;
    }

    // During shifts, active depends on who won auto
    boolean weWonAuto = getWeWonAuto();

    // Winner of AUTO scores during SHIFT_2 and SHIFT_4
    // Loser of AUTO scores during SHIFT_1 and SHIFT_3
    boolean winnerScoresNow = (phase == MatchPhase.SHIFT_2 || phase == MatchPhase.SHIFT_4);
    boolean loserScoresNow = (phase == MatchPhase.SHIFT_1 || phase == MatchPhase.SHIFT_3);

    if (weWonAuto) {
      return winnerScoresNow;
    } else {
      return loserScoresNow;
    }
  }

  /**
   * Get the match time in seconds. Returns 0 if not in a match.
   *
   * @return Match time in seconds
   */
  private double getMatchTime() {
    if (DriverStation.isAutonomous()) {
      return Timer.getMatchTime() > 0 ? (15 - Timer.getMatchTime()) : 0;
    } else if (DriverStation.isTeleop()) {
      // Teleop starts at 15s mark, runs for 135s
      double teleopTimeRemaining = Timer.getMatchTime();
      if (teleopTimeRemaining > 0) {
        // Convert remaining time to elapsed time since match start
        return 15 + (135 - teleopTimeRemaining);
      }
    }

    // For testing outside of a match, use elapsed time since enabled
    // This allows testing the phase logic in simulation
    if (DriverStation.isEnabled()) {
      return Timer.getFPGATimestamp() % 160; // Loop through phases for testing
    }

    return 0;
  }

  /**
   * Get time remaining until our next active window.
   *
   * @param isBlueAlliance True if we are on blue alliance
   * @return Seconds until our hub becomes active, or 0 if already active
   */
  public double getTimeUntilActive(boolean isBlueAlliance) {
    if (isOurHubActive(isBlueAlliance)) {
      return 0;
    }

    double matchTime = getMatchTime();
    MatchPhase currentPhase = getCurrentPhase();

    // Find the next phase where we're active
    boolean weWonAuto = getWeWonAuto();
    MatchPhase[] phases = MatchPhase.values();

    for (int i = currentPhase.ordinal(); i < phases.length; i++) {
      MatchPhase phase = phases[i];

      // Check if we'd be active in this phase
      boolean activeInPhase = false;
      if (phase == MatchPhase.AUTO
          || phase == MatchPhase.TRANSITION
          || phase == MatchPhase.END_GAME) {
        activeInPhase = true;
      } else if (weWonAuto && (phase == MatchPhase.SHIFT_2 || phase == MatchPhase.SHIFT_4)) {
        activeInPhase = true;
      } else if (!weWonAuto && (phase == MatchPhase.SHIFT_1 || phase == MatchPhase.SHIFT_3)) {
        activeInPhase = true;
      }

      if (activeInPhase && phase.startTime > matchTime) {
        return phase.startTime - matchTime;
      }
    }

    return Double.MAX_VALUE; // No more active windows
  }

  /**
   * Get time remaining in current active window.
   *
   * @param isBlueAlliance True if we are on blue alliance
   * @return Seconds remaining in current active window, or 0 if not active
   */
  public double getTimeRemainingActive(boolean isBlueAlliance) {
    if (!isOurHubActive(isBlueAlliance)) {
      return 0;
    }

    double matchTime = getMatchTime();
    MatchPhase currentPhase = getCurrentPhase();

    return Math.max(0, currentPhase.endTime - matchTime);
  }

  /** Update periodic logging. Call this from robotPeriodic(). */
  public void periodic() {
    double currentTime = Timer.getFPGATimestamp();

    // Update at 10Hz to reduce logging overhead
    if (currentTime - lastUpdateTime < 0.1) {
      return;
    }
    lastUpdateTime = currentTime;

    // Get alliance
    boolean isBlue =
        DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue)
            .equals(DriverStation.Alliance.Blue);

    // Update cached phase
    cachedPhase = getCurrentPhase();

    // Log status
    Logger.recordOutput("Shooting/MatchPhase", cachedPhase.toString());
    Logger.recordOutput("Shooting/MatchTime", getMatchTime());
    Logger.recordOutput("Shooting/OurHubActive", isOurHubActive(isBlue));
    Logger.recordOutput("Shooting/WeWonAuto", getWeWonAuto());
    Logger.recordOutput("Shooting/HubOverride", hubOverride.toString());
    Logger.recordOutput("Shooting/TimeUntilActive", getTimeUntilActive(isBlue));
    Logger.recordOutput("Shooting/TimeRemainingActive", getTimeRemainingActive(isBlue));
  }
}
