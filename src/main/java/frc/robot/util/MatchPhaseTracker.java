package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks match phases and determines when our alliance's hub is active for scoring. The 2026
 * REBUILT game has alternating scoring windows where only one alliance's hub is active at a time
 * during teleop.
 *
 * <p>FMS match structure:
 *
 * <ul>
 *   <li>AUTO: 20 seconds (DriverStation autonomous mode)
 *   <li>DELAY: ~5 seconds (FMS transition, robot disabled)
 *   <li>TRANSITION: First 5s of teleop (remaining > 130s) — both hubs active
 *   <li>SHIFT_1: Teleop remaining 105-130s — auto LOSER's hub active
 *   <li>SHIFT_2: Teleop remaining 80-105s — auto WINNER's hub active
 *   <li>SHIFT_3: Teleop remaining 55-80s — auto LOSER's hub active
 *   <li>SHIFT_4: Teleop remaining 30-55s — auto WINNER's hub active
 *   <li>END_GAME: Teleop remaining 0-30s — both hubs active
 * </ul>
 *
 * <p>Phase boundaries match the official WPILib 2026 game data documentation. Uses
 * Timer.getMatchTime() (teleop remaining time) directly for FMS accuracy.
 *
 * <p>FMS provides DriverStation.getGameSpecificMessage() which returns 'R' or 'B' indicating which
 * alliance's hub goes inactive first (i.e., who won auto). This arrives ~3 seconds after auto ends.
 * The manual dashboard tunable serves as a fallback when not on FMS.
 *
 * <p>Practice mode: When no FMS match timer is available, uses an internal elapsed timer that
 * simulates match progression starting from when the robot is first enabled.
 */
public class MatchPhaseTracker {

  /** Match phases for the 2026 REBUILT game. */
  public enum MatchPhase {
    /** Autonomous period (20s). Both hubs active. */
    AUTO,
    /** FMS transition between auto and teleop (~5s, robot disabled). Both hubs active. */
    DELAY,
    /** First 5s of teleop (remaining > 130s). Both hubs active. */
    TRANSITION,
    /** Teleop remaining 105-130s. Auto LOSER's hub active. */
    SHIFT_1,
    /** Teleop remaining 80-105s. Auto WINNER's hub active. */
    SHIFT_2,
    /** Teleop remaining 55-80s. Auto LOSER's hub active. */
    SHIFT_3,
    /** Teleop remaining 30-55s. Auto WINNER's hub active. */
    SHIFT_4,
    /** Last 30s of teleop (remaining <= 30s). Both hubs active. */
    END_GAME,
    /** Robot is disabled or match hasn't started. */
    DISABLED
  }

  // Teleop phase boundaries (seconds remaining on the FMS teleop countdown timer).
  // These match the WPILib 2026 game data documentation exactly.
  private static final double TRANSITION_BOUNDARY = 130.0; // >130 = transition
  private static final double SHIFT_1_BOUNDARY = 105.0; // 105-130 = shift 1
  private static final double SHIFT_2_BOUNDARY = 80.0; // 80-105 = shift 2
  private static final double SHIFT_3_BOUNDARY = 55.0; // 55-80 = shift 3
  private static final double SHIFT_4_BOUNDARY = 30.0; // 30-55 = shift 4
  // <=30 = end game

  // Practice mode timing (simulates match when no FMS timer is available)
  private static final double AUTO_DURATION = 20.0;
  private static final double DELAY_DURATION = 5.0;
  private static final double TELEOP_DURATION = 135.0;

  /** Hub override options for testing. */
  public enum HubOverride {
    AUTO, // Use match phase logic
    ON, // Force hub active
    OFF // Force hub inactive
  }

  private static MatchPhaseTracker instance;

  // Dashboard tunable: fallback for when FMS game data isn't available.
  // 1.0 = we won auto, 0.0 = we lost auto
  private static final LoggedTunableNumber weWonAutoNumber =
      new LoggedTunableNumber("Match/WeWonAuto", 1.0);

  private HubOverride hubOverride = HubOverride.AUTO;

  // Cache for efficiency
  private MatchPhase cachedPhase = MatchPhase.DISABLED;
  private double lastUpdateTime = 0;

  // Practice mode: internal timer for when FMS match timer isn't available.
  // Starts when robot is first enabled and tracks elapsed match time.
  private double practiceStartTime = -1;
  private boolean practiceTimerRunning = false;

  // Cache the FMS game data result to avoid repeated string parsing
  private Boolean fmsWeWonAutoCache = null;
  private boolean fmsDataChecked = false;

  private MatchPhaseTracker() {}

  /** Get the singleton instance. */
  public static MatchPhaseTracker getInstance() {
    if (instance == null) {
      instance = new MatchPhaseTracker();
    }
    return instance;
  }

  /**
   * Check if we won auto. First tries FMS game data (DriverStation.getGameSpecificMessage()), then
   * falls back to the dashboard tunable.
   *
   * <p>FMS sends a single character ('R' or 'B') indicating which alliance's hub goes inactive
   * first — that alliance is the auto WINNER. This data arrives ~3 seconds after auto ends.
   *
   * @return True if we won auto
   */
  public boolean getWeWonAuto() {
    // Try FMS game data first (only available during real matches)
    if (!fmsDataChecked) {
      String gameData = DriverStation.getGameSpecificMessage();
      if (gameData != null && !gameData.isEmpty()) {
        char goesInactiveFirst = gameData.charAt(0);
        boolean isBlue =
            DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Blue);
        // Alliance that goes inactive first = auto winner
        fmsWeWonAutoCache =
            (isBlue && goesInactiveFirst == 'B') || (!isBlue && goesInactiveFirst == 'R');
        fmsDataChecked = true;
      }
    }

    if (fmsWeWonAutoCache != null) {
      return fmsWeWonAutoCache;
    }

    // Fallback to dashboard tunable
    return weWonAutoNumber.get() > 0.5;
  }

  /**
   * Get whether we're using FMS game data or the manual fallback.
   *
   * @return true if FMS game data is being used
   */
  public boolean isUsingFmsGameData() {
    return fmsWeWonAutoCache != null;
  }

  /**
   * Set manual hub override for testing.
   *
   * @param override The override setting (AUTO, ON, or OFF)
   */
  public void setHubOverride(HubOverride override) {
    this.hubOverride = override;
    Logger.recordOutput("Match/HubOverride", override.toString());
  }

  /**
   * Get the current hub override setting.
   *
   * @return Current override
   */
  public HubOverride getHubOverride() {
    return hubOverride;
  }

  /** Reset FMS game data cache. Call at the start of each match (autonomousInit). */
  public void resetForNewMatch() {
    fmsWeWonAutoCache = null;
    fmsDataChecked = false;
    practiceStartTime = -1;
    practiceTimerRunning = false;
  }

  /**
   * Get the current match phase based on DriverStation state and match timer.
   *
   * <p>During FMS matches, uses Timer.getMatchTime() directly for accurate phase boundaries. In
   * practice mode (no FMS timer), uses an internal elapsed timer to simulate match progression.
   *
   * @return Current MatchPhase
   */
  public MatchPhase getCurrentPhase() {
    // Autonomous mode
    if (DriverStation.isAutonomous() && DriverStation.isEnabled()) {
      startPracticeTimerIfNeeded();
      return MatchPhase.AUTO;
    }

    // Teleop mode — determine phase from remaining time
    if (DriverStation.isTeleop() && DriverStation.isEnabled()) {
      double remaining = getTeleopRemainingTime();
      return getTeleopPhaseFromRemaining(remaining);
    }

    // Robot is enabled but neither auto nor teleop (FMS transition delay)
    if (DriverStation.isEnabled()) {
      return MatchPhase.DELAY;
    }

    // Disabled: check if we're in the auto-to-teleop gap during an FMS match
    // (DriverStation reports disabled briefly during the ~5s FMS transition)
    if (practiceTimerRunning) {
      double elapsed = Timer.getFPGATimestamp() - practiceStartTime;
      if (elapsed > AUTO_DURATION && elapsed < AUTO_DURATION + DELAY_DURATION) {
        return MatchPhase.DELAY;
      }
    }

    return MatchPhase.DISABLED;
  }

  /**
   * Determine the teleop phase from the remaining time on the teleop countdown.
   *
   * @param remaining Seconds remaining in teleop (135 to 0)
   * @return The current teleop phase
   */
  private MatchPhase getTeleopPhaseFromRemaining(double remaining) {
    if (remaining > TRANSITION_BOUNDARY) return MatchPhase.TRANSITION;
    if (remaining > SHIFT_1_BOUNDARY) return MatchPhase.SHIFT_1;
    if (remaining > SHIFT_2_BOUNDARY) return MatchPhase.SHIFT_2;
    if (remaining > SHIFT_3_BOUNDARY) return MatchPhase.SHIFT_3;
    if (remaining > SHIFT_4_BOUNDARY) return MatchPhase.SHIFT_4;
    return MatchPhase.END_GAME;
  }

  /**
   * Get the teleop remaining time. Uses FMS match timer when available, falls back to internal
   * practice timer.
   *
   * @return Seconds remaining in teleop (135 to 0)
   */
  private double getTeleopRemainingTime() {
    double fmsTime = Timer.getMatchTime();

    // FMS timer is available and valid
    if (fmsTime > 0) {
      return fmsTime;
    }

    // Practice mode: calculate remaining time from internal timer
    if (practiceTimerRunning && practiceStartTime > 0) {
      double elapsed = Timer.getFPGATimestamp() - practiceStartTime;
      double teleopElapsed = elapsed - AUTO_DURATION - DELAY_DURATION;
      if (teleopElapsed >= 0) {
        return Math.max(0, TELEOP_DURATION - teleopElapsed);
      }
    }

    // No timer available — assume start of teleop
    return TELEOP_DURATION;
  }

  /** Start the practice timer if it hasn't been started yet. */
  private void startPracticeTimerIfNeeded() {
    if (!practiceTimerRunning) {
      practiceStartTime = Timer.getFPGATimestamp();
      practiceTimerRunning = true;
    }
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

    // Both hubs active during AUTO, DELAY, TRANSITION, END_GAME, and DISABLED
    if (phase == MatchPhase.AUTO
        || phase == MatchPhase.DELAY
        || phase == MatchPhase.TRANSITION
        || phase == MatchPhase.END_GAME
        || phase == MatchPhase.DISABLED) {
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
   * Get time remaining in the current phase.
   *
   * @return Seconds remaining in the current phase
   */
  public double getTimeRemainingInPhase() {
    MatchPhase phase = getCurrentPhase();

    if (phase == MatchPhase.AUTO) {
      double autoRemaining = Timer.getMatchTime();
      return autoRemaining > 0 ? autoRemaining : 0;
    }

    if (phase == MatchPhase.DISABLED || phase == MatchPhase.DELAY) {
      return 0;
    }

    // Teleop phases: calculate from remaining time
    double remaining = getTeleopRemainingTime();
    switch (phase) {
      case TRANSITION:
        return remaining - TRANSITION_BOUNDARY;
      case SHIFT_1:
        return remaining - SHIFT_1_BOUNDARY;
      case SHIFT_2:
        return remaining - SHIFT_2_BOUNDARY;
      case SHIFT_3:
        return remaining - SHIFT_3_BOUNDARY;
      case SHIFT_4:
        return remaining - SHIFT_4_BOUNDARY;
      case END_GAME:
        return remaining;
      default:
        return 0;
    }
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

    double remaining = getTeleopRemainingTime();
    boolean weWonAuto = getWeWonAuto();

    // Find the next phase boundary where we become active.
    // Phases are checked in order of decreasing remaining time (forward through the match).
    // Winner is active during SHIFT_2 (80-105) and SHIFT_4 (30-55).
    // Loser is active during SHIFT_1 (105-130) and SHIFT_3 (55-80).
    double[] activeBoundaries;
    if (weWonAuto) {
      // Winner: active during SHIFT_2 (starts at remaining=105) and SHIFT_4 (starts at
      // remaining=55)
      activeBoundaries = new double[] {SHIFT_1_BOUNDARY, SHIFT_3_BOUNDARY, SHIFT_4_BOUNDARY};
    } else {
      // Loser: active during SHIFT_1 (starts at remaining=130) and SHIFT_3 (starts at
      // remaining=80)
      activeBoundaries = new double[] {TRANSITION_BOUNDARY, SHIFT_2_BOUNDARY, SHIFT_4_BOUNDARY};
    }

    for (double boundary : activeBoundaries) {
      if (remaining > boundary) {
        return remaining - boundary;
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

    return getTimeRemainingInPhase();
  }

  /**
   * Check if a shift change is approaching within the given warning window.
   *
   * @return Seconds until the next shift boundary, or -1 if no shift change is imminent
   */
  public double getTimeUntilNextShift() {
    MatchPhase phase = getCurrentPhase();
    if (phase == MatchPhase.DISABLED || phase == MatchPhase.AUTO || phase == MatchPhase.DELAY) {
      return -1;
    }
    return getTimeRemainingInPhase();
  }

  /** Update periodic logging. Call this from robotPeriodic(). */
  public void periodic() {
    double currentTime = Timer.getFPGATimestamp();

    // Update at 10Hz to reduce logging overhead
    if (currentTime - lastUpdateTime < 0.1) {
      return;
    }
    lastUpdateTime = currentTime;

    // Try to read FMS game data each cycle until we get it
    if (!fmsDataChecked || fmsWeWonAutoCache == null) {
      String gameData = DriverStation.getGameSpecificMessage();
      if (gameData != null && !gameData.isEmpty()) {
        char goesInactiveFirst = gameData.charAt(0);
        boolean isBlue =
            DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Blue);
        fmsWeWonAutoCache =
            (isBlue && goesInactiveFirst == 'B') || (!isBlue && goesInactiveFirst == 'R');
        fmsDataChecked = true;
      }
    }

    // Get alliance
    boolean isBlue =
        DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue)
            .equals(DriverStation.Alliance.Blue);

    // Update cached phase
    cachedPhase = getCurrentPhase();

    // Log status
    Logger.recordOutput("Match/MatchPhase", cachedPhase.toString());
    Logger.recordOutput("Match/TeleopRemaining", getTeleopRemainingTime());
    Logger.recordOutput("Match/OurHubActive", isOurHubActive(isBlue));
    Logger.recordOutput("Match/WeWonAuto", getWeWonAuto());
    Logger.recordOutput("Match/WeWonAutoSource", isUsingFmsGameData() ? "FMS" : "Dashboard");
    Logger.recordOutput("Match/HubOverride", hubOverride.toString());
    Logger.recordOutput("Match/TimeUntilActive", getTimeUntilActive(isBlue));
    Logger.recordOutput("Match/TimeRemainingActive", getTimeRemainingActive(isBlue));
    Logger.recordOutput("Match/TimeRemainingInPhase", getTimeRemainingInPhase());
    Logger.recordOutput("Match/TimeUntilNextShift", getTimeUntilNextShift());
  }
}
