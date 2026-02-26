package frc.robot.subsystems.led;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.util.MatchPhaseTracker;
import frc.robot.util.MatchPhaseTracker.MatchPhase;
import org.littletonrobotics.junction.Logger;

/**
 * LED subsystem that drives an addressable LED strip with match-phase patterns, hood trench danger
 * warnings, and general robot status indication. Self-contained: reads drive pose and hood angle
 * internally so no external command is needed.
 *
 * <p>Priority (highest to lowest):
 *
 * <ol>
 *   <li>Hood trench danger (fast red flash)
 *   <li>Match phase patterns (hub active/inactive/shift warnings/endgame)
 *   <li>Disabled rainbow
 * </ol>
 */
public class LEDSubsystem extends SubsystemBase {

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final int length;

  // Optional subsystem references for self-contained trench checking
  private final Drive drive;
  private final Hood hood;

  // Trench geometry (precomputed)
  private final double trenchHalfWidth;
  private final double allyHubX;
  private final double oppHubX;

  // Blink state for accelerating shift warnings
  private boolean blinkOn = true;
  private double lastBlinkTime = 0;

  // Rainbow state for disabled
  private int rainbowFirstPixelHue = 0;

  // Shift change warning window (seconds before shift boundary to start blinking)
  private static final double SHIFT_WARNING_SECONDS = 7.0;
  private static final double MIN_BLINK_PERIOD = 0.05;
  private static final double MAX_BLINK_PERIOD = 0.5;

  // Hood danger margin above stow angle (degrees)
  private static final double HOOD_DANGER_MARGIN_DEG = 3.0;

  /**
   * Create the LED subsystem.
   *
   * @param pwmPort PWM port the LED strip is connected to
   * @param length Number of LEDs in the strip
   * @param drive Drive subsystem for pose reading (nullable — disables trench check)
   * @param hood Hood subsystem for angle reading (nullable — disables trench check)
   */
  public LEDSubsystem(int pwmPort, int length, Drive drive, Hood hood) {
    this.length = length;
    this.drive = drive;
    this.hood = hood;

    led = new AddressableLED(pwmPort);
    buffer = new AddressableLEDBuffer(length);
    led.setLength(length);
    led.start();

    // Precompute trench geometry
    trenchHalfWidth = FieldConstants.LeftTrench.width / 2.0;
    allyHubX = FieldConstants.LinesVertical.hubCenter;
    oppHubX = FieldConstants.LinesVertical.oppHubCenter;
  }

  @Override
  public void periodic() {
    DriverStation.Alliance alliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    boolean isBlue = alliance == DriverStation.Alliance.Blue;

    double time = Timer.getFPGATimestamp();

    // Priority 1: Hood trench danger — fast red flash
    boolean hoodDanger = checkHoodTrenchDanger();
    if (hoodDanger) {
      applyFlash(new Color(255, 0, 0), new Color(0, 0, 0), 0.1, time);
      Logger.recordOutput("LED/Pattern", "HOOD_DANGER");
    }
    // Priority 2: Robot disabled — rainbow
    else if (DriverStation.isDisabled()) {
      applyRainbow();
      Logger.recordOutput("LED/Pattern", "DISABLED_RAINBOW");
    }
    // Priority 3: Match phase patterns
    else {
      applyMatchPhasePattern(isBlue, time);
    }

    led.setData(buffer);
    Logger.recordOutput("LED/HoodDanger", hoodDanger);
    publishLEDColors();
  }

  // ========== Trench Danger Check ==========

  /**
   * Check if the hood is raised while the robot is inside a trench zone. Both X and Y position must
   * be within a trench rectangle for the check to trigger.
   */
  private boolean checkHoodTrenchDanger() {
    if (drive == null || hood == null) {
      return false;
    }
    if (!hood.isRaisedAboveSafe(HOOD_DANGER_MARGIN_DEG)) {
      return false;
    }

    Pose2d pose = drive.getPose();
    double robotX = pose.getX();
    double robotY = pose.getY();

    // Y ranges for left/right trenches
    boolean inLeftTrenchY =
        robotY >= FieldConstants.LinesHorizontal.leftTrenchOpenEnd
            && robotY <= FieldConstants.LinesHorizontal.leftTrenchOpenStart;
    boolean inRightTrenchY =
        robotY >= FieldConstants.LinesHorizontal.rightTrenchOpenEnd
            && robotY <= FieldConstants.LinesHorizontal.rightTrenchOpenStart;

    // X ranges: alliance-side or opposing-side hub area
    boolean inAllyTrenchX =
        robotX >= allyHubX - trenchHalfWidth && robotX <= allyHubX + trenchHalfWidth;
    boolean inOppTrenchX =
        robotX >= oppHubX - trenchHalfWidth && robotX <= oppHubX + trenchHalfWidth;

    return (inAllyTrenchX || inOppTrenchX) && (inLeftTrenchY || inRightTrenchY);
  }

  // ========== Match Phase Patterns ==========

  private void applyMatchPhasePattern(boolean isBlue, double time) {
    MatchPhaseTracker tracker = MatchPhaseTracker.getInstance();
    MatchPhase phase = tracker.getCurrentPhase();
    boolean hubActive = tracker.isOurHubActive(isBlue);
    Color allianceColor = isBlue ? new Color(0, 0, 255) : new Color(255, 0, 0);

    switch (phase) {
      case AUTO:
        // Solid alliance color during auto
        applySolid(allianceColor);
        Logger.recordOutput("LED/Pattern", "AUTO_SOLID");
        break;

      case TRANSITION:
        // Rainbow during transition — get ready for teleop
        applyRainbow();
        Logger.recordOutput("LED/Pattern", "TRANSITION_RAINBOW");
        break;

      case SHIFT_1:
      case SHIFT_2:
      case SHIFT_3:
      case SHIFT_4:
        double timeRemaining = phase.getEndTime() - getMatchTimeEstimate();
        if (hubActive) {
          // Hub active: check for accelerating warning near shift boundary
          if (timeRemaining > 0 && timeRemaining <= SHIFT_WARNING_SECONDS) {
            applyAcceleratingBlink(allianceColor, timeRemaining, time);
            Logger.recordOutput("LED/Pattern", "SHIFT_ACTIVE_WARNING");
          } else {
            // Solid alliance color — GO!
            applySolid(allianceColor);
            Logger.recordOutput("LED/Pattern", "SHIFT_ACTIVE");
          }
        } else {
          // Hub inactive: check for approaching-active warning
          double timeUntilActive = tracker.getTimeUntilActive(isBlue);
          if (timeUntilActive > 0 && timeUntilActive <= SHIFT_WARNING_SECONDS) {
            applyAcceleratingBlink(allianceColor, timeUntilActive, time);
            Logger.recordOutput("LED/Pattern", "SHIFT_INCOMING_WARNING");
          } else {
            // Dim alliance color — HOLD
            applyDim(allianceColor, 0.1);
            Logger.recordOutput("LED/Pattern", "SHIFT_INACTIVE");
          }
        }
        break;

      case END_GAME:
        // Endgame: pulsing green (both hubs active, scoring rush)
        applyEndGamePulse(time);
        Logger.recordOutput("LED/Pattern", "ENDGAME_PULSE");
        break;
    }
    Logger.recordOutput("LED/Phase", phase.toString());
    Logger.recordOutput("LED/HubActive", hubActive);
  }

  // ========== Pattern Helpers ==========

  private void applySolid(Color color) {
    for (int i = 0; i < length; i++) {
      buffer.setLED(i, color);
    }
  }

  private void applyFlash(Color color1, Color color2, double periodSeconds, double time) {
    boolean on = (time % periodSeconds) < (periodSeconds / 2.0);
    applySolid(on ? color1 : color2);
  }

  /** Blink alliance color with accelerating speed as a shift change approaches. */
  private void applyAcceleratingBlink(Color color, double secondsUntilChange, double time) {
    double fraction = Math.max(0, Math.min(1, secondsUntilChange / SHIFT_WARNING_SECONDS));
    double blinkPeriod = MIN_BLINK_PERIOD + fraction * (MAX_BLINK_PERIOD - MIN_BLINK_PERIOD);

    if (time - lastBlinkTime >= blinkPeriod) {
      blinkOn = !blinkOn;
      lastBlinkTime = time;
    }

    if (blinkOn) {
      applySolid(color);
    } else {
      applySolid(new Color(0, 0, 0));
    }
  }

  /** Green pulse for endgame (both hubs active). */
  private void applyEndGamePulse(double time) {
    double brightness = 0.5 + 0.5 * Math.sin(time * 4 * Math.PI);
    int value = (int) (brightness * 255);
    for (int i = 0; i < length; i++) {
      buffer.setRGB(i, 0, value, 0);
    }
  }

  /** Rainbow effect for disabled state. */
  private void applyRainbow() {
    for (int i = 0; i < length; i++) {
      int hue = (rainbowFirstPixelHue + (i * 180 / length)) % 180;
      buffer.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue = (rainbowFirstPixelHue + 1) % 180;
  }

  /** Dim version of a color. */
  private void applyDim(Color color, double brightness) {
    int r = (int) (color.red * 255 * brightness);
    int g = (int) (color.green * 255 * brightness);
    int b = (int) (color.blue * 255 * brightness);
    for (int i = 0; i < length; i++) {
      buffer.setRGB(i, r, g, b);
    }
  }

  /** Publish LED buffer colors to NetworkTables for dashboard visualization. */
  private void publishLEDColors() {
    String[] colors = new String[length];
    for (int i = 0; i < length; i++) {
      Color c = buffer.getLED(i);
      colors[i] =
          String.format(
              "#%02X%02X%02X", (int) (c.red * 255), (int) (c.green * 255), (int) (c.blue * 255));
    }
    Logger.recordOutput("LED/Colors", colors);
  }

  /** Rough estimate of match elapsed time (same logic as MatchPhaseTracker). */
  private double getMatchTimeEstimate() {
    if (DriverStation.isAutonomous()) {
      return Timer.getMatchTime() > 0 ? (15 - Timer.getMatchTime()) : 0;
    } else if (DriverStation.isTeleop()) {
      double remaining = Timer.getMatchTime();
      if (remaining > 0) {
        return 15 + (135 - remaining);
      }
    }
    if (DriverStation.isEnabled()) {
      return Timer.getFPGATimestamp() % 160;
    }
    return 0;
  }
}
