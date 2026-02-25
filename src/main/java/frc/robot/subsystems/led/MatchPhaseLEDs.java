package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MatchPhaseTracker;
import frc.robot.util.MatchPhaseTracker.MatchPhase;
import org.littletonrobotics.junction.Logger;

/**
 * LED subsystem that displays match phase information using addressable LEDs. Provides visual
 * feedback to drivers about:
 *
 * <ul>
 *   <li>Current match phase (auto, teleop shifts, endgame)
 *   <li>Whether our hub is active (solid) or inactive (off/dim)
 *   <li>Countdown warnings before shift changes (blinking faster)
 *   <li>Alliance color context (blue/red tint)
 * </ul>
 *
 * <p>Modeled after the 2025 IndicatorLight subsystem pattern. Students should extend this with
 * additional effects and game-specific feedback.
 */
public class MatchPhaseLEDs extends SubsystemBase {

  // Hardware
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final int ledCount;

  // Pre-built color buffers for common states
  private final AddressableLEDBuffer blueBuffer;
  private final AddressableLEDBuffer redBuffer;
  private final AddressableLEDBuffer greenBuffer;
  private final AddressableLEDBuffer yellowBuffer;
  private final AddressableLEDBuffer offBuffer;

  // Blink state
  private boolean blinkOn = true;
  private double lastBlinkTime = 0;

  // Rainbow state (for disabled/celebration)
  private int rainbowFirstPixelHue = 0;

  // Shift change warning threshold (seconds before shift change to start blinking)
  private static final double SHIFT_WARNING_SECONDS = 7.0;

  // Minimum blink period at the shift boundary
  private static final double MIN_BLINK_PERIOD = 0.05;
  // Maximum blink period at the start of the warning window
  private static final double MAX_BLINK_PERIOD = 0.5;

  /**
   * Creates a new MatchPhaseLEDs subsystem.
   *
   * @param pwmPort The PWM port the LED strip is connected to
   * @param ledCount The number of LEDs in the strip
   */
  public MatchPhaseLEDs(int pwmPort, int ledCount) {
    this.ledCount = ledCount;

    led = new AddressableLED(pwmPort);
    buffer = new AddressableLEDBuffer(ledCount);
    led.setLength(ledCount);
    led.setData(buffer);
    led.start();

    // Pre-build solid color buffers
    blueBuffer = createSolidBuffer(Color.kBlue);
    redBuffer = createSolidBuffer(Color.kRed);
    greenBuffer = createSolidBuffer(Color.kGreen);
    yellowBuffer = createSolidBuffer(Color.kYellow);
    offBuffer = createSolidBuffer(Color.kBlack);
  }

  @Override
  public void periodic() {
    MatchPhaseTracker tracker = MatchPhaseTracker.getInstance();
    MatchPhase phase = tracker.getCurrentPhase();
    boolean isBlue =
        DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue)
            .equals(DriverStation.Alliance.Blue);
    boolean hubActive = tracker.isOurHubActive(isBlue);

    // Choose LED pattern based on match state
    if (phase == MatchPhase.DISABLED) {
      // Disabled: slow rainbow or alliance color
      doRainbow();
    } else if (phase == MatchPhase.AUTO) {
      // Auto: solid alliance color (both hubs active)
      setBuffer(isBlue ? blueBuffer : redBuffer);
    } else if (phase == MatchPhase.DELAY) {
      // Transition delay: dim alliance color
      setDimAllianceColor(isBlue, 0.3);
    } else if (phase == MatchPhase.END_GAME) {
      // End game: fast green pulse (both hubs active, go score!)
      doEndGamePulse();
    } else {
      // Teleop shift phases: show hub status with shift-change warnings
      double timeUntilShift = tracker.getTimeUntilNextShift();

      if (hubActive) {
        // Our hub is active — solid alliance color, blink warning near shift end
        if (timeUntilShift >= 0 && timeUntilShift <= SHIFT_WARNING_SECONDS) {
          doShiftWarningBlink(isBlue, timeUntilShift);
        } else {
          setBuffer(isBlue ? blueBuffer : redBuffer);
        }
      } else {
        // Our hub is inactive — show countdown to when we become active
        double timeUntilActive = tracker.getTimeUntilActive(isBlue);
        if (timeUntilActive <= SHIFT_WARNING_SECONDS) {
          // Getting close to our turn: blink faster as we approach
          doShiftWarningBlink(isBlue, timeUntilActive);
        } else {
          // Inactive and not close to a shift: off/dim
          setDimAllianceColor(isBlue, 0.1);
        }
      }
    }

    // Push buffer to hardware
    led.setData(buffer);

    // Log the current LED state
    Logger.recordOutput("LED/Phase", phase.toString());
    Logger.recordOutput("LED/HubActive", hubActive);

    // Publish LED colors so dashboards can display them in sim
    publishLEDColors();
  }

  /**
   * Blink the alliance color with increasing speed as a shift change approaches. Blink period
   * scales linearly from MAX_BLINK_PERIOD (far from shift) to MIN_BLINK_PERIOD (at the boundary).
   */
  private void doShiftWarningBlink(boolean isBlue, double secondsUntilChange) {
    // Calculate blink period: faster as we get closer
    double fraction = Math.max(0, Math.min(1, secondsUntilChange / SHIFT_WARNING_SECONDS));
    double blinkPeriod = MIN_BLINK_PERIOD + fraction * (MAX_BLINK_PERIOD - MIN_BLINK_PERIOD);

    double now = Timer.getFPGATimestamp();
    if (now - lastBlinkTime >= blinkPeriod) {
      blinkOn = !blinkOn;
      lastBlinkTime = now;
    }

    if (blinkOn) {
      setBuffer(isBlue ? blueBuffer : redBuffer);
    } else {
      setBuffer(offBuffer);
    }
  }

  /** Green pulsing effect for endgame (both hubs active, scoring rush). */
  private void doEndGamePulse() {
    double now = Timer.getFPGATimestamp();
    // Sine wave brightness oscillation (period ~0.5s)
    double brightness = 0.5 + 0.5 * Math.sin(now * 4 * Math.PI);
    int value = (int) (brightness * 255);
    for (int i = 0; i < ledCount; i++) {
      buffer.setRGB(i, 0, value, 0);
    }
  }

  /** Slow rainbow effect for disabled state. */
  private void doRainbow() {
    for (int i = 0; i < ledCount; i++) {
      int hue = (rainbowFirstPixelHue + (i * 180 / ledCount)) % 180;
      buffer.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue = (rainbowFirstPixelHue + 1) % 180;
  }

  /** Set all LEDs to a dim version of the alliance color. */
  private void setDimAllianceColor(boolean isBlue, double brightness) {
    int value = (int) (brightness * 255);
    for (int i = 0; i < ledCount; i++) {
      if (isBlue) {
        buffer.setRGB(i, 0, 0, value);
      } else {
        buffer.setRGB(i, value, 0, 0);
      }
    }
  }

  /** Copy a pre-built buffer into the active buffer. */
  private void setBuffer(AddressableLEDBuffer source) {
    for (int i = 0; i < ledCount; i++) {
      buffer.setLED(i, source.getLED(i));
    }
  }

  /**
   * Publish LED buffer colors to NetworkTables for dashboard visualization. Publishes a hex color
   * string array (e.g., "#FF0000") that can be dragged into Elastic or viewed in AdvantageScope.
   */
  private void publishLEDColors() {
    String[] colors = new String[ledCount];
    for (int i = 0; i < ledCount; i++) {
      Color c = buffer.getLED(i);
      colors[i] =
          String.format(
              "#%02X%02X%02X", (int) (c.red * 255), (int) (c.green * 255), (int) (c.blue * 255));
    }
    Logger.recordOutput("LED/Colors", colors);
  }

  /** Create a solid-color buffer. */
  private AddressableLEDBuffer createSolidBuffer(Color color) {
    AddressableLEDBuffer buf = new AddressableLEDBuffer(ledCount);
    for (int i = 0; i < ledCount; i++) {
      buf.setLED(i, color);
    }
    return buf;
  }
}
