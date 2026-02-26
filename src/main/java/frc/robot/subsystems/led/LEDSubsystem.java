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
 * LED subsystem that drives an addressable LED strip with match-phase patterns, hood danger
 * warnings, and general robot status indication.
 *
 * <p>Priority (highest → lowest):
 *
 * <ol>
 *   <li>Hood angle danger (red flash)
 *   <li>Match phase patterns (hub active/inactive/endgame)
 *   <li>Disabled / idle patterns
 * </ol>
 */
public class LEDSubsystem extends SubsystemBase {

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final int length;

  // Hood danger state (set externally via setHoodDanger)
  private boolean hoodDanger = false;

  // Pattern state
  private double patternStartTime = 0.0;

  // Alliance color cache
  private Color allianceColor = new Color(0, 0, 255); // default blue

  /**
   * Create the LED subsystem.
   *
   * @param pwmPort PWM port the LED strip is connected to
   * @param length Number of LEDs in the strip
   */
  public LEDSubsystem(int pwmPort, int length) {
    this.length = length;
    led = new AddressableLED(pwmPort);
    buffer = new AddressableLEDBuffer(length);
    led.setLength(length);
    led.start();
  }

  @Override
  public void periodic() {
    DriverStation.Alliance alliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    boolean isBlue = alliance == DriverStation.Alliance.Blue;
    allianceColor = isBlue ? new Color(0, 0, 255) : new Color(255, 0, 0);

    double time = Timer.getFPGATimestamp();

    // Priority 1: Hood danger — fast red flash
    if (hoodDanger) {
      applyFlash(new Color(255, 0, 0), new Color(0, 0, 0), 0.1, time);
      Logger.recordOutput("LED/Pattern", "HOOD_DANGER");
    }
    // Priority 2: Robot disabled — slow breathe in alliance color
    else if (DriverStation.isDisabled()) {
      applyBreathe(allianceColor, time, 2.0);
      Logger.recordOutput("LED/Pattern", "DISABLED_BREATHE");
    }
    // Priority 3: Match phase patterns
    else {
      applyMatchPhasePattern(isBlue, time);
    }

    led.setData(buffer);
  }

  // ========== Hood Trench Danger API ==========

  /**
   * Set whether the hood is raised while the robot is near a trench. When true, LEDs flash red to
   * warn the driver that the hood could collide with the trench ceiling.
   *
   * @param danger true if hood is raised AND robot is in a trench zone
   */
  public void setHoodDanger(boolean danger) {
    this.hoodDanger = danger;
  }

  /** @return true if the hood trench danger warning is active */
  public boolean isHoodDanger() {
    return hoodDanger;
  }

  // ========== Match Phase Patterns ==========

  private void applyMatchPhasePattern(boolean isBlue, double time) {
    MatchPhaseTracker tracker = MatchPhaseTracker.getInstance();
    MatchPhase phase = tracker.getCurrentPhase();
    boolean hubActive = tracker.isOurHubActive(isBlue);

    switch (phase) {
      case AUTO:
        // Solid alliance color during auto
        applySolid(allianceColor);
        Logger.recordOutput("LED/Pattern", "AUTO_SOLID");
        break;

      case TRANSITION:
        // Fast rainbow during transition — get ready
        applyRainbow(time, 1.0);
        Logger.recordOutput("LED/Pattern", "TRANSITION_RAINBOW");
        break;

      case SHIFT_1:
      case SHIFT_2:
      case SHIFT_3:
      case SHIFT_4:
        if (hubActive) {
          // Hub active: solid green pulse (GO!)
          applyPulse(new Color(0, 255, 0), time, 0.5);
          Logger.recordOutput("LED/Pattern", "SHIFT_ACTIVE");
        } else {
          // Hub inactive: dim orange slow pulse (HOLD)
          applyPulse(new Color(255, 100, 0), time, 2.0);
          Logger.recordOutput("LED/Pattern", "SHIFT_INACTIVE");
        }

        // Flash warning 3 seconds before phase ends
        double timeRemaining = phase.getEndTime() - getMatchTimeEstimate();
        if (timeRemaining > 0 && timeRemaining <= 3.0) {
          applyFlash(new Color(255, 255, 0), allianceColor, 0.15, time);
          Logger.recordOutput("LED/Pattern", "SHIFT_WARNING");
        }
        break;

      case END_GAME:
        // Endgame: fast alliance color flash — intensity ramps up
        applyFlash(allianceColor, new Color(255, 255, 255), 0.2, time);
        Logger.recordOutput("LED/Pattern", "ENDGAME_FLASH");
        break;
    }
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

  private void applyBreathe(Color color, double time, double periodSeconds) {
    // Sine wave brightness from 0.1 to 1.0
    double brightness = 0.55 + 0.45 * Math.sin(2 * Math.PI * time / periodSeconds);
    Color dimmed =
        new Color(
            (int) (color.red * 255 * brightness),
            (int) (color.green * 255 * brightness),
            (int) (color.blue * 255 * brightness));
    applySolid(dimmed);
  }

  private void applyPulse(Color color, double time, double periodSeconds) {
    double brightness = 0.3 + 0.7 * Math.abs(Math.sin(Math.PI * time / periodSeconds));
    Color pulsed =
        new Color(
            (int) (color.red * 255 * brightness),
            (int) (color.green * 255 * brightness),
            (int) (color.blue * 255 * brightness));
    applySolid(pulsed);
  }

  private void applyRainbow(double time, double speedMultiplier) {
    int offset = (int) (time * 180.0 * speedMultiplier) % 180;
    for (int i = 0; i < length; i++) {
      int hue = (offset + (i * 180 / length)) % 180;
      buffer.setHSV(i, hue, 255, 128);
    }
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
