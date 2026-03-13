package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.led.IndicatorLightConstants.LED_EFFECTS;
import frc.robot.util.HubShiftUtil;
import java.util.Random;
import org.littletonrobotics.junction.Logger;

public class IndicatorLight extends SubsystemBase {

  /** Dashboard-selectable light mode: match auto-lighting, off, or pit (blue ombre). */
  public enum LightMode {
    MATCH,
    OFF,
    PIT
  }

  private final SendableChooser<LightMode> lightModeChooser = new SendableChooser<>();

  private LED_EFFECTS currentColor_GOAL = LED_EFFECTS.BLACK;
  private LED_EFFECTS LED_State = LED_EFFECTS.BLACK;

  private final Alert autoWinnerNotSet =
      new Alert("!!! AUTO WINNER NOT SET - Hub shift schedule unknown !!!", AlertType.kError);

  // Define constants for the blink period (in seconds)
  private static final double MAX_BLINK_PERIOD = 0.5; // far from target, slow blink
  private static final double MIN_BLINK_PERIOD = 0.001; // very close, fast blink

  public static final double MIN_TOLERANCE_BLINK = 0.025; // below this, blink is at min period
  public static final double MAX_TOLERANCE_BLINK = 0.150; // above this, blink is at max period

  // Variable to hold the current blink period computed from the error
  private double currentBlinkPeriod = MAX_BLINK_PERIOD;

  private AddressableLED wlLED;
  private RGBWBuffer wlLEDBuffer;
  private RGBWBuffer wlGreenLEDBuffer;
  private RGBWBuffer wlOrangeLEDBuffer;
  private RGBWBuffer wlPurpleLEDBuffer;
  private RGBWBuffer wlRedLEDBuffer;
  private RGBWBuffer wlYellowLEDBuffer;
  private RGBWBuffer wlBlueLEDBuffer;
  private RGBWBuffer wlIndigoLEDBuffer;
  private RGBWBuffer wlVioletLEDBuffer;
  private RGBWBuffer wlWhiteLEDBuffer;
  private RGBWBuffer wlBlackLEDBuffer;

  // Store what the last hue of the first pixel is
  private int rainbowFirstPixelHue = 0;
  private int currentSaturation = 100;
  private boolean forward = true;
  private int counter = 0;
  private double lastTime = 0.0;
  private double blinkTime = 0.0;
  private boolean on = false;
  private int skittleCount = 0;

  private double countdownRemainingTime = 7.0;
  private double warningRemainingTime = 5.0;

  private Random random = new Random();

  private Timer effectTimer = new Timer();
  private final double restartInterval = 5.0; // Restart effect every 5 seconds
  private int effectPhase = 0;
  private final int maxBrightness = 255;
  private int center = 9;
  private final double updateInterval = 0.05; // Interval in seconds for updates

  private RGBWBuffer currentActiveBuffer;

  public IndicatorLight() {
    // Dashboard chooser: Match (default auto-lighting), Off, or Pit (blue ombre)
    lightModeChooser.setDefaultOption("Match", LightMode.MATCH);
    lightModeChooser.addOption("Off", LightMode.OFF);
    lightModeChooser.addOption("Pit", LightMode.PIT);
    SmartDashboard.putData("Match/Light Mode", lightModeChooser);

    // Default brightness to 50% — adjustable via dashboard (0.0 to 1.0)
    SmartDashboard.putNumber("Match/LED Brightness", 0.5);
    RGBWBuffer.setBrightnessScalar(0.5);

    int numLEDs = IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH;

    wlLED = new AddressableLED(IndicatorLightConstants.ADDRESSABLE_LED_PORT);
    wlLEDBuffer = new RGBWBuffer(numLEDs);
    wlLED.setLength(wlLEDBuffer.getInternalBuffer().getLength());
    center = wlLEDBuffer.getLength() / 2;
    currentActiveBuffer = wlLEDBuffer;
    wlLEDBuffer.flushToBuffer();
    wlLED.setData(wlLEDBuffer.getInternalBuffer());
    wlLED.start();

    wlGreenLEDBuffer = new RGBWBuffer(numLEDs);
    for (var i = 0; i < wlGreenLEDBuffer.getLength(); i++) {
      wlGreenLEDBuffer.setHSV(i, IndicatorLightConstants.GREEN_HUE, 255, 128);
    }

    wlOrangeLEDBuffer = new RGBWBuffer(numLEDs);
    for (var i = 0; i < wlOrangeLEDBuffer.getLength(); i++) {
      wlOrangeLEDBuffer.setLED(i, new Color8Bit(255, 27, 0));
    }

    wlPurpleLEDBuffer = new RGBWBuffer(numLEDs);
    for (var i = 0; i < wlPurpleLEDBuffer.getLength(); i++) {
      wlPurpleLEDBuffer.setHSV(i, IndicatorLightConstants.PURPLE_HUE, 63, 92);
    }

    wlRedLEDBuffer = new RGBWBuffer(numLEDs);
    for (var i = 0; i < wlRedLEDBuffer.getLength(); i++) {
      wlRedLEDBuffer.setHSV(i, IndicatorLightConstants.RED_HUE, 255, 128);
    }

    wlYellowLEDBuffer = new RGBWBuffer(numLEDs);
    for (int i = 0; i < wlYellowLEDBuffer.getLength(); i++) {
      wlYellowLEDBuffer.setLED(i, Color.kYellow);
    }

    wlBlueLEDBuffer = new RGBWBuffer(numLEDs);
    for (var i = 0; i < wlBlueLEDBuffer.getLength(); i++) {
      wlBlueLEDBuffer.setLED(i, Color.kBlue);
    }

    wlIndigoLEDBuffer = new RGBWBuffer(numLEDs);
    for (var i = 0; i < wlIndigoLEDBuffer.getLength(); i++) {
      wlIndigoLEDBuffer.setLED(i, Color.kIndigo);
    }

    wlVioletLEDBuffer = new RGBWBuffer(numLEDs);
    for (var i = 0; i < wlVioletLEDBuffer.getLength(); i++) {
      wlVioletLEDBuffer.setLED(i, Color.kViolet);
    }

    wlWhiteLEDBuffer = new RGBWBuffer(numLEDs);
    for (var i = 0; i < wlWhiteLEDBuffer.getLength(); i++) {
      // Use the dedicated white channel for true white on RGBW strips
      wlWhiteLEDBuffer.setRGBW(i, 0, 0, 0, 255);
    }

    wlBlackLEDBuffer = new RGBWBuffer(numLEDs);
    for (var i = 0; i < wlBlackLEDBuffer.getLength(); i++) {
      wlBlackLEDBuffer.setLED(i, Color.kBlack);
    }

    effectTimer.start();
  }

  @Override
  public void periodic() {
    RGBWBuffer.setBrightnessScalar(SmartDashboard.getNumber("Match/LED Brightness", 0.5));

    LightMode mode = lightModeChooser.getSelected();
    if (mode == null) mode = LightMode.MATCH;

    // Disabled RSL shows in all modes
    if (DriverStation.isDisabled()) {
      LED_State = LED_EFFECTS.RSL;
      doRsl();
      publishLEDColors();
      Logger.recordOutput("LED/Pattern", LED_State.toString());
      return;
    }

    if (mode == LightMode.OFF) {
      LED_State = LED_EFFECTS.BLACK;
      setActiveBuffer(wlBlackLEDBuffer);
      publishLEDColors();
      Logger.recordOutput("LED/Pattern", LED_State.toString());
      return;
    }

    if (mode == LightMode.PIT) {
      LED_State = LED_EFFECTS.BLUEOMBRE;
      doBlueOmbre();
      publishLEDColors();
      Logger.recordOutput("LED/Pattern", LED_State.toString());
      return;
    }

    // MATCH mode: normal auto-lighting logic
    currentColor_GOAL = updateLightingGoal();

    if (LED_State != LED_EFFECTS.BLINK) {
      LED_State = currentColor_GOAL;
    }
    switch (LED_State) {
      case RED -> setActiveBuffer(wlRedLEDBuffer);
      case YELLOW -> setActiveBuffer(wlYellowLEDBuffer);
      case GREEN -> setActiveBuffer(wlGreenLEDBuffer);
      case ORANGE -> setActiveBuffer(wlOrangeLEDBuffer);
      case PURPLE -> setActiveBuffer(wlPurpleLEDBuffer);
      case BLUE -> setActiveBuffer(wlBlueLEDBuffer);
      case BLACK -> setActiveBuffer(wlBlackLEDBuffer);
      case WHITE -> setActiveBuffer(wlWhiteLEDBuffer);
      case BLINK_RED -> doBlinkRed();
      case RAINBOW -> doRainbow();
      case BLUEOMBRE -> doBlueOmbre();
      case BLINK -> doBlink();
      case COUNTDOWN_BLINK -> doCountdownBlink();
      case BLINK_PURPLE -> blinkPurple();
      case PARTY -> doParty();
      case RSL -> doRsl();
      case SEGMENTPARTY -> doSegmentParty();
      case EXPLOSION -> doExplosionEffect();
      case POLKADOT -> doPokadot();
      case SEARCH_LIGHT -> doSearchlightSingleEffect();
      case DYNAMIC_BLINK -> dynamicBlink();
      case GREEN_RED_WARNING -> doGreenRedWarning();
      default -> {}
    }

    publishLEDColors();
    Logger.recordOutput("LED/Pattern", LED_State.toString());
  }

  /** Publish LED colors to NetworkTables for Elastic Multi Color View widget (sim only). */
  private void publishLEDColors() {
    if (!RobotBase.isSimulation()) return;
    int len = currentActiveBuffer.getLength();
    String[] colors = new String[len];
    for (int i = 0; i < len; i++) {
      Color c = currentActiveBuffer.getLED(i);
      colors[i] =
          String.format(
              "#%02X%02X%02X", (int) (c.red * 255), (int) (c.green * 255), (int) (c.blue * 255));
    }
    SmartDashboard.putStringArray("LED/Colors", colors);
  }

  // ========== Public setters for LED effects ==========

  public void setEffect(LED_EFFECTS effect) {
    currentColor_GOAL = effect;
  }

  public void blueOmbre() {
    currentColor_GOAL = LED_EFFECTS.BLUEOMBRE;
  }

  public void rainbow() {
    currentColor_GOAL = LED_EFFECTS.RAINBOW;
  }

  public void party() {
    currentColor_GOAL = LED_EFFECTS.PARTY;
  }

  public void segmentParty() {
    currentColor_GOAL = LED_EFFECTS.SEGMENTPARTY;
  }

  public void explosion() {
    currentColor_GOAL = LED_EFFECTS.EXPLOSION;
  }

  public void polkadot() {
    currentColor_GOAL = LED_EFFECTS.POLKADOT;
  }

  public void blink() {
    currentColor_GOAL = LED_EFFECTS.BLINK;
  }

  public void blinkRed() {
    LED_State = LED_EFFECTS.BLINK_RED;
  }

  public void green() {
    currentColor_GOAL = LED_EFFECTS.GREEN;
  }

  public void orange() {
    currentColor_GOAL = LED_EFFECTS.ORANGE;
  }

  public void purple() {
    currentColor_GOAL = LED_EFFECTS.PURPLE;
  }

  public void red() {
    currentColor_GOAL = LED_EFFECTS.RED;
  }

  public void yellow() {
    currentColor_GOAL = LED_EFFECTS.YELLOW;
  }

  public void blue() {
    currentColor_GOAL = LED_EFFECTS.BLUE;
  }

  // ========== Effect implementations ==========

  public void doExplosionEffect() {
    double elapsedTime = effectTimer.get();

    // Automatically restart the effect after a specific interval
    if (elapsedTime > restartInterval) {
      effectPhase = 1; // Reset to start phase
      effectTimer.reset();
    }

    // Determine the update step based on the elapsed time
    int step = (int) (elapsedTime / updateInterval);

    if (effectPhase == 1) {
      // Expansion phase
      if (step <= center) {
        for (int i = 0; i <= step; i++) {
          int brightness = Math.max(0, maxBrightness - ((maxBrightness / center) * i));
          wlLEDBuffer.setRGB(center + i, brightness, brightness, 0);
          wlLEDBuffer.setRGB(center - i, brightness, brightness, 0);
        }
        setActiveBuffer(wlLEDBuffer);
      } else {
        effectPhase = 2; // Move to fading phase
      }
    } else if (effectPhase == 2) {
      // Fading phase
      int fadeStep = maxBrightness - (int) (step * 5.0 / updateInterval);
      if (fadeStep > 0) {
        for (int i = 0; i < wlLEDBuffer.getLength(); i++) {
          int distance = Math.abs(center - i);
          int brightness = Math.max(0, fadeStep - ((maxBrightness / center) * distance));
          wlLEDBuffer.setRGB(i, brightness, brightness, 0);
        }
        setActiveBuffer(wlLEDBuffer);
      } else {
        effectPhase = 0; // End the effect and wait for the next restart
      }
    }
  }

  public void doParty() {
    for (var i = 0; i < wlLEDBuffer.getLength(); i++) {
      int red = random.nextInt(256);
      int green = random.nextInt(256);
      int blue = random.nextInt(256);
      wlLEDBuffer.setRGB(i, red, green, blue);
    }
    setActiveBuffer(wlLEDBuffer);
  }

  private void doSegmentParty() {
    if (counter > IndicatorLightConstants.UPDATE_FREQUENCY) {
      counter = 0;

      int numberOfSegments = 1 + random.nextInt(10);

      for (int segment = 0; segment < numberOfSegments; segment++) {
        int[] color =
            IndicatorLightConstants.colorPalette[
                random.nextInt(IndicatorLightConstants.colorPalette.length)];

        int start = random.nextInt(wlLEDBuffer.getLength());
        int length = 1 + random.nextInt(wlLEDBuffer.getLength() - start);

        for (int i = start; i < start + length; i++) {
          wlLEDBuffer.setRGB(i, color[0], color[1], color[2]);
        }
      }
    } else counter++;

    setActiveBuffer(wlLEDBuffer);
  }

  private void doPokadot() {
    for (var i = 0; i < wlLEDBuffer.getLength(); i++) {
      int red = random.nextInt(256);
      int green = random.nextInt(256);
      int blue = random.nextInt(256);
      wlLEDBuffer.setRGB(i, red, green, blue);
    }
    setActiveBuffer(wlLEDBuffer);
  }

  public void doRainbow() {
    for (var i = 0; i < wlLEDBuffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / wlLEDBuffer.getLength())) % 180;
      wlLEDBuffer.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
    setActiveBuffer(wlLEDBuffer);
  }

  public void doBlueOmbre() {
    for (var i = 0; i < wlLEDBuffer.getLength(); i++) {
      final var saturation = (currentSaturation + (i * 255 / wlLEDBuffer.getLength())) % 255;
      wlLEDBuffer.setHSV(i, 103, 255, saturation);
    }

    if (forward) {
      currentSaturation += 3;
      if (currentSaturation >= 255) {
        currentSaturation = 255;
        forward = false;
      }
    } else {
      currentSaturation -= 3;
      if (currentSaturation <= 0) {
        currentSaturation = 0;
        forward = true;
      }
    }

    setActiveBuffer(wlLEDBuffer);
  }

  public void doBlink() {
    LED_State = LED_EFFECTS.BLINK;
    double timeStamp = Timer.getFPGATimestamp();

    if (blinkTime == 0.0) {
      blinkTime = timeStamp;
    }
    if (timeStamp - lastTime > 0.05) {
      on = !on;
      lastTime = timeStamp;
    }
    if (timeStamp - blinkTime > 1.0) {
      blinkTime = 0.0;
      LED_State = currentColor_GOAL;
    }

    Color blinkColor = on ? Color.kWhite : Color.kBlack;
    for (int i = 0; i < wlLEDBuffer.getLength(); i++) {
      wlLEDBuffer.setLED(i, blinkColor);
    }

    setActiveBuffer(wlLEDBuffer);
  }

  public void doCountdownBlink() {
    double timeStamp = Timer.getFPGATimestamp();
    int numLEDs = wlLEDBuffer.getLength(); // 42
    int half = numLEDs / 2; // 21

    // Phase 1: progressive fill (7.0s → 3.0s), Phase 2: full bar blink (3.0s → 0.0s)
    boolean phase2 = countdownRemainingTime <= 3.0;

    double blinkPeriod;
    if (!phase2) {
      // Phase 1: blink period 0.4s → 0.15s as remainingTime goes 7.0 → 3.0
      double t = 1.0 - (countdownRemainingTime - 3.0) / 4.0; // 0.0 → 1.0
      t = Math.max(0.0, Math.min(1.0, t));
      blinkPeriod = 0.4 - t * (0.4 - 0.15);
    } else {
      // Phase 2: blink period 0.12s → 0.05s as remainingTime goes 3.0 → 0.0
      double t = 1.0 - countdownRemainingTime / 3.0; // 0.0 → 1.0
      t = Math.max(0.0, Math.min(1.0, t));
      blinkPeriod = 0.12 - t * (0.12 - 0.05);
    }

    // Toggle on/off using existing blink fields
    if (timeStamp - lastTime >= blinkPeriod) {
      on = !on;
      lastTime = timeStamp;
    }

    if (!phase2) {
      // Phase 1: fill from both ends inward
      double progress = 1.0 - (countdownRemainingTime - 3.0) / 4.0; // 0.0 → 1.0
      progress = Math.max(0.0, Math.min(1.0, progress));
      int ledsPerSide = Math.max(2, (int) Math.ceil(progress * half));

      for (int i = 0; i < numLEDs; i++) {
        // Light LED if it's within ledsPerSide from either end
        boolean lit = (i < ledsPerSide) || (i >= numLEDs - ledsPerSide);
        if (lit && on) {
          wlLEDBuffer.setRGBW(i, 0, 0, 0, 255);
        } else {
          wlLEDBuffer.setRGB(i, 0, 0, 0);
        }
      }
    } else {
      // Phase 2: all LEDs blink
      for (int i = 0; i < numLEDs; i++) {
        if (on) {
          wlLEDBuffer.setRGBW(i, 0, 0, 0, 255);
        } else {
          wlLEDBuffer.setRGB(i, 0, 0, 0);
        }
      }
    }

    setActiveBuffer(wlLEDBuffer);
  }

  public void doBlinkRed() {
    LED_State = LED_EFFECTS.BLINK_RED;
    double timeStamp = Timer.getFPGATimestamp();

    if (timeStamp - lastTime > 0.1) {
      on = !on;
      lastTime = timeStamp;
    }
    if (on) {
      setActiveBuffer(wlRedLEDBuffer);
    } else {
      setActiveBuffer(wlBlackLEDBuffer);
    }
  }

  public void rainbowBlink() {
    LED_State = LED_EFFECTS.BLINK;
    double timeStamp = Timer.getFPGATimestamp();
    if (blinkTime == 0.0) {
      blinkTime = timeStamp;
    }
    if (timeStamp - lastTime > 0.1) {
      on = !on;
      lastTime = timeStamp;
      skittleCount++;
      skittleCount = skittleCount % 7;
      switch (skittleCount) {
        case 0:
          setActiveBuffer(wlRedLEDBuffer);
          break;
        case 1:
          setActiveBuffer(wlOrangeLEDBuffer);
          break;
        case 2:
          setActiveBuffer(wlYellowLEDBuffer);
          break;
        case 3:
          setActiveBuffer(wlGreenLEDBuffer);
          break;
        case 4:
          setActiveBuffer(wlBlueLEDBuffer);
          break;
        case 5:
          setActiveBuffer(wlIndigoLEDBuffer);
          break;
        case 6:
          setActiveBuffer(wlVioletLEDBuffer);
          break;
        default:
          break;
      }
    }
    if (timeStamp - blinkTime > 1.5) {
      blinkTime = 0.0;
      LED_State = currentColor_GOAL;
    }
  }

  public void blinkPurple() {
    LED_State = LED_EFFECTS.BLINK_PURPLE;
    double timeStamp = Timer.getFPGATimestamp();
    if (blinkTime == 0.0) {
      blinkTime = timeStamp;
    }
    if (timeStamp - lastTime > 0.05) {
      on = !on;
      lastTime = timeStamp;
    }
    if (timeStamp - blinkTime > 1.0) {
      blinkTime = 0.0;
      LED_State = currentColor_GOAL;
    }
    if (on) {
      setActiveBuffer(wlWhiteLEDBuffer);
    } else {
      setActiveBuffer(wlPurpleLEDBuffer);
    }
  }

  public void doSearchlightSingleEffect() {
    if (effectTimer.get() < updateInterval) {
      return;
    }

    for (int i = 0; i < wlLEDBuffer.getLength(); i++) {
      wlLEDBuffer.setLED(i, Color.kBlack);
    }

    int half = wlLEDBuffer.getLength() / 2;
    int effectiveCounter = counter / 5;

    int range = half - 1;
    int pos = effectiveCounter % (2 * range);
    if (pos > range) {
      pos = 2 * range - pos;
    }

    Color searchlightColor = Color.kYellow;

    wlLEDBuffer.setLED(pos, searchlightColor);
    wlLEDBuffer.setLED(half + pos, searchlightColor);

    setActiveBuffer(wlLEDBuffer);

    counter++;
    effectTimer.reset();
  }

  public void dynamicBlink() {
    double timeStamp = Timer.getFPGATimestamp();

    if (blinkTime == 0.0) {
      blinkTime = timeStamp;
      lastTime = timeStamp;
    }

    if (timeStamp - lastTime >= currentBlinkPeriod) {
      on = !on;
      lastTime = timeStamp;

      if (on) {
        setActiveBuffer(wlGreenLEDBuffer);
      } else {
        setActiveBuffer(wlBlackLEDBuffer);
      }
    }
  }

  public void doGreenRedWarning() {
    double timeStamp = Timer.getFPGATimestamp();
    int numLEDs = wlLEDBuffer.getLength();

    // First 3s (5.0→2.0): green/red split blink. Last 2s (2.0→0.0): blink red.
    boolean redOnlyPhase = warningRemainingTime <= 2.0;

    if (redOnlyPhase) {
      // Blink red on/off at ~5 Hz
      if (timeStamp - lastTime >= 0.1) {
        on = !on;
        lastTime = timeStamp;
      }
      if (on) {
        setActiveBuffer(wlRedLEDBuffer);
      } else {
        setActiveBuffer(wlBlackLEDBuffer);
      }
    } else {
      int half = numLEDs / 2;

      // Flip-flop at ~4 Hz (0.25s period)
      if (timeStamp - lastTime >= 0.125) {
        on = !on;
        lastTime = timeStamp;
      }

      for (int i = 0; i < numLEDs; i++) {
        boolean firstHalf = i < half;
        // on=true: first half green, second half red. on=false: flip.
        boolean greenPixel = (firstHalf && on) || (!firstHalf && !on);
        if (greenPixel) {
          wlLEDBuffer.setHSV(i, IndicatorLightConstants.GREEN_HUE, 255, 128);
        } else {
          wlLEDBuffer.setHSV(i, IndicatorLightConstants.RED_HUE, 255, 128);
        }
      }

      setActiveBuffer(wlLEDBuffer);
    }
  }

  public void updateBlinkPeriod(double lateralError) {
    if (lateralError <= MIN_TOLERANCE_BLINK) {
      currentBlinkPeriod = MIN_BLINK_PERIOD;
    } else if (lateralError >= MAX_TOLERANCE_BLINK) {
      currentBlinkPeriod = MAX_BLINK_PERIOD;
    } else {
      double fraction =
          (lateralError - MIN_TOLERANCE_BLINK) / (MAX_TOLERANCE_BLINK - MIN_TOLERANCE_BLINK);
      currentBlinkPeriod = MIN_BLINK_PERIOD + fraction * (MAX_BLINK_PERIOD - MIN_BLINK_PERIOD);
    }
  }

  // ========== Default lighting logic ==========

  private LED_EFFECTS updateLightingGoal() {
    // Disabled: orange RSL-style
    if (DriverStation.isDisabled()) {
      return LED_EFFECTS.RSL;
    }
    if (DriverStation.isAutonomous()) {
      return LED_EFFECTS.PURPLE;
    }

    // Alert: no game data and no dashboard override — shift schedule is unknown
    // Skip this alert when "Ignore Hub State" is on (e.g. practice without FMS)
    boolean ignoreHubState = SmartDashboard.getBoolean("Match/Ignore Hub State", true);
    boolean gameDataMissing =
        !ignoreHubState
            && DriverStation.getGameSpecificMessage().isEmpty()
            && HubShiftUtil.getAllianceWinOverride().isEmpty();
    autoWinnerNotSet.set(gameDataMissing);
    if (gameDataMissing) {
      return LED_EFFECTS.BLINK_RED;
    }

    // Teleop: green when active, red when inactive, blink white 7s before going active
    HubShiftUtil.ShiftInfo shiftInfo = HubShiftUtil.getOfficialShiftInfo();
    if (!shiftInfo.active() && shiftInfo.remainingTime() <= 7.0) {
      countdownRemainingTime = shiftInfo.remainingTime();
      return LED_EFFECTS.COUNTDOWN_BLINK;
    } else if (shiftInfo.active() && shiftInfo.remainingTime() <= 5.0) {
      warningRemainingTime = shiftInfo.remainingTime();
      return LED_EFFECTS.GREEN_RED_WARNING;
    } else if (shiftInfo.active()) {
      return LED_EFFECTS.GREEN;
    } else {
      return LED_EFFECTS.RED;
    }
  }

  private void doRsl() {
    setActiveBuffer(wlOrangeLEDBuffer);
  }

  private void setActiveBuffer(RGBWBuffer buffer) {
    currentActiveBuffer = buffer;
    buffer.flushToBuffer();
    wlLED.setData(buffer.getInternalBuffer());
  }
}
