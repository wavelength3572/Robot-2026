package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.led.IndicatorLightConstants.LED_EFFECTS;
import frc.robot.util.HubShiftUtil;
import java.util.Random;
import org.littletonrobotics.junction.Logger;

public class IndicatorLight extends SubsystemBase {

  private LED_EFFECTS currentColor_GOAL = LED_EFFECTS.BLACK;
  private LED_EFFECTS LED_State = LED_EFFECTS.BLACK;

  // Define constants for the blink period (in seconds)
  private static final double MAX_BLINK_PERIOD = 0.5; // far from target, slow blink
  private static final double MIN_BLINK_PERIOD = 0.001; // very close, fast blink

  public static final double MIN_TOLERANCE_BLINK = 0.025; // below this, blink is at min period
  public static final double MAX_TOLERANCE_BLINK = 0.150; // above this, blink is at max period

  // Variable to hold the current blink period computed from the error
  private double currentBlinkPeriod = MAX_BLINK_PERIOD;

  private AddressableLED wlLED;
  private AddressableLEDBuffer wlLEDBuffer;
  private AddressableLEDBuffer wlGreenLEDBuffer;
  private AddressableLEDBuffer wlOrangeLEDBuffer;
  private AddressableLEDBuffer wlPurpleLEDBuffer;
  private AddressableLEDBuffer wlRedLEDBuffer;
  private AddressableLEDBuffer wlYellowLEDBuffer;
  private AddressableLEDBuffer wlBlueLEDBuffer;
  private AddressableLEDBuffer wlIndigoLEDBuffer;
  private AddressableLEDBuffer wlVioletLEDBuffer;
  private AddressableLEDBuffer wlWhiteLEDBuffer;
  private AddressableLEDBuffer wlBlackLEDBuffer;

  // Store what the last hue of the first pixel is
  private int rainbowFirstPixelHue = 0;
  private int currentSaturation = 100;
  private boolean forward = true;
  private int counter = 0;
  private double lastTime = 0.0;
  private double blinkTime = 0.0;
  private boolean on = false;
  private int skittleCount = 0;

  private Random random = new Random();

  private Timer effectTimer = new Timer();
  private final double restartInterval = 5.0; // Restart effect every 5 seconds
  private int effectPhase = 0;
  private final int maxBrightness = 255;
  private int center = 9;
  private final double updateInterval = 0.05; // Interval in seconds for updates

  private AddressableLEDBuffer currentActiveBuffer;

  public IndicatorLight() {

    wlLED = new AddressableLED(IndicatorLightConstants.ADDRESSABLE_LED_PORT);
    wlLEDBuffer = new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    wlLED.setLength(wlLEDBuffer.getLength());
    center = wlLEDBuffer.getLength() / 2;
    currentActiveBuffer = wlLEDBuffer; // Set the default active buffer
    wlLED.setData(wlLEDBuffer);
    wlLED.start();

    wlGreenLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlGreenLEDBuffer.getLength(); i++) {
      wlGreenLEDBuffer.setHSV(i, IndicatorLightConstants.GREEN_HUE, 255, 128);
    }

    wlOrangeLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlOrangeLEDBuffer.getLength(); i++) {
      wlOrangeLEDBuffer.setLED(i, new Color8Bit(255, 27, 0));
    }

    wlPurpleLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlPurpleLEDBuffer.getLength(); i++) {
      wlPurpleLEDBuffer.setHSV(i, IndicatorLightConstants.PURPLE_HUE, 63, 92);
    }

    wlRedLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlRedLEDBuffer.getLength(); i++) {
      wlRedLEDBuffer.setHSV(i, IndicatorLightConstants.RED_HUE, 255, 128);
    }

    wlYellowLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (int i = 0; i < wlYellowLEDBuffer.getLength(); i++) {
      wlYellowLEDBuffer.setLED(i, Color.kYellow);
    }

    wlBlueLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlBlueLEDBuffer.getLength(); i++) {
      wlBlueLEDBuffer.setLED(i, Color.kBlue);
    }

    wlIndigoLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlIndigoLEDBuffer.getLength(); i++) {
      wlIndigoLEDBuffer.setLED(i, Color.kIndigo);
    }

    wlVioletLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlVioletLEDBuffer.getLength(); i++) {
      wlVioletLEDBuffer.setLED(i, Color.kViolet);
    }

    wlWhiteLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlWhiteLEDBuffer.getLength(); i++) {
      wlWhiteLEDBuffer.setLED(i, Color.kWhite);
    }

    wlBlackLEDBuffer =
        new AddressableLEDBuffer(IndicatorLightConstants.ADDRESSABLE_LED_BUFFER_LENGTH);
    for (var i = 0; i < wlBlackLEDBuffer.getLength(); i++) {
      wlBlackLEDBuffer.setLED(i, Color.kBlack);
    }

    effectTimer.start();
  }

  @Override
  public void periodic() {
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
      case BLINK_PURPLE -> blinkPurple();
      case PARTY -> doParty();
      case RSL -> doRsl();
      case SEGMENTPARTY -> doSegmentParty();
      case EXPLOSION -> doExplosionEffect();
      case POLKADOT -> doPokadot();
      case SEARCH_LIGHT -> doSearchlightSingleEffect();
      case DYNAMIC_BLINK -> dynamicBlink();
      default -> {}
    }

    publishLEDColors();
    Logger.recordOutput("LED/Pattern", LED_State.toString());
  }

  /** Publish LED buffer colors as a hex string array for AdvantageScope visualization. */
  private void publishLEDColors() {
    int length = currentActiveBuffer.getLength();
    String[] colors = new String[length];
    for (int i = 0; i < length; i++) {
      Color c = currentActiveBuffer.getLED(i);
      colors[i] =
          String.format(
              "#%02X%02X%02X", (int) (c.red * 255), (int) (c.green * 255), (int) (c.blue * 255));
    }
    Logger.recordOutput("LED/Colors", colors);
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
        wlLED.setData(wlLEDBuffer);
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
        wlLED.setData(wlLEDBuffer);
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
    wlLED.setData(wlLEDBuffer);
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

    wlLED.setData(wlLEDBuffer);
  }

  private void doPokadot() {
    for (var i = 0; i < wlLEDBuffer.getLength(); i++) {
      int red = random.nextInt(256);
      int green = random.nextInt(256);
      int blue = random.nextInt(256);
      wlLEDBuffer.setRGB(i, red, green, blue);
    }
    wlLED.setData(wlLEDBuffer);
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

    // Teleop: green when active, red when inactive, blink white 7s before going active
    HubShiftUtil.ShiftInfo shiftInfo = HubShiftUtil.getOfficialShiftInfo();
    if (!shiftInfo.active() && shiftInfo.remainingTime() <= 7.0) {
      return LED_EFFECTS.BLINK;
    } else if (shiftInfo.active()) {
      return LED_EFFECTS.GREEN;
    } else {
      return LED_EFFECTS.RED;
    }
  }

  private void doRsl() {
    setActiveBuffer(wlOrangeLEDBuffer);
  }

  private void setActiveBuffer(AddressableLEDBuffer buffer) {
    currentActiveBuffer = buffer;
    wlLED.setData(buffer);
  }
}
