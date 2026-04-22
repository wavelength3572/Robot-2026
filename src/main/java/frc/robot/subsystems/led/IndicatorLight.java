package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.led.IndicatorLightConstants.LED_EFFECTS;
import frc.robot.subsystems.shooting.ShootingCoordinator;
import frc.robot.util.HubShiftUtil;
import java.util.Random;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class IndicatorLight extends SubsystemBase {

  /** Turret absolute encoder validation state, checked at startup. */
  public enum TurretEncoderStatus {
    VALID,
    WARNING,
    ERROR
  }

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

  private double countdownRemainingTime = 10.0;
  private double warningRemainingTime = 5.0;

  // Turret encoder validation supplier (set by RobotContainer)
  private Supplier<TurretEncoderStatus> turretEncoderStatusSupplier =
      () -> TurretEncoderStatus.VALID;

  // Shooting coordinator supplier — drives edge-LED SmartLaunch status overlay
  private Supplier<ShootingCoordinator.CoordinatorState> coordinatorStateSupplier = null;
  private BooleanSupplier smartLaunchActiveSupplier = () -> false;

  // The buffer most recently chosen by the current effect — written to hardware once at end of
  // periodic() so overlays (like SmartLaunch edge LEDs) don't cause a double-flush flicker.
  private RGBWBuffer pendingBuffer = null;

  // Emergency strobe state
  private double strobePhaseStartTime = 0.0;
  private int strobePhase = 0; // 0=strobe, 1=converge, 2=flash
  private boolean strobeOn = false;
  private double strobeLastToggle = 0.0;

  // Amber breathing state
  private double amberBreathAngle = 0.0;

  private Random random = new Random();

  private Timer effectTimer = new Timer();
  private final double restartInterval = 5.0; // Restart effect every 5 seconds
  private int effectPhase = 0;
  private final int maxBrightness = 255;
  private int center = 9;
  private final double updateInterval = 0.05; // Interval in seconds for updates

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

    // Turret encoder validation overrides everything (except disabled RSL)
    TurretEncoderStatus encoderStatus = turretEncoderStatusSupplier.get();
    if (encoderStatus == TurretEncoderStatus.ERROR) {
      LED_State = LED_EFFECTS.TURRET_ENCODER_ERROR;
      doTurretEncoderError();
    } else if (encoderStatus == TurretEncoderStatus.WARNING) {
      LED_State = LED_EFFECTS.TURRET_ENCODER_WARNING;
      doTurretEncoderWarning();
    } else if (DriverStation.isDisabled()) {
      // Disabled: blue ombre in pit mode, RSL otherwise
      if (mode == LightMode.PIT || Constants.currentMode == Constants.Mode.PIT) {
        LED_State = LED_EFFECTS.BLUEOMBRE;
        doBlueOmbre();
      } else {
        LED_State = LED_EFFECTS.RSL;
        doRsl();
      }
    } else if (mode == LightMode.OFF) {
      LED_State = LED_EFFECTS.BLACK;
      setActiveBuffer(wlBlackLEDBuffer);
    } else {
      // MATCH mode: normal auto-lighting logic
      currentColor_GOAL = updateLightingGoal();

      if (LED_State != LED_EFFECTS.BLINK) {
        LED_State = currentColor_GOAL;
      }
      Logger.recordOutput("LEDs/State", LED_State.name());
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
        case REDOMBRE -> doRedOmbre();
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
        case TURRET_ENCODER_WARNING -> doTurretEncoderWarning();
        case TURRET_ENCODER_ERROR -> doTurretEncoderError();
        default -> {}
      }
    }

    // SmartLaunch status overlay: paint edge LEDs (0-2, 39-41) based on coordinator state.
    // Runs after the main effect so it overlays whatever the normal lighting chose.
    applySmartLaunchOverlay();

    // Single flush to hardware — after all effects and overlays have written into pendingBuffer.
    flushToHardware();
  }

  /** Paint edge LEDs with SmartLaunch coordinator state when active. */
  private void applySmartLaunchOverlay() {
    if (coordinatorStateSupplier == null || !smartLaunchActiveSupplier.getAsBoolean()) {
      return;
    }
    ShootingCoordinator.CoordinatorState state = coordinatorStateSupplier.get();
    if (state == ShootingCoordinator.CoordinatorState.INACTIVE) {
      return;
    }

    // Pick color based on coordinator state
    int r, g, b;
    switch (state) {
      case FIRING -> {
        r = 0;
        g = 255;
        b = 0;
      } // green — actively shooting or ready to fire
      case HELD -> {
        r = 0;
        g = 255;
        b = 255;
      } // cyan — ready, holding fire (release to shoot)
      case AIMING, SETTLING, UNARMED -> {
        r = 255;
        g = 180;
        b = 0;
      } // yellow/amber — acquiring
      case NO_FIRE_ZONE -> {
        r = 255;
        g = 0;
        b = 0;
      } // red — zone blocks shooting
      default -> {
        return;
      }
    }

    if (pendingBuffer == null) return;

    // If the pending buffer is a shared pre-built buffer (solid color), copy it into wlLEDBuffer
    // so we don't permanently corrupt the shared buffer with overlay pixels.
    if (pendingBuffer != wlLEDBuffer) {
      wlLEDBuffer.copyFrom(pendingBuffer);
      pendingBuffer = wlLEDBuffer;
    }

    // Write edge LEDs: 0-2 (left end) and 39-41 (right end)
    int len = pendingBuffer.getLength();
    for (int i = 0; i < 3 && i < len; i++) {
      pendingBuffer.setRGB(i, r, g, b);
    }
    for (int i = Math.max(0, len - 3); i < len; i++) {
      pendingBuffer.setRGB(i, r, g, b);
    }

    Logger.recordOutput("LEDs/SmartLaunchOverlay", state.name());
  }

  /**
   * Set the supplier for turret encoder validation status. When ERROR or WARNING, the LED pattern
   * overrides normal match lighting.
   */
  public void setTurretEncoderStatusSupplier(Supplier<TurretEncoderStatus> supplier) {
    this.turretEncoderStatusSupplier = supplier;
  }

  /**
   * Set the shooting coordinator suppliers for SmartLaunch status overlay on edge LEDs. When
   * SmartLaunch is active, LEDs 0-2 and 39-41 show coordinator state: green = FIRING/ready, yellow
   * = AIMING/SETTLING, cyan = HELD (ready, release to fire), red = NO_FIRE_ZONE.
   */
  public void setShootingCoordinatorSuppliers(
      Supplier<ShootingCoordinator.CoordinatorState> stateSupplier,
      BooleanSupplier activeSupplier) {
    this.coordinatorStateSupplier = stateSupplier;
    this.smartLaunchActiveSupplier = activeSupplier;
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

  public void doRedOmbre() {
    for (var i = 0; i < wlLEDBuffer.getLength(); i++) {
      final var saturation = (currentSaturation + (i * 255 / wlLEDBuffer.getLength())) % 255;
      wlLEDBuffer.setHSV(i, 0, 255, saturation);
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

    // Phase 1: progressive fill (cutoff → 3.0s), Phase 2: full bar blink (3.0s → 0.0s)
    double cutoff = HubShiftUtil.preActiveCutoffSeconds.get();
    double phase1Duration = Math.max(cutoff - 3.0, 0.1); // avoid divide-by-zero
    boolean phase2 = countdownRemainingTime <= 3.0;

    double blinkPeriod;
    if (!phase2) {
      // Phase 1: blink period 0.4s → 0.15s as remainingTime goes cutoff → 3.0
      double t = 1.0 - (countdownRemainingTime - 3.0) / phase1Duration; // 0.0 → 1.0
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
      double progress = 1.0 - (countdownRemainingTime - 3.0) / phase1Duration; // 0.0 → 1.0
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

  // ========== Turret encoder validation patterns ==========

  /**
   * Emergency strobe — 3-phase repeating cycle for critical encoder error.
   *
   * <p>Phase 0 (0.4s): Rapid red/white alternating strobe every 50ms. Phase 1 (0.3s): Red pixels
   * converge from both ends to center. Phase 2 (0.3s): Full white flash then instant blackout.
   */
  private void doTurretEncoderError() {
    double now = Timer.getFPGATimestamp();
    int numLEDs = wlLEDBuffer.getLength();

    // Initialize phase timer on first call
    if (strobePhaseStartTime == 0.0) {
      strobePhaseStartTime = now;
      strobePhase = 0;
      strobeLastToggle = now;
    }

    double phaseElapsed = now - strobePhaseStartTime;

    // Advance phases
    if (strobePhase == 0 && phaseElapsed >= 0.4) {
      strobePhase = 1;
      strobePhaseStartTime = now;
      phaseElapsed = 0.0;
    } else if (strobePhase == 1 && phaseElapsed >= 0.3) {
      strobePhase = 2;
      strobePhaseStartTime = now;
      phaseElapsed = 0.0;
    } else if (strobePhase == 2 && phaseElapsed >= 0.3) {
      strobePhase = 0;
      strobePhaseStartTime = now;
      phaseElapsed = 0.0;
    }

    if (strobePhase == 0) {
      // Phase 0: rapid red/white alternating strobe
      if (now - strobeLastToggle >= 0.05) {
        strobeOn = !strobeOn;
        strobeLastToggle = now;
      }
      for (int i = 0; i < numLEDs; i++) {
        // Alternate even/odd pixels, swap each toggle
        boolean even = (i % 2 == 0);
        boolean redPixel = (even == strobeOn);
        if (redPixel) {
          wlLEDBuffer.setRGB(i, 255, 0, 0);
        } else {
          wlLEDBuffer.setRGBW(i, 0, 0, 0, 255);
        }
      }
    } else if (strobePhase == 1) {
      // Phase 1: red converge from both ends to center
      double progress = phaseElapsed / 0.3; // 0.0 → 1.0
      int reachIndex = (int) (progress * (numLEDs / 2));

      for (int i = 0; i < numLEDs; i++) {
        wlLEDBuffer.setRGB(i, 0, 0, 0); // black base
      }
      // Leading edge from left
      for (int i = Math.max(0, reachIndex - 2); i <= Math.min(reachIndex, numLEDs / 2); i++) {
        int brightness = 255 - (reachIndex - i) * 80;
        wlLEDBuffer.setRGB(i, Math.max(0, brightness), 0, 0);
      }
      // Leading edge from right (mirror)
      for (int i = Math.max(numLEDs / 2, numLEDs - 1 - reachIndex);
          i <= Math.min(numLEDs - 1, numLEDs - 1 - reachIndex + 2);
          i++) {
        int brightness = 255 - (i - (numLEDs - 1 - reachIndex)) * 80;
        wlLEDBuffer.setRGB(i, Math.max(0, brightness), 0, 0);
      }
    } else {
      // Phase 2: white flash then blackout
      boolean flash = phaseElapsed < 0.12;
      for (int i = 0; i < numLEDs; i++) {
        if (flash) {
          wlLEDBuffer.setRGBW(i, 0, 0, 0, 255);
        } else {
          wlLEDBuffer.setRGB(i, 0, 0, 0);
        }
      }
    }

    setActiveBuffer(wlLEDBuffer);
  }

  /** Amber breathing pulse for encoder warning (5-15° offset). Smooth sine-wave brightness. */
  private void doTurretEncoderWarning() {
    int numLEDs = wlLEDBuffer.getLength();

    // Advance breath angle (~1.5s full cycle)
    amberBreathAngle += 0.06;
    if (amberBreathAngle > 2.0 * Math.PI) {
      amberBreathAngle -= 2.0 * Math.PI;
    }

    // Sine wave: 0.0 → 1.0 brightness
    double brightness = (Math.sin(amberBreathAngle) + 1.0) / 2.0;
    int value = (int) (brightness * 200) + 20; // never fully off, range 20-220

    for (int i = 0; i < numLEDs; i++) {
      // Warm amber: HSV hue 15 (yellow-orange), full saturation
      wlLEDBuffer.setHSV(i, IndicatorLightConstants.YELLOW_HUE, 255, value);
    }

    setActiveBuffer(wlLEDBuffer);
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

    // Teleop: green when active, red when inactive, countdown blink before going active
    HubShiftUtil.ShiftInfo shiftInfo = HubShiftUtil.getOfficialShiftInfo();
    if (!shiftInfo.active()
        && shiftInfo.remainingTime() <= HubShiftUtil.preActiveCutoffSeconds.get()) {
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
    pendingBuffer = buffer;
  }

  /** Flush the pending buffer to hardware once, after all overlays have been applied. */
  private void flushToHardware() {
    if (pendingBuffer == null) return;
    pendingBuffer.flushToBuffer();
    wlLED.setData(pendingBuffer.getInternalBuffer());

    // Sim-only: publish LED colors as hex strings for Elastic Multi Color View widget
    if (Constants.currentMode == Constants.Mode.SIM) {
      String[] colors = new String[pendingBuffer.getLength()];
      for (int i = 0; i < pendingBuffer.getLength(); i++) {
        colors[i] = pendingBuffer.getLED(i).toHexString();
      }
      SmartDashboard.putStringArray("Sim/LED Strip", colors);
    }
  }
}
