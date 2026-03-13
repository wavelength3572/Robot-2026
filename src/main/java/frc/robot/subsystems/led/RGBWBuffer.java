package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Buffer wrapper for RGBW LED strips (e.g., SK6812 RGBW NeoPixels). WPILib's AddressableLED sends 3
 * bytes per pixel (GRB), but RGBW strips need 4 bytes per pixel (G, R, B, W). This class packs RGBW
 * data into an oversized RGB buffer so the total byte count on the wire matches.
 *
 * <p>For N physical RGBW LEDs, we allocate ceil(N * 4 / 3) virtual RGB LEDs in the internal buffer.
 */
public class RGBWBuffer {
  /** Global brightness scalar applied to all LED output (0.0 = off, 1.0 = full). */
  private static double brightnessScalar = 1.0;

  private final int numLEDs;
  private final AddressableLEDBuffer internalBuffer;
  private final byte[] rawBytes; // GRBW wire bytes for all LEDs
  private final Color[] logicalColors; // Intended display colors (for logging)

  public RGBWBuffer(int numLEDs) {
    this.numLEDs = numLEDs;
    int virtualLength = (numLEDs * 4 + 2) / 3; // ceil(numLEDs * 4 / 3)
    this.internalBuffer = new AddressableLEDBuffer(virtualLength);
    this.rawBytes = new byte[virtualLength * 3]; // Total wire bytes
    this.logicalColors = new Color[numLEDs];
    for (int i = 0; i < numLEDs; i++) {
      logicalColors[i] = Color.kBlack;
    }
  }

  /** Set the global brightness scalar (0.0 = off, 1.0 = full brightness). Clamped to [0.0, 1.0]. */
  public static void setBrightnessScalar(double scalar) {
    brightnessScalar = Math.max(0.0, Math.min(1.0, scalar));
  }

  /** Returns the current global brightness scalar. */
  public static double getBrightnessScalar() {
    return brightnessScalar;
  }

  /** Returns the number of physical RGBW LEDs. */
  public int getLength() {
    return numLEDs;
  }

  /** Returns the internal WPILib buffer (call {@link #flushToBuffer()} first). */
  public AddressableLEDBuffer getInternalBuffer() {
    return internalBuffer;
  }

  /** Returns the logical color of a pixel (for logging/display, ignores white channel). */
  public Color getLED(int index) {
    return logicalColors[index];
  }

  /**
   * Set a pixel to an RGBW color.
   *
   * @param index Physical LED index (0 to numLEDs-1)
   * @param r Red (0-255)
   * @param g Green (0-255)
   * @param b Blue (0-255)
   * @param w White (0-255)
   */
  public void setRGBW(int index, int r, int g, int b, int w) {
    logicalColors[index] = new Color(r / 255.0, g / 255.0, b / 255.0);
    // SK6812 RGBW wire order: Green, Red, Blue, White
    // Brightness scaling is applied later in flushToBuffer()
    int base = index * 4;
    rawBytes[base] = (byte) g;
    rawBytes[base + 1] = (byte) r;
    rawBytes[base + 2] = (byte) b;
    rawBytes[base + 3] = (byte) w;
  }

  /**
   * Set a pixel to an RGB color (white channel = 0).
   *
   * @param index Physical LED index
   * @param r Red (0-255)
   * @param g Green (0-255)
   * @param b Blue (0-255)
   */
  public void setRGB(int index, int r, int g, int b) {
    setRGBW(index, r, g, b, 0);
  }

  /**
   * Set a pixel using HSV values (white channel = 0).
   *
   * @param index Physical LED index
   * @param h Hue (0-180, WPILib convention)
   * @param s Saturation (0-255)
   * @param v Value/brightness (0-255)
   */
  public void setHSV(int index, int h, int s, int v) {
    Color c = Color.fromHSV(h, s, v);
    setRGBW(index, (int) (c.red * 255), (int) (c.green * 255), (int) (c.blue * 255), 0);
  }

  /**
   * Set a pixel to a WPILib Color (white channel = 0).
   *
   * @param index Physical LED index
   * @param color The color to set
   */
  public void setLED(int index, Color color) {
    setRGBW(index, (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0);
  }

  /**
   * Set a pixel to a WPILib Color8Bit (white channel = 0).
   *
   * @param index Physical LED index
   * @param color The color to set
   */
  public void setLED(int index, Color8Bit color) {
    setRGBW(index, color.red, color.green, color.blue, 0);
  }

  /**
   * Flush the RGBW raw bytes into the internal WPILib buffer. Must be called after setting pixels
   * and before passing the buffer to AddressableLED.setData().
   *
   * <p>Maps 4-byte RGBW data onto 3-byte virtual RGB LEDs. WPILib sends virtual LED bytes in GRB
   * order on the wire, so we map: wire byte at position v*3+0 = green channel, v*3+1 = red channel,
   * v*3+2 = blue channel.
   */
  public void flushToBuffer() {
    int virtualCount = internalBuffer.getLength();
    for (int v = 0; v < virtualCount; v++) {
      int wireBase = v * 3;
      // rawBytes are in wire order; WPILib sends [G, R, B] per virtual LED
      // So rawBytes[wireBase+0] should go into the G slot, +1 into R, +2 into B
      int g = (wireBase < rawBytes.length) ? Byte.toUnsignedInt(rawBytes[wireBase]) : 0;
      int r = (wireBase + 1 < rawBytes.length) ? Byte.toUnsignedInt(rawBytes[wireBase + 1]) : 0;
      int b = (wireBase + 2 < rawBytes.length) ? Byte.toUnsignedInt(rawBytes[wireBase + 2]) : 0;
      // Apply brightness scalar at output time so it works with pre-built buffers
      g = (int) (g * brightnessScalar);
      r = (int) (r * brightnessScalar);
      b = (int) (b * brightnessScalar);
      internalBuffer.setRGB(v, r, g, b);
    }
  }
}
