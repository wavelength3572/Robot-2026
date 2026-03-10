package frc.robot.subsystems.led;

public class IndicatorLightConstants {
  public static int ADDRESSABLE_LED_PORT = 0;
  public static int ADDRESSABLE_LED_BUFFER_LENGTH = 42;
  public static int YELLOW_HUE = 15;
  public static int ORANGE_HUE = 36;
  public static int GREEN_HUE = 60;
  public static int RED_HUE = 360;
  public static int BLUE_HUE = 87;
  public static int PURPLE_HUE = 141;
  public static int BLUE_OMBRE_MIN = 87;
  public static int BLUE_OMBRE_MAX = 107;
  public static int UPDATE_FREQUENCY = 15;

  public static enum LED_EFFECTS {
    RED,
    ORANGE,
    YELLOW,
    PURPLE,
    GREEN,
    BLUE,
    BLACK,
    WHITE,
    RAINBOW,
    BLUEOMBRE,
    BLINK,
    BLINK_RED,
    BLINK_PURPLE,
    POLKADOT,
    PARTY,
    RSL,
    SEGMENTPARTY,
    EXPLOSION,
    SEARCH_LIGHT,
    DYNAMIC_BLINK
  }

  // Define an array with 20 chosen colors (as RGB values)
  public static int[][] colorPalette = {
    {255, 0, 0}, // Red
    {0, 255, 0}, // Green
    {0, 0, 255}, // Blue
    {255, 255, 0}, // Yellow
    {255, 0, 255}, // Magenta
    {0, 255, 255}, // Cyan
    {128, 0, 0}, // Maroon
    {128, 128, 0}, // Olive
    {0, 128, 0}, // Dark Green
    {128, 0, 128}, // Purple
    {0, 128, 128}, // Teal
    {0, 0, 128}, // Navy
    {255, 165, 0}, // Orange
    {255, 20, 147}, // Deep Pink
    {75, 0, 130}, // Indigo
    {240, 128, 128}, // Light Coral
    {255, 215, 0}, // Gold
    {64, 224, 208}, // Turquoise
    {255, 105, 180}, // Hot Pink
    {0, 255, 127} // Spring Green
  };
}
