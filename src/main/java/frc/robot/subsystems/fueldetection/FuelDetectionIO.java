package frc.robot.subsystems.fueldetection;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for fuel detection from a coprocessor (Raspberry Pi). The Pi runs a trained vision
 * model to detect fuel on the field, calculates distance and angle, then publishes the
 * field-relative position to NetworkTables for the roboRIO to consume.
 */
public interface FuelDetectionIO {
  @AutoLog
  public static class FuelDetectionIOInputs {
    /** Whether the coprocessor is connected and publishing data. */
    public boolean connected = false;

    /** Whether the coprocessor currently sees fuel in its camera frame. */
    public boolean hasFuelTarget = false;

    /** Field-relative X position of the detected fuel (meters, blue alliance origin). */
    public double fuelFieldX = 0.0;

    /** Field-relative Y position of the detected fuel (meters, blue alliance origin). */
    public double fuelFieldY = 0.0;

    /** Confidence score from the detection model (0.0 to 1.0). */
    public double confidence = 0.0;

    /** Timestamp of the detection (FPGA seconds). */
    public double timestamp = 0.0;

    /** Distance from camera to detected fuel (meters). */
    public double distanceMeters = 0.0;

    /** Angle to the fuel relative to the camera's forward axis (radians, positive = left). */
    public double angleRadians = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FuelDetectionIOInputs inputs) {}
}
