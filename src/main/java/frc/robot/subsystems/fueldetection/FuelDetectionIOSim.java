package frc.robot.subsystems.fueldetection;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.FuelSim;
import java.util.function.Supplier;

/**
 * Simulated fuel detection IO that reads fuel positions from FuelSim. Simulates what the Raspberry
 * Pi would detect by finding the nearest fuel to the robot and computing the same fields the real Pi
 * would publish.
 */
public class FuelDetectionIOSim implements FuelDetectionIO {
  private final Supplier<Translation2d> robotPositionSupplier;

  /**
   * @param robotPositionSupplier Supplies the robot's current field-relative position
   */
  public FuelDetectionIOSim(Supplier<Translation2d> robotPositionSupplier) {
    this.robotPositionSupplier = robotPositionSupplier;
  }

  @Override
  public void updateInputs(FuelDetectionIOInputs inputs) {
    inputs.connected = true;

    Translation2d robotPos = robotPositionSupplier.get();
    Translation2d nearestFuel = FuelSim.getInstance().getNearestFuelPosition(robotPos);

    if (nearestFuel != null) {
      inputs.hasFuelTarget = true;
      inputs.fuelFieldX = nearestFuel.getX();
      inputs.fuelFieldY = nearestFuel.getY();
      inputs.confidence = 0.95;
      inputs.timestamp = Timer.getFPGATimestamp();

      Translation2d robotToFuel = nearestFuel.minus(robotPos);
      inputs.distanceMeters = robotToFuel.getNorm();
      inputs.angleRadians = robotToFuel.getAngle().getRadians();
    } else {
      inputs.hasFuelTarget = false;
      inputs.fuelFieldX = 0.0;
      inputs.fuelFieldY = 0.0;
      inputs.confidence = 0.0;
      inputs.timestamp = Timer.getFPGATimestamp();
      inputs.distanceMeters = 0.0;
      inputs.angleRadians = 0.0;
    }
  }
}
