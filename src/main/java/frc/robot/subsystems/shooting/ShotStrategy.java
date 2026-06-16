package frc.robot.subsystems.shooting;

/**
 * Strategy interface for computing shot parameters. Implementations provide different ways to
 * determine launcher RPM, hood angle, motivator RPM, and spindexer RPM for a given distance.
 *
 * <p>Strategies are pure distance→commands functions. Velocity compensation, turret alignment, and
 * target selection are handled by the coordinator.
 */
public interface ShotStrategy {

  /**
   * Calculate shot parameters for a target at the given distance.
   *
   * @param distanceM Horizontal distance from turret to target in meters
   * @return Shot result with the 4 mechanical commands
   */
  ShotCalculator.ShotResult calculateShot(double distanceM);

  /**
   * Estimate the ball exit velocity for a given launcher RPM at a given distance. Used by the
   * coordinator for velocity compensation (time-of-flight estimation).
   *
   * @param launcherRPM Launcher wheel RPM
   * @param distanceM Horizontal distance to target in meters
   * @return Estimated exit velocity in m/s
   */
  double estimateExitVelocity(double launcherRPM, double distanceM);

  /** Human-readable name for logging/dashboard. */
  String getName();
}
