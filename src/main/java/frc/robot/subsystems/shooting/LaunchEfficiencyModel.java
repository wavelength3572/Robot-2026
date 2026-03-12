package frc.robot.subsystems.shooting;

import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * RPM-dependent launch efficiency model. Back-calculates the actual efficiency (ratio of ball exit
 * velocity to wheel surface velocity) from empirical LUT data, then fits a linear model so the
 * parametric system can predict accurate RPMs at any distance.
 *
 * <p>Why not a single constant? Higher RPM = more slip between wheel and ball = lower efficiency.
 * A static 0.55 might be close at 2200 RPM but way off at 3000 RPM — which is exactly why
 * parametric drifts at longer distances.
 *
 * <p>The model is: efficiency(rpm) = intercept + slope * rpm
 *
 * <p>Refit anytime by calling {@link #fitFromLUTData} with current LUT entries. A dashboard button
 * in ShootingCoordinator triggers this.
 */
public class LaunchEfficiencyModel {

  private static final double GRAVITY = 9.81;
  private static final double WHEEL_RADIUS_METERS = 0.0508;

  // Linear fit coefficients: efficiency = intercept + slope * RPM
  private double intercept;
  private double slope;
  private int dataPointCount = 0;

  // Hub geometry (must match TrajectoryOptimizer)
  private static final double HUB_CENTER_HEIGHT = 1.43;

  // Turret height — set once at construction
  private final double turretHeightM;

  /**
   * Create a new efficiency model with default fit from initial LUT data analysis.
   *
   * @param turretHeightM Turret launch height above ground in meters
   */
  public LaunchEfficiencyModel(double turretHeightM) {
    this.turretHeightM = turretHeightM;
    // Default fit derived from 10-point LUT data (RPM range 2200-3000):
    //   efficiency ≈ 0.56 - 0.000035 * RPM
    // This gives ~0.49 at 2200 RPM and ~0.45 at 3000 RPM
    this.intercept = 0.56;
    this.slope = -0.000035;
    this.dataPointCount = 0;
  }

  /**
   * Get the efficiency for a given RPM.
   *
   * @param rpm Launcher wheel RPM
   * @return Efficiency factor (typically 0.4-0.55)
   */
  public double getEfficiency(double rpm) {
    double eff = intercept + slope * rpm;
    // Clamp to reasonable range — don't let bad extrapolation go wild
    return Math.max(0.2, Math.min(0.8, eff));
  }

  /**
   * Refit the linear model from current LUT data. For each entry, back-calculates the exit
   * velocity that must have been produced (from projectile physics to the hub), computes the wheel
   * surface velocity from RPM, and derives efficiency = exitVelocity / surfaceVelocity.
   *
   * <p>Then fits a line through (RPM, efficiency) using least-squares regression.
   *
   * @param entries LUT entries with empirically verified RPM, hood angle, and distance
   * @return Number of valid data points used in the fit
   */
  public int fitFromLUTData(List<StationaryShotBatchRecorder.LUTEntry> entries) {
    if (entries.size() < 2) {
      Logger.recordOutput("Shots/Efficiency/FitStatus", "Not enough data (need >= 2)");
      return 0;
    }

    // Back-calculate efficiency for each entry
    double[] rpms = new double[entries.size()];
    double[] efficiencies = new double[entries.size()];
    int validCount = 0;

    for (StationaryShotBatchRecorder.LUTEntry entry : entries) {
      double exitVelocity = backCalculateExitVelocity(entry.distanceM(), entry.hoodAngleDeg());
      if (Double.isNaN(exitVelocity) || exitVelocity <= 0) {
        continue; // Skip entries where physics doesn't work out
      }

      double surfaceVelocity = rpmToSurfaceVelocity(entry.rpm());
      if (surfaceVelocity <= 0) {
        continue;
      }

      double efficiency = exitVelocity / surfaceVelocity;

      // Sanity check — efficiency should be between 0.2 and 0.8
      if (efficiency < 0.2 || efficiency > 0.8) {
        continue;
      }

      rpms[validCount] = entry.rpm();
      efficiencies[validCount] = efficiency;
      validCount++;
    }

    if (validCount < 2) {
      Logger.recordOutput("Shots/Efficiency/FitStatus", "Not enough valid points (need >= 2)");
      return 0;
    }

    // Least-squares linear regression: efficiency = intercept + slope * RPM
    double sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
    for (int i = 0; i < validCount; i++) {
      sumX += rpms[i];
      sumY += efficiencies[i];
      sumXX += rpms[i] * rpms[i];
      sumXY += rpms[i] * efficiencies[i];
    }

    double n = validCount;
    double denominator = n * sumXX - sumX * sumX;
    if (Math.abs(denominator) < 1e-10) {
      // All RPMs are the same — can't fit a line, use mean efficiency
      this.intercept = sumY / n;
      this.slope = 0.0;
    } else {
      this.slope = (n * sumXY - sumX * sumY) / denominator;
      this.intercept = (sumY - this.slope * sumX) / n;
    }

    this.dataPointCount = validCount;

    // Log the fit results
    Logger.recordOutput("Shots/Efficiency/FitStatus", "OK (" + validCount + " points)");
    Logger.recordOutput("Shots/Efficiency/Intercept", intercept);
    Logger.recordOutput("Shots/Efficiency/Slope", slope);
    Logger.recordOutput("Shots/Efficiency/At2200RPM", getEfficiency(2200));
    Logger.recordOutput("Shots/Efficiency/At2600RPM", getEfficiency(2600));
    Logger.recordOutput("Shots/Efficiency/At3000RPM", getEfficiency(3000));

    System.out.println(
        String.format(
            "[LaunchEfficiencyModel] Refit from %d points: efficiency = %.4f + %.7f * RPM"
                + "  (%.3f @ 2200, %.3f @ 2600, %.3f @ 3000)",
            validCount,
            intercept,
            slope,
            getEfficiency(2200),
            getEfficiency(2600),
            getEfficiency(3000)));

    return validCount;
  }

  /**
   * Back-calculate the exit velocity required to reach the hub center from a given distance and
   * hood angle, using projectile physics (no drag).
   *
   * <p>The ball launches from turretHeight at launchAngle and must arrive at HUB_CENTER_HEIGHT at
   * the given horizontal distance.
   *
   * @param distanceM Horizontal distance to hub center
   * @param hoodAngleDeg Hood mechanical angle in degrees
   * @return Required exit velocity in m/s, or NaN if geometry is invalid
   */
  private double backCalculateExitVelocity(double distanceM, double hoodAngleDeg) {
    double launchAngleRad = Math.toRadians(90.0 - hoodAngleDeg);
    double cosTheta = Math.cos(launchAngleRad);
    double tanTheta = Math.tan(launchAngleRad);

    double deltaH = HUB_CENTER_HEIGHT - turretHeightM;
    double denominator = 2.0 * cosTheta * cosTheta * (distanceM * tanTheta - deltaH);

    if (denominator <= 0) {
      return Double.NaN; // Geometry doesn't work — can't reach target
    }

    double vSquared = GRAVITY * distanceM * distanceM / denominator;
    if (vSquared <= 0) {
      return Double.NaN;
    }

    return Math.sqrt(vSquared);
  }

  /** Convert RPM to wheel surface velocity (before efficiency). */
  private static double rpmToSurfaceVelocity(double rpm) {
    return (rpm * 2.0 * Math.PI * WHEEL_RADIUS_METERS) / 60.0;
  }

  /** Get the current linear fit intercept. */
  public double getIntercept() {
    return intercept;
  }

  /** Get the current linear fit slope. */
  public double getSlope() {
    return slope;
  }

  /** Get the number of data points used in the last fit. */
  public int getDataPointCount() {
    return dataPointCount;
  }

  /** Whether the model has been fit from real data (vs using defaults). */
  public boolean isFitted() {
    return dataPointCount > 0;
  }
}
