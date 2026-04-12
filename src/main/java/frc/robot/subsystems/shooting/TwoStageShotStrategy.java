package frc.robot.subsystems.shooting;

import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ZoneDetector;
import org.littletonrobotics.junction.Logger;

/**
 * Two-stage velocity model shot strategy for hub shots. Uses the same fixed-height parabola solver
 * as {@link FixedHeightShotStrategy} but with a physically-correct velocity model that accounts for
 * the motivator wheel's contribution.
 *
 * <p>The two-stage model:
 *
 * <pre>
 *   v_m    = k_m * v_motivator_surface           (ball speed after motivator)
 *   v_exit = v_m + k_L * (v_roller_avg - v_m)    (launcher adjusts from there)
 * </pre>
 *
 * <p>Where {@code k_m} (motivator coupling) is how much of the motivator's surface speed the ball
 * achieves after the motivator stage, and {@code k_L} (launcher coupling) is how much of the
 * remaining speed gap the launcher closes. This correctly models the speed-dependent interaction:
 * when rollers are faster than the motivator exit speed, the launcher speeds the ball up; when the
 * motivator is faster, the launcher slows it down. The effective efficiency naturally varies with
 * distance without needing per-zone tuning.
 *
 * <p>The key advantage: two parameters replace four distance-interpolated efficiency values, and
 * the model holds when motivator RPM changes.
 */
public class TwoStageShotStrategy implements ShotStrategy {

  private static final double INCHES_TO_METERS = 0.0254;
  private static final RobotConfig config = Constants.getRobotConfig();

  // All spatial tunables in inches (same defaults as FixedHeight)
  private static final LoggedTunableNumber peakHeightIn =
      new LoggedTunableNumber("Shots/TwoStage/PeakHeightIn", config.getFixedHeightPeakHeightIn());

  private static final LoggedTunableNumber passThroughHeightIn =
      new LoggedTunableNumber(
          "Shots/TwoStage/PassThroughHeightIn", config.getFixedHeightPassThroughHeightIn());

  private static final LoggedTunableNumber horizontalOffsetIn =
      new LoggedTunableNumber(
          "Shots/TwoStage/HorizontalOffsetIn", config.getFixedHeightHorizontalOffsetIn());

  // RPM limits
  private static final LoggedTunableNumber minRPM =
      new LoggedTunableNumber("Shots/TwoStage/MinRPM", config.getFixedHeightMinRPM());

  private static final LoggedTunableNumber maxRPM =
      new LoggedTunableNumber("Shots/TwoStage/MaxRPM", config.getFixedHeightMaxRPM());

  // ========== Two-Stage Velocity Model ==========
  // v_exit = efficiency * v_roller_avg + coupling * v_motivator_surface

  // Motivator coupling (k_m): fraction of motivator surface speed the ball achieves after
  // passing through the motivator wheel. 0 = motivator does nothing, 1 = ball matches motivator.
  private static final LoggedTunableNumber motivatorCoupling =
      new LoggedTunableNumber("Shots/TwoStage/MotivatorCoupling", 0.80);

  // Launcher coupling (k_L): fraction of the speed gap between motivator exit speed and roller
  // surface speed that the launcher closes. 0 = ball passes through unchanged, 1 = ball fully
  // matches roller speed. This captures roller grip and contact time.
  private static final LoggedTunableNumber launcherCoupling =
      new LoggedTunableNumber("Shots/TwoStage/LauncherCoupling", 0.76);

  // Motivator wheel RPM used for surface speed calculation (should match what we command).
  private static final LoggedTunableNumber motivatorWheelRPM =
      new LoggedTunableNumber("Shots/TwoStage/MotivatorWheelRPM", config.getShootingMotivatorRPM());

  // ========== Hood Angle Offsets (distance-interpolated, degrees) ==========
  // Additive offset applied after the parabola solver. Starting at 0 — if the two-stage model
  // is accurate, these should stay near zero.
  private static final LoggedTunableNumber hoodAngleOffsetClose =
      new LoggedTunableNumber("Shots/TwoStage/HoodAngleOffset/Close", 0.0);
  private static final LoggedTunableNumber hoodAngleOffsetMid =
      new LoggedTunableNumber("Shots/TwoStage/HoodAngleOffset/Mid", 0.0);
  private static final LoggedTunableNumber hoodAngleOffsetFar =
      new LoggedTunableNumber("Shots/TwoStage/HoodAngleOffset/Far", 0.0);
  private static final LoggedTunableNumber hoodAngleOffsetCorner =
      new LoggedTunableNumber("Shots/TwoStage/HoodAngleOffset/Corner", 0.0);

  // ========== Motivator / Spindexer Output Commands ==========
  private static final LoggedTunableNumber motivatorOutputRPM =
      new LoggedTunableNumber(
          "Shots/TwoStage/MotivatorOutputRPM", config.getShootingMotivatorRPM());
  private static final LoggedTunableNumber spindexerOutputRPM =
      new LoggedTunableNumber("Shots/TwoStage/SpindexerRPM", config.getSpindexerCloseRPM());

  // Hood limits from hardware config
  private static final double HOOD_MIN_DEG = config.getHoodMinAngleDegrees();
  private static final double HOOD_MAX_DEG = config.getHoodMaxAngleDegrees();

  // ========== Velocity Model ==========

  /** Calculate the motivator wheel surface speed in m/s from its RPM. */
  private double motivatorSurfaceSpeed() {
    return (motivatorWheelRPM.get() * 2.0 * Math.PI * ShotCalculator.MOTIVATOR_WHEEL_RADIUS_METERS)
        / 60.0;
  }

  /** Calculate the average launcher roller surface speed in m/s from RPM. */
  private double rollerAvgSurfaceSpeed(double launcherRPM) {
    double mainSurface =
        (launcherRPM * 2.0 * Math.PI * ShotCalculator.MAIN_WHEEL_RADIUS_METERS) / 60.0;
    double hoodSurface = mainSurface * ShotCalculator.HOOD_SURFACE_SPEED_RATIO;
    return (mainSurface + hoodSurface) / 2.0;
  }

  /**
   * Two-stage forward model: ball exits motivator at k_m * v_motivator, then the launcher closes
   * k_L of the gap between that speed and the roller surface speed.
   *
   * <pre>
   *   v_m = k_m * v_motivator_surface           (stage 1: motivator)
   *   v_exit = v_m + k_L * (v_roller - v_m)     (stage 2: launcher)
   * </pre>
   *
   * <p>When rollers are faster than motivator exit speed → launcher speeds ball up. When motivator
   * exit speed is faster → launcher slows ball down. The interaction naturally varies with
   * distance.
   */
  @Override
  public double estimateExitVelocity(double launcherRPM, double distanceM) {
    double kM = motivatorCoupling.get();
    double kL = launcherCoupling.get();
    double vMotivator = motivatorSurfaceSpeed();
    double vRoller = rollerAvgSurfaceSpeed(launcherRPM);

    double vM = kM * vMotivator; // ball speed after motivator
    return vM + kL * (vRoller - vM); // launcher adjusts from there
  }

  /**
   * Reverse the two-stage model: given a target exit velocity, compute the required launcher RPM.
   *
   * <pre>
   *   v_exit = v_m + k_L * (v_roller - v_m)
   *   v_exit = v_m * (1 - k_L) + k_L * v_roller
   *   v_roller = (v_exit - v_m * (1 - k_L)) / k_L
   * </pre>
   */
  private double calculateRPMForVelocity(double targetExitVelocity) {
    double kM = motivatorCoupling.get();
    double kL = launcherCoupling.get();
    double vMotivator = motivatorSurfaceSpeed();

    double vM = kM * vMotivator; // ball speed after motivator

    // Guard: avoid division by zero if launcher coupling is 0
    if (kL <= 0.001) return minRPM.get();

    double rollerSpeedNeeded = (targetExitVelocity - vM * (1.0 - kL)) / kL;

    // Guard: if motivator alone provides enough velocity, floor at minRPM
    if (rollerSpeedNeeded <= 0) return minRPM.get();

    // Reverse the two-roller average: avg = main × (1 + hoodRatio) / 2
    double mainSurfaceVelocity =
        rollerSpeedNeeded * 2.0 / (1.0 + ShotCalculator.HOOD_SURFACE_SPEED_RATIO);
    return (mainSurfaceVelocity * 60.0) / (2.0 * Math.PI * ShotCalculator.MAIN_WHEEL_RADIUS_METERS);
  }

  /** Get hood angle offset (degrees) interpolated by distance. */
  private double getHoodAngleOffset(double distanceMeters) {
    double dClose = ZoneDetector.getZoneBoundaryClose();
    double dMid = ZoneDetector.getZoneBoundaryMid();
    double dFar = ZoneDetector.getZoneBoundaryFar();
    double dCorner = ZoneDetector.getZoneBoundaryCorner();
    double fClose = hoodAngleOffsetClose.get();
    double fMid = hoodAngleOffsetMid.get();
    double fFar = hoodAngleOffsetFar.get();
    double fCorner = hoodAngleOffsetCorner.get();

    if (distanceMeters <= dClose) return fClose;
    else if (distanceMeters <= dMid) {
      double t = (distanceMeters - dClose) / (dMid - dClose);
      return fClose + t * (fMid - fClose);
    } else if (distanceMeters <= dFar) {
      double t = (distanceMeters - dMid) / (dFar - dMid);
      return fMid + t * (fFar - fMid);
    } else if (distanceMeters <= dCorner) {
      double t = (distanceMeters - dFar) / (dCorner - dFar);
      return fFar + t * (fCorner - fFar);
    } else return fCorner;
  }

  // ========== Shot Calculation ==========

  @Override
  public ShotCalculator.ShotResult calculateShot(double distanceM) {
    // Convert tunable inches to meters
    double peakHeightM = peakHeightIn.get() * INCHES_TO_METERS;
    double passThroughHeightM = passThroughHeightIn.get() * INCHES_TO_METERS;
    double horizontalOffsetM = horizontalOffsetIn.get() * INCHES_TO_METERS;
    double h0 = config.getTurretHeightMeters();

    // Pass-through distance
    double x_p = distanceM - horizontalOffsetM;

    // Solve the parabola (same solver as FixedHeight)
    ShotCalculator.FixedHeightResult result =
        ShotCalculator.solveFixedHeightParabola(
            h0, peakHeightM, passThroughHeightM, x_p, HOOD_MIN_DEG, HOOD_MAX_DEG);

    if (!result.achievable()) {
      logFailure("%s at D=%.2fm", result.failureReason(), distanceM);
      return new ShotCalculator.ShotResult(
          minRPM.get(), HOOD_MAX_DEG, motivatorOutputRPM.get(), spindexerOutputRPM.get());
    }

    double thetaDeg = result.launchAngleDeg();
    double hoodAngleDeg = 90.0 - thetaDeg;
    double velocity = result.exitVelocityMps();

    // Apply hood angle offset (distance-interpolated)
    double hoodOffset = getHoodAngleOffset(distanceM);
    hoodAngleDeg += hoodOffset;

    // Convert exit velocity to RPM using two-stage model (the key difference from FixedHeight)
    double rpm = calculateRPMForVelocity(velocity);

    // Compute the model's components for diagnostic logging
    double kM = motivatorCoupling.get();
    double kL = launcherCoupling.get();
    double vMotivator = motivatorSurfaceSpeed();
    double rollerAvg = rollerAvgSurfaceSpeed(rpm);
    double vM = kM * vMotivator; // ball speed after motivator stage
    double modelExitVelocity = vM + kL * (rollerAvg - vM); // two-stage result
    // Effective efficiency: what single efficiency would produce this exit velocity?
    double effectiveEfficiency = rollerAvg > 0 ? modelExitVelocity / rollerAvg : 0;

    // Log diagnostics
    Logger.recordOutput("Shots/TwoStage/DistanceM", distanceM);
    Logger.recordOutput("Shots/TwoStage/DistanceIn", distanceM / INCHES_TO_METERS);
    Logger.recordOutput("Shots/TwoStage/MotivatorCoupling_kM", kM);
    Logger.recordOutput("Shots/TwoStage/LauncherCoupling_kL", kL);
    Logger.recordOutput("Shots/TwoStage/MotivatorSurfaceMps", vMotivator);
    Logger.recordOutput("Shots/TwoStage/MotivatorExitMps", vM);
    Logger.recordOutput("Shots/TwoStage/RollerAvgMps", rollerAvg);
    Logger.recordOutput("Shots/TwoStage/SpeedGapMps", rollerAvg - vM);
    Logger.recordOutput("Shots/TwoStage/ModelExitVelocityMps", modelExitVelocity);
    Logger.recordOutput("Shots/TwoStage/SolverExitVelocityMps", velocity);
    Logger.recordOutput("Shots/TwoStage/EffectiveEfficiency", effectiveEfficiency);
    Logger.recordOutput("Shots/TwoStage/RPM", rpm);
    Logger.recordOutput("Shots/TwoStage/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("Shots/TwoStage/HoodAngleOffsetDeg", hoodOffset);
    Logger.recordOutput("Shots/TwoStage/LaunchAngleDeg", 90.0 - hoodAngleDeg);
    Logger.recordOutput("Shots/TwoStage/Clamped", result.clamped());

    // Clamp RPM to limits (always return valid commands)
    if (rpm < minRPM.get()) {
      logStatus("RPM %.0f below min %.0f at D=%.2fm, clamped", rpm, minRPM.get(), distanceM);
      rpm = minRPM.get();
    } else if (rpm > maxRPM.get()) {
      logStatus("RPM %.0f above max %.0f at D=%.2fm, clamped", rpm, maxRPM.get(), distanceM);
      rpm = maxRPM.get();
    } else {
      if (result.clamped()) {
        Logger.recordOutput(
            "Shots/TwoStage/Status",
            String.format(
                "CLAMPED: hood at %s %.0f°, peak %s to %.0fin (tuned %.0fin) at D=%.2fm",
                result.clampedLow() ? "min" : "max",
                hoodAngleDeg,
                result.clampedLow() ? "raised" : "lowered",
                result.actualPeakHeightM() / INCHES_TO_METERS,
                peakHeightIn.get(),
                distanceM));
      } else {
        Logger.recordOutput("Shots/TwoStage/Status", "OK");
      }
    }
    Logger.recordOutput("Shots/TwoStage/Achievable", true);

    return new ShotCalculator.ShotResult(
        rpm, hoodAngleDeg, motivatorOutputRPM.get(), spindexerOutputRPM.get());
  }

  private static void logFailure(String format, Object... args) {
    String msg = "FAIL: " + String.format(format, args);
    Logger.recordOutput("Shots/TwoStage/Status", msg);
    Logger.recordOutput("Shots/TwoStage/Achievable", false);
  }

  private static void logStatus(String format, Object... args) {
    Logger.recordOutput("Shots/TwoStage/Status", String.format(format, args));
  }

  @Override
  public String getName() {
    return "TwoStage";
  }
}
