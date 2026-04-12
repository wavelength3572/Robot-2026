package frc.robot.subsystems.shooting;

import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ZoneDetector;
import org.littletonrobotics.junction.Logger;

/**
 * Fixed-height parabola strategy for passes. Same vertex-form math as {@link
 * FixedHeightShotStrategy} but tuned for passing: the ball arcs to a fixed peak height and passes
 * through a target point at ground level, then continues on the parabola to land beyond it.
 *
 * <p>Graceful degradation matches the hub strategy: when the hood must clamp to mechanical limits,
 * velocity is re-derived to still hit the pass-through point. Peak height shifts but the pass still
 * clears the obstacle.
 */
public class FixedHeightPassStrategy implements ShotStrategy {

  private static final double INCHES_TO_METERS = 0.0254;
  private static final RobotConfig config = Constants.getRobotConfig();

  // Peak height for passes — lower than hub shots since we're clearing an obstacle, not scoring
  private static final LoggedTunableNumber peakHeightIn =
      new LoggedTunableNumber(
          "Shots/FixedHeightPass/PeakHeightIn", config.getFixedHeightPassPeakHeightIn());

  // Pass-through height (ground level for most passes)
  private static final LoggedTunableNumber passThroughHeightIn =
      new LoggedTunableNumber("Shots/FixedHeightPass/PassThroughHeightIn", 0.0);

  // RPM limits for passes
  private static final LoggedTunableNumber minRPM =
      new LoggedTunableNumber("Shots/FixedHeightPass/MinRPM", config.getFixedHeightPassMinRPM());

  private static final LoggedTunableNumber maxRPM =
      new LoggedTunableNumber("Shots/FixedHeightPass/MaxRPM", config.getFixedHeightPassMaxRPM());

  // Hood floor for passes — prevents near-vertical launches
  private static final LoggedTunableNumber hoodMinFloor =
      new LoggedTunableNumber(
          "Shots/FixedHeightPass/HoodMinDeg", config.getFixedHeightPassHoodMinDeg());

  // ========== Efficiency (distance-interpolated) ==========
  // Uses the same breakpoint structure as FixedHeightShotStrategy but with its own tunables.
  private static final LoggedTunableNumber efficiencyClose =
      new LoggedTunableNumber(
          "Shots/FixedHeightPass/Efficiency/Close", config.getShotEfficiencyClose());
  private static final LoggedTunableNumber efficiencyMid =
      new LoggedTunableNumber(
          "Shots/FixedHeightPass/Efficiency/Mid", config.getShotEfficiencyMid());
  private static final LoggedTunableNumber efficiencyFar =
      new LoggedTunableNumber(
          "Shots/FixedHeightPass/Efficiency/Far", config.getShotEfficiencyFar());
  private static final LoggedTunableNumber efficiencyCorner =
      new LoggedTunableNumber(
          "Shots/FixedHeightPass/Efficiency/Corner", config.getShotEfficiencyCorner());

  // ========== Motivator / Spindexer ==========
  private static final LoggedTunableNumber motivatorRPM =
      new LoggedTunableNumber(
          "Shots/FixedHeightPass/MotivatorRPM", config.getPassingMotivatorRPM());
  private static final LoggedTunableNumber spindexerRPM =
      new LoggedTunableNumber("Shots/FixedHeightPass/SpindexerRPM", config.getSpindexerPassRPM());

  // Hood limits from hardware config
  private static final double HOOD_MIN_DEG = config.getHoodMinAngleDegrees();
  private static final double HOOD_MAX_DEG = config.getHoodMaxAngleDegrees();

  // ========== Velocity Model ==========

  private double getEfficiency(double distanceMeters) {
    double dClose = ZoneDetector.getZoneBoundaryClose();
    double dMid = ZoneDetector.getZoneBoundaryMid();
    double dFar = ZoneDetector.getZoneBoundaryFar();
    double dCorner = ZoneDetector.getZoneBoundaryCorner();
    double eClose = efficiencyClose.get();
    double eMid = efficiencyMid.get();
    double eFar = efficiencyFar.get();
    double eCorner = efficiencyCorner.get();

    if (distanceMeters <= dClose) return eClose;
    else if (distanceMeters <= dMid) {
      double t = (distanceMeters - dClose) / (dMid - dClose);
      return eClose + t * (eMid - eClose);
    } else if (distanceMeters <= dFar) {
      double t = (distanceMeters - dMid) / (dFar - dMid);
      return eMid + t * (eFar - eMid);
    } else if (distanceMeters <= dCorner) {
      double t = (distanceMeters - dFar) / (dCorner - dFar);
      return eFar + t * (eCorner - eFar);
    } else return eCorner;
  }

  private double calculateRPMForVelocity(double targetExitVelocity, double distanceMeters) {
    double averageSurfaceVelocity = targetExitVelocity / getEfficiency(distanceMeters);
    double mainSurfaceVelocity =
        averageSurfaceVelocity * 2.0 / (1.0 + ShotCalculator.HOOD_SURFACE_SPEED_RATIO);
    return (mainSurfaceVelocity * 60.0) / (2.0 * Math.PI * ShotCalculator.MAIN_WHEEL_RADIUS_METERS);
  }

  @Override
  public double estimateExitVelocity(double launcherRPM, double distanceM) {
    double mainSurfaceVelocity =
        (launcherRPM * 2.0 * Math.PI * ShotCalculator.MAIN_WHEEL_RADIUS_METERS) / 60.0;
    double hoodSurfaceVelocity = mainSurfaceVelocity * ShotCalculator.HOOD_SURFACE_SPEED_RATIO;
    double averageSurfaceVelocity = (mainSurfaceVelocity + hoodSurfaceVelocity) / 2.0;
    return averageSurfaceVelocity * getEfficiency(distanceM);
  }

  // ========== Shot Calculation ==========

  @Override
  public ShotCalculator.ShotResult calculateShot(double distanceM) {
    return calculateShot(distanceM, peakHeightIn.get());
  }

  /**
   * Calculate pass shot with a specific peak height. Used by LONG_PASS to arc higher than normal
   * passes, keeping the hood angle within mechanical limits at longer distances.
   *
   * @param distanceM Horizontal distance to target in meters
   * @param peakHeightOverrideIn Peak height in inches
   * @return Shot result with the 4 mechanical commands
   */
  public ShotCalculator.ShotResult calculateShot(double distanceM, double peakHeightOverrideIn) {
    double peakHeightM = peakHeightOverrideIn * INCHES_TO_METERS;
    double passThroughHeightM = passThroughHeightIn.get() * INCHES_TO_METERS;
    double h0 = config.getTurretHeightMeters();

    // Apply pass-specific hood floor
    double effectiveHoodMin = Math.max(HOOD_MIN_DEG, hoodMinFloor.get());

    // Log inputs
    Logger.recordOutput("Shots/FixedHeightPass/DistanceM", distanceM);
    Logger.recordOutput("Shots/FixedHeightPass/DistanceIn", distanceM / INCHES_TO_METERS);
    Logger.recordOutput("Shots/FixedHeightPass/PeakHeightIn", peakHeightOverrideIn);
    Logger.recordOutput("Shots/FixedHeightPass/PassThroughHeightIn", passThroughHeightIn.get());

    // Solve the parabola — distance IS the pass-through distance (no horizontal offset for passes)
    ShotCalculator.FixedHeightResult result =
        ShotCalculator.solveFixedHeightParabola(
            h0, peakHeightM, passThroughHeightM, distanceM, effectiveHoodMin, HOOD_MAX_DEG);

    if (!result.achievable()) {
      logFailure("%s at D=%.2fm", result.failureReason(), distanceM);
      return new ShotCalculator.ShotResult(
          minRPM.get(), HOOD_MAX_DEG, motivatorRPM.get(), spindexerRPM.get());
    }

    double thetaDeg = result.launchAngleDeg();
    double hoodAngleDeg = 90.0 - thetaDeg;
    double velocity = result.exitVelocityMps();

    // Log trajectory values
    Logger.recordOutput("Shots/FixedHeightPass/VertexXM", result.vertexXM());
    Logger.recordOutput("Shots/FixedHeightPass/LaunchAngleDeg", thetaDeg);
    Logger.recordOutput("Shots/FixedHeightPass/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("Shots/FixedHeightPass/Clamped", result.clamped());
    if (result.clamped()) {
      Logger.recordOutput(
          "Shots/FixedHeightPass/ActualPeakHeightIn",
          result.actualPeakHeightM() / INCHES_TO_METERS);
    }

    // Convert to RPM
    double rpm = calculateRPMForVelocity(velocity, distanceM);

    Logger.recordOutput("Shots/FixedHeightPass/ExitVelocityMps", velocity);
    Logger.recordOutput("Shots/FixedHeightPass/RPM", rpm);

    // Clamp RPM (always return valid commands)
    if (rpm < minRPM.get()) {
      logStatus("RPM %.0f below min %.0f at D=%.2fm, clamped", rpm, minRPM.get(), distanceM);
      rpm = minRPM.get();
    } else if (rpm > maxRPM.get()) {
      logStatus("RPM %.0f above max %.0f at D=%.2fm, clamped", rpm, maxRPM.get(), distanceM);
      rpm = maxRPM.get();
    } else {
      if (result.clamped()) {
        Logger.recordOutput(
            "Shots/FixedHeightPass/Status",
            String.format(
                "CLAMPED: hood at %s %.0f°, peak %s to %.0fin (tuned %.0fin) at D=%.2fm",
                result.clampedLow() ? "min" : "max",
                hoodAngleDeg,
                result.clampedLow() ? "raised" : "lowered",
                result.actualPeakHeightM() / INCHES_TO_METERS,
                peakHeightOverrideIn,
                distanceM));
      } else {
        Logger.recordOutput("Shots/FixedHeightPass/Status", "OK");
      }
    }
    Logger.recordOutput("Shots/FixedHeightPass/Achievable", true);

    return new ShotCalculator.ShotResult(rpm, hoodAngleDeg, motivatorRPM.get(), spindexerRPM.get());
  }

  private static void logFailure(String format, Object... args) {
    String msg = "FAIL: " + String.format(format, args);
    Logger.recordOutput("Shots/FixedHeightPass/Status", msg);
    Logger.recordOutput("Shots/FixedHeightPass/Achievable", false);
  }

  private static void logStatus(String format, Object... args) {
    Logger.recordOutput("Shots/FixedHeightPass/Status", String.format(format, args));
  }

  @Override
  public String getName() {
    return "FixedHeightPass";
  }
}
