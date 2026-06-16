package frc.robot.subsystems.shooting;

import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ZoneDetector;
import org.littletonrobotics.junction.Logger;

/**
 * Fixed-height parabola shot strategy for hub shots. Defines a unique trajectory using three
 * constraints:
 *
 * <ol>
 *   <li>Launch point (turret position)
 *   <li>Fixed peak height (same for every shot regardless of distance)
 *   <li>Pass-through point on the descent, at a fixed height and horizontal offset from hub center
 * </ol>
 *
 * <p>Delegates core parabola math to {@link ShotCalculator#solveFixedHeightParabola}. When the hood
 * must be clamped to mechanical limits, the solver re-derives velocity so the shot still hits the
 * pass-through point — peak height shifts but the ball still lands.
 *
 * <p>This strategy owns its efficiency and hood angle offset tunables. Different strategies may use
 * different velocity models with different tuning.
 */
public class FixedHeightShotStrategy implements ShotStrategy {

  private static final double INCHES_TO_METERS = 0.0254;
  private static final RobotConfig config = Constants.getRobotConfig();

  // All spatial tunables in inches
  private static final LoggedTunableNumber peakHeightIn =
      new LoggedTunableNumber(
          "Shots/FixedHeight/PeakHeightIn", config.getFixedHeightPeakHeightIn());

  private static final LoggedTunableNumber passThroughHeightIn =
      new LoggedTunableNumber(
          "Shots/FixedHeight/PassThroughHeightIn", config.getFixedHeightPassThroughHeightIn());

  private static final LoggedTunableNumber horizontalOffsetIn =
      new LoggedTunableNumber(
          "Shots/FixedHeight/HorizontalOffsetIn", config.getFixedHeightHorizontalOffsetIn());

  // RPM limits
  private static final LoggedTunableNumber minRPM =
      new LoggedTunableNumber("Shots/FixedHeight/MinRPM", config.getFixedHeightMinRPM());

  private static final LoggedTunableNumber maxRPM =
      new LoggedTunableNumber("Shots/FixedHeight/MaxRPM", config.getFixedHeightMaxRPM());

  // ========== Efficiency (distance-interpolated) ==========
  // Mechanical roller-to-ball transfer efficiency. Higher RPM (longer range) tends to have
  // more ball compression and slip, reducing effective efficiency.
  private static final LoggedTunableNumber efficiencyClose =
      new LoggedTunableNumber(
          "Shots/FixedHeight/Efficiency/Close", config.getShotEfficiencyClose());
  private static final LoggedTunableNumber efficiencyMid =
      new LoggedTunableNumber("Shots/FixedHeight/Efficiency/Mid", config.getShotEfficiencyMid());
  private static final LoggedTunableNumber efficiencyFar =
      new LoggedTunableNumber("Shots/FixedHeight/Efficiency/Far", config.getShotEfficiencyFar());
  private static final LoggedTunableNumber efficiencyCorner =
      new LoggedTunableNumber(
          "Shots/FixedHeight/Efficiency/Corner", config.getShotEfficiencyCorner());

  // ========== Hood Angle Offset (distance-interpolated, degrees) ==========
  // Additive offset applied after the parabola solver. Positive = flatter, negative = steeper.
  private static final LoggedTunableNumber hoodAngleOffsetClose =
      new LoggedTunableNumber(
          "Shots/FixedHeight/HoodAngleOffset/Close", config.getShotHoodAngleFudgeClose());
  private static final LoggedTunableNumber hoodAngleOffsetMid =
      new LoggedTunableNumber(
          "Shots/FixedHeight/HoodAngleOffset/Mid", config.getShotHoodAngleFudgeMid());
  private static final LoggedTunableNumber hoodAngleOffsetFar =
      new LoggedTunableNumber(
          "Shots/FixedHeight/HoodAngleOffset/Far", config.getShotHoodAngleFudgeFar());
  private static final LoggedTunableNumber hoodAngleOffsetCorner =
      new LoggedTunableNumber(
          "Shots/FixedHeight/HoodAngleOffset/Corner", config.getShotHoodAngleFudgeCorner());

  // ========== Motivator / Spindexer ==========
  private static final LoggedTunableNumber motivatorRPM =
      new LoggedTunableNumber("Shots/FixedHeight/MotivatorRPM", config.getShootingMotivatorRPM());
  private static final LoggedTunableNumber spindexerRPM =
      new LoggedTunableNumber("Shots/FixedHeight/SpindexerRPM", config.getSpindexerCloseRPM());

  // Hood limits from hardware config
  private static final double HOOD_MIN_DEG = config.getHoodMinAngleDegrees();
  private static final double HOOD_MAX_DEG = config.getHoodMaxAngleDegrees();

  // ========== Velocity Model ==========

  /**
   * Get roller-to-ball efficiency interpolated by distance. Uses zone boundaries as breakpoints.
   */
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

  /** Calculate RPM needed for a target exit velocity at a given distance. */
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
    // Convert tunable inches to meters
    double peakHeightM = peakHeightIn.get() * INCHES_TO_METERS;
    double passThroughHeightM = passThroughHeightIn.get() * INCHES_TO_METERS;
    double horizontalOffsetM = horizontalOffsetIn.get() * INCHES_TO_METERS;
    double h0 = config.getTurretHeightMeters();

    // Pass-through distance
    double x_p = distanceM - horizontalOffsetM;

    // Log inputs
    Logger.recordOutput("Shots/FixedHeight/DistanceM", distanceM);
    Logger.recordOutput("Shots/FixedHeight/DistanceIn", distanceM / INCHES_TO_METERS);
    Logger.recordOutput("Shots/FixedHeight/TurretHeightIn", h0 / INCHES_TO_METERS);
    Logger.recordOutput("Shots/FixedHeight/PeakHeightIn", peakHeightIn.get());
    Logger.recordOutput("Shots/FixedHeight/PassThroughHeightIn", passThroughHeightIn.get());
    Logger.recordOutput("Shots/FixedHeight/PassThroughDistM", x_p);

    // Solve the parabola
    ShotCalculator.FixedHeightResult result =
        ShotCalculator.solveFixedHeightParabola(
            h0, peakHeightM, passThroughHeightM, x_p, HOOD_MIN_DEG, HOOD_MAX_DEG);

    if (!result.achievable()) {
      logFailure("%s at D=%.2fm", result.failureReason(), distanceM);
      // Return clamped safe defaults
      return new ShotCalculator.ShotResult(
          minRPM.get(), HOOD_MAX_DEG, motivatorRPM.get(), spindexerRPM.get());
    }

    double thetaDeg = result.launchAngleDeg();
    double hoodAngleDeg = 90.0 - thetaDeg;
    double velocity = result.exitVelocityMps();

    // Apply hood angle offset (distance-interpolated)
    double hoodOffset = getHoodAngleOffset(distanceM);
    hoodAngleDeg += hoodOffset;

    // Log trajectory values
    Logger.recordOutput("Shots/FixedHeight/VertexXM", result.vertexXM());
    Logger.recordOutput("Shots/FixedHeight/LaunchAngleDeg", 90.0 - hoodAngleDeg);
    Logger.recordOutput("Shots/FixedHeight/HoodAngleOffsetDeg", hoodOffset);
    Logger.recordOutput("Shots/FixedHeight/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("Shots/FixedHeight/Clamped", result.clamped());
    if (result.clamped()) {
      Logger.recordOutput(
          "Shots/FixedHeight/ActualPeakHeightIn", result.actualPeakHeightM() / INCHES_TO_METERS);
    }

    // Convert to RPM
    double rpm = calculateRPMForVelocity(velocity, distanceM);

    Logger.recordOutput("Shots/FixedHeight/ExitVelocityMps", velocity);
    Logger.recordOutput("Shots/FixedHeight/RPM", rpm);
    Logger.recordOutput("Shots/FixedHeight/Efficiency", getEfficiency(distanceM));

    // Clamp RPM to limits (always return valid commands)
    if (rpm < minRPM.get()) {
      logStatus("RPM %.0f below min %.0f at D=%.2fm, clamped", rpm, minRPM.get(), distanceM);
      rpm = minRPM.get();
    } else if (rpm > maxRPM.get()) {
      logStatus("RPM %.0f above max %.0f at D=%.2fm, clamped", rpm, maxRPM.get(), distanceM);
      rpm = maxRPM.get();
    } else {
      // Status logging
      if (result.clamped()) {
        Logger.recordOutput(
            "Shots/FixedHeight/Status",
            String.format(
                "CLAMPED: hood at %s %.0f°, peak %s to %.0fin (tuned %.0fin) at D=%.2fm",
                result.clampedLow() ? "min" : "max",
                hoodAngleDeg,
                result.clampedLow() ? "raised" : "lowered",
                result.actualPeakHeightM() / INCHES_TO_METERS,
                peakHeightIn.get(),
                distanceM));
      } else {
        Logger.recordOutput("Shots/FixedHeight/Status", "OK");
      }
    }
    Logger.recordOutput("Shots/FixedHeight/Achievable", true);

    return new ShotCalculator.ShotResult(rpm, hoodAngleDeg, motivatorRPM.get(), spindexerRPM.get());
  }

  private static void logFailure(String format, Object... args) {
    String msg = "FAIL: " + String.format(format, args);
    Logger.recordOutput("Shots/FixedHeight/Status", msg);
    Logger.recordOutput("Shots/FixedHeight/Achievable", false);
  }

  private static void logStatus(String format, Object... args) {
    Logger.recordOutput("Shots/FixedHeight/Status", String.format(format, args));
  }

  @Override
  public String getName() {
    return "FixedHeight";
  }
}
