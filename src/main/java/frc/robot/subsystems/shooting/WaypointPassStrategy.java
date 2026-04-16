package frc.robot.subsystems.shooting;

import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ZoneDetector;
import org.littletonrobotics.junction.Logger;

/**
 * Waypoint-constrained pass strategy. Instead of arcing to a fixed peak height and landing at a
 * ground-level target, the ball must pass through a 3D waypoint above the bump/trench. The parabola
 * solver finds the trajectory that starts at turret height, peaks at the tuned peak height, and
 * passes through the waypoint at its specified height and horizontal distance.
 *
 * <p>This produces much flatter arcs than {@link FixedHeightPassStrategy} because the pass-through
 * point is already elevated (e.g. 48" above the bump), so the peak only needs to be slightly higher
 * than that.
 *
 * <p>Uses the same {@link ShotCalculator#solveFixedHeightParabola} solver — just feeds it the
 * waypoint height as the pass-through height instead of ground level.
 */
public class WaypointPassStrategy implements ShotStrategy {

  private static final double INCHES_TO_METERS = 0.0254;
  private static final RobotConfig config = Constants.getRobotConfig();

  // Peak height — only needs to be slightly above the waypoint height for a flat arc
  private static final LoggedTunableNumber peakHeightIn =
      new LoggedTunableNumber(
          "Shots/WaypointPass/PeakHeightIn", config.getWaypointPassPeakHeightIn());

  // RPM limits
  private static final LoggedTunableNumber minRPM =
      new LoggedTunableNumber("Shots/WaypointPass/MinRPM", config.getWaypointPassMinRPM());

  private static final LoggedTunableNumber maxRPM =
      new LoggedTunableNumber("Shots/WaypointPass/MaxRPM", config.getWaypointPassMaxRPM());

  // Hood floor — prevents near-vertical launches
  private static final LoggedTunableNumber hoodMinFloor =
      new LoggedTunableNumber("Shots/WaypointPass/HoodMinDeg", config.getWaypointPassHoodMinDeg());

  // ========== Efficiency (distance-interpolated) ==========
  private static final LoggedTunableNumber efficiencyClose =
      new LoggedTunableNumber(
          "Shots/WaypointPass/Efficiency/Close", config.getShotEfficiencyClose());
  private static final LoggedTunableNumber efficiencyMid =
      new LoggedTunableNumber("Shots/WaypointPass/Efficiency/Mid", config.getShotEfficiencyMid());
  private static final LoggedTunableNumber efficiencyFar =
      new LoggedTunableNumber("Shots/WaypointPass/Efficiency/Far", config.getShotEfficiencyFar());
  private static final LoggedTunableNumber efficiencyCorner =
      new LoggedTunableNumber(
          "Shots/WaypointPass/Efficiency/Corner", config.getShotEfficiencyCorner());

  // ========== Motivator / Spindexer ==========
  private static final LoggedTunableNumber motivatorRPM =
      new LoggedTunableNumber("Shots/WaypointPass/MotivatorRPM", config.getPassingMotivatorRPM());
  private static final LoggedTunableNumber spindexerRPM =
      new LoggedTunableNumber("Shots/WaypointPass/SpindexerRPM", config.getSpindexerPassRPM());

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

  /**
   * Default calculateShot — not typically used for waypoint passes since the coordinator provides
   * the waypoint distance and height. Falls back to using the peak height as a basic pass.
   */
  @Override
  public ShotCalculator.ShotResult calculateShot(double distanceM) {
    // Fallback: treat distance as waypoint distance with ground-level pass-through.
    // The coordinator should call the overloaded version with waypoint height.
    return calculateShot(distanceM, 0.0, peakHeightIn.get());
  }

  /**
   * Calculate a pass that arcs through a waypoint at a specific height and horizontal distance.
   *
   * <p>The parabola is constrained to:
   *
   * <ol>
   *   <li>Start at turret height
   *   <li>Peak at the tuned peak height
   *   <li>Pass through the waypoint at (waypointDistM, waypointHeightM)
   * </ol>
   *
   * <p>After the waypoint, the ball continues on the parabola and descends naturally.
   *
   * @param waypointDistM Horizontal distance from turret to the waypoint in meters
   * @param waypointHeightM Height of the waypoint above ground in meters
   * @param peakHeightOverrideIn Peak height in inches (overrides tunable)
   * @return Shot result with the 4 mechanical commands
   */
  public ShotCalculator.ShotResult calculateShot(
      double waypointDistM, double waypointHeightM, double peakHeightOverrideIn) {
    double peakHeightM = peakHeightOverrideIn * INCHES_TO_METERS;
    double h0 = config.getTurretHeightMeters();

    // Apply pass-specific hood floor
    double effectiveHoodMin = Math.max(HOOD_MIN_DEG, hoodMinFloor.get());

    // Log inputs
    Logger.recordOutput("Shots/WaypointPass/WaypointDistM", waypointDistM);
    Logger.recordOutput("Shots/WaypointPass/WaypointDistIn", waypointDistM / INCHES_TO_METERS);
    Logger.recordOutput("Shots/WaypointPass/WaypointHeightIn", waypointHeightM / INCHES_TO_METERS);
    Logger.recordOutput("Shots/WaypointPass/PeakHeightIn", peakHeightOverrideIn);

    // Solve the parabola — waypoint IS the pass-through point
    ShotCalculator.FixedHeightResult result =
        ShotCalculator.solveFixedHeightParabola(
            h0, peakHeightM, waypointHeightM, waypointDistM, effectiveHoodMin, HOOD_MAX_DEG);

    if (!result.achievable()) {
      logFailure(
          "%s at WP=%.2fm, WPH=%.0fin",
          result.failureReason(), waypointDistM, waypointHeightM / INCHES_TO_METERS);
      return new ShotCalculator.ShotResult(
          minRPM.get(), HOOD_MAX_DEG, motivatorRPM.get(), spindexerRPM.get());
    }

    double thetaDeg = result.launchAngleDeg();
    double hoodAngleDeg = 90.0 - thetaDeg;
    double velocity = result.exitVelocityMps();

    // Log trajectory values
    Logger.recordOutput("Shots/WaypointPass/VertexXM", result.vertexXM());
    Logger.recordOutput("Shots/WaypointPass/LaunchAngleDeg", thetaDeg);
    Logger.recordOutput("Shots/WaypointPass/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("Shots/WaypointPass/Clamped", result.clamped());
    if (result.clamped()) {
      Logger.recordOutput(
          "Shots/WaypointPass/ActualPeakHeightIn", result.actualPeakHeightM() / INCHES_TO_METERS);
    }

    // Convert to RPM (use waypoint distance for efficiency lookup)
    double rpm = calculateRPMForVelocity(velocity, waypointDistM);

    Logger.recordOutput("Shots/WaypointPass/ExitVelocityMps", velocity);
    Logger.recordOutput("Shots/WaypointPass/RPM", rpm);

    // Clamp RPM (always return valid commands)
    if (rpm < minRPM.get()) {
      logStatus("RPM %.0f below min %.0f at WP=%.2fm, clamped", rpm, minRPM.get(), waypointDistM);
      rpm = minRPM.get();
    } else if (rpm > maxRPM.get()) {
      logStatus("RPM %.0f above max %.0f at WP=%.2fm, clamped", rpm, maxRPM.get(), waypointDistM);
      rpm = maxRPM.get();
    } else {
      if (result.clamped()) {
        Logger.recordOutput(
            "Shots/WaypointPass/Status",
            String.format(
                "CLAMPED: hood at %s %.0f\u00b0, peak %s to %.0fin (tuned %.0fin) at WP=%.2fm",
                result.clampedLow() ? "min" : "max",
                hoodAngleDeg,
                result.clampedLow() ? "raised" : "lowered",
                result.actualPeakHeightM() / INCHES_TO_METERS,
                peakHeightOverrideIn,
                waypointDistM));
      } else {
        Logger.recordOutput("Shots/WaypointPass/Status", "OK");
      }
    }
    Logger.recordOutput("Shots/WaypointPass/Achievable", true);

    return new ShotCalculator.ShotResult(rpm, hoodAngleDeg, motivatorRPM.get(), spindexerRPM.get());
  }

  private static void logFailure(String format, Object... args) {
    String msg = "FAIL: " + String.format(format, args);
    Logger.recordOutput("Shots/WaypointPass/Status", msg);
    Logger.recordOutput("Shots/WaypointPass/Achievable", false);
  }

  private static void logStatus(String format, Object... args) {
    Logger.recordOutput("Shots/WaypointPass/Status", String.format(format, args));
  }

  @Override
  public String getName() {
    return "WaypointPass";
  }
}
