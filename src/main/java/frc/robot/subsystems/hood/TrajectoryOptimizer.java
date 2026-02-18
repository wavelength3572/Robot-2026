package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.turret.TurretCalculator;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Trajectory optimizer for hybrid RPM+hood control. Calculates the optimal combination of launcher
 * RPM and hood angle to achieve a valid arc trajectory.
 *
 * <p>The trajectory is defined by:
 *
 * <ol>
 *   <li>Descent angle (tunable) - angle of line from hub edge entry to hub center, parallel to hub
 *       wall
 *   <li>This determines the required height at hub edge
 *   <li>Clearance must be within min/max bounds (safety constraints)
 * </ol>
 *
 * <p>Given valid geometry, there is exactly ONE trajectory (launch angle + RPM) that passes through
 * both points with the specified descent angle.
 */
public class TrajectoryOptimizer {
  private static final double GRAVITY = 9.81; // m/s^2

  // Hub geometry (from FuelSim and welded field measurements)
  private static final double HUB_LIP_HEIGHT = 1.83; // meters - top of hub lip (ENTRY_HEIGHT)
  private static final double HUB_CENTER_HEIGHT = 1.43; // meters - where ball should land
  // Verified against welded field: 20.865 inches radius
  private static final double HUB_ENTRY_RADIUS = 0.530; // meters - 20.865 inches

  // PRIMARY TUNABLE: Descent angle (angle of line from hub edge to hub center)
  // Tune this to match the hub wall angle visually (60° matches well)
  private static final LoggedTunableNumber descentAngleDeg =
      new LoggedTunableNumber("Tuning/Trajectory/DescentAngleDeg", 60.0);

  // Minimum descent angle for fallback. When the preferred angle requires a hood position
  // below the mechanical limit (too close to hub), the optimizer steps down in 1° increments
  // until it finds an achievable shot or hits this floor.
  private static final LoggedTunableNumber minDescentAngleDeg =
      new LoggedTunableNumber("Tuning/Trajectory/MinDescentAngleDeg", 40.0);

  // Clearance constraints (inches above the lip)
  private static final LoggedTunableNumber minClearanceInches =
      new LoggedTunableNumber("Tuning/Trajectory/MinClearanceInches", 2.0); // Safety margin
  private static final LoggedTunableNumber maxClearanceInches =
      new LoggedTunableNumber("Tuning/Trajectory/MaxClearanceInches", 24.0); // Sanity check

  // RPM limits
  private static final LoggedTunableNumber minRPM =
      new LoggedTunableNumber("Tuning/Trajectory/MinRPM", 1500.0);
  private static final LoggedTunableNumber maxRPM =
      new LoggedTunableNumber("Tuning/Trajectory/MaxRPM", 4000.0);
  private static final LoggedTunableNumber maxPeakHeightFt =
      new LoggedTunableNumber("Tuning/Trajectory/MaxPeakHeightFt", 13.0);

  // Hood angle limits (mechanical position). The ball's physics launch angle = 90 - hoodAngle.
  private static final LoggedTunableNumber hoodMinAngleDeg =
      new LoggedTunableNumber("Tuning/Trajectory/HoodAngleMinDeg", 16.0);
  private static final LoggedTunableNumber hoodMaxAngleDeg =
      new LoggedTunableNumber("Tuning/Trajectory/HoodAngleMaxDeg", 46.0);

  /** Result of trajectory optimization. */
  public static class OptimalShot {
    public final double rpm;
    public final double hoodAngleDeg; // Mechanical hood position to command
    public final double launchAngleDeg; // Physics angle from horizontal (for sim/visualization)
    public final double exitVelocityMps;
    public final double peakHeightM;
    public final double descentAngleDeg;
    public final boolean achievable;
    public final String notes;

    public OptimalShot(
        double rpm,
        double launchAngleDeg,
        double hoodAngleDeg,
        double exitVelocityMps,
        double peakHeightM,
        double descentAngleDeg,
        boolean achievable,
        String notes) {
      this.rpm = rpm;
      this.launchAngleDeg = launchAngleDeg;
      this.hoodAngleDeg = hoodAngleDeg;
      this.exitVelocityMps = exitVelocityMps;
      this.peakHeightM = peakHeightM;
      this.descentAngleDeg = descentAngleDeg;
      this.achievable = achievable;
      this.notes = notes;
    }
  }

  /**
   * Calculate the optimal RPM and hood angle for a shot.
   *
   * <p>Tries the preferred descent angle first for the best hub entry. If that fails due to hood
   * angle limits (too close to hub), steps down in 1° increments until an achievable shot is found
   * or the minimum descent angle is reached.
   *
   * <p>The descent angle determines the required height at the hub edge. The trajectory must pass
   * through:
   *
   * <ul>
   *   <li>Point A (hub edge): height = hub_center + R * tan(descent_angle)
   *   <li>Point B (hub center): height = 1.43m
   * </ul>
   *
   * <p>The resulting clearance (height above lip) must be within min/max bounds.
   */
  public static OptimalShot calculateOptimalShot(
      Translation3d turretPosition, Translation3d target) {
    // Calculate geometry (shared across all descent angle attempts)
    double dx = target.getX() - turretPosition.getX();
    double dy = target.getY() - turretPosition.getY();
    double D = Math.sqrt(dx * dx + dy * dy); // Horizontal distance to hub center
    double turretHeightM = turretPosition.getZ();

    double R = HUB_ENTRY_RADIUS;
    double D_edge = D - R; // Distance to hub edge

    // Log distance info (constant across attempts)
    Logger.recordOutput("Match/Trajectory/Input/HorizontalDistanceM", D);
    Logger.recordOutput("Match/Trajectory/Input/TurretHeightM", turretHeightM);
    Logger.recordOutput("Match/Trajectory/Input/HubEdgeDistanceM", D_edge);

    // Try preferred descent angle first, then step down if hood limits prevent the shot
    double preferredDescent = descentAngleDeg.get();
    double minDescent = minDescentAngleDeg.get();
    OptimalShot lastFailure = null;

    for (double descent = preferredDescent; descent >= minDescent; descent -= 1.0) {
      OptimalShot shot = tryDescentAngle(descent, D, D_edge, turretHeightM);

      if (shot.achievable) {
        // Log the descent angle actually used (may differ from preferred)
        Logger.recordOutput("Match/Trajectory/Input/DescentAngleDeg", descent);
        Logger.recordOutput("Match/Trajectory/Input/PreferredDescentDeg", preferredDescent);
        Logger.recordOutput("Match/Trajectory/Input/DescentWasReduced", descent < preferredDescent);
        logResult(shot);
        return shot;
      }

      lastFailure = shot;
    }

    // No descent angle worked — return the last failure
    Logger.recordOutput("Match/Trajectory/Input/DescentAngleDeg", minDescent);
    Logger.recordOutput("Match/Trajectory/Input/PreferredDescentDeg", preferredDescent);
    Logger.recordOutput("Match/Trajectory/Input/DescentWasReduced", true);
    logResult(lastFailure);
    return lastFailure;
  }

  /**
   * Try a single descent angle and return the resulting shot (achievable or not).
   *
   * @param descent Descent angle in degrees
   * @param D Horizontal distance to hub center
   * @param D_edge Horizontal distance to hub edge
   * @param turretHeightM Turret height in meters
   * @return OptimalShot result (check achievable flag)
   */
  private static OptimalShot tryDescentAngle(
      double descent, double D, double D_edge, double turretHeightM) {
    double descentRad = Math.toRadians(descent);
    double heightDrop = HUB_ENTRY_RADIUS * Math.tan(descentRad);
    double heightAtEdge = HUB_CENTER_HEIGHT + heightDrop;

    // Calculate clearance and check bounds
    double clearanceM = heightAtEdge - HUB_LIP_HEIGHT;
    double clearanceInchesComputed = clearanceM / 0.0254;

    Logger.recordOutput("Match/Trajectory/Input/HeightAtEdgeM", heightAtEdge);
    Logger.recordOutput("Match/Trajectory/Input/ClearanceInches", clearanceInchesComputed);

    if (clearanceInchesComputed < minClearanceInches.get()) {
      return new OptimalShot(
          0,
          0,
          0,
          0,
          0,
          descent,
          false,
          String.format(
              "Clearance %.1f in < min %.1f in - increase descent angle",
              clearanceInchesComputed, minClearanceInches.get()));
    }
    if (clearanceInchesComputed > maxClearanceInches.get()) {
      return new OptimalShot(
          0,
          0,
          0,
          0,
          0,
          descent,
          false,
          String.format(
              "Clearance %.1f in > max %.1f in - decrease descent angle",
              clearanceInchesComputed, maxClearanceInches.get()));
    }

    // Heights relative to turret (for trajectory math)
    double H_edge = heightAtEdge - turretHeightM;
    double H_target = HUB_CENTER_HEIGHT - turretHeightM;

    return calculateTrajectoryThroughTwoPoints(D_edge, H_edge, D, H_target, turretHeightM);
  }

  /**
   * Calculate the unique trajectory that passes through two specified points.
   *
   * <p>Given two points (x1, y1) and (x2, y2) on a parabolic trajectory, we can solve for the
   * launch angle theta and velocity v.
   *
   * <p>Projectile motion: y = x*tan(theta) - g*x^2 / (2*v^2*cos^2(theta))
   *
   * <p>Let K = g / (2*v^2*cos^2(theta)), then: y = x*tan(theta) - K*x^2
   *
   * <p>From two points: y1 = x1*tan(theta) - K*x1^2 y2 = x2*tan(theta) - K*x2^2
   *
   * <p>Solving: tan(theta) = (y1*x2^2 - y2*x1^2) / (x1*x2^2 - x2*x1^2) = (y1*x2^2 - y2*x1^2) /
   * (x1*x2*(x2 - x1))
   */
  private static OptimalShot calculateTrajectoryThroughTwoPoints(
      double x1, double y1, double x2, double y2, double turretHeightM) {

    // Compute descent angle for this trajectory (angle of line from point A to point B)
    double heightAtEdge = turretHeightM + y1;
    double heightDrop = heightAtEdge - HUB_CENTER_HEIGHT;
    double descentAngleDeg = Math.toDegrees(Math.atan(heightDrop / HUB_ENTRY_RADIUS));

    // Solve for launch angle
    double denominator = x1 * x2 * (x2 - x1);
    if (Math.abs(denominator) < 0.001) {
      return new OptimalShot(
          0, 0, 0, 0, 0, descentAngleDeg, false, "Geometry error: points too close");
    }

    double tanTheta = (y1 * x2 * x2 - y2 * x1 * x1) / denominator;
    double theta = Math.atan(tanTheta);
    double thetaDeg = Math.toDegrees(theta); // physics launch angle from horizontal
    double hoodAngleDeg = 90.0 - thetaDeg; // convert to hood mechanical angle

    // Check against hood mechanical limits
    if (hoodAngleDeg < hoodMinAngleDeg.get() || hoodAngleDeg > hoodMaxAngleDeg.get()) {
      return new OptimalShot(
          0,
          thetaDeg,
          hoodAngleDeg,
          0,
          0,
          descentAngleDeg,
          false,
          String.format(
              "Hood angle %.1f deg outside range [%.0f-%.0f]",
              hoodAngleDeg, hoodMinAngleDeg.get(), hoodMaxAngleDeg.get()));
    }

    // Solve for K = g / (2*v^2*cos^2(theta)) using first point
    double K = (x1 * tanTheta - y1) / (x1 * x1);
    if (K <= 0) {
      return new OptimalShot(
          0,
          thetaDeg,
          hoodAngleDeg,
          0,
          0,
          descentAngleDeg,
          false,
          "Invalid trajectory: K <= 0 (unreachable)");
    }

    // Calculate velocity: v^2 = g / (2*K*cos^2(theta))
    double cosTheta = Math.cos(theta);
    double vSquared = GRAVITY / (2 * K * cosTheta * cosTheta);
    if (vSquared <= 0) {
      return new OptimalShot(
          0, thetaDeg, hoodAngleDeg, 0, 0, descentAngleDeg, false, "Invalid velocity calculation");
    }
    double velocity = Math.sqrt(vSquared);

    // Convert to RPM
    double rpm = TurretCalculator.calculateRPMForVelocity(velocity);

    // Check RPM limits
    if (rpm < minRPM.get() || rpm > maxRPM.get()) {
      return new OptimalShot(
          rpm,
          thetaDeg,
          hoodAngleDeg,
          velocity,
          0,
          descentAngleDeg,
          false,
          String.format("RPM %.0f outside range [%.0f-%.0f]", rpm, minRPM.get(), maxRPM.get()));
    }

    // Calculate peak height
    double sinTheta = Math.sin(theta);
    double vy0 = velocity * sinTheta;
    double peakHeight = turretHeightM + (vy0 * vy0) / (2 * GRAVITY);

    // Check peak height limit
    double maxPeakHeightM = maxPeakHeightFt.get() * 0.3048;
    if (peakHeight > maxPeakHeightM) {
      return new OptimalShot(
          rpm,
          thetaDeg,
          hoodAngleDeg,
          velocity,
          peakHeight,
          descentAngleDeg,
          false,
          String.format(
              "Peak %.1fft exceeds max %.1fft", peakHeight / 0.3048, maxPeakHeightFt.get()));
    }

    // Check that peak is before hub edge (ball should be descending at point A)
    double vx = velocity * cosTheta;
    double timeToPeak = vy0 / GRAVITY;
    double distanceToPeak = vx * timeToPeak;
    if (distanceToPeak >= x1) {
      return new OptimalShot(
          rpm,
          thetaDeg,
          hoodAngleDeg,
          velocity,
          peakHeight,
          descentAngleDeg,
          false,
          String.format(
              "Peak at %.2fm, hub edge at %.2fm - ball still rising", distanceToPeak, x1));
    }

    // All constraints satisfied!
    double clearanceM = heightAtEdge - HUB_LIP_HEIGHT;
    return new OptimalShot(
        rpm,
        thetaDeg,
        hoodAngleDeg,
        velocity,
        peakHeight,
        descentAngleDeg,
        true,
        String.format(
            "OK - clearance %.1f in, descent %.1f deg", clearanceM / 0.0254, descentAngleDeg));
  }

  private static void logResult(OptimalShot shot) {
    Logger.recordOutput("Match/Trajectory/Ideal/RPM", shot.rpm);
    Logger.recordOutput("Match/Trajectory/Ideal/HoodAngleDeg", shot.hoodAngleDeg);
    Logger.recordOutput("Match/Trajectory/Ideal/LaunchAngleDeg", shot.launchAngleDeg);
    Logger.recordOutput("Match/Trajectory/Ideal/ExitVelocityMps", shot.exitVelocityMps);
    Logger.recordOutput("Match/Trajectory/Ideal/PeakHeightM", shot.peakHeightM);
    Logger.recordOutput("Match/Trajectory/Ideal/DescentAngleDeg", shot.descentAngleDeg);
    Logger.recordOutput("Match/Trajectory/Ideal/Achievable", shot.achievable);
    Logger.recordOutput("Match/Trajectory/Ideal/Notes", shot.notes);
  }

  /** Get the current descent angle setting in degrees. */
  public static double getDescentAngleDeg() {
    return descentAngleDeg.get();
  }

  /** Get the computed clearance for the current descent angle, in inches. */
  public static double getComputedClearanceInches() {
    double descentRad = Math.toRadians(descentAngleDeg.get());
    double heightDrop = HUB_ENTRY_RADIUS * Math.tan(descentRad);
    double heightAtEdge = HUB_CENTER_HEIGHT + heightDrop;
    double clearanceM = heightAtEdge - HUB_LIP_HEIGHT;
    return clearanceM / 0.0254;
  }
}
