package frc.robot.subsystems.shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Per-actuator dashboard overrides and launcher RPM trim.
 *
 * <p>Each actuator (launcher, hood, motivator, spindexer) has an independent dashboard toggle. When
 * a toggle is on, its tunable value replaces the strategy-calculated value. Any combination of 0–4
 * overrides can be active simultaneously.
 *
 * <p>The launcher trim is an additive offset applied on top of both override and calculated values.
 * It lets drivers bump RPM up/down during a match without redeploying.
 */
public final class ShotOverrides {

  // Dashboard keys for the on/off toggles
  private static final String KEY_LAUNCHER = "Overrides/Launcher";
  private static final String KEY_HOOD = "Overrides/Hood";
  private static final String KEY_MOTIVATOR = "Overrides/Motivator";
  private static final String KEY_SPINDEXER = "Overrides/Spindexer";

  // Override target values — replaces strategy output when the matching toggle is on
  private static final LoggedTunableNumber overrideLauncherRPM =
      new LoggedTunableNumber(
          "Overrides/LauncherRPM", Constants.getRobotConfig().getOverrideLauncherRPM());
  private static final LoggedTunableNumber overrideHoodDeg =
      new LoggedTunableNumber(
          "Overrides/HoodDeg", Constants.getRobotConfig().getOverrideHoodDeg());
  private static final LoggedTunableNumber overrideMotivatorRPM =
      new LoggedTunableNumber(
          "Overrides/MotivatorRPM", Constants.getRobotConfig().getOverrideMotivatorRPM());
  private static final LoggedTunableNumber overrideSpindexerRPM =
      new LoggedTunableNumber(
          "Overrides/SpindexerRPM", Constants.getRobotConfig().getOverrideSpindexerRPM());

  // Additive trim applied to all launcher RPM targets (both smart launch and fixed shots).
  // Adjusted via button box knob during a match. Knob positions: −50, 0, +75, +150 RPM.
  private static double launcherTrimRPM = 0.0;

  // Safety cap matches LauncherIOSparkFlex.MAX_VELOCITY_RPM hardware limit
  private static final double MAX_LAUNCHER_RPM = 5000.0;

  private ShotOverrides() {}

  /** Publish all override tunables and toggle defaults to SmartDashboard. */
  public static void initDashboard() {
    overrideLauncherRPM.get();
    overrideHoodDeg.get();
    overrideMotivatorRPM.get();
    overrideSpindexerRPM.get();
    SmartDashboard.putBoolean(KEY_LAUNCHER, false);
    SmartDashboard.putBoolean(KEY_HOOD, false);
    SmartDashboard.putBoolean(KEY_MOTIVATOR, false);
    SmartDashboard.putBoolean(KEY_SPINDEXER, false);
    SmartDashboard.putNumber("Trim/LauncherRPM", launcherTrimRPM);
    Logger.recordOutput("Trim/LauncherRPM", launcherTrimRPM);
  }

  // ===== Trim API =====

  /**
   * Set the launcher RPM trim offset. Added to all launcher RPM targets (both smart launch and
   * fixed shots). Called by the button box knob bindings.
   */
  public static void setLauncherTrimRPM(double trimRPM) {
    if (launcherTrimRPM != trimRPM) {
      launcherTrimRPM = trimRPM;
      SmartDashboard.putNumber("Trim/LauncherRPM", trimRPM);
      Logger.recordOutput("Trim/LauncherRPM", trimRPM);
    }
  }

  public static double getLauncherTrimRPM() {
    return launcherTrimRPM;
  }

  // ===== Effective value getters =====
  // Each returns the override value when its toggle is on, otherwise the strategy value.
  // Trim is applied only to launcher (it's an RPM offset, not a position offset).

  /** Effective launcher RPM: override or strategy value, plus trim, capped at hardware limit. */
  public static double getLauncherRPM(ShotCalculator.ShotResult shot) {
    double base =
        SmartDashboard.getBoolean(KEY_LAUNCHER, false)
            ? overrideLauncherRPM.get()
            : (shot != null ? shot.launcherRPM() : 0.0);
    return Math.min(base + launcherTrimRPM, MAX_LAUNCHER_RPM);
  }

  /** Effective hood angle in degrees: override or strategy value. */
  public static double getHoodDeg(ShotCalculator.ShotResult shot) {
    return SmartDashboard.getBoolean(KEY_HOOD, false)
        ? overrideHoodDeg.get()
        : (shot != null ? shot.hoodAngleDeg() : 0.0);
  }

  /** Effective motivator RPM: override or strategy value. */
  public static double getMotivatorRPM(ShotCalculator.ShotResult shot) {
    return SmartDashboard.getBoolean(KEY_MOTIVATOR, false)
        ? overrideMotivatorRPM.get()
        : (shot != null ? shot.motivatorRPM() : 0.0);
  }

  /** Effective spindexer RPM: override or strategy value. */
  public static double getSpindexerRPM(ShotCalculator.ShotResult shot) {
    return SmartDashboard.getBoolean(KEY_SPINDEXER, false)
        ? overrideSpindexerRPM.get()
        : (shot != null ? shot.spindexerRPM() : 0.0);
  }
}
