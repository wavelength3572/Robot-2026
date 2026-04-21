package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.motivator.Motivator;
import frc.robot.subsystems.shooting.ShootingCoordinator;
import frc.robot.subsystems.shooting.ShotCalculator;
import frc.robot.subsystems.shooting.ShotOverrides;
import frc.robot.subsystems.shooting.ShotVisualizer;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.FuelSim;
import org.littletonrobotics.junction.Logger;

/**
 * Factory class for shooting commands. The coordinator owns all shooting behavior; these are thin
 * wrappers kept for call-site compatibility and simulation/reset utilities.
 *
 * <p>Prefer calling coordinator methods directly for new code:
 *
 * <ul>
 *   <li>{@link ShootingCoordinator#shootCommand()} — smart launch (teleop)
 *   <li>{@link ShootingCoordinator#shootCommand(ShootingCoordinator.ArmTrigger)} — with arm trigger
 *   <li>{@link ShootingCoordinator#hubShotCommand()} — fixed hub shot
 *   <li>{@link ShootingCoordinator#leftTrenchShotCommand()} — fixed left-trench shot
 *   <li>{@link ShootingCoordinator#rightTrenchShotCommand()} — fixed right-trench shot
 * </ul>
 */
public class ShootingCommands {

  private ShootingCommands() {}

  /** Initialize all tunables so they appear in the dashboard immediately on boot. */
  public static void initTunables() {
    ShotOverrides.initDashboard();

    SmartDashboard.putString("Match/Status/Goal", ShootingCoordinator.Goal.IDLE.name());
    SmartDashboard.putNumber("Match/Status/CurrentRPM", 0.0);
    SmartDashboard.putBoolean("Match/Status/ReadyAll", false);
    SmartDashboard.putBoolean("Match/Status/ReadyHood", false);
    SmartDashboard.putBoolean("Match/Status/ReadyLauncher", false);
    SmartDashboard.putBoolean("Match/Status/ReadyMotivators", false);
    SmartDashboard.putBoolean("Match/Status/ReadyTurret", false);
    SmartDashboard.putString("Match/Status/State", "Idle");
  }

  // ===== Launcher Trim =====

  public static void setLauncherTrimRPM(double trimRPM) {
    ShotOverrides.setLauncherTrimRPM(trimRPM);
  }

  public static double getLauncherTrimRPM() {
    return ShotOverrides.getLauncherTrimRPM();
  }

  // ===== Per-Actuator Override Helpers (backward compat) =====

  public static double getEffectiveRPM(ShotCalculator.ShotResult shot) {
    return ShotOverrides.getLauncherRPM(shot);
  }

  public static double getEffectiveHoodDeg(ShotCalculator.ShotResult shot) {
    return ShotOverrides.getHoodDeg(shot);
  }

  public static double getEffectiveMotivatorRPM(ShotCalculator.ShotResult shot) {
    return ShotOverrides.getMotivatorRPM(shot);
  }

  public static double getEffectiveSpindexerRPM(ShotCalculator.ShotResult shot) {
    return ShotOverrides.getSpindexerRPM(shot);
  }

  // ===== Simulation Commands =====

  /** Reset simulation state: clear field, reset scores, refill hopper to 40 balls. */
  public static Command resetSimulationCommand(ShootingCoordinator coordinator) {
    return Commands.runOnce(
            () -> {
              FuelSim.getInstance().clearFuel();
              FuelSim.Hub.BLUE_HUB.resetScore();
              FuelSim.Hub.RED_HUB.resetScore();
              coordinator.resetShotCounts();
              ShotVisualizer visualizer = coordinator.getVisualizer();
              if (visualizer != null) {
                visualizer.setFuelCount(40);
                Logger.recordOutput("ShotLog/FuelRemaining", 40);
              }
              Logger.recordOutput("ShotLog/SimReset", true);
              System.out.println("[Shooting] Simulation reset: field cleared, 40 balls in hopper");
            })
        .ignoringDisable(true)
        .withName("Reset Simulation");
  }

  /** Reset simulation AND spawn all starting fuel on the field. */
  public static Command resetStartingFieldCommand(ShootingCoordinator coordinator) {
    return Commands.runOnce(
            () -> {
              FuelSim.getInstance().clearFuel();
              FuelSim.Hub.BLUE_HUB.resetScore();
              FuelSim.Hub.RED_HUB.resetScore();
              coordinator.resetShotCounts();
              FuelSim.getInstance().spawnStartingFuel();
              ShotVisualizer visualizer = coordinator.getVisualizer();
              if (visualizer != null) {
                visualizer.setFuelCount(40);
                Logger.recordOutput("ShotLog/FuelRemaining", 40);
              }
              Logger.recordOutput("ShotLog/SimReset", true);
              System.out.println(
                  "[Shooting] Field reset: all starting fuel spawned, 40 balls in hopper");
            })
        .ignoringDisable(true)
        .withName("Reset Starting Field");
  }

  // ===== Command Wrappers (delegate to coordinator) =====
  // Kept so existing call sites (AutoWrapperFactory, ButtonsAndDashboardBindings) compile
  // unchanged. Prefer coordinator.shootCommand() / coordinator.hubShotCommand() for new code.

  public static Command hubShotCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {
    return coordinator.hubShotCommand();
  }

  public static Command leftTrenchShotCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {
    return coordinator.leftTrenchShotCommand();
  }

  public static Command rightTrenchShotCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {
    return coordinator.rightTrenchShotCommand();
  }

  public static Command smartLaunchDangerousCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {
    return coordinator.shootCommand();
  }

  public static Command smartLaunchDangerousCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer,
      ShootingCoordinator.ArmTrigger armTrigger) {
    return coordinator.shootCommand(armTrigger);
  }
}
