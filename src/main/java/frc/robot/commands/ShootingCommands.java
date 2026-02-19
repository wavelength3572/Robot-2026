package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.motivator.Motivator;
import frc.robot.subsystems.shooting.ShootingCoordinator;
import frc.robot.subsystems.shooting.ShotVisualizer;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.util.BenchTestMetrics;
import frc.robot.util.FuelSim;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
//

/**
 * Factory class for shooting commands. Provides a unified launch command that works for both
 * simulation (spawns fuel balls) and physical robot (runs motivator).
 *
 * <p>Supports two modes:
 *
 * <ul>
 *   <li>AUTO: Auto-calculates optimal trajectory to hub (Tuning/Shooting/*)
 *   <li>MANUAL: Uses manual BenchTest/Shooting/* dashboard values for controlled testing
 * </ul>
 *
 * <p>The launch command:
 *
 * <ol>
 *   <li>Positions turret + hood (manual angles or auto-calculated)
 *   <li>Spins up launcher and motivator to RPM
 *   <li>Waits for all subsystems to reach setpoint (5s timeout)
 *   <li>Activates spindexer to feed
 *   <li>On release: everything stops, back to IDLE
 * </ol>
 */
public class ShootingCommands {

  /** Shooting mode determines trajectory calculation and parameter source. */
  public enum ShootingMode {
    /** Auto-calculated trajectory to hub, optimized RPM/angle for distance (Tuning/Shooting/*). */
    AUTO,
    /** Manual parameters from BenchTest/Shooting/* dashboard values. */
    MANUAL
  }

  // Current shooting mode - defaults to manual for safe bringup
  private static ShootingMode currentMode = ShootingMode.MANUAL;

  /**
   * Get the current shooting mode.
   *
   * @return Current ShootingMode
   */
  public static ShootingMode getMode() {
    return currentMode;
  }

  /**
   * Set the shooting mode.
   *
   * @param mode The mode to set
   */
  public static void setMode(ShootingMode mode) {
    if (currentMode != mode) {
      currentMode = mode;
      SmartDashboard.putString("Match/Status/Mode", mode.toString());
      System.out.println("[Shooting] Mode changed to: " + mode);
    }
  }

  /**
   * Check if currently in manual mode.
   *
   * @return True if in MANUAL mode
   */
  public static boolean isManualMode() {
    return currentMode == ShootingMode.MANUAL;
  }

  // ===== BenchTest/Shooting/* Values =====

  private static final LoggedTunableNumber testLauncherRPM =
      new LoggedTunableNumber("BenchTest/Shooting/LauncherRPM", 2500.0);

  private static final LoggedTunableNumber testMotivatorRPM =
      new LoggedTunableNumber("BenchTest/Shooting/MotivatorRPM", 1500.0);

  private static final LoggedTunableNumber testTurretAngleDeg =
      new LoggedTunableNumber("BenchTest/Shooting/AngleDegTurret", 0.0);

  private static final LoggedTunableNumber testTurretVolts =
      new LoggedTunableNumber("BenchTest/Shooting/TurretVolts", 0.0);

  private static final LoggedTunableNumber testHoodAngleDeg =
      new LoggedTunableNumber("BenchTest/Shooting/AngleDegHood", 16.0);

  private static final LoggedTunableNumber testSpindexerRPM =
      new LoggedTunableNumber("BenchTest/Shooting/SpindexerRPM", 750.0);

  // ===== Command Behavior Constants =====

  // Minimum time between shots (prevents multiple fires per frame)
  private static final double MIN_SHOT_INTERVAL_SECONDS = 0.05;

  // Always wait for setpoint recovery before firing next shot
  private static final boolean WAIT_FOR_RECOVERY = true;

  private ShootingCommands() {
    // Static factory class
  }

  /** Initialize tunables so they appear in the dashboard immediately. */
  public static void initTunables() {
    testLauncherRPM.get();
    testMotivatorRPM.get();
    testTurretAngleDeg.get();
    testTurretVolts.get();
    testHoodAngleDeg.get();
    testSpindexerRPM.get();

    SmartDashboard.putString("Match/Status/Mode", currentMode.toString());
    SmartDashboard.putNumber("Match/Status/CurrentRPM", 0.0);
    SmartDashboard.putBoolean("Match/Status/ReadyAll", false);
    SmartDashboard.putBoolean("Match/Status/ReadyHood", false);
    SmartDashboard.putBoolean("Match/Status/ReadyLauncher", false);
    SmartDashboard.putBoolean("Match/Status/ReadyMotivators", false);
    SmartDashboard.putBoolean("Match/Status/ReadyTurret", false);
    SmartDashboard.putString("Match/Status/State", "Idle");
  }

  // ===== Public Getters for BenchTest Tunables =====

  public static LoggedTunableNumber getTestLauncherRPM() {
    return testLauncherRPM;
  }

  public static LoggedTunableNumber getTestMotivatorRPM() {
    return testMotivatorRPM;
  }

  public static LoggedTunableNumber getTestSpindexerRPM() {
    return testSpindexerRPM;
  }

  public static LoggedTunableNumber getTestTurretAngleDeg() {
    return testTurretAngleDeg;
  }

  public static LoggedTunableNumber getTestTurretVolts() {
    return testTurretVolts;
  }

  /**
   * Unified launch command. Checks ShootingMode at startup to determine parameter source. In MANUAL
   * mode, uses BenchTest/Shooting/* dashboard values and positions turret + hood. In AUTO mode,
   * uses Tuning/Shooting/* RPMs and lets the coordinator handle turret + hood aiming.
   *
   * <p>Pipeline: position + spin up → wait for setpoint (5s timeout) → feed and fire → cleanup.
   *
   * @param launcher The launcher subsystem
   * @param coordinator The shooting coordinator
   * @param motivator The motivator subsystem (can be null if not present)
   * @param hood The hood subsystem (can be null if not present)
   * @param spindexer The spindexer subsystem (can be null if not present)
   * @return Command that launches while held
   */
  public static Command launchCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Hood hood,
      Spindexer spindexer) {
    return Commands.sequence(
            // Phase 0: Setup — read mode, reset metrics, enable launch mode, set parameters
            Commands.runOnce(
                () -> {
                  BenchTestMetrics.getInstance().reset();
                  coordinator.enableLaunchMode();

                  if (isManualMode()) {
                    // MANUAL: position turret + hood from dashboard values
                    double turretAngle = testTurretAngleDeg.get();
                    coordinator.setTurretAngle(turretAngle);

                    double hoodAngle = testHoodAngleDeg.get();
                    if (hood != null) {
                      hood.setAngle(hoodAngle);
                    }

                    double launcherRPM = testLauncherRPM.get();
                    launcher.setVelocity(launcherRPM);
                    coordinator.setManualShotParameters(launcherRPM, hoodAngle, turretAngle);

                    if (motivator != null) {
                      motivator.setMotivatorVelocity(testMotivatorRPM.get());
                    }

                    SmartDashboard.putString("Match/Status/State", "Positioning & Spinning Up");
                    Logger.recordOutput("Match/ShotLog/TargetRPM", launcherRPM);
                    System.out.println(
                        "[Launch] MANUAL: Positioning turret to "
                            + turretAngle
                            + "° and spinning up to "
                            + launcherRPM
                            + " RPM");
                  } else {
                    // AUTO: coordinator handles turret + hood aiming
                    // TODO: get RPMs from coordinator's calculated shot instead of BenchTest values
                    double targetRPM = testLauncherRPM.get();
                    launcher.setVelocity(targetRPM);
                    if (motivator != null) {
                      motivator.setMotivatorVelocity(testMotivatorRPM.get());
                    }

                    SmartDashboard.putString("Match/Status/State", "Spinning Up");
                    Logger.recordOutput("Match/ShotLog/TargetRPM", targetRPM);
                    System.out.println("[Launch] AUTO: Spinning up to " + targetRPM + " RPM");
                  }
                }),

            // Phase 1: Wait for all subsystems to reach setpoint (5s timeout)
            Commands.race(
                Commands.sequence(
                    Commands.waitUntil(
                        () -> {
                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.isMotivatorAtSetpoint();
                          boolean turretReady = !isManualMode() || coordinator.turretAtTarget();
                          boolean hoodReady = !isManualMode() || hood == null || hood.atTarget();

                          boolean allReady =
                              launcherReady && motivatorReady && turretReady && hoodReady;

                          SmartDashboard.putBoolean("Match/Status/ReadyLauncher", launcherReady);
                          SmartDashboard.putBoolean("Match/Status/ReadyMotivators", motivatorReady);
                          SmartDashboard.putBoolean("Match/Status/ReadyTurret", turretReady);
                          SmartDashboard.putBoolean("Match/Status/ReadyHood", hoodReady);
                          SmartDashboard.putBoolean("Match/Status/ReadyAll", allReady);

                          return allReady;
                        }),
                    Commands.waitSeconds(0.1),
                    Commands.waitUntil(
                        () -> {
                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.isMotivatorAtSetpoint();
                          boolean turretReady = !isManualMode() || coordinator.turretAtTarget();
                          boolean hoodReady = !isManualMode() || hood == null || hood.atTarget();
                          return launcherReady && motivatorReady && turretReady && hoodReady;
                        })),
                Commands.sequence(
                    Commands.waitSeconds(5.0),
                    Commands.runOnce(
                        () -> {
                          System.out.println("[Launch] WARNING: Setup timeout - continuing anyway");
                          SmartDashboard.putString(
                              "Match/Status/State", "TIMEOUT - continuing anyway");
                        }))),

            // Phase 2: Enable feeding, log ready
            Commands.runOnce(
                () -> {
                  SmartDashboard.putString("Match/Status/State", "Firing");
                  System.out.println("[Launch] At setpoint, starting prefeed and firing");
                  launcher.setFeedingActive(true);
                }),

            // Phase 3: Fire repeatedly while held
            Commands.parallel(
                // Keep launcher at speed and update status continuously
                Commands.run(
                    () -> {
                      double rpm = testLauncherRPM.get();
                      launcher.setVelocity(rpm);
                      SmartDashboard.putNumber("Match/Status/CurrentRPM", launcher.getVelocity());

                      // In MANUAL mode, keep updating manual shot parameters for trajectory
                      if (isManualMode()) {
                        coordinator.setManualShotParameters(
                            rpm, testHoodAngleDeg.get(), testTurretAngleDeg.get());
                      }

                      // Update ready indicators live during firing
                      boolean launcherReady = launcher.atSetpoint();
                      boolean motivatorReady =
                          motivator == null || motivator.isMotivatorAtSetpoint();
                      boolean turretReady = !isManualMode() || coordinator.turretAtTarget();
                      boolean hoodReady = !isManualMode() || hood == null || hood.atTarget();
                      SmartDashboard.putBoolean("Match/Status/ReadyLauncher", launcherReady);
                      SmartDashboard.putBoolean("Match/Status/ReadyMotivators", motivatorReady);
                      SmartDashboard.putBoolean("Match/Status/ReadyTurret", turretReady);
                      SmartDashboard.putBoolean("Match/Status/ReadyHood", hoodReady);
                      SmartDashboard.putBoolean(
                          "Match/Status/ReadyAll",
                          launcherReady && motivatorReady && turretReady && hoodReady);
                    },
                    launcher),

                // Keep turret positioned in MANUAL mode (checked inside lambda)
                Commands.run(
                    () -> {
                      if (isManualMode()) {
                        coordinator.setTurretAngle(testTurretAngleDeg.get());
                      }
                    }),

                // Keep hood positioned in MANUAL mode
                hood != null
                    ? Commands.run(
                        () -> {
                          if (isManualMode()) {
                            hood.setAngle(testHoodAngleDeg.get());
                          }
                        },
                        hood)
                    : Commands.none(),

                // Keep motivator feeders running
                motivator != null
                    ? Commands.run(
                        () -> motivator.setMotivatorVelocity(testMotivatorRPM.get()), motivator)
                    : Commands.none(),

                // Start spindexer to feed fuel into the pipeline
                spindexer != null
                    ? Commands.run(
                        () -> spindexer.setSpindexerVelocity(testSpindexerRPM.get()), spindexer)
                    : Commands.none(),

                // Fire balls repeatedly
                createFiringLoop(coordinator, launcher)))
        .finallyDo(
            () -> {
              launcher.setFeedingActive(false);
              launcher.stop();
              if (motivator != null) {
                motivator.stopMotivator();
              }
              if (spindexer != null) {
                spindexer.stopSpindexer();
              }
              coordinator.clearManualShotParameters();
              coordinator.disableLaunchMode();
              SmartDashboard.putString("Match/Status/State", "Stopped");
              System.out.println("[Launch] Stopped");
            })
        .withName("Launch");
  }

  /**
   * Creates the firing loop that repeatedly launches fuel balls. Records BenchTestMetrics on every
   * shot (harmless in auto mode).
   *
   * @param coordinator The shooting coordinator
   * @param launcher The launcher subsystem (for recovery notification in sim)
   * @return Command that fires repeatedly until out of fuel
   */
  private static Command createFiringLoop(ShootingCoordinator coordinator, Launcher launcher) {
    return Commands.sequence(
            WAIT_FOR_RECOVERY
                ? Commands.waitUntil(launcher::atSetpoint)
                : Commands.waitSeconds(MIN_SHOT_INTERVAL_SECONDS),
            Commands.runOnce(
                () -> {
                  coordinator.launchFuel();
                  Logger.recordOutput("Match/ShotLog/LastShotTime", Timer.getFPGATimestamp());

                  launcher.notifyBallFired();
                  BenchTestMetrics.getInstance().recordShot();

                  ShotVisualizer visualizer = coordinator.getVisualizer();
                  int fuelRemaining = visualizer != null ? visualizer.getFuelCount() : 0;
                  Logger.recordOutput("Match/ShotLog/FuelRemaining", fuelRemaining);
                }),
            Commands.waitSeconds(MIN_SHOT_INTERVAL_SECONDS))
        .repeatedly()
        .until(
            () -> {
              ShotVisualizer visualizer = coordinator.getVisualizer();
              boolean outOfFuel = visualizer == null || visualizer.getFuelCount() <= 0;
              if (outOfFuel) {
                Logger.recordOutput("Match/ShotLog/Status", "Out of Fuel");
                System.out.println("[Launch] Out of fuel");
              }
              return outOfFuel;
            });
  }

  /**
   * Command to reset the simulation for testing.
   *
   * @param coordinator The shooting coordinator
   * @return Command that resets simulation state
   */
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
                Logger.recordOutput("Match/ShotLog/FuelRemaining", 40);
              }

              Logger.recordOutput("Match/ShotLog/SimReset", true);
              System.out.println("[Shooting] Simulation reset: field cleared, 40 balls in hopper");
            })
        .ignoringDisable(true)
        .withName("Reset Simulation");
  }

  /**
   * Resets the simulation AND spawns all starting fuel on the field.
   *
   * @param coordinator The shooting coordinator
   * @return Command that resets simulation with full field fuel
   */
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
                Logger.recordOutput("Match/ShotLog/FuelRemaining", 40);
              }

              Logger.recordOutput("Match/ShotLog/SimReset", true);
              System.out.println(
                  "[Shooting] Field reset: all starting fuel spawned, 40 balls in hopper");
            })
        .ignoringDisable(true)
        .withName("Reset Starting Field");
  }
}
