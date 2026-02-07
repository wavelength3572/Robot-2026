package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.motivator.Motivator;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretVisualizer;
import frc.robot.util.BenchTestMetrics;
import frc.robot.util.FuelSim;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Factory class for shooting commands. Provides a unified launch command that works for both
 * simulation (spawns fuel balls) and physical robot (runs motivator/prefeed).
 *
 * <p>Supports two modes:
 *
 * <ul>
 *   <li>COMPETITION: Auto-calculates optimal trajectory to hub (Tuning/Shooting/*)
 *   <li>TEST: Uses manual BenchTest/Shooting/* dashboard values for controlled testing
 * </ul>
 *
 * <p>The launch command:
 *
 * <ol>
 *   <li>Spins up launcher (and motivator if present)
 *   <li>Waits for launcher to reach setpoint
 *   <li>Fires balls repeatedly until button released or out of fuel
 * </ol>
 */
public class ShootingCommands {

  /** Shooting mode determines trajectory calculation and parameter source. */
  public enum ShootingMode {
    /** Auto-calculated trajectory to hub, optimized RPM/angle for distance (Tuning/Shooting/*). */
    COMPETITION,
    /** Manual parameters from BenchTest/Shooting/* dashboard values. */
    TEST
  }

  // Current shooting mode - defaults to competition
  private static ShootingMode currentMode = ShootingMode.COMPETITION;

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
      // Use SmartDashboard for dashboard-visible status
      SmartDashboard.putString("Match/Status/Mode", mode.toString());
      SmartDashboard.putBoolean("Match/Status/Active", mode == ShootingMode.TEST);
      System.out.println("[Shooting] Mode changed to: " + mode);
    }
  }

  /**
   * Check if currently in test mode.
   *
   * @return True if in TEST mode
   */
  public static boolean isTestMode() {
    return currentMode == ShootingMode.TEST;
  }

  // ===== Robot Tuning (affects real robot behavior) =====

  // Target velocities for shooting
  private static final LoggedTunableNumber launchVelocityRPM =
      new LoggedTunableNumber("Tuning/Shooting/LaunchVelocityRPM", 1700.0);

  private static final LoggedTunableNumber motivatorVelocityRPM =
      new LoggedTunableNumber("Tuning/Shooting/MotivatorVelocityRPM", 1000.0);

  private static final LoggedTunableNumber prefeedVelocityRPM =
      new LoggedTunableNumber("Tuning/Shooting/PrefeedVelocityRPM", 1000.0);

  // ===== BenchTest/Shooting/* Override Values (for controlled manual testing) =====
  // These allow manually setting all mechanism parameters while obeying safety limits

  private static final LoggedTunableNumber testLauncherRPM =
      new LoggedTunableNumber("BenchTest/Shooting/LauncherRPM", 1700.0);

  private static final LoggedTunableNumber testMotivatorRPM =
      new LoggedTunableNumber("BenchTest/Shooting/MotivatorRPM", 1000.0);

  private static final LoggedTunableNumber testPrefeedRPM =
      new LoggedTunableNumber("BenchTest/Shooting/PrefeedRPM", 1000.0);

  private static final LoggedTunableNumber testTurretAngleDeg =
      new LoggedTunableNumber("BenchTest/Shooting/AngleDegTurret", 0.0);

  private static final LoggedTunableNumber testHoodAngleDeg =
      new LoggedTunableNumber("BenchTest/Shooting/AngleDegHood", 45.0);

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
    // Normal shooting tunables
    launchVelocityRPM.get();
    motivatorVelocityRPM.get();
    prefeedVelocityRPM.get();

    // TestShooting override tunables
    testLauncherRPM.get();
    testMotivatorRPM.get();
    testPrefeedRPM.get();
    testTurretAngleDeg.get();
    testHoodAngleDeg.get();

    // Initialize all shooting status values so they appear on dashboard immediately
    SmartDashboard.putString("Match/Status/Mode", currentMode.toString());
    SmartDashboard.putBoolean("Match/Status/Active", currentMode == ShootingMode.TEST);
    SmartDashboard.putNumber("Match/Status/CurrentRPM", 0.0);
    SmartDashboard.putBoolean("Match/Status/ReadyAll", false);
    SmartDashboard.putBoolean("Match/Status/ReadyHood", false);
    SmartDashboard.putBoolean("Match/Status/ReadyLauncher", false);
    SmartDashboard.putBoolean("Match/Status/ReadyMotivators", false);
    SmartDashboard.putBoolean("Match/Status/ReadyTurret", false);
    SmartDashboard.putString("Match/Status/State", "Idle");
  }

  /**
   * Main launch command. Spins up, waits for setpoint, then fires repeatedly.
   *
   * <p>Sequence:
   *
   * <ol>
   *   <li>Spin up launcher + motivator feeder wheels (no prefeed yet)
   *   <li>Wait for launcher AND motivator feeders to reach setpoint
   *   <li>Start prefeed and begin firing balls
   * </ol>
   *
   * <p>Works for both simulation (calls turret.launchFuel()) and physical robot (runs motivator
   * prefeed to feed balls).
   *
   * @param launcher The launcher subsystem
   * @param turret The turret subsystem (for sim fuel launching)
   * @param motivator The motivator subsystem (can be null if not present)
   * @return Command that launches while held
   */
  public static Command launchCommand(Launcher launcher, Turret turret, Motivator motivator) {
    return Commands.sequence(
            // Set competition mode for auto-calculated trajectories
            Commands.runOnce(() -> setMode(ShootingMode.COMPETITION)),

            // Phase 1: Spin up launcher and motivator feeder wheels (NO prefeed yet)
            Commands.runOnce(
                () -> {
                  double targetRPM = launchVelocityRPM.get();
                  launcher.setVelocity(targetRPM);
                  turret.enableLaunchMode();
                  if (motivator != null) {
                    // Only spin up the two feeder wheels, not prefeed
                    motivator.setMotivatorsVelocity(motivatorVelocityRPM.get());
                  }
                  SmartDashboard.putString("Match/Status/State", "Spinning Up");
                  Logger.recordOutput("Match/ShotLog/TargetRPM", targetRPM);
                  System.out.println("[Launch] Spinning up launcher to " + targetRPM + " RPM");
                }),

            // Phase 2: Wait for launcher AND motivator feeders to reach setpoint
            Commands.race(
                // Wait for all wheels at setpoint, then debounce for stability
                Commands.sequence(
                    Commands.waitUntil(
                        () -> {
                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.areMotivatorsAtSetpoint();
                          boolean allReady = launcherReady && motivatorReady;
                          SmartDashboard.putBoolean("Match/Status/ReadyLauncher", launcherReady);
                          SmartDashboard.putBoolean("Match/Status/ReadyMotivators", motivatorReady);
                          SmartDashboard.putBoolean("Match/Status/ReadyAll", allReady);
                          return allReady;
                        }),
                    Commands.waitSeconds(0.1), // Debounce - must stay at setpoint for 100ms
                    Commands.waitUntil(
                        () -> {
                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.areMotivatorsAtSetpoint();
                          return launcherReady && motivatorReady;
                        })),
                // Timeout after 3 seconds
                Commands.sequence(
                    Commands.waitSeconds(3.0),
                    Commands.runOnce(
                        () -> System.out.println("[Launch] Spinup timeout - continuing anyway")))),

            // Log that we're ready to fire
            Commands.runOnce(
                () -> {
                  SmartDashboard.putString("Match/Status/State", "Firing");
                  System.out.println("[Launch] At setpoint, starting prefeed and firing");
                  launcher.setFeedingActive(true);
                }),

            // Phase 3: Start prefeed and fire repeatedly
            Commands.parallel(
                // Keep launcher at speed continuously
                Commands.run(
                    () -> {
                      launcher.setVelocity(launchVelocityRPM.get());
                      SmartDashboard.putNumber("Match/Status/CurrentRPM", launcher.getVelocity());
                    },
                    launcher),

                // Keep motivator feeders running AND now add prefeed
                motivator != null
                    ? Commands.run(
                        () ->
                            motivator.setVelocities(
                                motivatorVelocityRPM.get(), prefeedVelocityRPM.get()),
                        motivator)
                    : Commands.none(),

                // Fire balls repeatedly in simulation
                createFiringLoop(turret, launcher)))
        .finallyDo(
            () -> {
              launcher.setFeedingActive(false);
              launcher.stop();
              turret.disableLaunchMode();
              if (motivator != null) {
                motivator.stop();
              }
              SmartDashboard.putString("Match/Status/State", "Stopped");
              System.out.println("[Launch] Stopped");
            })
        .withName("Launch");
  }

  /**
   * Creates the firing loop that repeatedly launches fuel balls. Firing rate is naturally limited
   * by recovery time (if waitForRecovery is enabled) or by minShotInterval.
   *
   * @param turret The turret subsystem
   * @param launcher The launcher subsystem (for recovery notification in sim)
   * @return Command that fires repeatedly until out of fuel
   */
  private static Command createFiringLoop(Turret turret, Launcher launcher) {
    return Commands.sequence(
            // Wait for launcher recovery before firing next shot
            WAIT_FOR_RECOVERY
                ? Commands.waitUntil(launcher::atSetpoint)
                : Commands.waitSeconds(MIN_SHOT_INTERVAL_SECONDS),

            // Fire one ball
            Commands.runOnce(
                () -> {
                  turret.launchFuel();
                  Logger.recordOutput("Match/ShotLog/LastShotTime", Timer.getFPGATimestamp());

                  // Notify launcher of shot (triggers recovery in sim, no-op on real robot)
                  launcher.notifyBallFired();

                  // Log fuel count
                  TurretVisualizer visualizer = turret.getVisualizer();
                  int fuelRemaining = visualizer != null ? visualizer.getFuelCount() : 0;
                  Logger.recordOutput("Match/ShotLog/FuelRemaining", fuelRemaining);
                }),

            // Small delay to prevent multiple fires in same frame
            Commands.waitSeconds(MIN_SHOT_INTERVAL_SECONDS))
        .repeatedly()
        .until(
            () -> {
              TurretVisualizer visualizer = turret.getVisualizer();
              boolean outOfFuel = visualizer == null || visualizer.getFuelCount() <= 0;
              if (outOfFuel) {
                Logger.recordOutput("Match/ShotLog/Status", "Out of Fuel");
                System.out.println("[Launch] Out of fuel");
              }
              return outOfFuel;
            });
  }

  /**
   * Command to reset the simulation for testing. Clears all balls from the field and puts 40 in the
   * hopper.
   *
   * @param turret The turret subsystem (to access visualizer)
   * @return Command that resets simulation state
   */
  public static Command resetSimulationCommand(Turret turret) {
    return Commands.runOnce(
            () -> {
              // Clear field and reset hub scores
              FuelSim.getInstance().clearFuel();
              FuelSim.Hub.BLUE_HUB.resetScore();
              FuelSim.Hub.RED_HUB.resetScore();

              // Put 40 balls in hopper
              TurretVisualizer visualizer = turret.getVisualizer();
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
   * Resets the simulation AND spawns all starting fuel on the field. Use this to set up a
   * game-accurate field for autonomous testing. Clears existing fuel, resets hub scores, spawns all
   * 408 starting fuel (neutral zone + depots), and loads 50 balls in hopper.
   *
   * @param turret The turret subsystem (to access visualizer)
   * @return Command that resets simulation with full field fuel
   */
  public static Command resetStartingFieldCommand(Turret turret) {
    return Commands.runOnce(
            () -> {
              // Clear field and reset hub scores
              FuelSim.getInstance().clearFuel();
              FuelSim.Hub.BLUE_HUB.resetScore();
              FuelSim.Hub.RED_HUB.resetScore();

              // Spawn all starting fuel (neutral zone + depots)
              FuelSim.getInstance().spawnStartingFuel();

              // Put 40 balls in hopper
              TurretVisualizer visualizer = turret.getVisualizer();
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

  /**
   * Manual test launch command using BenchTest/Shooting/* dashboard values for turret angle, hood
   * angle, and RPM. Positions all mechanisms to the specified values (obeying safety limits), waits
   * for everything to reach setpoint, then fires.
   *
   * <p>Use this for controlled testing where you want to specify exact values for:
   *
   * <ul>
   *   <li>Launcher RPM (BenchTest/Shooting/LauncherRPM)
   *   <li>Motivator RPM (BenchTest/Shooting/MotivatorRPM)
   *   <li>Prefeed RPM (BenchTest/Shooting/PrefeedRPM)
   *   <li>Turret angle in degrees (BenchTest/Shooting/AngleDegTurret)
   *   <li>Hood angle in degrees (BenchTest/Shooting/AngleDegHood)
   * </ul>
   *
   * <p>Sequence:
   *
   * <ol>
   *   <li>Set turret and hood angles (they clamp internally to safety limits)
   *   <li>Spin up launcher and motivator wheels
   *   <li>Wait for launcher, motivator, turret, AND hood to reach setpoint
   *   <li>Start prefeed and begin firing
   * </ol>
   *
   * @param launcher The launcher subsystem
   * @param turret The turret subsystem
   * @param motivator The motivator subsystem (can be null)
   * @param hood The hood subsystem (can be null)
   * @return Command that positions, spins up, and fires while held
   */
  public static Command testLaunchCommand(
      Launcher launcher, Turret turret, Motivator motivator, Hood hood) {
    return Commands.sequence(
            // Set test mode for manual trajectory parameters
            Commands.runOnce(() -> setMode(ShootingMode.TEST)),

            // Reset metrics at start of each run
            Commands.runOnce(() -> BenchTestMetrics.getInstance().reset()),

            // Phase 1: Start positioning turret and hood, spin up wheels
            Commands.runOnce(
                () -> {
                  // Enable full turret range for test shooting
                  turret.enableLaunchMode();

                  // Set turret angle (subsystem clamps to limits)
                  double turretAngle = testTurretAngleDeg.get();
                  turret.setAngle(turretAngle);

                  // Set hood angle if available (subsystem clamps to limits)
                  double hoodAngle = testHoodAngleDeg.get();
                  if (hood != null) {
                    hood.setAngle(hoodAngle);
                  }

                  // Spin up launcher
                  double launcherRPM = testLauncherRPM.get();
                  launcher.setVelocity(launcherRPM);

                  // Set manual shot parameters for trajectory visualization
                  // This makes the sim trajectory match our test parameters
                  // Use TARGET turret angle so trajectory shows where shot WILL go
                  turret.setManualShotParameters(launcherRPM, hoodAngle, turretAngle);

                  // Spin up motivator feeders (no prefeed yet)
                  if (motivator != null) {
                    motivator.setMotivatorsVelocity(testMotivatorRPM.get());
                  }

                  SmartDashboard.putString("Match/Status/State", "Positioning & Spinning Up");
                  System.out.println(
                      "[ManualFire] Positioning turret to "
                          + turretAngle
                          + "° and spinning up to "
                          + launcherRPM
                          + " RPM");
                }),

            // Phase 2: Wait for everything to reach setpoint
            Commands.race(
                Commands.sequence(
                    Commands.waitUntil(
                        () -> {
                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.areMotivatorsAtSetpoint();
                          boolean turretReady = turret.atTarget();
                          boolean hoodReady = hood == null || hood.atTarget();

                          boolean allReady =
                              launcherReady && motivatorReady && turretReady && hoodReady;

                          SmartDashboard.putBoolean("Match/Status/ReadyLauncher", launcherReady);
                          SmartDashboard.putBoolean("Match/Status/ReadyMotivators", motivatorReady);
                          SmartDashboard.putBoolean("Match/Status/ReadyTurret", turretReady);
                          SmartDashboard.putBoolean("Match/Status/ReadyHood", hoodReady);
                          SmartDashboard.putBoolean("Match/Status/ReadyAll", allReady);

                          return allReady;
                        }),
                    // Debounce - must stay at setpoint for 100ms
                    Commands.waitSeconds(0.1),
                    Commands.waitUntil(
                        () -> {
                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.areMotivatorsAtSetpoint();
                          boolean turretReady = turret.atTarget();
                          boolean hoodReady = hood == null || hood.atTarget();
                          return launcherReady && motivatorReady && turretReady && hoodReady;
                        })),
                // Timeout after 5 seconds (longer than normal to allow positioning)
                Commands.sequence(
                    Commands.waitSeconds(5.0),
                    Commands.runOnce(
                        () -> {
                          System.out.println(
                              "[ManualFire] WARNING: Setup timeout - check turret/hood positions!");
                          SmartDashboard.putString(
                              "Match/Status/State", "TIMEOUT - continuing anyway");
                        }))),

            // Log ready state
            Commands.runOnce(
                () -> {
                  SmartDashboard.putString("Match/Status/State", "Ready - Starting Prefeed");
                  System.out.println("[ManualFire] All mechanisms ready, starting prefeed");
                  launcher.setFeedingActive(true);
                }),

            // Phase 3: Start prefeed and fire repeatedly
            Commands.parallel(
                // Keep launcher at speed and update manual shot parameters for trajectory
                Commands.run(
                    () -> {
                      double rpm = testLauncherRPM.get();
                      double hoodAngle = testHoodAngleDeg.get();
                      double turretAngle = testTurretAngleDeg.get();
                      launcher.setVelocity(rpm);
                      turret.setManualShotParameters(rpm, hoodAngle, turretAngle);
                      SmartDashboard.putNumber("Match/Status/CurrentRPM", launcher.getVelocity());
                    },
                    launcher),

                // Keep turret positioned
                Commands.run(() -> turret.setAngle(testTurretAngleDeg.get()), turret),

                // Keep hood positioned
                hood != null
                    ? Commands.run(() -> hood.setAngle(testHoodAngleDeg.get()), hood)
                    : Commands.none(),

                // Keep motivator feeders running AND add prefeed
                motivator != null
                    ? Commands.run(
                        () -> motivator.setVelocities(testMotivatorRPM.get(), testPrefeedRPM.get()),
                        motivator)
                    : Commands.none(),

                // Fire balls repeatedly with metrics recording
                createBenchTestFiringLoop(turret, launcher)))
        .finallyDo(
            () -> {
              launcher.setFeedingActive(false);
              launcher.stop();
              if (motivator != null) {
                motivator.stop();
              }
              if (hood != null) {
                hood.stop();
              }
              // Clear manual shot parameters, disable launch mode, and return to competition mode
              turret.clearManualShotParameters();
              turret.disableLaunchMode();
              setMode(ShootingMode.COMPETITION);
              SmartDashboard.putString("Match/Status/State", "Stopped");
              System.out.println("[ManualFire] Stopped");
            })
        .withName("ManualFire");
  }

  /**
   * Bench test launch command. Stripped-down version that only waits for launcher + motivator (no
   * turret/hood check, no timeout delay). Uses BenchTest/Shooting/* tunables for live RPM
   * adjustment. Integrates BenchTestMetrics for shot counting and rate tracking.
   *
   * @param launcher The launcher subsystem
   * @param turret The turret subsystem (for sim fuel launching)
   * @param motivator The motivator subsystem (can be null)
   * @return Command that launches while held
   */
  public static Command benchTestLaunchCommand(
      Launcher launcher, Turret turret, Motivator motivator) {
    return Commands.sequence(
            Commands.runOnce(() -> setMode(ShootingMode.TEST)),

            // Reset metrics at start of each run
            Commands.runOnce(() -> BenchTestMetrics.getInstance().reset()),

            // Phase 1: Spin up launcher and motivator feeders (no prefeed yet)
            Commands.runOnce(
                () -> {
                  double targetRPM = testLauncherRPM.get();
                  launcher.setVelocity(targetRPM);
                  turret.enableLaunchMode();
                  if (motivator != null) {
                    motivator.setMotivatorsVelocity(testMotivatorRPM.get());
                  }
                  SmartDashboard.putString("Match/Status/State", "BenchTest: Spinning Up");
                  System.out.println("[BenchTest] Spinning up to " + targetRPM + " RPM");
                }),

            // Phase 2: Wait for launcher + motivator only (no turret/hood check)
            Commands.race(
                Commands.sequence(
                    Commands.waitUntil(
                        () -> {
                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.areMotivatorsAtSetpoint();
                          return launcherReady && motivatorReady;
                        }),
                    Commands.waitSeconds(0.1),
                    Commands.waitUntil(
                        () -> {
                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.areMotivatorsAtSetpoint();
                          return launcherReady && motivatorReady;
                        })),
                Commands.sequence(
                    Commands.waitSeconds(3.0),
                    Commands.runOnce(
                        () ->
                            System.out.println("[BenchTest] Spinup timeout - continuing anyway")))),

            // Start firing
            Commands.runOnce(
                () -> {
                  SmartDashboard.putString("Match/Status/State", "BenchTest: Firing");
                  System.out.println("[BenchTest] Firing");
                  launcher.setFeedingActive(true);
                }),

            // Phase 3: Fire with live tunable RPM + metrics integration
            Commands.parallel(
                // Keep launcher at speed (reads tunable each cycle)
                Commands.run(
                    () -> {
                      launcher.setVelocity(testLauncherRPM.get());
                      SmartDashboard.putNumber("Match/Status/CurrentRPM", launcher.getVelocity());
                    },
                    launcher),

                // Keep motivator + prefeed running
                motivator != null
                    ? Commands.run(
                        () -> motivator.setVelocities(testMotivatorRPM.get(), testPrefeedRPM.get()),
                        motivator)
                    : Commands.none(),

                // Fire balls with metrics recording
                createBenchTestFiringLoop(turret, launcher)))
        .finallyDo(
            () -> {
              launcher.setFeedingActive(false);
              launcher.stop();
              turret.disableLaunchMode();
              if (motivator != null) {
                motivator.stop();
              }
              setMode(ShootingMode.COMPETITION);
              SmartDashboard.putString("Match/Status/State", "Stopped");
              System.out.println("[BenchTest] Stopped");
            })
        .withName("BenchTestLaunch");
  }

  /**
   * Firing loop for bench testing with BenchTestMetrics integration.
   *
   * @param turret The turret subsystem
   * @param launcher The launcher subsystem
   * @return Command that fires repeatedly and records metrics
   */
  private static Command createBenchTestFiringLoop(Turret turret, Launcher launcher) {
    return Commands.sequence(
            Commands.waitUntil(launcher::atSetpoint),
            Commands.runOnce(
                () -> {
                  turret.launchFuel();
                  launcher.notifyBallFired();
                  BenchTestMetrics.getInstance().recordShot();
                  Logger.recordOutput("Match/ShotLog/LastShotTime", Timer.getFPGATimestamp());

                  TurretVisualizer visualizer = turret.getVisualizer();
                  int fuelRemaining = visualizer != null ? visualizer.getFuelCount() : 0;
                  Logger.recordOutput("Match/ShotLog/FuelRemaining", fuelRemaining);
                }),
            Commands.waitSeconds(MIN_SHOT_INTERVAL_SECONDS))
        .repeatedly()
        .until(
            () -> {
              TurretVisualizer visualizer = turret.getVisualizer();
              return visualizer == null || visualizer.getFuelCount() <= 0;
            });
  }
}
