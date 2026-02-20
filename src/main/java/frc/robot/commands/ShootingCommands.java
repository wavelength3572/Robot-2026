package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.motivator.Motivator;
import frc.robot.subsystems.shooting.ShootingCoordinator;
import frc.robot.subsystems.shooting.ShotCalculator;
import frc.robot.subsystems.shooting.ShotVisualizer;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
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

  // ===== Fixed Shot Presets =====

  // Hub shot — close-range shot into the hub
  private static final double HUB_SHOT_LAUNCHER_RPM = 2600.0;
  private static final double HUB_SHOT_HOOD_ANGLE_DEG = 16.0;
  private static final double HUB_SHOT_TURRET_ANGLE_DEG = -90.0;

  // Left trench shot
  private static final double LEFT_TRENCH_LAUNCHER_RPM = 3080.0;
  private static final double LEFT_TRENCH_HOOD_ANGLE_DEG = 19.0;
  private static final double LEFT_TRENCH_TURRET_ANGLE_DEG = 208.0;

  // Right trench shot
  private static final double RIGHT_TRENCH_LAUNCHER_RPM = 3080.0;
  private static final double RIGHT_TRENCH_HOOD_ANGLE_DEG = 19.0;
  private static final double RIGHT_TRENCH_TURRET_ANGLE_DEG = -25.0;

  // ===== Robot Tuning (affects real robot behavior) =====

  // Target velocities for shooting
  private static final LoggedTunableNumber launchVelocityRPM =
      new LoggedTunableNumber("Tuning/Shooting/LaunchVelocityRPM", 1700.0);

  private static final LoggedTunableNumber motivatorVelocityRPM =
      new LoggedTunableNumber("Tuning/Shooting/MotivatorVelocityRPM", 1100.0);

  private static final LoggedTunableNumber spindexerFeedRPM =
      new LoggedTunableNumber("Tuning/Shooting/SpindexerFeedRPM", 750.0);

  // ===== BenchTest/Shooting/* Override Values (for controlled manual testing)
  // =====

  private static final LoggedTunableNumber testLauncherRPM =
      new LoggedTunableNumber("BenchTest/Shooting/LauncherRPM", 1700.0);

  private static final LoggedTunableNumber testMotivatorRPM =
      new LoggedTunableNumber("BenchTest/Shooting/MotivatorRPM", 1000.0);

  private static final LoggedTunableNumber testOutsideTurretAngleDeg =
      new LoggedTunableNumber("BenchTest/Shooting/OutsideTurretAngleDeg", 0.0);

  private static final LoggedTunableNumber testInsideTurretAngleDeg =
      new LoggedTunableNumber("BenchTest/Shooting/InsideTurretAngleDeg", 0.0);

  private static final LoggedTunableNumber testTurretVolts =
      new LoggedTunableNumber("BenchTest/Shooting/TurretVolts", 0.0);

  private static final LoggedTunableNumber testHoodAngleDeg =
      new LoggedTunableNumber("BenchTest/Shooting/AngleDegHood", 45.0);

  // Placeholder for future indexer subsystem
  private static final LoggedTunableNumber testSpindexerRPM =
      new LoggedTunableNumber("BenchTest/Shooting/SpindexerRPM", 1000.0);

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
    launchVelocityRPM.get();
    motivatorVelocityRPM.get();

    testLauncherRPM.get();
    testMotivatorRPM.get();
    testOutsideTurretAngleDeg.get();
    testInsideTurretAngleDeg.get();
    testTurretVolts.get();
    testHoodAngleDeg.get();
    testSpindexerRPM.get();

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

  public static LoggedTunableNumber getOutsideTurretAngleDeg() {
    return testOutsideTurretAngleDeg;
  }

  public static LoggedTunableNumber getInsideTurretAngleDeg() {
    return testInsideTurretAngleDeg;
  }

  public static LoggedTunableNumber getTestTurretVolts() {
    return testTurretVolts;
  }

  /**
   * Main launch command. Spins up, waits for setpoint, then fires repeatedly.
   *
   * @param launcher The launcher subsystem
   * @param coordinator The shooting coordinator
   * @param motivator The motivator subsystem (can be null if not present)
   * @return Command that launches while held
   */
  public static Command launchCommand(
      Launcher launcher, ShootingCoordinator coordinator, Motivator motivator) {
    return Commands.sequence(
            // Set competition mode for auto-calculated trajectories
            Commands.runOnce(() -> setMode(ShootingMode.COMPETITION)),

            // Phase 1: Spin up launcher and motivator feeder wheels (NO prefeed yet)
            Commands.runOnce(
                () -> {
                  double targetRPM = launchVelocityRPM.get();
                  launcher.setVelocity(targetRPM);
                  coordinator.enableLaunchMode();
                  if (motivator != null) {
                    motivator.setMotivatorVelocity(motivatorVelocityRPM.get());
                  }
                  SmartDashboard.putString("Match/Status/State", "Spinning Up");
                  Logger.recordOutput("Match/ShotLog/TargetRPM", targetRPM);
                  System.out.println("[Launch] Spinning up launcher to " + targetRPM + " RPM");
                }),

            // Phase 2: Wait for launcher AND motivator feeders to reach setpoint
            Commands.race(
                Commands.sequence(
                    Commands.waitUntil(
                        () -> {
                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.isMotivatorAtSetpoint();
                          boolean allReady = launcherReady && motivatorReady;
                          SmartDashboard.putBoolean("Match/Status/ReadyLauncher", launcherReady);
                          SmartDashboard.putBoolean("Match/Status/ReadyMotivators", motivatorReady);
                          SmartDashboard.putBoolean("Match/Status/ReadyAll", allReady);
                          return allReady;
                        }),
                    Commands.waitSeconds(0.1),
                    Commands.waitUntil(
                        () -> {
                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.isMotivatorAtSetpoint();
                          return launcherReady && motivatorReady;
                        })),
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
                        () -> motivator.setMotivatorVelocity(motivatorVelocityRPM.get()), motivator)
                    : Commands.none(),

                // Fire balls repeatedly in simulation
                createFiringLoop(coordinator, launcher)))
        .finallyDo(
            () -> {
              launcher.setFeedingActive(false);
              launcher.stop();
              coordinator.disableLaunchMode();
              if (motivator != null) {
                motivator.stopMotivator();
              }
              SmartDashboard.putString("Match/Status/State", "Stopped");
              System.out.println("[Launch] Stopped");
            })
        .withName("Launch");
  }

  /**
   * Creates the firing loop that repeatedly launches fuel balls.
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

  /**
   * Manual test launch command using BenchTest/Shooting/* dashboard values.
   *
   * @param launcher The launcher subsystem
   * @param coordinator The shooting coordinator
   * @param motivator The motivator subsystem (can be null)
   * @param hood The hood subsystem (can be null)
   * @return Command that positions, spins up, and fires while held
   */
  public static Command testLaunchCommand(
      Launcher launcher, ShootingCoordinator coordinator, Motivator motivator, Hood hood) {
    return Commands.sequence(
            Commands.runOnce(() -> setMode(ShootingMode.TEST)),

            // Reset metrics at start of each run
            Commands.runOnce(() -> BenchTestMetrics.getInstance().reset()),

            // Phase 1: Start positioning turret and hood, spin up wheels
            Commands.runOnce(
                () -> {
                  coordinator.enableLaunchMode();

                  double turretAngle = testOutsideTurretAngleDeg.get();
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
                              motivator == null || motivator.isMotivatorAtSetpoint();
                          boolean turretReady = coordinator.turretAtTarget();
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
                    Commands.waitSeconds(0.1),
                    Commands.waitUntil(
                        () -> {
                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.isMotivatorAtSetpoint();
                          boolean turretReady = coordinator.turretAtTarget();
                          boolean hoodReady = hood == null || hood.atTarget();
                          return launcherReady && motivatorReady && turretReady && hoodReady;
                        })),
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
                      double turretAngle = testOutsideTurretAngleDeg.get();
                      launcher.setVelocity(rpm);
                      coordinator.setManualShotParameters(rpm, hoodAngle, turretAngle);
                      SmartDashboard.putNumber("Match/Status/CurrentRPM", launcher.getVelocity());
                    },
                    launcher),

                // Keep turret positioned
                Commands.run(() -> coordinator.setTurretAngle(testOutsideTurretAngleDeg.get())),

                // Keep hood positioned
                hood != null
                    ? Commands.run(() -> hood.setAngle(testHoodAngleDeg.get()), hood)
                    : Commands.none(),

                // Keep motivator feeders running AND add prefeed
                motivator != null
                    ? Commands.run(
                        () -> motivator.setMotivatorVelocity(testMotivatorRPM.get()), motivator)
                    : Commands.none(),

                // Fire balls repeatedly with metrics recording
                createBenchTestFiringLoop(coordinator, launcher)))
        .finallyDo(
            () -> {
              launcher.setFeedingActive(false);
              launcher.stop();
              if (motivator != null) {
                motivator.stopMotivator();
              }
              coordinator.clearManualShotParameters();
              coordinator.disableLaunchMode();
              setMode(ShootingMode.COMPETITION);
              SmartDashboard.putString("Match/Status/State", "Stopped");
              System.out.println("[ManualFire] Stopped");
            })
        .withName("ManualFire");
  }

  /** Hub shot — close-range fixed-position launch into the hub. */
  public static Command hubShotCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {
    return fixedPositionLaunchCommand(
        launcher,
        coordinator,
        motivator,
        turret,
        hood,
        spindexer,
        HUB_SHOT_LAUNCHER_RPM,
        HUB_SHOT_HOOD_ANGLE_DEG,
        HUB_SHOT_TURRET_ANGLE_DEG);
  }

  /** Left trench — fixed-position launch toward the left trench. */
  public static Command leftTrenchShotCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {
    return fixedPositionLaunchCommand(
        launcher,
        coordinator,
        motivator,
        turret,
        hood,
        spindexer,
        LEFT_TRENCH_LAUNCHER_RPM,
        LEFT_TRENCH_HOOD_ANGLE_DEG,
        LEFT_TRENCH_TURRET_ANGLE_DEG);
  }

  /** Right trench — fixed-position launch toward the right trench. */
  public static Command rightTrenchShotCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {
    return fixedPositionLaunchCommand(
        launcher,
        coordinator,
        motivator,
        turret,
        hood,
        spindexer,
        RIGHT_TRENCH_LAUNCHER_RPM,
        RIGHT_TRENCH_HOOD_ANGLE_DEG,
        RIGHT_TRENCH_TURRET_ANGLE_DEG);
  }

  /**
   * Fixed-position launch command. Moves all subsystems to hardcoded setpoints in parallel, waits
   * for ready (with 5s timeout), then feeds via spindexer. Parameterized so multiple field
   * positions can reuse this with different values.
   *
   * @param launcher The launcher subsystem
   * @param coordinator The shooting coordinator
   * @param motivator The motivator subsystem (can be null)
   * @param turret The turret subsystem
   * @param hood The hood subsystem (can be null)
   * @param spindexer The spindexer subsystem (can be null)
   * @param launcherRPM Target launcher RPM
   * @param hoodAngleDeg Target hood angle in degrees
   * @param turretAngleDeg Target outside turret angle in degrees
   * @return Command that positions, spins up, feeds, and fires while held
   */
  public static Command fixedPositionLaunchCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer,
      double launcherRPM,
      double hoodAngleDeg,
      double turretAngleDeg) {
    return Commands.sequence(
            Commands.runOnce(() -> setMode(ShootingMode.TEST)),
            Commands.runOnce(() -> BenchTestMetrics.getInstance().reset()),

            // Phase 1: Spin up and position all subsystems in parallel
            Commands.runOnce(
                () -> {
                  if (coordinator != null) {
                    coordinator.enableLaunchMode();
                  }

                  turret.setOutsideTurretAngle(turretAngleDeg);

                  if (hood != null) {
                    hood.setAngle(hoodAngleDeg);
                  }

                  launcher.setVelocity(launcherRPM);

                  if (coordinator != null) {
                    coordinator.setManualShotParameters(launcherRPM, hoodAngleDeg, turretAngleDeg);
                  }

                  if (motivator != null) {
                    motivator.setMotivatorVelocity(motivatorVelocityRPM.get());
                  }

                  SmartDashboard.putString("Match/Status/State", "Positioning & Spinning Up");
                  System.out.println(
                      "[HubShot] Positioning turret to "
                          + turretAngleDeg
                          + "° and spinning up to "
                          + launcherRPM
                          + " RPM");
                }),

            // Phase 2: Wait for everything to reach setpoint (with 5s timeout)
            Commands.race(
                Commands.sequence(
                    Commands.waitUntil(
                        () -> {
                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.isMotivatorAtSetpoint();
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
                    Commands.waitSeconds(0.1),
                    Commands.waitUntil(
                        () -> {
                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.isMotivatorAtSetpoint();
                          boolean turretReady = turret.atTarget();
                          boolean hoodReady = hood == null || hood.atTarget();
                          return launcherReady && motivatorReady && turretReady && hoodReady;
                        })),
                Commands.sequence(
                    Commands.waitSeconds(5.0),
                    Commands.runOnce(
                        () -> {
                          System.out.println(
                              "[HubShot] WARNING: Setup timeout - continuing anyway!");
                          SmartDashboard.putString(
                              "Match/Status/State", "TIMEOUT - continuing anyway");
                        }))),

            // Log ready state
            Commands.runOnce(
                () -> {
                  SmartDashboard.putString("Match/Status/State", "Ready - Feeding");
                  System.out.println("[HubShot] All mechanisms ready, starting feed");
                  launcher.setFeedingActive(true);
                }),

            // Phase 3: Feed via spindexer while keeping all subsystems running
            Commands.parallel(
                // Keep launcher at speed
                Commands.run(
                    () -> {
                      launcher.setVelocity(launcherRPM);
                      if (coordinator != null) {
                        coordinator.setManualShotParameters(
                            launcherRPM, hoodAngleDeg, turretAngleDeg);
                      }
                      SmartDashboard.putNumber("Match/Status/CurrentRPM", launcher.getVelocity());
                    },
                    launcher),

                // Keep turret positioned
                Commands.run(() -> turret.setOutsideTurretAngle(turretAngleDeg), turret),

                // Keep hood positioned
                hood != null
                    ? Commands.run(() -> hood.setAngle(hoodAngleDeg), hood)
                    : Commands.none(),

                // Keep motivator running
                motivator != null
                    ? Commands.run(
                        () -> motivator.setMotivatorVelocity(motivatorVelocityRPM.get()), motivator)
                    : Commands.none(),

                // Run spindexer to feed fuel
                spindexer != null
                    ? Commands.run(
                        () -> spindexer.setSpindexerVelocity(spindexerFeedRPM.get()), spindexer)
                    : Commands.none(),

                // Fire balls in simulation
                coordinator != null
                    ? createBenchTestFiringLoop(coordinator, launcher)
                    : Commands.none()))
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
              if (coordinator != null) {
                coordinator.clearManualShotParameters();
                coordinator.disableLaunchMode();
              }
              setMode(ShootingMode.COMPETITION);
              SmartDashboard.putString("Match/Status/State", "Stopped");
              System.out.println("[HubShot] Stopped");
            })
        .withName("HubShot");
  }

  /**
   * Smart launch command. Continuously reads shot parameters from the ShootingCoordinator's
   * odometry-based calculations (hub or pass depending on field position) and commands all
   * subsystems accordingly. The coordinator calculates, this command executes.
   *
   * @param launcher The launcher subsystem
   * @param coordinator The shooting coordinator (provides shot calculations)
   * @param motivator The motivator subsystem (can be null)
   * @param turret The turret subsystem
   * @param hood The hood subsystem (can be null)
   * @param spindexer The spindexer subsystem (can be null)
   * @return Command that aims and fires based on odometry while held
   */
  public static Command smartLaunchCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {
    return Commands.sequence(
            Commands.runOnce(() -> setMode(ShootingMode.COMPETITION)),

            // Phase 1: Start subsystems using initial shot calculation
            Commands.runOnce(
                () -> {
                  coordinator.enableLaunchMode();

                  ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                  if (shot != null) {
                    double rpm = ShotCalculator.calculateRPMForVelocity(shot.exitVelocityMps());
                    launcher.setVelocity(rpm);
                    turret.setOutsideTurretAngle(shot.turretAngleDeg());
                    if (hood != null) {
                      hood.setAngle(shot.hoodAngleDeg());
                    }
                  }

                  if (motivator != null) {
                    motivator.setMotivatorVelocity(motivatorVelocityRPM.get());
                  }

                  SmartDashboard.putString("Match/Status/State", "Smart Launch - Positioning");
                  System.out.println("[SmartLaunch] Starting odometry-based launch");
                }),

            // Phase 2: Wait for all subsystems to reach setpoint (with 5s timeout)
            Commands.race(
                Commands.sequence(
                    Commands.waitUntil(
                        () -> {
                          // Continuously update targets while waiting
                          ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                          if (shot != null) {
                            double rpm =
                                ShotCalculator.calculateRPMForVelocity(shot.exitVelocityMps());
                            launcher.setVelocity(rpm);
                            turret.setOutsideTurretAngle(shot.turretAngleDeg());
                            if (hood != null) {
                              hood.setAngle(shot.hoodAngleDeg());
                            }
                            coordinator.setManualShotParameters(
                                rpm, shot.hoodAngleDeg(), shot.turretAngleDeg());
                          }

                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.isMotivatorAtSetpoint();
                          boolean turretReady = turret.atTarget();
                          boolean hoodReady = hood == null || hood.atTarget();
                          boolean hasShot = shot != null && shot.achievable();

                          boolean allReady =
                              launcherReady
                                  && motivatorReady
                                  && turretReady
                                  && hoodReady
                                  && hasShot;

                          SmartDashboard.putBoolean("Match/Status/ReadyLauncher", launcherReady);
                          SmartDashboard.putBoolean("Match/Status/ReadyMotivators", motivatorReady);
                          SmartDashboard.putBoolean("Match/Status/ReadyTurret", turretReady);
                          SmartDashboard.putBoolean("Match/Status/ReadyHood", hoodReady);
                          SmartDashboard.putBoolean("Match/Status/ReadyAchievable", hasShot);
                          SmartDashboard.putBoolean("Match/Status/ReadyAll", allReady);
                          if (shot != null) {
                            SmartDashboard.putNumber(
                                "Match/Status/SmartLaunchRPM",
                                ShotCalculator.calculateRPMForVelocity(shot.exitVelocityMps()));
                            SmartDashboard.putNumber(
                                "Match/Status/SmartLaunchHoodDeg", shot.hoodAngleDeg());
                            SmartDashboard.putNumber(
                                "Match/Status/SmartLaunchTurretDeg", shot.turretAngleDeg());
                            SmartDashboard.putNumber(
                                "Match/Status/SmartLaunchExitVel", shot.exitVelocityMps());
                          }

                          return allReady;
                        }),
                    Commands.waitSeconds(0.1),
                    Commands.waitUntil(
                        () -> {
                          ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.isMotivatorAtSetpoint();
                          boolean turretReady = turret.atTarget();
                          boolean hoodReady = hood == null || hood.atTarget();
                          boolean hasShot = shot != null && shot.achievable();
                          return launcherReady
                              && motivatorReady
                              && turretReady
                              && hoodReady
                              && hasShot;
                        })),
                Commands.sequence(
                    Commands.waitSeconds(5.0),
                    Commands.runOnce(
                        () -> {
                          ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.isMotivatorAtSetpoint();
                          boolean turretReady = turret.atTarget();
                          boolean hoodReady = hood == null || hood.atTarget();
                          boolean hasShot = shot != null && shot.achievable();
                          System.out.println(
                              "[SmartLaunch] WARNING: Setup timeout! Conditions: "
                                  + "launcher="
                                  + launcherReady
                                  + " motivator="
                                  + motivatorReady
                                  + " turret="
                                  + turretReady
                                  + " hood="
                                  + hoodReady
                                  + " achievable="
                                  + hasShot
                                  + (shot != null
                                      ? " rpm="
                                          + ShotCalculator.calculateRPMForVelocity(
                                              shot.exitVelocityMps())
                                          + " hood="
                                          + shot.hoodAngleDeg()
                                          + " turret="
                                          + shot.turretAngleDeg()
                                      : " shot=null"));
                          SmartDashboard.putString(
                              "Match/Status/State", "TIMEOUT - continuing anyway");
                        }))),

            // Log ready state
            Commands.runOnce(
                () -> {
                  SmartDashboard.putString("Match/Status/State", "Smart Launch - Feeding");
                  System.out.println("[SmartLaunch] Ready, starting feed");
                  launcher.setFeedingActive(true);
                }),

            // Phase 3: Continuously update from odometry while feeding
            Commands.parallel(
                // Keep launcher + turret + hood tracking the shot
                Commands.run(
                    () -> {
                      ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                      if (shot != null) {
                        double rpm = ShotCalculator.calculateRPMForVelocity(shot.exitVelocityMps());
                        launcher.setVelocity(rpm);
                        coordinator.setManualShotParameters(
                            rpm, shot.hoodAngleDeg(), shot.turretAngleDeg());
                        SmartDashboard.putNumber("Match/Status/CurrentRPM", launcher.getVelocity());
                      }
                    },
                    launcher),

                // Keep turret tracking
                Commands.run(
                    () -> {
                      ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                      if (shot != null) {
                        turret.setOutsideTurretAngle(shot.turretAngleDeg());
                      }
                    },
                    turret),

                // Keep hood tracking
                hood != null
                    ? Commands.run(
                        () -> {
                          ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                          if (shot != null) {
                            hood.setAngle(shot.hoodAngleDeg());
                          }
                        },
                        hood)
                    : Commands.none(),

                // Keep motivator running
                motivator != null
                    ? Commands.run(
                        () -> motivator.setMotivatorVelocity(motivatorVelocityRPM.get()), motivator)
                    : Commands.none(),

                // Run spindexer to feed fuel
                spindexer != null
                    ? Commands.run(
                        () -> spindexer.setSpindexerVelocity(spindexerFeedRPM.get()), spindexer)
                    : Commands.none(),

                // Fire balls in simulation
                createBenchTestFiringLoop(coordinator, launcher)))
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
              setMode(ShootingMode.COMPETITION);
              SmartDashboard.putString("Match/Status/State", "Stopped");
              System.out.println("[SmartLaunch] Stopped");
            })
        .withName("SmartLaunch");
  }

  /**
   * Bench test launch command. Stripped-down version that only waits for launcher + motivator.
   *
   * @param launcher The launcher subsystem
   * @param coordinator The shooting coordinator
   * @param motivator The motivator subsystem (can be null)
   * @return Command that launches while held
   */
  public static Command benchTestLaunchCommand(
      Launcher launcher, ShootingCoordinator coordinator, Motivator motivator) {
    return Commands.sequence(
            Commands.runOnce(() -> setMode(ShootingMode.TEST)),

            // Reset metrics at start of each run
            Commands.runOnce(() -> BenchTestMetrics.getInstance().reset()),

            // Phase 1: Spin up launcher and motivator feeders (no prefeed yet)
            Commands.runOnce(
                () -> {
                  double targetRPM = testLauncherRPM.get();
                  launcher.setVelocity(targetRPM);
                  coordinator.enableLaunchMode();
                  if (motivator != null) {
                    motivator.setMotivatorVelocity(testMotivatorRPM.get());
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
                              motivator == null || motivator.isMotivatorAtSetpoint();
                          return launcherReady && motivatorReady;
                        }),
                    Commands.waitSeconds(0.1),
                    Commands.waitUntil(
                        () -> {
                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.isMotivatorAtSetpoint();
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
                Commands.run(
                    () -> {
                      launcher.setVelocity(testLauncherRPM.get());
                      SmartDashboard.putNumber("Match/Status/CurrentRPM", launcher.getVelocity());
                    },
                    launcher),
                motivator != null
                    ? Commands.run(
                        () -> motivator.setMotivatorVelocity(testMotivatorRPM.get()), motivator)
                    : Commands.none(),
                createBenchTestFiringLoop(coordinator, launcher)))
        .finallyDo(
            () -> {
              launcher.setFeedingActive(false);
              launcher.stop();
              coordinator.disableLaunchMode();
              if (motivator != null) {
                motivator.stopMotivator();
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
   * @param coordinator The shooting coordinator
   * @param launcher The launcher subsystem
   * @return Command that fires repeatedly and records metrics
   */
  private static Command createBenchTestFiringLoop(
      ShootingCoordinator coordinator, Launcher launcher) {
    return Commands.sequence(
            Commands.waitUntil(launcher::atSetpoint),
            Commands.runOnce(
                () -> {
                  coordinator.launchFuel();
                  launcher.notifyBallFired();
                  BenchTestMetrics.getInstance().recordShot();
                  Logger.recordOutput("Match/ShotLog/LastShotTime", Timer.getFPGATimestamp());

                  ShotVisualizer visualizer = coordinator.getVisualizer();
                  int fuelRemaining = visualizer != null ? visualizer.getFuelCount() : 0;
                  Logger.recordOutput("Match/ShotLog/FuelRemaining", fuelRemaining);
                }),
            Commands.waitSeconds(MIN_SHOT_INTERVAL_SECONDS))
        .repeatedly()
        .until(
            () -> {
              ShotVisualizer visualizer = coordinator.getVisualizer();
              return visualizer == null || visualizer.getFuelCount() <= 0;
            });
  }
}
