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
 *   <li>COMPETITION: Auto-calculates optimal trajectory to hub (Shooting/Auto/*)
 *   <li>TEST: Uses manual Shooting/Test/* dashboard values for controlled testing
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
    /** Auto-calculated trajectory to hub, optimized RPM/angle for distance (Shooting/Auto/*). */
    COMPETITION,
    /** Manual parameters from Shooting/Test/* dashboard values. */
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
      SmartDashboard.putString("Shooting/Status/Mode", mode.toString());
      SmartDashboard.putBoolean("Shooting/Status/Active", mode == ShootingMode.TEST);
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
      new LoggedTunableNumber("Shooting/Auto/LaunchVelocityRPM", 1700.0);

  private static final LoggedTunableNumber motivatorVelocityRPM =
      new LoggedTunableNumber("Shooting/Auto/MotivatorVelocityRPM", 1000.0);

  private static final LoggedTunableNumber prefeedVelocityRPM =
      new LoggedTunableNumber("Shooting/Auto/PrefeedVelocityRPM", 1000.0);

  // ===== Shooting/Test/* Override Values (for controlled manual testing) =====
  // These allow manually setting all mechanism parameters while obeying safety limits

  private static final LoggedTunableNumber testLauncherRPM =
      new LoggedTunableNumber("Shooting/Test/LauncherRPM", 1700.0);

  private static final LoggedTunableNumber testMotivatorRPM =
      new LoggedTunableNumber("Shooting/Test/MotivatorRPM", 1000.0);

  private static final LoggedTunableNumber testPrefeedRPM =
      new LoggedTunableNumber("Shooting/Test/PrefeedRPM", 1000.0);

  private static final LoggedTunableNumber testTurretAngleDeg =
      new LoggedTunableNumber("Shooting/Test/AngleDegTurret", 0.0);

  private static final LoggedTunableNumber testHoodAngleDeg =
      new LoggedTunableNumber("Shooting/Test/AngleDegHood", 45.0);

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
    SmartDashboard.putString("Shooting/Status/Mode", currentMode.toString());
    SmartDashboard.putBoolean("Shooting/Status/Active", currentMode == ShootingMode.TEST);
    SmartDashboard.putNumber("Shooting/Status/CurrentRPM", 0.0);
    SmartDashboard.putBoolean("Shooting/Status/ReadyAll", false);
    SmartDashboard.putBoolean("Shooting/Status/ReadyHood", false);
    SmartDashboard.putBoolean("Shooting/Status/ReadyLauncher", false);
    SmartDashboard.putBoolean("Shooting/Status/ReadyMotivators", false);
    SmartDashboard.putBoolean("Shooting/Status/ReadyTurret", false);
    SmartDashboard.putString("Shooting/Status/State", "Idle");
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
                  if (motivator != null) {
                    // Only spin up the two feeder wheels, not prefeed
                    motivator.setMotivatorsVelocity(motivatorVelocityRPM.get());
                  }
                  SmartDashboard.putString("Shooting/Status/State", "Spinning Up");
                  Logger.recordOutput("Shooting/TargetRPM", targetRPM);
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
                          SmartDashboard.putBoolean("Shooting/Status/ReadyLauncher", launcherReady);
                          SmartDashboard.putBoolean(
                              "Shooting/Status/ReadyMotivators", motivatorReady);
                          SmartDashboard.putBoolean("Shooting/Status/ReadyAll", allReady);
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
                  SmartDashboard.putString("Shooting/Status/State", "Firing");
                  System.out.println("[Launch] At setpoint, starting prefeed and firing");
                  launcher.setFeedingActive(true);
                }),

            // Phase 3: Start prefeed and fire repeatedly
            Commands.parallel(
                // Keep launcher at speed continuously
                Commands.run(
                    () -> {
                      launcher.setVelocity(launchVelocityRPM.get());
                      SmartDashboard.putNumber(
                          "Shooting/Status/CurrentRPM", launcher.getVelocity());
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
              if (motivator != null) {
                motivator.stop();
              }
              SmartDashboard.putString("Shooting/Status/State", "Stopped");
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
                  Logger.recordOutput("Shooting/LastShotTime", Timer.getFPGATimestamp());

                  // Notify launcher of shot (triggers recovery in sim, no-op on real robot)
                  launcher.notifyBallFired();

                  // Log fuel count
                  TurretVisualizer visualizer = turret.getVisualizer();
                  int fuelRemaining = visualizer != null ? visualizer.getFuelCount() : 0;
                  Logger.recordOutput("Shooting/FuelRemaining", fuelRemaining);
                }),

            // Small delay to prevent multiple fires in same frame
            Commands.waitSeconds(MIN_SHOT_INTERVAL_SECONDS))
        .repeatedly()
        .until(
            () -> {
              TurretVisualizer visualizer = turret.getVisualizer();
              boolean outOfFuel = visualizer == null || visualizer.getFuelCount() <= 0;
              if (outOfFuel) {
                Logger.recordOutput("Shooting/State", "Out of Fuel");
                System.out.println("[Launch] Out of fuel");
              }
              return outOfFuel;
            });
  }

  /**
   * Command to reset the simulation for testing. Clears all balls from the field and puts 50 in the
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

              // Put 50 balls in hopper
              TurretVisualizer visualizer = turret.getVisualizer();
              if (visualizer != null) {
                visualizer.setFuelCount(50);
                Logger.recordOutput("Shooting/FuelRemaining", 50);
              }

              Logger.recordOutput("Shooting/SimReset", true);
              System.out.println("[Shooting] Simulation reset: field cleared, 50 balls in hopper");
            })
        .ignoringDisable(true)
        .withName("Reset Simulation");
  }

  /**
   * Manual test launch command using Shooting/Test/* dashboard values. Positions all mechanisms to
   * the specified values (obeying safety limits), waits for everything to reach setpoint, then
   * fires.
   *
   * <p>Use this for controlled testing where you want to specify exact values for:
   *
   * <ul>
   *   <li>Launcher RPM (Shooting/Test/LauncherRPM)
   *   <li>Motivator RPM (Shooting/Test/MotivatorRPM)
   *   <li>Prefeed RPM (Shooting/Test/PrefeedRPM)
   *   <li>Turret angle in degrees (Shooting/Test/AngleDegTurret)
   *   <li>Hood angle in degrees (Shooting/Test/AngleDegHood)
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

            // Phase 1: Start positioning turret and hood, spin up wheels
            Commands.runOnce(
                () -> {
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

                  SmartDashboard.putString("Shooting/Status/State", "Positioning & Spinning Up");
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

                          SmartDashboard.putBoolean("Shooting/Status/ReadyLauncher", launcherReady);
                          SmartDashboard.putBoolean(
                              "Shooting/Status/ReadyMotivators", motivatorReady);
                          SmartDashboard.putBoolean("Shooting/Status/ReadyTurret", turretReady);
                          SmartDashboard.putBoolean("Shooting/Status/ReadyHood", hoodReady);
                          SmartDashboard.putBoolean("Shooting/Status/ReadyAll", allReady);

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
                              "Shooting/Status/State", "TIMEOUT - continuing anyway");
                        }))),

            // Log ready state
            Commands.runOnce(
                () -> {
                  SmartDashboard.putString("Shooting/Status/State", "Ready - Starting Prefeed");
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
                      SmartDashboard.putNumber(
                          "Shooting/Status/CurrentRPM", launcher.getVelocity());
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

                // Fire balls repeatedly
                createFiringLoop(turret, launcher)))
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
              // Clear manual shot parameters and return to competition mode
              turret.clearManualShotParameters();
              setMode(ShootingMode.COMPETITION);
              SmartDashboard.putString("Shooting/Status/State", "Stopped");
              System.out.println("[ManualFire] Stopped");
            })
        .withName("ManualFire");
  }
}
