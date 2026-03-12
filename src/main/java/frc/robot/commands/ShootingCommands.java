package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.motivator.Motivator;
import frc.robot.subsystems.shooting.ShootingCoordinator;
import frc.robot.subsystems.shooting.ShotCalculator;
import frc.robot.subsystems.shooting.ShotVisualizer;
import frc.robot.subsystems.shooting.StationaryShotBatchRecorder;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.BenchTestMetrics;
import frc.robot.util.FuelSim;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Factory class for shooting commands. Provides a unified launch command that works for both
 * simulation (spawns fuel balls) and physical robot (runs motivator).
 *
 * <p>Supports two modes:
 *
 * <ul>
 *   <li>COMPETITION: Auto-calculates optimal trajectory to hub (Shots/*)
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
    /** Auto-calculated trajectory to hub, optimized RPM/angle for distance (Shots/*). */
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

  // ===== Fixed Shot Presets (tunable from dashboard) =====

  // Hub shot — close-range shot into the hub
  private static final LoggedTunableNumber hubShotLauncherRPM =
      new LoggedTunableNumber("Shots/HubShot/LauncherRPM", 2250.0);
  private static final LoggedTunableNumber hubShotHoodAngleDeg =
      new LoggedTunableNumber("Shots/HubShot/HoodAngleDeg", 13.0);
  private static final LoggedTunableNumber hubShotTurretAngleDeg =
      new LoggedTunableNumber("Shots/HubShot/TurretAngleDeg", -90);
  private static final LoggedTunableNumber hubShotMotivatorRPM =
      new LoggedTunableNumber("Shots/HubShot/MotivatorRPM", 1100.0);
  private static final LoggedTunableNumber hubShotSpindexerRPM =
      new LoggedTunableNumber("Shots/HubShot/SpindexerRPM", 350.0);

  // Left trench shot
  private static final LoggedTunableNumber leftTrenchLauncherRPM =
      new LoggedTunableNumber("Shots/LeftTrench/LauncherRPM", 2500.0);
  private static final LoggedTunableNumber leftTrenchHoodAngleDeg =
      new LoggedTunableNumber("Shots/LeftTrench/HoodAngleDeg", 26.0);
  private static final LoggedTunableNumber leftTrenchTurretAngleDeg =
      new LoggedTunableNumber("Shots/LeftTrench/TurretAngleDeg", 208.0);
  private static final LoggedTunableNumber leftTrenchMotivatorRPM =
      new LoggedTunableNumber("Shots/LeftTrench/MotivatorRPM", 1800.0);
  private static final LoggedTunableNumber leftTrenchSpindexerRPM =
      new LoggedTunableNumber("Shots/LeftTrench/SpindexerRPM", 325.0);

  // Right trench shot
  private static final LoggedTunableNumber rightTrenchLauncherRPM =
      new LoggedTunableNumber("Shots/RightTrench/LauncherRPM", 2500.0);
  private static final LoggedTunableNumber rightTrenchHoodAngleDeg =
      new LoggedTunableNumber("Shots/RightTrench/HoodAngleDeg", 26.0);
  private static final LoggedTunableNumber rightTrenchTurretAngleDeg =
      new LoggedTunableNumber("Shots/RightTrench/TurretAngleDeg", -25.0);
  private static final LoggedTunableNumber rightTrenchMotivatorRPM =
      new LoggedTunableNumber("Shots/RightTrench/MotivatorRPM", 1800.0);
  private static final LoggedTunableNumber rightTrenchSpindexerRPM =
      new LoggedTunableNumber("Shots/RightTrench/SpindexerRPM", 325.0);

  // ===== Robot Tuning (affects real robot behavior) =====

  // Target velocities for the basic launch command
  private static final LoggedTunableNumber launchVelocityRPM =
      new LoggedTunableNumber("Match/Shooting/LaunchVelocityRPM", 1700.0);

  private static final LoggedTunableNumber motivatorVelocityRPM =
      new LoggedTunableNumber("Match/Shooting/MotivatorVelocityRPM", 1100.0);

  private static final LoggedTunableNumber spindexerFeedRPM =
      new LoggedTunableNumber("Match/Shooting/SpindexerFeedRPM", 750.0);

  // Smart shot motivator/spindexer speeds (separate from basic launch)
  private static final LoggedTunableNumber smartShotMotivatorRPM =
      new LoggedTunableNumber("Shots/SmartLaunch/MotivatorRPM", 1800.0);
  private static final LoggedTunableNumber smartShotSpindexerRPM =
      new LoggedTunableNumber("Shots/SmartLaunch/SpindexerRPM", 325.0);

  // Max drive speed (m/s) while smart launch speed-limit mode is active
  private static final LoggedTunableNumber smartLaunchSpeedLimitCapMps =
      new LoggedTunableNumber("Shots/SmartLaunch/SpeedLimitCapMps", 1.5);

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
    // Basic launch tunables
    launchVelocityRPM.get();
    motivatorVelocityRPM.get();
    spindexerFeedRPM.get();

    // Fixed shot preset tunables
    hubShotLauncherRPM.get();
    hubShotHoodAngleDeg.get();
    hubShotTurretAngleDeg.get();
    hubShotMotivatorRPM.get();
    hubShotSpindexerRPM.get();

    leftTrenchLauncherRPM.get();
    leftTrenchHoodAngleDeg.get();
    leftTrenchTurretAngleDeg.get();
    leftTrenchMotivatorRPM.get();
    leftTrenchSpindexerRPM.get();

    rightTrenchLauncherRPM.get();
    rightTrenchHoodAngleDeg.get();
    rightTrenchTurretAngleDeg.get();
    rightTrenchMotivatorRPM.get();
    rightTrenchSpindexerRPM.get();

    // Smart shot tunables
    smartShotMotivatorRPM.get();
    smartShotSpindexerRPM.get();

    // BenchTest tunables
    testLauncherRPM.get();
    testMotivatorRPM.get();
    testOutsideTurretAngleDeg.get();
    testInsideTurretAngleDeg.get();
    testTurretVolts.get();
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
                Commands.run(() -> launcher.setVelocity(launchVelocityRPM.get()), launcher),

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
        hubShotLauncherRPM::get,
        hubShotHoodAngleDeg::get,
        hubShotTurretAngleDeg::get,
        hubShotMotivatorRPM::get,
        hubShotSpindexerRPM::get);
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
        leftTrenchLauncherRPM::get,
        leftTrenchHoodAngleDeg::get,
        leftTrenchTurretAngleDeg::get,
        leftTrenchMotivatorRPM::get,
        leftTrenchSpindexerRPM::get);
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
        rightTrenchLauncherRPM::get,
        rightTrenchHoodAngleDeg::get,
        rightTrenchTurretAngleDeg::get,
        rightTrenchMotivatorRPM::get,
        rightTrenchSpindexerRPM::get);
  }

  /**
   * Fixed-position launch command. Moves all subsystems to tunable setpoints in parallel, waits for
   * ready (with 5s timeout), then feeds via spindexer. All parameters are suppliers so dashboard
   * tunables take effect immediately without redeploying.
   *
   * @param launcher The launcher subsystem
   * @param coordinator The shooting coordinator
   * @param motivator The motivator subsystem (can be null)
   * @param turret The turret subsystem
   * @param hood The hood subsystem (can be null)
   * @param spindexer The spindexer subsystem (can be null)
   * @param launcherRPMSupplier Supplier for target launcher RPM
   * @param hoodAngleDegSupplier Supplier for target hood angle in degrees
   * @param turretAngleDegSupplier Supplier for target outside turret angle in degrees
   * @param motivatorRPMSupplier Supplier for target motivator RPM
   * @param spindexerRPMSupplier Supplier for target spindexer RPM
   * @return Command that positions, spins up, feeds, and fires while held
   */
  public static Command fixedPositionLaunchCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer,
      DoubleSupplier launcherRPMSupplier,
      DoubleSupplier hoodAngleDegSupplier,
      DoubleSupplier turretAngleDegSupplier,
      DoubleSupplier motivatorRPMSupplier,
      DoubleSupplier spindexerRPMSupplier) {
    return Commands.sequence(
            Commands.runOnce(() -> setMode(ShootingMode.TEST)),
            Commands.runOnce(() -> BenchTestMetrics.getInstance().reset()),

            // Phase 1: Spin up and position all subsystems in parallel
            Commands.runOnce(
                () -> {
                  if (coordinator != null) {}

                  double turretAngle = turretAngleDegSupplier.getAsDouble();
                  double hoodAngle = hoodAngleDegSupplier.getAsDouble();
                  double launcherRPM = launcherRPMSupplier.getAsDouble();

                  turret.setOutsideTurretAngle(turretAngle);

                  if (hood != null) {
                    hood.setHoodAngle(hoodAngle);
                  }

                  launcher.setVelocity(launcherRPM);

                  if (coordinator != null) {
                    coordinator.setManualShotParameters(launcherRPM, hoodAngle, turretAngle);
                  }

                  if (motivator != null) {
                    motivator.setMotivatorVelocity(motivatorRPMSupplier.getAsDouble());
                  }

                  SmartDashboard.putString("Match/Status/State", "Positioning & Spinning Up");
                  System.out.println(
                      "[FixedShot] Positioning turret to "
                          + turretAngle
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

                          return launcherReady && motivatorReady && turretReady && hoodReady;
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
                              "[FixedShot] WARNING: Setup timeout - continuing anyway!");
                          SmartDashboard.putString(
                              "Match/Status/State", "TIMEOUT - continuing anyway");
                        }))),

            // Log ready state
            Commands.runOnce(
                () -> {
                  SmartDashboard.putString("Match/Status/State", "Ready - Feeding");
                  System.out.println("[FixedShot] All mechanisms ready, starting feed");
                  launcher.setFeedingActive(true);
                }),

            // Phase 3: Feed via spindexer while keeping all subsystems running
            Commands.parallel(
                // Keep launcher at speed (reads tunable each cycle)
                Commands.run(
                    () -> {
                      double rpm = launcherRPMSupplier.getAsDouble();
                      double hoodAngle = hoodAngleDegSupplier.getAsDouble();
                      double turretAngle = turretAngleDegSupplier.getAsDouble();
                      launcher.setVelocity(rpm);
                      if (coordinator != null) {
                        coordinator.setManualShotParameters(rpm, hoodAngle, turretAngle);
                      }
                    },
                    launcher),

                // Keep turret positioned (reads tunable each cycle)
                Commands.run(
                    () -> turret.setOutsideTurretAngle(turretAngleDegSupplier.getAsDouble()),
                    turret),

                // Keep hood positioned (reads tunable each cycle)
                hood != null
                    ? Commands.run(
                        () -> hood.setHoodAngle(hoodAngleDegSupplier.getAsDouble()), hood)
                    : Commands.none(),

                // Keep motivator running (gated on turret alignment)
                motivator != null
                    ? Commands.run(
                        () -> {
                          if (turret.atTarget()) {
                            motivator.setMotivatorVelocity(motivatorRPMSupplier.getAsDouble());
                          } else {
                            motivator.stopMotivator();
                          }
                        },
                        motivator)
                    : Commands.none(),

                // Run spindexer to feed fuel (gated on turret alignment)
                spindexer != null
                    ? Commands.run(
                        () -> {
                          if (turret.atTarget()) {
                            spindexer.setSpindexerVelocity(spindexerRPMSupplier.getAsDouble());
                          } else {
                            spindexer.stopSpindexer();
                          }
                          Logger.recordOutput("FixedShot/FeedingSuppressed", !turret.atTarget());
                        },
                        spindexer)
                    : Commands.none(),

                // Fire balls in simulation
                coordinator != null
                    ? createBenchTestFiringLoop(coordinator, launcher, turret)
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
              }
              setMode(ShootingMode.COMPETITION);
              SmartDashboard.putString("Match/Status/State", "Stopped");
              System.out.println("[FixedShot] Stopped");
            })
        .withName("FixedShot");
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
  private static boolean isRobotSlowEnoughToFeed(ShootingCoordinator coordinator) {
    if (coordinator.getFieldSpeedsSupplier() == null) return true;
    edu.wpi.first.math.kinematics.ChassisSpeeds speeds = coordinator.getFieldSpeedsSupplier().get();
    double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    return speed <= coordinator.getMaxFeedSpeedMps();
  }

  public static Command smartLaunchCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {
    return smartLaunchCommand(launcher, coordinator, motivator, turret, hood, spindexer, true);
  }

  /**
   * Core smart launch implementation.
   *
   * @param gateOnSpeed When true, feeding is suppressed if the robot exceeds the max feed speed.
   *     When false, feeding is allowed at any speed (used by speed-limited variant where the driver
   *     already accepts reduced speed).
   */
  private static Command smartLaunchCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer,
      boolean gateOnSpeed) {
    return Commands.sequence(
            Commands.runOnce(() -> setMode(ShootingMode.COMPETITION)),

            // Phase 1: Start subsystems using initial shot calculation
            Commands.runOnce(
                () -> {
                  ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                  if (shot != null) {
                    double rpm = shot.launcherRPM();
                    launcher.setVelocity(rpm);
                    turret.setOutsideTurretAngle(shot.turretAngleDeg());
                    if (hood != null) {
                      hood.setHoodAngle(shot.hoodAngleDeg());
                    }
                  }

                  if (motivator != null) {
                    motivator.setMotivatorVelocity(smartShotMotivatorRPM.get());
                  }

                  // Start a LUT batch — captures fuel count for batch tracking
                  ShotVisualizer vis = coordinator.getVisualizer();
                  if (vis != null) {
                    coordinator.getBatchRecorder().startBatch(vis.getFuelCount());
                  }

                  Logger.recordOutput("SmartLaunch/Active", true);
                  Logger.recordOutput("SmartLaunch/Phase", "Positioning");
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
                              hood.setHoodAngle(shot.hoodAngleDeg());
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
                          boolean robotSlow = !gateOnSpeed || isRobotSlowEnoughToFeed(coordinator);

                          boolean allReady =
                              launcherReady
                                  && motivatorReady
                                  && turretReady
                                  && hoodReady
                                  && hasShot
                                  && robotSlow;

                          Logger.recordOutput("SmartLaunch/Ready/RobotSlow", robotSlow);
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
                          boolean robotSlow = !gateOnSpeed || isRobotSlowEnoughToFeed(coordinator);
                          return launcherReady
                              && motivatorReady
                              && turretReady
                              && hoodReady
                              && hasShot
                              && robotSlow;
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
                          Logger.recordOutput("SmartLaunch/Phase", "Timeout");
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
                                          + shot.launcherRPM()
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
                  Logger.recordOutput("SmartLaunch/Phase", "Feeding");
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
                        double rpm = shot.launcherRPM();
                        launcher.setVelocity(rpm);
                        coordinator.setManualShotParameters(
                            rpm, shot.hoodAngleDeg(), shot.turretAngleDeg());
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
                            hood.setHoodAngle(shot.hoodAngleDeg());
                          }
                        },
                        hood)
                    : Commands.none(),

                // Keep motivator running at target RPM throughout firing phase.
                // The motivator just stages fuel — it doesn't need to be gated on
                // turret alignment or robot speed. Stopping it causes RPM drops
                // that disrupt shots, especially during shoot-on-the-move when the
                // turret is continuously tracking and atTarget() flickers.
                motivator != null
                    ? Commands.run(
                        () -> motivator.setMotivatorVelocity(smartShotMotivatorRPM.get()),
                        motivator)
                    : Commands.none(),

                // Run spindexer to feed fuel (gated on turret alignment + optionally robot speed)
                spindexer != null
                    ? Commands.run(
                        () -> {
                          boolean feedOk =
                              turret.atTarget()
                                  && (!gateOnSpeed || isRobotSlowEnoughToFeed(coordinator));
                          if (feedOk) {
                            spindexer.setSpindexerVelocity(smartShotSpindexerRPM.get());
                          } else {
                            spindexer.stopSpindexer();
                          }
                          Logger.recordOutput("SmartLaunch/FeedingSuppressed", !feedOk);
                        },
                        spindexer)
                    : Commands.none(),

                // Fire balls in simulation (same feed conditions)
                createBenchTestFiringLoop(coordinator, launcher, turret)))
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
              // Drive hood back to stow angle so it doesn't stay raised after
              // event zones end (the PathPlanner auto group holds the hood
              // subsystem requirement, so the HoodStow default command can't run).
              if (hood != null) {
                hood.setHoodAngle(hood.getMinAngle());
              }
              coordinator.clearManualShotParameters();
              setMode(ShootingMode.COMPETITION);
              Logger.recordOutput("SmartLaunch/Active", false);
              Logger.recordOutput("SmartLaunch/Phase", "Idle");
              SmartDashboard.putString("Match/Status/State", "Stopped");
              System.out.println("[SmartLaunch] Stopped");
            })
        .withName("SmartLaunch");
  }

  /**
   * Smart launch with drive speed limiting. Runs the smart launch command in parallel with a
   * speed-limited joystick drive command, so the robot can't exceed the tunable speed limit while
   * shooting on the move. When released, the default drive command resumes automatically.
   *
   * @param launcher The launcher subsystem
   * @param coordinator The shooting coordinator
   * @param motivator The motivator subsystem (can be null)
   * @param turret The turret subsystem
   * @param hood The hood subsystem (can be null)
   * @param spindexer The spindexer subsystem (can be null)
   * @param drive The drive subsystem
   * @param xSupplier Joystick X axis supplier
   * @param ySupplier Joystick Y axis supplier
   * @param omegaSupplier Joystick rotation supplier
   * @return Command that aims, fires, and limits drive speed while held
   */
  public static Command smartLaunchWithSpeedLimitCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer,
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.parallel(
            smartLaunchCommand(launcher, coordinator, motivator, turret, hood, spindexer, false),
            DriveCommands.joystickDriveSpeedLimited(
                drive, xSupplier, ySupplier, omegaSupplier, smartLaunchSpeedLimitCapMps::get))
        .withName("SmartLaunch (Speed Limited)");
  }

  /**
   * Auto-tracking command. Continuously aims turret and hood at the calculated target based on
   * odometry. When used as a toggle, keeps the turret/hood pre-aimed so that firing commands
   * (smartLaunch, hub shot, etc.) can skip the positioning phase — they interrupt this command via
   * subsystem requirements, and it resumes automatically when they end.
   *
   * @param coordinator The shooting coordinator (provides shot calculations)
   * @param turret The turret subsystem
   * @param hood The hood subsystem (can be null)
   * @return Command that continuously tracks the target while active
   */
  public static Command autoTrackCommand(
      ShootingCoordinator coordinator, Turret turret, Hood hood) {
    return Commands.run(
            () -> {
              ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
              if (shot != null) {
                // Command turret only — hood stays on its default (stow) command
                turret.setOutsideTurretAngle(shot.turretAngleDeg());

                // Auto-track specific: aim readiness (turret only, hood handled by fire commands)
                boolean turretReady = turret.atTarget();
                boolean aimReady = turretReady && shot.achievable();
                Logger.recordOutput("SmartLaunch/AutoTrack/AimReady", aimReady);

                SmartDashboard.putString(
                    "Match/Status/AutoTrackAimMode",
                    shot.achievable() ? "Tracking" : "Out of Range");
                SmartDashboard.putBoolean("Match/Status/AutoTrackAimReady", aimReady);
              } else {
                Logger.recordOutput("SmartLaunch/AutoTrack/AimReady", false);
              }
            },
            turret)
        .beforeStarting(
            () -> {
              Logger.recordOutput("SmartLaunch/AutoTrack/Active", true);
              SmartDashboard.putBoolean("Match/Status/AutoTracking", true);
              System.out.println("[AutoTrack] Started — turret tracking target");
            })
        .finallyDo(
            () -> {
              Logger.recordOutput("SmartLaunch/AutoTrack/Active", false);
              Logger.recordOutput("SmartLaunch/AutoTrack/AimReady", false);
              SmartDashboard.putBoolean("Match/Status/AutoTracking", false);
              SmartDashboard.putBoolean("Match/Status/AutoTrackAimReady", false);
              SmartDashboard.putString("Match/Status/AutoTrackAimMode", "Off");
              System.out.println("[AutoTrack] Stopped");
            })
        .withName("AutoTrack");
  }

  /**
   * Firing loop for bench testing with BenchTestMetrics integration.
   *
   * @param coordinator The shooting coordinator
   * @param launcher The launcher subsystem
   * @return Command that fires repeatedly and records metrics
   */
  private static Command createBenchTestFiringLoop(
      ShootingCoordinator coordinator, Launcher launcher, Turret turret) {
    return Commands.sequence(
            Commands.waitUntil(
                () ->
                    turret.atTarget()
                        && launcher.atSetpoint()
                        && isRobotSlowEnoughToFeed(coordinator)),
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

  // ===== LUT Dev Mode Commands =====

  /**
   * Calculate horizontal distance from turret to hub center.
   *
   * @param robotPose Current robot pose
   * @param config Turret geometry config
   * @return Distance in meters
   */
  private static double calculateDistanceToHub(
      Pose2d robotPose, ShotCalculator.TurretConfig config) {
    double robotHeadingRad = robotPose.getRotation().getRadians();
    double[] turretFieldPos =
        ShotCalculator.getTurretFieldPosition(
            robotPose.getX(), robotPose.getY(), robotHeadingRad, config);
    double hubX = frc.robot.FieldConstants.Hub.innerCenterPoint.getX();
    double hubY = frc.robot.FieldConstants.Hub.innerCenterPoint.getY();
    return Math.sqrt(Math.pow(hubX - turretFieldPos[0], 2) + Math.pow(hubY - turretFieldPos[1], 2));
  }

  /**
   * End a batch and record LUT data. Call after firing a hopper of fuel from a stationary position.
   * Press success if the shots scored, miss if they didn't. Writes clean LUT data (success only)
   * and full batch log (all attempts).
   *
   * @param coordinator Provides pose, current shot, and batch recorder
   * @param launcher For target RPM
   * @param turret For turret angle
   * @param hood For hood angle (can be null)
   * @param successful Whether the batch scored
   * @return Command that ends the batch and records data
   */
  public static Command recordBatchCommand(
      ShootingCoordinator coordinator,
      Launcher launcher,
      Turret turret,
      Hood hood,
      boolean successful) {
    return Commands.runOnce(
            () -> {
              if (coordinator.getRobotPoseSupplier() == null) {
                System.out.println("[LUTDev] Cannot record — no pose supplier");
                return;
              }

              StationaryShotBatchRecorder recorder = coordinator.getBatchRecorder();

              Pose2d robotPose = coordinator.getRobotPoseSupplier().get();
              ShotCalculator.TurretConfig config = coordinator.getTurretConfig();
              double distance = calculateDistanceToHub(robotPose, config);

              // Get shot parameters from current calculation
              ShotCalculator.ShotResult currentShot = coordinator.getCurrentShot();
              double rpm = launcher.getTargetVelocity();
              double hoodAngle =
                  currentShot != null
                      ? currentShot.hoodAngleDeg()
                      : (hood != null ? hood.getCurrentAngle() : 0.0);
              double turretAngle = turret.getOutsideCurrentAngle();
              double motivatorRPM = smartShotMotivatorRPM.get();
              double spindexerRPM = smartShotSpindexerRPM.get();

              // Calculate TOF
              double exitVelocity = ShotCalculator.calculateExitVelocityFromRPM(rpm);
              double tof = 0.0;
              if (currentShot != null && currentShot.exitVelocityMps() > 0) {
                tof =
                    ShotCalculator.calculateTimeOfFlight(
                        currentShot.exitVelocityMps(), currentShot.launchAngleRad(), distance);
              } else {
                tof = distance / Math.max(exitVelocity * 0.8, 1.0);
              }

              // Get current fuel count for batch calculation
              ShotVisualizer visualizer = coordinator.getVisualizer();
              int currentFuel = visualizer != null ? visualizer.getFuelCount() : 0;

              int fuelFired =
                  recorder.endBatch(
                      Timer.getFPGATimestamp(),
                      robotPose.getX(),
                      robotPose.getY(),
                      distance,
                      rpm,
                      hoodAngle,
                      turretAngle,
                      tof,
                      motivatorRPM,
                      spindexerRPM,
                      currentFuel,
                      successful);

              // Auto-reload LUT after recording a success
              if (successful) {
                coordinator.reloadLUTData();
              }

              // Log to AdvantageKit
              Logger.recordOutput("LUTDev/LastDistance", distance);
              Logger.recordOutput("LUTDev/LastRPM", rpm);
              Logger.recordOutput("LUTDev/LastHoodDeg", hoodAngle);
              Logger.recordOutput("LUTDev/LastFuelFired", fuelFired);
              Logger.recordOutput("LUTDev/LastSuccessful", successful);
              Logger.recordOutput("LUTDev/LUTEntries", recorder.getLUTEntryCount());
              Logger.recordOutput("LUTDev/BatchCount", recorder.getBatchCount());
              Logger.recordOutput("LUTDev/SuccessCount", recorder.getSuccessCount());
              Logger.recordOutput("LUTDev/MissCount", recorder.getMissCount());
              Logger.recordOutput("LUTDev/BatchLog", recorder.getBatchSummaries());

              // Console output
              System.out.println(
                  "[LUTDev] Batch "
                      + (successful ? "SUCCESS" : "MISS")
                      + " at "
                      + String.format("%.2f", distance)
                      + "m | RPM="
                      + String.format("%.0f", rpm)
                      + " Hood="
                      + String.format("%.1f", hoodAngle)
                      + "° | "
                      + fuelFired
                      + " fuel fired | "
                      + recorder.getSuccessCount()
                      + "/"
                      + recorder.getBatchCount()
                      + " batches successful | "
                      + recorder.getLUTEntryCount()
                      + " LUT entries");
            })
        .ignoringDisable(true)
        .withName("Record Batch " + (successful ? "Success" : "Miss"));
  }

  /**
   * Command to reload LUT data from disk into the active lookup table.
   *
   * @param coordinator The shooting coordinator
   * @return Command that reloads LUT data
   */
  public static Command reloadLUTCommand(ShootingCoordinator coordinator) {
    return Commands.runOnce(coordinator::reloadLUTData)
        .ignoringDisable(true)
        .withName("Reload LUT Data");
  }

  /**
   * Command to clear all recorded LUT and batch log data.
   *
   * @param coordinator The shooting coordinator
   * @return Command that clears all data
   */
  public static Command clearLUTDataCommand(ShootingCoordinator coordinator) {
    return Commands.runOnce(
            () -> {
              coordinator.getBatchRecorder().clearAll();
              coordinator.reloadLUTData();
              System.out.println("[LUTDev] All data cleared (LUT + batch log)");
            })
        .ignoringDisable(true)
        .withName("Clear LUT Data");
  }

  /**
   * LUT dev mode logging command. Runs continuously to output useful info to the dashboard while
   * collecting LUT data.
   *
   * @param coordinator The shooting coordinator
   * @param turret For current distance calculation
   * @return Command that logs LUT dev info each cycle
   */
  public static Command lutDevModeCommand(ShootingCoordinator coordinator, Turret turret) {
    return Commands.run(
            () -> {
              if (coordinator.getRobotPoseSupplier() == null) return;

              Pose2d robotPose = coordinator.getRobotPoseSupplier().get();
              double distance = calculateDistanceToHub(robotPose, coordinator.getTurretConfig());

              ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
              StationaryShotBatchRecorder recorder = coordinator.getBatchRecorder();

              Logger.recordOutput("LUTDev/CurrentDistance", distance);
              Logger.recordOutput("LUTDev/SuggestedRPM", shot != null ? shot.launcherRPM() : 0.0);
              Logger.recordOutput(
                  "LUTDev/SuggestedHoodDeg", shot != null ? shot.hoodAngleDeg() : 0.0);
              Logger.recordOutput("LUTDev/BatchActive", recorder.isBatchActive());
              Logger.recordOutput("LUTDev/LUTEntries", coordinator.getLookupTable().size());
              Logger.recordOutput("LUTDev/BatchCount", recorder.getBatchCount());
              Logger.recordOutput("LUTDev/SuccessCount", recorder.getSuccessCount());
              Logger.recordOutput("LUTDev/MissCount", recorder.getMissCount());
            })
        .ignoringDisable(true)
        .withName("LUT Dev Mode");
  }
}
