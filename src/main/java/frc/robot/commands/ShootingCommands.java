package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
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
      new LoggedTunableNumber("Shots/HubShot/LauncherRPM", 2300.0);
  private static final LoggedTunableNumber hubShotHoodAngleDeg =
      new LoggedTunableNumber("Shots/HubShot/HoodAngleDeg", 13.0);
  private static final LoggedTunableNumber hubShotTurretAngleDeg =
      new LoggedTunableNumber("Shots/HubShot/TurretAngleDeg", -90);
  private static final LoggedTunableNumber hubShotMotivatorRPM =
      new LoggedTunableNumber("Shots/HubShot/MotivatorRPM", 1300.0);
  private static final LoggedTunableNumber hubShotSpindexerRPM =
      new LoggedTunableNumber("Shots/HubShot/SpindexerRPM", 325.0);

  // Left trench shot
  private static final LoggedTunableNumber leftTrenchLauncherRPM =
      new LoggedTunableNumber("Shots/LeftTrench/LauncherRPM", 2650.0);
  private static final LoggedTunableNumber leftTrenchHoodAngleDeg =
      new LoggedTunableNumber("Shots/LeftTrench/HoodAngleDeg", 18.0);
  private static final LoggedTunableNumber leftTrenchTurretAngleDeg =
      new LoggedTunableNumber("Shots/LeftTrench/TurretAngleDeg", 186.5);
  private static final LoggedTunableNumber leftTrenchMotivatorRPM =
      new LoggedTunableNumber("Shots/LeftTrench/MotivatorRPM", 1800.0);
  private static final LoggedTunableNumber leftTrenchSpindexerRPM =
      new LoggedTunableNumber("Shots/LeftTrench/SpindexerRPM", 325.0);

  // Right trench shot
  private static final LoggedTunableNumber rightTrenchLauncherRPM =
      new LoggedTunableNumber("Shots/RightTrench/LauncherRPM", 3169.0);
  private static final LoggedTunableNumber rightTrenchHoodAngleDeg =
      new LoggedTunableNumber("Shots/RightTrench/HoodAngleDeg", 18.0);
  private static final LoggedTunableNumber rightTrenchTurretAngleDeg =
      new LoggedTunableNumber("Shots/RightTrench/TurretAngleDeg", -4.84);
  private static final LoggedTunableNumber rightTrenchMotivatorRPM =
      new LoggedTunableNumber("Shots/RightTrench/MotivatorRPM", 1800.0);
  private static final LoggedTunableNumber rightTrenchSpindexerRPM =
      new LoggedTunableNumber("Shots/RightTrench/SpindexerRPM", 325.0);

  // ===== Robot Tuning (affects real robot behavior) =====

  // Smart shot motivator/spindexer speeds
  private static final LoggedTunableNumber smartShotMotivatorRPM =
      new LoggedTunableNumber("Shots/SmartLaunch/MotivatorRPM", 1500.0);
  private static final LoggedTunableNumber smartShotSpindexerRPM =
      new LoggedTunableNumber("Shots/SmartLaunch/SpindexerRPM", 325.0);

  // Motivator RPM as a ratio of launcher RPM. When > 0, motivator RPM is derived from
  // launcherRPM * ratio instead of being an independent LUT/tunable value. Set to 0 to
  // disable and fall back to the old per-shot motivator RPM behavior.
  private static final LoggedTunableNumber motivatorLauncherRatio =
      new LoggedTunableNumber("Shots/SmartLaunch/MotivatorLauncherRatio", 0.565);

  // Spindexer RPM as a ratio of launcher RPM. When > 0, spindexer RPM is derived from
  // launcherRPM * ratio instead of the fixed per-shot tunable. Set to 0 to disable and
  // fall back to the old independent spindexer RPM behavior.
  private static final LoggedTunableNumber spindexerLauncherRatio =
      new LoggedTunableNumber("Shots/SmartLaunch/SpindexerLauncherRatio", 0.0);

  // Max drive speed (m/s) while smart launch speed-limit mode is active
  private static final LoggedTunableNumber smartLaunchSpeedLimitCapMps =
      new LoggedTunableNumber("Shots/SmartLaunch/SpeedLimitCapMps", 0.5);

  // ===== LUT Dev Overrides (manual RPM/hood for data collection) =====
  private static final LoggedTunableNumber lutDevOverrideRPM =
      new LoggedTunableNumber("LUTDev/OverrideRPM", 2500.0);
  private static final LoggedTunableNumber lutDevOverrideHoodDeg =
      new LoggedTunableNumber("LUTDev/OverrideHoodDeg", 25.0);

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

  /**
   * Derive motivator RPM from launcher RPM using the configured ratio. When the ratio tunable is >
   * 0, motivator RPM = launcherRPM * ratio. Otherwise falls back to the shot's motivator RPM (if
   * available) or the smartShotMotivatorRPM tunable.
   */
  private static double getMotivatorRPM(double launcherRPM) {
    double ratio = motivatorLauncherRatio.get();
    if (ratio > 0) {
      return launcherRPM * ratio;
    }
    return smartShotMotivatorRPM.get();
  }

  /**
   * Derive spindexer RPM from launcher RPM using the configured ratio. When the ratio tunable is >
   * 0, spindexer RPM = launcherRPM * ratio. Otherwise falls back to the smartShotSpindexerRPM
   * tunable.
   */
  private static double getSpindexerRPM(double launcherRPM) {
    double ratio = spindexerLauncherRatio.get();
    if (ratio > 0) {
      return launcherRPM * ratio;
    }
    return smartShotSpindexerRPM.get();
  }

  /** Initialize tunables so they appear in the dashboard immediately. */
  public static void initTunables() {
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
    spindexerLauncherRatio.get();

    // LUT Dev override tunables
    lutDevOverrideRPM.get();
    lutDevOverrideHoodDeg.get();

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

    // Measured TOF input — students fill this in from slow-mo camera analysis (seconds)
    SmartDashboard.putNumber("LUTDev/MeasuredTOF_s", 0.0);
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
   * Main launch command. Spins up, waits for setpoint, then fires repeatedly. /** Command to reset
   * the simulation for testing.
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
                    double rpm = getEffectiveRPM(shot);
                    launcher.setVelocity(rpm);
                    turret.setOutsideTurretAngle(shot.turretAngleDeg());
                    if (hood != null) {
                      hood.setHoodAngle(getEffectiveHoodDeg(shot));
                    }
                    if (motivator != null) {
                      double motRPM = getMotivatorRPM(rpm);
                      motivator.setMotivatorVelocity(motRPM);
                    }
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
                            double rpm = getEffectiveRPM(shot);
                            double hoodDeg = getEffectiveHoodDeg(shot);
                            launcher.setVelocity(rpm);
                            turret.setOutsideTurretAngle(shot.turretAngleDeg());
                            if (hood != null) {
                              hood.setHoodAngle(hoodDeg);
                            }
                          }

                          boolean launcherReady = launcher.atSetpoint();
                          boolean motivatorReady =
                              motivator == null || motivator.isMotivatorAtSetpoint();
                          boolean turretReady = turret.atTarget();
                          boolean hoodReady = hood == null || hood.atTarget();
                          boolean hasShot =
                              shot != null && (isLutDevOverrideActive() || shot.achievable());
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
                          boolean hasShot =
                              shot != null && (isLutDevOverrideActive() || shot.achievable());
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
                          boolean hasShot =
                              shot != null && (isLutDevOverrideActive() || shot.achievable());
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
                        double rpm = getEffectiveRPM(shot);
                        double hoodDeg = getEffectiveHoodDeg(shot);
                        launcher.setVelocity(rpm);
                        // Cache params so recordBatchCommand can read them after SmartLaunch ends
                        coordinator.getBatchRecorder().cacheParams(rpm, hoodDeg);
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
                            hood.setHoodAngle(getEffectiveHoodDeg(shot));
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
                        () -> {
                          ShotCalculator.ShotResult s = coordinator.getCurrentShot();
                          double launcherRPM = getEffectiveRPM(s);
                          motivator.setMotivatorVelocity(getMotivatorRPM(launcherRPM));
                        },
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
                            ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                            double launcherRPM = getEffectiveRPM(shot);
                            double spnRPM = getSpindexerRPM(launcherRPM);
                            spindexer.setSpindexerVelocity(spnRPM);
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
   * Smart launch with drive speed limiting. Sets a global speed limit on the default drive command
   * while active. When released, the speed limit ramps back up smoothly (via {@link
   * DriveCommands#clearSpeedLimit()}) to prevent sudden acceleration if the driver is pushing the
   * stick forward.
   *
   * @param launcher The launcher subsystem
   * @param coordinator The shooting coordinator
   * @param motivator The motivator subsystem (can be null)
   * @param turret The turret subsystem
   * @param hood The hood subsystem (can be null)
   * @param spindexer The spindexer subsystem (can be null)
   * @return Command that aims, fires, and limits drive speed while held
   */
  public static Command smartLaunchWithSpeedLimitCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {
    return smartLaunchCommand(launcher, coordinator, motivator, turret, hood, spindexer, false)
        .beforeStarting(() -> DriveCommands.setSpeedLimit(smartLaunchSpeedLimitCapMps.get()))
        .finallyDo(() -> DriveCommands.clearSpeedLimit())
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

  // ===== LUT Dev Override Helpers =====

  private static boolean isLutDevOverrideActive() {
    return SmartDashboard.getBoolean("LUTDev/UseOverrides", false);
  }

  private static double getEffectiveRPM(ShotCalculator.ShotResult shot) {
    if (isLutDevOverrideActive()) {
      return lutDevOverrideRPM.get();
    }
    return shot != null ? shot.launcherRPM() : 0.0;
  }

  private static double getEffectiveHoodDeg(ShotCalculator.ShotResult shot) {
    if (isLutDevOverrideActive()) {
      return lutDevOverrideHoodDeg.get();
    }
    return shot != null ? shot.hoodAngleDeg() : 0.0;
  }

  /**
   * Seeds the LUT dev override sliders from the current calculated shot, so you can start from the
   * physics answer and fine-tune. Also enables UseOverrides automatically.
   */
  public static Command seedOverridesFromCalculatedCommand(ShootingCoordinator coordinator) {
    return Commands.runOnce(
        () -> {
          ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
          if (shot == null) {
            System.out.println("[LUTDev] No calculated shot available to seed from");
            return;
          }
          lutDevOverrideRPM.set(shot.launcherRPM());
          lutDevOverrideHoodDeg.set(shot.hoodAngleDeg());
          SmartDashboard.putBoolean("LUTDev/UseOverrides", true);
          System.out.printf(
              "[LUTDev] Seeded overrides from calculated shot: RPM=%.0f, Hood=%.1f deg%n",
              shot.launcherRPM(), shot.hoodAngleDeg());
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

              // Use cached params from SmartLaunch (survive after command ends)
              double rpm = recorder.getCachedRPM();
              double hoodAngle = recorder.getCachedHoodAngleDeg();
              double turretAngle = turret.getOutsideCurrentAngle();
              ShotCalculator.ShotResult currentShot = coordinator.getCurrentShot();

              // Calculate theoretical TOF
              double exitVelocity = ShotCalculator.calculateExitVelocityFromRPM(rpm);
              double theoreticalTOF = 0.0;
              if (currentShot != null && currentShot.exitVelocityMps() > 0) {
                theoreticalTOF =
                    ShotCalculator.calculateTimeOfFlight(
                        currentShot.exitVelocityMps(), currentShot.launchAngleRad(), distance);
              } else {
                theoreticalTOF = distance / Math.max(exitVelocity * 0.8, 1.0);
              }

              // Read measured TOF from dashboard (students input from slow-mo camera)
              double measuredTOF = SmartDashboard.getNumber("LUTDev/MeasuredTOF_s", 0.0);

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
                      theoreticalTOF,
                      measuredTOF,
                      currentFuel,
                      successful);

              // Reset measured TOF input after recording
              SmartDashboard.putNumber("LUTDev/MeasuredTOF_s", 0.0);

              // Auto-reload LUT after recording a success
              if (successful) {
                coordinator.reloadLUTData();
              }

              // Log to AdvantageKit
              Logger.recordOutput("LUTDev/LastDistance", distance);
              Logger.recordOutput("LUTDev/LastRPM", rpm);
              Logger.recordOutput("LUTDev/LastHoodDeg", hoodAngle);
              Logger.recordOutput("LUTDev/LastTheoreticalTOF", theoreticalTOF);
              Logger.recordOutput("LUTDev/LastMeasuredTOF", measuredTOF);
              Logger.recordOutput("LUTDev/LastFuelFired", fuelFired);
              Logger.recordOutput("LUTDev/LastSuccessful", successful);
              Logger.recordOutput("LUTDev/LUTEntries", recorder.getLUTEntryCount());
              Logger.recordOutput("LUTDev/BatchCount", recorder.getBatchCount());
              Logger.recordOutput("LUTDev/SuccessCount", recorder.getSuccessCount());
              Logger.recordOutput("LUTDev/MissCount", recorder.getMissCount());
              Logger.recordOutput("LUTDev/BatchLog", recorder.getBatchSummaries());

              // Summary string for quick dashboard verification
              String summary =
                  (successful ? "HIT" : "MISS")
                      + " | "
                      + String.format("%.2fm", distance)
                      + " | RPM="
                      + String.format("%.0f", rpm)
                      + " | Hood="
                      + String.format("%.1f°", hoodAngle)
                      + " | TOF="
                      + String.format("%.3fs", measuredTOF > 0 ? measuredTOF : theoreticalTOF);
              SmartDashboard.putString("LUTDev/LastEntry", summary);

              // Console output
              System.out.println("[LUTDev] Batch " + summary + " | " + fuelFired + " fuel fired");
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
   * Command to remove the last LUT entry (undo a bad recording).
   *
   * @param coordinator The shooting coordinator
   * @return Command that removes the last entry
   */
  public static Command undoLastLUTEntryCommand(ShootingCoordinator coordinator) {
    return Commands.runOnce(
            () -> {
              String removed = coordinator.getBatchRecorder().removeLastLUTEntry();
              if (removed != null) {
                coordinator.reloadLUTData();
                SmartDashboard.putString("LUTDev/LastEntry", "UNDONE: " + removed);
                System.out.println("[LUTDev] Removed last entry: " + removed);
              } else {
                SmartDashboard.putString("LUTDev/LastEntry", "Nothing to undo");
                System.out.println("[LUTDev] No entries to remove");
              }
            })
        .ignoringDisable(true)
        .withName("Undo Last LUT Entry");
  }

  /**
   * Command to remove a specific LUT entry by index (shown in LUTDev/LUTData).
   *
   * @param coordinator The shooting coordinator
   * @return Command that removes the entry at the index in LUTDev/RemoveIndex
   */
  public static Command removeLUTEntryCommand(ShootingCoordinator coordinator) {
    return Commands.runOnce(
            () -> {
              int index = (int) SmartDashboard.getNumber("LUTDev/RemoveIndex", -1);
              if (index < 0) {
                SmartDashboard.putString("LUTDev/LastEntry", "Set RemoveIndex first");
                return;
              }
              String removed = coordinator.getBatchRecorder().removeLUTEntry(index);
              if (removed != null) {
                coordinator.reloadLUTData();
                SmartDashboard.putString("LUTDev/LastEntry", "REMOVED: " + removed);
                System.out.println("[LUTDev] Removed entry: " + removed);
              } else {
                SmartDashboard.putString("LUTDev/LastEntry", "Invalid index: " + index);
                System.out.println("[LUTDev] Invalid index: " + index);
              }
            })
        .ignoringDisable(true)
        .withName("Remove LUT Entry");
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
              Logger.recordOutput("LUTDev/OverridesActive", isLutDevOverrideActive());
              Logger.recordOutput("LUTDev/LUTData", recorder.getLUTSummaries());
            })
        .ignoringDisable(true)
        .withName("LUT Dev Mode");
  }
}
