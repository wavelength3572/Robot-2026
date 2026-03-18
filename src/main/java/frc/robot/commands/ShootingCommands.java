package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.util.FuelSim;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ZoneDetector;
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
 *   <li>TEST: Uses fixed preset parameters (hub shot, trench shots)
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
    /** Fixed preset parameters (hub shot, trench shots). */
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
      new LoggedTunableNumber("Shots/HubShot/SpindexerRPM", 500.0);

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

  // Motivator RPM as a ratio of launcher RPM: motivatorRPM = launcherRPM * ratio
  private static final LoggedTunableNumber motivatorLauncherRatio =
      new LoggedTunableNumber("Shots/SmartLaunch/MotivatorLauncherRatio", 0.565);

  // Spindexer RPM lerped by distance: close = max, far = min
  private static final LoggedTunableNumber spindexerCloseRPM =
      new LoggedTunableNumber("Shots/SmartLaunch/SpindexerCloseRPM", 225.0);
  private static final LoggedTunableNumber spindexerFarRPM =
      new LoggedTunableNumber("Shots/SmartLaunch/SpindexerFarRPM", 225.0);
  // Fixed spindexer RPM used in pass/neutral zones (no distance lerp)
  private static final LoggedTunableNumber spindexerPassRPM =
      new LoggedTunableNumber("Shots/SmartLaunch/SpindexerPassRPM", 350.0);

  // Feed suppression during large turret movements (flips).
  // Suppress feeding when turret error exceeds threshold; resume with hysteresis.
  private static final LoggedTunableNumber feedSuppressionThresholdDeg =
      new LoggedTunableNumber("Shots/SmartLaunch/FeedSuppressionThresholdDeg", 45.0);
  private static final LoggedTunableNumber feedResumeThresholdDeg =
      new LoggedTunableNumber("Shots/SmartLaunch/FeedResumeThresholdDeg", 35.0);

  // ===== LUT Dev Overrides (manual RPM/hood for data collection) =====
  private static final LoggedTunableNumber lutDevOverrideRPM =
      new LoggedTunableNumber("LUTDev/OverrideRPM", 2500.0);
  private static final LoggedTunableNumber lutDevOverrideHoodDeg =
      new LoggedTunableNumber("LUTDev/OverrideHoodDeg", 25.0);

  // ===== Launcher RPM Trim =====

  // Trim value added to all launcher RPM targets. Adjusted via button box 1 axis knob.
  // Knob positions: (0,-1)=neutral, (0,1)=-50, (-1,1)=+50, (1,1)=+100
  private static double launcherTrimRPM = 0.0;

  /**
   * Set the launcher RPM trim offset. This value is added to all launcher RPM targets (both smart
   * launch and fixed shots).
   *
   * @param trimRPM The RPM offset to apply
   */
  public static void setLauncherTrimRPM(double trimRPM) {
    if (launcherTrimRPM != trimRPM) {
      launcherTrimRPM = trimRPM;
      SmartDashboard.putNumber("Trim/LauncherRPM", trimRPM);
      Logger.recordOutput("Trim/LauncherRPM", trimRPM);
      System.out.println("[Trim] Launcher RPM trim set to " + trimRPM);
    }
  }

  /**
   * Get the current launcher RPM trim offset.
   *
   * @return Current trim RPM value
   */
  public static double getLauncherTrimRPM() {
    return launcherTrimRPM;
  }

  // ===== Command Behavior Constants =====

  // Minimum time between shots (prevents multiple fires per frame)
  private static final double MIN_SHOT_INTERVAL_SECONDS = 0.05;

  // Always wait for setpoint recovery before firing next shot
  private static final boolean WAIT_FOR_RECOVERY = true;

  private ShootingCommands() {
    // Static factory class
  }

  /** Derive motivator RPM from launcher RPM: motivatorRPM = launcherRPM * ratio. */
  public static double getMotivatorRPM(double launcherRPM) {
    return launcherRPM * motivatorLauncherRatio.get();
  }

  /** Lerp spindexer RPM from distance — close (1.16m) to far (5.35m). */
  public static double getSpindexerRPM(double distanceM) {
    double minDist = 1.16, maxDist = 5.35;
    double t = Math.max(0, Math.min(1, (distanceM - minDist) / (maxDist - minDist)));
    return spindexerCloseRPM.get() + t * (spindexerFarRPM.get() - spindexerCloseRPM.get());
  }

  /** Fixed spindexer RPM for pass/neutral zones. */
  public static double getSpindexerPassRPM() {
    return spindexerPassRPM.get();
  }

  /**
   * Check if feed should be suppressed due to a large turret movement (flip). Uses hysteresis to
   * prevent chatter near the threshold.
   *
   * @param turret The turret subsystem
   * @param wasSuppressed Single-element array holding the previous suppression state
   * @return true if feed should be suppressed
   */
  private static boolean shouldSuppressFeed(Turret turret, boolean[] wasSuppressed) {
    double error = Math.abs(turret.getOutsideCurrentAngle() - turret.getOutsideTargetAngle());
    if (wasSuppressed[0]) {
      wasSuppressed[0] = error > feedResumeThresholdDeg.get();
    } else {
      wasSuppressed[0] = error > feedSuppressionThresholdDeg.get();
    }
    return wasSuppressed[0];
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

    // LUT Dev override tunables
    lutDevOverrideRPM.get();
    lutDevOverrideHoodDeg.get();

    // Trim initial value on dashboard
    SmartDashboard.putNumber("Trim/LauncherRPM", launcherTrimRPM);
    Logger.recordOutput("Trim/LauncherRPM", launcherTrimRPM);

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
                Logger.recordOutput("ShotLog/FuelRemaining", 40);
              }

              Logger.recordOutput("ShotLog/SimReset", true);
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
                Logger.recordOutput("ShotLog/FuelRemaining", 40);
              }

              Logger.recordOutput("ShotLog/SimReset", true);
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
    // Wrap the launcher RPM supplier to include trim offset
    DoubleSupplier trimmedLauncherRPM = () -> launcherRPMSupplier.getAsDouble() + launcherTrimRPM;

    return Commands.sequence(
            Commands.runOnce(() -> setMode(ShootingMode.TEST)),

            // Phase 1: Spin up and position all subsystems in parallel
            Commands.runOnce(
                () -> {
                  if (coordinator != null) {}

                  double turretAngle = turretAngleDegSupplier.getAsDouble();
                  double hoodAngle = hoodAngleDegSupplier.getAsDouble();
                  double launcherRPM = trimmedLauncherRPM.getAsDouble();

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
                          boolean launcherReady = launcher.isReady();
                          boolean motivatorReady =
                              motivator == null
                                  || motivator.getState() == Motivator.MotivatorState.READY;
                          boolean turretReady = turret.getState() == Turret.TurretState.READY;
                          boolean hoodReady =
                              hood == null || hood.getState() == Hood.HoodState.READY;

                          return launcherReady && motivatorReady && turretReady && hoodReady;
                        }),
                    Commands.waitSeconds(0.1),
                    Commands.waitUntil(
                        () -> {
                          boolean launcherReady = launcher.isReady();
                          boolean motivatorReady =
                              motivator == null
                                  || motivator.getState() == Motivator.MotivatorState.READY;
                          boolean turretReady = turret.getState() == Turret.TurretState.READY;
                          boolean hoodReady =
                              hood == null || hood.getState() == Hood.HoodState.READY;
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
                  logShotStatus(
                      "FixedShot", launcher, hood, motivator, trimmedLauncherRPM.getAsDouble());
                  launcher.setFeedingActive(true);
                }),

            // Phase 3: Feed via spindexer while keeping all subsystems running
            Commands.parallel(
                // Keep launcher at speed (reads tunable each cycle)
                Commands.run(
                    () -> {
                      double rpm = trimmedLauncherRPM.getAsDouble();
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

                // Run spindexer to feed fuel continuously (fixed shots are stationary presets)
                spindexer != null
                    ? Commands.run(
                        () -> spindexer.setSpindexerVelocity(spindexerRPMSupplier.getAsDouble()),
                        spindexer)
                    : Commands.none(),

                // Fire balls in simulation
                coordinator != null
                    ? createSimFiringLoop(coordinator, launcher, turret)
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
  public static Command smartLaunchCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {
    return smartLaunchCommandImpl(launcher, coordinator, motivator, turret, hood, spindexer);
  }

  /**
   * Core smart launch Sequenced. Uses zone-aware speed gating for both readiness checks and
   * spindexer feeding.
   */
  private static Command smartLaunchCommandSeq(
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
                  ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                  if (shot != null) {
                    double rpm = getEffectiveRPM(shot);
                    launcher.setVelocity(rpm);
                    turret.setOutsideTurretAngle(shot.turretAngleDeg());
                    if (hood != null) {
                      hood.setHoodAngle(getEffectiveHoodDeg(shot));
                    }
                    if (motivator != null) {
                      motivator.stopMotivator();
                    }
                  }

                  // Start a LUT batch — captures fuel count for batch tracking
                  ShotVisualizer vis = coordinator.getVisualizer();
                  if (vis != null) {
                    coordinator.getBatchRecorder().startBatch(vis.getFuelCount());
                  }

                  Logger.recordOutput("SmartLaunch/Phase", "SPIN_UP");
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
                            // Start motivator once launcher is at setpoint
                            if (motivator != null
                                && launcher.isReady()
                                && turret.atTarget()
                                && hood.atTarget()) {
                              motivator.setMotivatorVelocity(getMotivatorRPM(rpm));
                            }
                          }

                          boolean launcherReady = launcher.isReady();
                          boolean motivatorReady =
                              motivator == null
                                  || motivator.getState() == Motivator.MotivatorState.READY;
                          boolean turretReady = turret.getState() == Turret.TurretState.READY;
                          boolean hoodReady =
                              hood == null || hood.getState() == Hood.HoodState.READY;
                          boolean achievable =
                              shot != null && (isLutDevOverrideActive() || shot.achievable());
                          boolean robotSlow = coordinator.isRobotSlowEnoughForCurrentZone();

                          boolean allReady =
                              launcherReady
                                  && motivatorReady
                                  && turretReady
                                  && hoodReady
                                  && achievable
                                  && robotSlow;

                          Logger.recordOutput("SmartLaunch/Ready/Launcher", launcherReady);
                          Logger.recordOutput("SmartLaunch/Ready/Motivator", motivatorReady);
                          Logger.recordOutput("SmartLaunch/Ready/Turret", turretReady);
                          Logger.recordOutput("SmartLaunch/Ready/Hood", hoodReady);
                          Logger.recordOutput("SmartLaunch/Ready/Achievable", achievable);
                          Logger.recordOutput("SmartLaunch/Ready/RobotSlow", robotSlow);
                          Logger.recordOutput("SmartLaunch/Ready/All", allReady);
                          return allReady;
                        })),
                Commands.sequence(
                    Commands.waitSeconds(5.0),
                    Commands.runOnce(
                        () -> {
                          ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                          boolean launcherReady = launcher.isReady();
                          boolean motivatorReady =
                              motivator == null
                                  || motivator.getState() == Motivator.MotivatorState.READY;
                          boolean turretReady = turret.getState() == Turret.TurretState.READY;
                          boolean hoodReady =
                              hood == null || hood.getState() == Hood.HoodState.READY;
                          boolean achievable =
                              shot != null && (isLutDevOverrideActive() || shot.achievable());
                          Logger.recordOutput("SmartLaunch/Phase", "TIMED_OUT");
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
                                  + achievable
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
                  Logger.recordOutput("SmartLaunch/Phase", "FIRING");
                  SmartDashboard.putString("Match/Status/State", "Smart Launch - Feeding");
                  ShotCalculator.ShotResult feedShot = coordinator.getCurrentShot();
                  logShotStatus(
                      "SmartLaunch", launcher, hood, motivator, getEffectiveRPM(feedShot));
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
                // Feed is suppressed during large turret movements (flips).
                motivator != null
                    ? Commands.run(
                        new Runnable() {
                          boolean[] flipSuppressed = {false};

                          @Override
                          public void run() {
                            if (turret != null && shouldSuppressFeed(turret, flipSuppressed)) {
                              motivator.stopMotivator();
                            } else {
                              ShotCalculator.ShotResult s = coordinator.getCurrentShot();
                              double launcherRPM = getEffectiveRPM(s);
                              motivator.setMotivatorVelocity(getMotivatorRPM(launcherRPM));
                            }
                          }
                        },
                        motivator)
                    : Commands.none(),

                // Run spindexer: teleop always feeds (operator is holding shoot button),
                // auto feeds when slow enough, reciprocates when too fast (transiting).
                // Feed is suppressed during large turret movements (flips).
                spindexer != null
                    ? Commands.run(
                        new Runnable() {
                          boolean wasReciprocating = false;
                          boolean[] flipSuppressed = {false};

                          @Override
                          public void run() {
                            boolean suppressed =
                                turret != null && shouldSuppressFeed(turret, flipSuppressed);
                            Logger.recordOutput("SmartLaunch/FlipSuppressed", suppressed);
                            if (suppressed) {
                              spindexer.stopSpindexer();
                              return;
                            }
                            double dist = coordinator.getDistanceToTarget();
                            double spnRPM =
                                coordinator.isInPassZone()
                                    ? getSpindexerPassRPM()
                                    : getSpindexerRPM(dist > 0 ? dist : 1.16);
                            if (DriverStation.isTeleop()
                                || coordinator.isRobotSlowEnoughForCurrentZone()) {
                              wasReciprocating = false;
                              spindexer.setSpindexerVelocity(spnRPM);
                            } else if (DriverStation.isAutonomous()) {
                              wasReciprocating = true;
                              spindexer.reciprocate();
                            } else if (wasReciprocating) {
                              wasReciprocating = false;
                              spindexer.stopReciprocateWithKick();
                            } else {
                              spindexer.stopSpindexer();
                            }
                            Logger.recordOutput(
                                "SmartLaunch/Feeding",
                                DriverStation.isTeleop()
                                    || coordinator.isRobotSlowEnoughForCurrentZone());
                          }
                        },
                        spindexer)
                    : Commands.none(),

                // Fire balls in simulation (same feed conditions)
                createSimFiringLoop(coordinator, launcher, turret)))
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
              Logger.recordOutput("SmartLaunch/Phase", "IDLE");
              SmartDashboard.putString("Match/Status/State", "Stopped");
              System.out.println("[SmartLaunch] Stopped");
            })
        .withName("SmartLaunch");
  }

  /**
   * Core smart launch implementation. Uses zone-aware speed gating for both readiness checks and
   * spindexer feeding.
   */
  private static Command smartLaunchCommandImpl(
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

                  Logger.recordOutput("SmartLaunch/Phase", "SPIN_UP");
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
                            // Start motivator once launcher is at setpoint
                            if (motivator != null && launcher.isReady()) {
                              motivator.setMotivatorVelocity(getMotivatorRPM(rpm));
                            }
                          }

                          boolean launcherReady = launcher.isReady();
                          boolean motivatorReady =
                              motivator == null
                                  || motivator.getState() == Motivator.MotivatorState.READY;
                          boolean turretReady = turret.getState() == Turret.TurretState.READY;
                          boolean hoodReady =
                              hood == null || hood.getState() == Hood.HoodState.READY;
                          boolean achievable =
                              shot != null && (isLutDevOverrideActive() || shot.achievable());
                          boolean robotSlow = coordinator.isRobotSlowEnoughForCurrentZone();

                          boolean allReady =
                              launcherReady
                                  && motivatorReady
                                  && turretReady
                                  && hoodReady
                                  && achievable
                                  && robotSlow;

                          Logger.recordOutput("SmartLaunch/Ready/Launcher", launcherReady);
                          Logger.recordOutput("SmartLaunch/Ready/Motivator", motivatorReady);
                          Logger.recordOutput("SmartLaunch/Ready/Turret", turretReady);
                          Logger.recordOutput("SmartLaunch/Ready/Hood", hoodReady);
                          Logger.recordOutput("SmartLaunch/Ready/Achievable", achievable);
                          Logger.recordOutput("SmartLaunch/Ready/RobotSlow", robotSlow);
                          Logger.recordOutput("SmartLaunch/Ready/All", allReady);
                          return allReady;
                        }),
                    Commands.waitSeconds(0.1),
                    Commands.waitUntil(
                        () -> {
                          ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                          boolean launcherReady = launcher.isReady();
                          boolean motivatorReady =
                              motivator == null
                                  || motivator.getState() == Motivator.MotivatorState.READY;
                          boolean turretReady = turret.getState() == Turret.TurretState.READY;
                          boolean hoodReady =
                              hood == null || hood.getState() == Hood.HoodState.READY;
                          boolean achievable =
                              shot != null && (isLutDevOverrideActive() || shot.achievable());
                          boolean robotSlow = coordinator.isRobotSlowEnoughForCurrentZone();
                          return launcherReady
                              && motivatorReady
                              && turretReady
                              && hoodReady
                              && achievable
                              && robotSlow;
                        })),
                Commands.sequence(
                    Commands.waitSeconds(5.0),
                    Commands.runOnce(
                        () -> {
                          ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                          boolean launcherReady = launcher.isReady();
                          boolean motivatorReady =
                              motivator == null
                                  || motivator.getState() == Motivator.MotivatorState.READY;
                          boolean turretReady = turret.getState() == Turret.TurretState.READY;
                          boolean hoodReady =
                              hood == null || hood.getState() == Hood.HoodState.READY;
                          boolean achievable =
                              shot != null && (isLutDevOverrideActive() || shot.achievable());
                          Logger.recordOutput("SmartLaunch/Phase", "TIMED_OUT");
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
                                  + achievable
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
                  Logger.recordOutput("SmartLaunch/Phase", "FIRING");
                  SmartDashboard.putString("Match/Status/State", "Smart Launch - Feeding");
                  ShotCalculator.ShotResult feedShot = coordinator.getCurrentShot();
                  logShotStatus(
                      "SmartLaunch", launcher, hood, motivator, getEffectiveRPM(feedShot));
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
                // Feed is suppressed during large turret movements (flips).
                motivator != null
                    ? Commands.run(
                        new Runnable() {
                          boolean[] flipSuppressed = {false};

                          @Override
                          public void run() {
                            if (turret != null && shouldSuppressFeed(turret, flipSuppressed)) {
                              motivator.stopMotivator();
                            } else {
                              ShotCalculator.ShotResult s = coordinator.getCurrentShot();
                              double launcherRPM = getEffectiveRPM(s);
                              motivator.setMotivatorVelocity(getMotivatorRPM(launcherRPM));
                            }
                          }
                        },
                        motivator)
                    : Commands.none(),

                // Run spindexer: teleop always feeds (operator is holding shoot button),
                // auto feeds when slow enough, reciprocates when too fast (transiting).
                // Feed is suppressed during large turret movements (flips).
                spindexer != null
                    ? Commands.run(
                        new Runnable() {
                          boolean wasReciprocating = false;
                          boolean[] flipSuppressed = {false};

                          @Override
                          public void run() {
                            boolean suppressed =
                                turret != null && shouldSuppressFeed(turret, flipSuppressed);
                            Logger.recordOutput("SmartLaunch/FlipSuppressed", suppressed);
                            if (suppressed) {
                              spindexer.stopSpindexer();
                              return;
                            }
                            double dist = coordinator.getDistanceToTarget();
                            double spnRPM =
                                coordinator.isInPassZone()
                                    ? getSpindexerPassRPM()
                                    : getSpindexerRPM(dist > 0 ? dist : 1.16);
                            if (DriverStation.isTeleop()
                                || coordinator.isRobotSlowEnoughForCurrentZone()) {
                              wasReciprocating = false;
                              spindexer.setSpindexerVelocity(spnRPM);
                            } else if (DriverStation.isAutonomous()) {
                              wasReciprocating = true;
                              spindexer.reciprocate();
                            } else if (wasReciprocating) {
                              wasReciprocating = false;
                              spindexer.stopReciprocateWithKick();
                            } else {
                              spindexer.stopSpindexer();
                            }
                            Logger.recordOutput(
                                "SmartLaunch/Feeding",
                                DriverStation.isTeleop()
                                    || coordinator.isRobotSlowEnoughForCurrentZone());
                          }
                        },
                        spindexer)
                    : Commands.none(),

                // Fire balls in simulation (same feed conditions)
                createSimFiringLoop(coordinator, launcher, turret)))
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
              Logger.recordOutput("SmartLaunch/Phase", "IDLE");
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
    return smartLaunchCommandImpl(launcher, coordinator, motivator, turret, hood, spindexer)
        .beforeStarting(() -> DriveCommands.setSpeedLimit(coordinator.getShootOnTheMoveSpeedMps()))
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
                boolean turretReady = turret.getState() == Turret.TurretState.READY;
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

  // ===== Shared Readiness Check =====

  /**
   * Check if all shooting subsystems are ready to fire. Uses state machines rather than raw
   * atSetpoint() calls — single source of truth for readiness gating.
   *
   * @param launcher The launcher subsystem
   * @param motivator The motivator subsystem (can be null)
   * @param turret The turret subsystem
   * @param hood The hood subsystem (can be null)
   * @param shot The current shot result (can be null)
   * @return true if all subsystems are at setpoint and shot is achievable
   */
  static boolean isAllSubsystemsReady(
      Launcher launcher,
      Motivator motivator,
      Turret turret,
      Hood hood,
      ShotCalculator.ShotResult shot) {
    return launcher.isReady()
        && (motivator == null || motivator.getState() == Motivator.MotivatorState.READY)
        && turret.getState() == Turret.TurretState.READY
        && (hood == null || hood.getState() == Hood.HoodState.READY)
        && shot != null
        && (isLutDevOverrideActive() || shot.achievable());
  }

  // ===== Continuous Smart Launch (for autonomous auto-shoot) =====

  /**
   * Continuous smart launch command for use during autonomous paths. Unlike smartLaunchCommand,
   * this has no initial wait phase — it starts tracking and firing immediately (launcher may
   * already be spinning from initialSmartLaunch). Runs until interrupted (path ends → parallel
   * group ends).
   *
   * <p>Uses zone-based speed gating from the coordinator for hub vs pass speed limits. Controls all
   * shooting subsystems (launcher RPM, turret angle, hood angle, motivator, spindexer) from live
   * shot calculations.
   *
   * @param launcher The launcher subsystem
   * @param coordinator The shooting coordinator (provides shot calculations and zone checks)
   * @param motivator The motivator subsystem (can be null)
   * @param turret The turret subsystem
   * @param hood The hood subsystem (can be null)
   * @param spindexer The spindexer subsystem (can be null)
   * @param armOnPassZone When true, feeding is suppressed until the robot first enters a PASS or
   *     LONG_PASS zone. Used by SPRINT autos to prevent firing preloads at the start position.
   *     Launcher/turret/hood still track the live shot while waiting.
   * @return Command that continuously tracks and fires while active
   */
  public static Command continuousSmartLaunchCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer,
      boolean armOnPassZone) {
    // Mutable flags captured by lambdas.
    // feedingArmed: flips to true once the robot transitions into a pass zone.
    //   Once armed, stays armed permanently (robot can return to alliance zone and fire).
    // wasOutsidePassZone: prevents arming if the robot starts in/near the pass zone.
    //   Must leave the pass zone first, then re-enter to arm. For SPRINT autos starting
    //   in the alliance zone this is immediate; for edge cases it prevents false arming.
    final boolean[] feedingArmed = {!armOnPassZone};
    final boolean[] wasOutsidePassZone = {false};
    final boolean[] motivatorStarted = {false};

    return Commands.parallel(
            // Arming monitor — detects transition INTO a pass zone. Requires the robot
            // to have been outside the pass zone first (prevents false arming if the
            // starting position happens to be in/near the pass zone boundary).
            Commands.run(
                () -> {
                  if (feedingArmed[0]) return;
                  boolean inPassZone = coordinator.isInPassZone();
                  if (!inPassZone) {
                    wasOutsidePassZone[0] = true;
                  } else if (wasOutsidePassZone[0]) {
                    // Transition: was outside → now inside → arm subsystems.
                    // Don't set feedingActive yet — that enables recovery boost which
                    // makes no sense before the launcher is at speed. It gets set once
                    // the spindexer actually starts feeding (launcher at setpoint).
                    feedingArmed[0] = true;
                    Logger.recordOutput("ContinuousSmartLaunch/FeedingArmed", true);
                    System.out.println("[ContinuousSmartLaunch] Armed — entered pass zone");
                  }
                }),

            // Launcher — pre-spin to idle RPM before armed, full RPM once armed
            Commands.run(
                () -> {
                  if (!feedingArmed[0]) {
                    // Pre-spin so wheels are already moving when we enter the pass zone
                    launcher.setVelocity(2000);
                    return;
                  }
                  ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                  if (shot != null) {
                    double rpm = getEffectiveRPM(shot);
                    double hoodDeg = getEffectiveHoodDeg(shot);
                    launcher.setVelocity(rpm);
                    coordinator.getBatchRecorder().cacheParams(rpm, hoodDeg);
                  }
                },
                launcher),

            // Turret — always track target so it's pre-aimed when feeding arms
            Commands.run(
                () -> {
                  ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                  if (shot != null) {
                    turret.setOutsideTurretAngle(shot.turretAngleDeg());
                  }
                },
                turret),

            // Hood — track target angle (only when armed)
            hood != null
                ? Commands.run(
                    () -> {
                      if (!feedingArmed[0]) return;
                      ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                      if (shot != null) {
                        hood.setHoodAngle(getEffectiveHoodDeg(shot));
                      }
                    },
                    hood)
                : Commands.none(),

            // Motivator — wait for armed + launcher ready to start, then keep running
            motivator != null
                ? Commands.run(
                    () -> {
                      if (!feedingArmed[0]) return;
                      if (!motivatorStarted[0]) {
                        if (!launcher.isReady()) return;
                        motivatorStarted[0] = true;
                      }
                      ShotCalculator.ShotResult s = coordinator.getCurrentShot();
                      double launcherRPM = getEffectiveRPM(s);
                      motivator.setMotivatorVelocity(getMotivatorRPM(launcherRPM));
                    },
                    motivator)
                : Commands.none(),

            // Spindexer — feed fuel (gated on armed + launcher ready + turret aimed + zone speed,
            // reciprocate in neutral zone only when launcher is at speed, hold still otherwise)
            spindexer != null
                ? Commands.run(
                    new Runnable() {
                      boolean wasReciprocating = false;

                      @Override
                      public void run() {
                        boolean armed = feedingArmed[0];
                        boolean launcherReady = launcher.isReady();
                        boolean turretAimed = turret.getState() == Turret.TurretState.READY;
                        boolean speedOk = coordinator.isRobotSlowEnoughForCurrentZone();
                        Logger.recordOutput("ContinuousSmartLaunch/Gate/Armed", armed);
                        Logger.recordOutput(
                            "ContinuousSmartLaunch/Gate/LauncherReady", launcherReady);
                        Logger.recordOutput("ContinuousSmartLaunch/Gate/TurretAimed", turretAimed);
                        Logger.recordOutput("ContinuousSmartLaunch/Gate/SpeedOk", speedOk);
                        boolean feedOk = armed && launcherReady && turretAimed && speedOk;
                        boolean spindexerFeedOk = armed && turretAimed && speedOk;
                        if (feedOk) {
                          launcher.setFeedingActive(true);
                        }
                        if (spindexerFeedOk) {
                          wasReciprocating = false;
                          double dist = coordinator.getDistanceToTarget();
                          double spnRPM =
                              coordinator.isInPassZone()
                                  ? getSpindexerPassRPM()
                                  : getSpindexerRPM(dist > 0 ? dist : 1.16);
                          spindexer.setSpindexerVelocity(spnRPM);
                        } else if (!speedOk) {
                          wasReciprocating = true;
                          spindexer.reciprocate();
                        } else if (wasReciprocating) {
                          wasReciprocating = false;
                          spindexer.stopReciprocateWithKick();
                        } else {
                          spindexer.stopSpindexer();
                        }
                        Logger.recordOutput("ContinuousSmartLaunch/FeedingSuppressed", !feedOk);
                      }
                    },
                    spindexer)
                : Commands.none(),

            // Fire balls in simulation
            coordinator != null
                ? createContinuousSimFiringLoop(
                    coordinator, launcher, motivator, turret, hood, feedingArmed)
                : Commands.none())
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
              if (hood != null) {
                hood.setHoodAngle(hood.getMinAngle());
              }
              Logger.recordOutput("ContinuousSmartLaunch/Active", false);
            })
        .beforeStarting(
            () -> {
              // Only set feeding active immediately if not waiting for pass zone
              if (!armOnPassZone) {
                launcher.setFeedingActive(true);
              }
              Logger.recordOutput("ContinuousSmartLaunch/Active", true);
              Logger.recordOutput("ContinuousSmartLaunch/FeedingArmed", feedingArmed[0]);
              System.out.println(
                  "[ContinuousSmartLaunch] Started"
                      + (armOnPassZone ? " (waiting for pass zone)" : " (armed immediately)"));
            })
        .withName("ContinuousSmartLaunch");
  }

  /**
   * Auto-tracking stationary shooting command. Continuously tracks the hub (turret, launcher,
   * motivator) in far trench, near trench, and alliance zones, but only feeds the spindexer when
   * the robot is stationary in near trench or alliance zone. Hood tracks target angle in near
   * trench and alliance zone only — in far trench the hood stays at min angle for safety.
   *
   * <p>Designed for autos where the path stops in a shooting zone to fire automatically, without
   * needing explicit shoot commands in the path.
   *
   * @param launcher The launcher subsystem
   * @param coordinator The shooting coordinator
   * @param motivator The motivator subsystem (can be null)
   * @param turret The turret subsystem
   * @param hood The hood subsystem (can be null)
   * @param spindexer The spindexer subsystem (can be null)
   * @return Command that tracks continuously and fires when stationary in a safe zone
   */
  public static Command autoTrackingStationaryCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {
    final boolean[] motivatorStarted = {false};

    return Commands.parallel(
            // Launcher — spin to hub RPM in tracking zones, idle otherwise
            Commands.run(
                () -> {
                  ZoneDetector.Zone zone = coordinator.getCurrentZone();
                  boolean inTrackZone =
                      zone == ZoneDetector.Zone.TRENCH_FAR
                          || zone == ZoneDetector.Zone.TRENCH_NEAR
                          || zone == ZoneDetector.Zone.ALLIANCE;
                  if (!inTrackZone) {
                    launcher.setVelocity(2000);
                    return;
                  }
                  ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                  if (shot != null) {
                    double rpm = getEffectiveRPM(shot);
                    double hoodDeg = getEffectiveHoodDeg(shot);
                    launcher.setVelocity(rpm);
                    coordinator.getBatchRecorder().cacheParams(rpm, hoodDeg);
                  }
                },
                launcher),

            // Turret — track hub in tracking zones only
            Commands.run(
                () -> {
                  ZoneDetector.Zone zone = coordinator.getCurrentZone();
                  boolean inTrackZone =
                      zone == ZoneDetector.Zone.TRENCH_FAR
                          || zone == ZoneDetector.Zone.TRENCH_NEAR
                          || zone == ZoneDetector.Zone.ALLIANCE;
                  if (!inTrackZone) return;
                  ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                  if (shot != null) {
                    turret.setOutsideTurretAngle(shot.turretAngleDeg());
                  }
                },
                turret),

            // Hood — track target in near trench and alliance only.
            // In far trench, hold at min angle (not safe to raise under trench ceiling).
            // Outside tracking zones, do nothing.
            hood != null
                ? Commands.run(
                    () -> {
                      ZoneDetector.Zone zone = coordinator.getCurrentZone();
                      if (zone == ZoneDetector.Zone.TRENCH_FAR) {
                        hood.setHoodAngle(hood.getMinAngle());
                        return;
                      }
                      if (zone != ZoneDetector.Zone.TRENCH_NEAR
                          && zone != ZoneDetector.Zone.ALLIANCE) {
                        return;
                      }
                      ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                      if (shot != null) {
                        hood.setHoodAngle(getEffectiveHoodDeg(shot));
                      }
                    },
                    hood)
                : Commands.none(),

            // Motivator — spin in tracking zones once launcher is ready
            motivator != null
                ? Commands.run(
                    () -> {
                      ZoneDetector.Zone zone = coordinator.getCurrentZone();
                      boolean inTrackZone =
                          zone == ZoneDetector.Zone.TRENCH_FAR
                              || zone == ZoneDetector.Zone.TRENCH_NEAR
                              || zone == ZoneDetector.Zone.ALLIANCE;
                      if (!inTrackZone) {
                        motivatorStarted[0] = false;
                        return;
                      }
                      if (!motivatorStarted[0]) {
                        if (!launcher.isReady()) return;
                        motivatorStarted[0] = true;
                      }
                      ShotCalculator.ShotResult s = coordinator.getCurrentShot();
                      double launcherRPM = getEffectiveRPM(s);
                      motivator.setMotivatorVelocity(getMotivatorRPM(launcherRPM));
                    },
                    motivator)
                : Commands.none(),

            // Spindexer — feed only when stationary in near trench or alliance zone
            spindexer != null
                ? Commands.run(
                    () -> {
                      ZoneDetector.Zone zone = coordinator.getCurrentZone();
                      boolean inShootZone =
                          zone == ZoneDetector.Zone.TRENCH_NEAR
                              || zone == ZoneDetector.Zone.ALLIANCE;
                      boolean stationary = coordinator.isRobotStationary();
                      boolean launcherReady = launcher.isReady();
                      boolean turretAimed = turret.getState() == Turret.TurretState.READY;

                      Logger.recordOutput("AutoTrackStationary/Gate/InShootZone", inShootZone);
                      Logger.recordOutput("AutoTrackStationary/Gate/Stationary", stationary);
                      Logger.recordOutput("AutoTrackStationary/Gate/LauncherReady", launcherReady);
                      Logger.recordOutput("AutoTrackStationary/Gate/TurretAimed", turretAimed);

                      boolean feedOk = inShootZone && stationary && launcherReady && turretAimed;
                      if (feedOk) {
                        launcher.setFeedingActive(true);
                        double dist = coordinator.getDistanceToTarget();
                        double spnRPM = getSpindexerRPM(dist > 0 ? dist : 1.16);
                        spindexer.setSpindexerVelocity(spnRPM);
                      } else {
                        spindexer.reciprocate();
                      }
                      Logger.recordOutput("AutoTrackStationary/FeedingSuppressed", !feedOk);
                    },
                    spindexer)
                : Commands.none(),

            // Fire balls in simulation
            coordinator != null
                ? createAutoTrackingSimFiringLoop(coordinator, launcher, motivator, turret, hood)
                : Commands.none())
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
              if (hood != null) {
                hood.setHoodAngle(hood.getMinAngle());
              }
              Logger.recordOutput("AutoTrackStationary/Active", false);
            })
        .beforeStarting(
            () -> {
              launcher.setFeedingActive(true);
              Logger.recordOutput("AutoTrackStationary/Active", true);
              System.out.println("[AutoTrackStationary] Started");
            })
        .withName("AutoTrackStationary");
  }

  /**
   * Sim firing loop for auto-tracking stationary. Gates on stationary + in shoot zone (near trench
   * or alliance).
   */
  private static Command createAutoTrackingSimFiringLoop(
      ShootingCoordinator coordinator,
      Launcher launcher,
      Motivator motivator,
      Turret turret,
      Hood hood) {
    return Commands.sequence(
            Commands.waitUntil(
                () -> {
                  ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                  boolean subsReady = isAllSubsystemsReady(launcher, motivator, turret, hood, shot);
                  boolean stationary = coordinator.isRobotStationary();
                  ZoneDetector.Zone zone = coordinator.getCurrentZone();
                  boolean inShootZone =
                      zone == ZoneDetector.Zone.TRENCH_NEAR || zone == ZoneDetector.Zone.ALLIANCE;
                  return subsReady && stationary && inShootZone;
                }),
            Commands.runOnce(
                () -> {
                  ShotVisualizer visualizer = coordinator.getVisualizer();
                  if (visualizer != null && visualizer.getFuelCount() <= 0) return;
                  coordinator.launchFuel();
                  launcher.notifyBallFired();
                  Logger.recordOutput("ShotLog/LastShotTime", Timer.getFPGATimestamp());

                  int fuelRemaining = visualizer != null ? visualizer.getFuelCount() : 0;
                  Logger.recordOutput("ShotLog/FuelRemaining", fuelRemaining);
                }),
            Commands.waitSeconds(MIN_SHOT_INTERVAL_SECONDS))
        .repeatedly();
  }

  /**
   * Sim firing loop for continuous smart launch. Uses isAllSubsystemsReady + zone speed gating +
   * feeding armed check.
   */
  private static Command createContinuousSimFiringLoop(
      ShootingCoordinator coordinator,
      Launcher launcher,
      Motivator motivator,
      Turret turret,
      Hood hood,
      boolean[] feedingArmed) {
    return Commands.sequence(
            Commands.waitUntil(
                () -> {
                  ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                  boolean armed = feedingArmed[0];
                  boolean subsReady = isAllSubsystemsReady(launcher, motivator, turret, hood, shot);
                  boolean speedOk = coordinator.isRobotSlowEnoughForCurrentZone();
                  Logger.recordOutput("ContinuousSimFire/Armed", armed);
                  Logger.recordOutput("ContinuousSimFire/SubsReady", subsReady);
                  Logger.recordOutput("ContinuousSimFire/SpeedOk", speedOk);
                  Logger.recordOutput(
                      "ContinuousSimFire/LauncherState", launcher.getState().name());
                  Logger.recordOutput(
                      "ContinuousSimFire/MotivatorState",
                      motivator != null ? motivator.getState().name() : "NULL");
                  return armed && subsReady && speedOk;
                }),
            Commands.runOnce(
                () -> {
                  ShotVisualizer visualizer = coordinator.getVisualizer();
                  if (visualizer != null && visualizer.getFuelCount() <= 0) return;
                  coordinator.launchFuel();
                  launcher.notifyBallFired();
                  Logger.recordOutput("ShotLog/LastShotTime", Timer.getFPGATimestamp());

                  int fuelRemaining = visualizer != null ? visualizer.getFuelCount() : 0;
                  Logger.recordOutput("ShotLog/FuelRemaining", fuelRemaining);
                }),
            Commands.waitSeconds(MIN_SHOT_INTERVAL_SECONDS))
        .repeatedly();
  }

  /**
   * Simulation firing loop. Waits for alignment and setpoint, then launches fuel repeatedly.
   *
   * @param coordinator The shooting coordinator
   * @param launcher The launcher subsystem
   * @param turret The turret subsystem
   * @return Command that fires repeatedly until out of fuel
   */
  private static Command createSimFiringLoop(
      ShootingCoordinator coordinator, Launcher launcher, Turret turret) {
    return Commands.sequence(
            Commands.waitUntil(
                () ->
                    turret.getState() == Turret.TurretState.READY
                        && launcher.isReady()
                        && coordinator.isRobotSlowEnoughForCurrentZone()),
            Commands.runOnce(
                () -> {
                  coordinator.launchFuel();
                  launcher.notifyBallFired();
                  Logger.recordOutput("ShotLog/LastShotTime", Timer.getFPGATimestamp());

                  ShotVisualizer visualizer = coordinator.getVisualizer();
                  int fuelRemaining = visualizer != null ? visualizer.getFuelCount() : 0;
                  Logger.recordOutput("ShotLog/FuelRemaining", fuelRemaining);
                }),
            Commands.waitSeconds(MIN_SHOT_INTERVAL_SECONDS))
        .repeatedly()
        .until(
            () -> {
              ShotVisualizer visualizer = coordinator.getVisualizer();
              return visualizer == null || visualizer.getFuelCount() <= 0;
            });
  }

  // ===== Shot Logging =====

  /** Log target vs actual for all subsystems when feeding starts. */
  private static void logShotStatus(
      String label, Launcher launcher, Hood hood, Motivator motivator, double targetRPM) {
    double actualRPM = launcher != null ? launcher.getVelocity() : 0;
    double hoodTarget = hood != null ? hood.getTargetAngle() : 0;
    double hoodActual = hood != null ? hood.getCurrentAngle() : 0;
    double motTarget = motivator != null ? motivator.getMotivatorTargetRPM() : 0;
    double motActual = motivator != null ? motivator.getMotivatorWheelVelocity() : 0;
    System.out.printf(
        "[%s] Feeding — Launcher: %.0f/%.0f RPM | Hood: %.1f/%.1f° | Motivator: %.0f/%.0f RPM%n",
        label, targetRPM, actualRPM, hoodTarget, hoodActual, motTarget, motActual);
  }

  // ===== LUT Dev Override Helpers =====

  private static boolean isLutDevOverrideActive() {
    return SmartDashboard.getBoolean("LUTDev/UseOverrides", false);
  }

  // Safety cap: matches LauncherIOSparkFlex.MAX_VELOCITY_RPM hardware limit
  private static final double MAX_LAUNCHER_RPM = 5000.0;

  private static double getEffectiveRPM(ShotCalculator.ShotResult shot) {
    double rpm;
    if (isLutDevOverrideActive()) {
      rpm = lutDevOverrideRPM.get() + launcherTrimRPM;
    } else {
      rpm = shot != null ? shot.launcherRPM() + launcherTrimRPM : 0.0;
    }
    return Math.min(rpm, MAX_LAUNCHER_RPM);
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
