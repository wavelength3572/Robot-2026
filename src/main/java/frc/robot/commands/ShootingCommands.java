package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.motivator.Motivator;
import frc.robot.subsystems.shooting.ShootingCoordinator;
import frc.robot.subsystems.shooting.ShotCalculator;
import frc.robot.subsystems.shooting.ShotVisualizer;
import frc.robot.subsystems.shooting.StationaryShotBatchRecorder;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretState;
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
      // System.out.println("[Shooting] Mode changed to: " + mode);
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
      new LoggedTunableNumber("Shots/HubShot/LauncherRPM", 2450.0);
  private static final LoggedTunableNumber hubShotHoodAngleDeg =
      new LoggedTunableNumber("Shots/HubShot/HoodAngleDeg", 15.0);
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

  // Motivator RPM as a ratio of launcher RPM: motivatorRPM = launcherRPM * ratio
  private static final LoggedTunableNumber motivatorLauncherRatio =
      new LoggedTunableNumber(
          "Shots/SmartLaunch/MotivatorLauncherRatio", 0.2); // consider .68 was .565

  // Spindexer RPM lerped by distance: close = max, far = min
  private static final LoggedTunableNumber spindexerCloseRPM =
      new LoggedTunableNumber("Shots/SmartLaunch/SpindexerCloseRPM", 300.0);
  private static final LoggedTunableNumber spindexerFarRPM =
      new LoggedTunableNumber("Shots/SmartLaunch/SpindexerFarRPM", 300.0);
  // Fixed spindexer RPM used in pass/neutral zones (no distance lerp)
  private static final LoggedTunableNumber spindexerPassRPM =
      new LoggedTunableNumber("Shots/SmartLaunch/SpindexerPassRPM", 375.0);

  // ===== LUT Dev Overrides (manual RPM/hood for data collection) =====
  private static final LoggedTunableNumber lutDevOverrideRPM =
      new LoggedTunableNumber("LUTDev/OverrideRPM", 2500.0);
  private static final LoggedTunableNumber lutDevOverrideHoodDeg =
      new LoggedTunableNumber("LUTDev/OverrideHoodDeg", 25.0);

  // ===== Launcher RPM Trim =====

  // Trim value added to all launcher RPM targets. Adjusted via button box 1 axis
  // knob.
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
      // System.out.println("[Trim] Launcher RPM trim set to " + trimRPM);
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
  private static final double MIN_SHOT_INTERVAL_SECONDS = 0.14;

  // Tracks last sim fire time so the throttle works across all feeding paths
  private static double lastSimFireTimestamp = 0;

  /**
   * Fire a simulated ball if in SIM/REPLAY mode, throttled to MIN_SHOT_INTERVAL_SECONDS. Call this
   * from the real feeding code path — the same conditions that gate real feeding automatically gate
   * sim firing, so sim behavior matches the real robot exactly.
   */
  private static void fireSimBallIfReady(ShootingCoordinator coordinator, Launcher launcher) {
    if (Constants.currentMode == Constants.Mode.REAL) return;
    double now = Timer.getFPGATimestamp();
    if (now - lastSimFireTimestamp < MIN_SHOT_INTERVAL_SECONDS) return;
    ShotVisualizer visualizer = coordinator.getVisualizer();
    if (visualizer != null && visualizer.getFuelCount() <= 0) return;
    coordinator.launchFuel();
    launcher.notifyBallFired();
    lastSimFireTimestamp = now;
    Logger.recordOutput("ShotLog/LastShotTime", now);
    int fuelRemaining = visualizer != null ? visualizer.getFuelCount() : 0;
    Logger.recordOutput("ShotLog/FuelRemaining", fuelRemaining);
  }

  // Always wait for setpoint recovery before firing next shot
  private static final boolean WAIT_FOR_RECOVERY = true;

  private ShootingCommands() {
    // Static factory class
  }

  /** Derive motivator RPM from launcher RPM: motivatorRPM = launcherRPM * ratio. */
  public static double getMotivatorRPM(double launcherRPM) {
    return launcherRPM * motivatorLauncherRatio.get();
  }

  /**
   * Derive motivator RPM from launcher RPM using a fixed ratio.
   *
   * @param launcherRPM current launcher RPM
   * @param coordinator the shooting coordinator (unused, kept for API compatibility)
   */
  public static double getMotivatorRPM(double launcherRPM, ShootingCoordinator coordinator) {
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

    // Measured TOF input — students fill this in from slow-mo camera analysis
    // (seconds)
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
                  double turretAngle = turretAngleDegSupplier.getAsDouble();
                  double hoodAngle = hoodAngleDegSupplier.getAsDouble();
                  double launcherRPM = trimmedLauncherRPM.getAsDouble();

                  launcher.setVelocity(launcherRPM);
                  if (hood != null) {
                    hood.setHoodAngle(hoodAngle);
                  }
                  turret.setOutsideTurretAngle(turretAngle);

                  if (motivator != null) {
                    motivator.stopMotivator();
                    // motivator.setMotivatorVelocity(motivatorRPMSupplier.getAsDouble());
                  }

                  if (coordinator != null) {
                    coordinator.setManualShotParameters(launcherRPM, hoodAngle, turretAngle);
                  }

                  SmartDashboard.putString("Match/Status/State", "Fixed Launch - Positioning");
                }),

            // Phase 1.5: Brief reverse pulse to clear balls from motivator/spindexer
            // while the launcher is spinning up
            // Commands.sequence(
            //     Commands.runOnce(
            //         () -> {
            //           if (motivator != null) {
            //             motivator.setMotivatorVoltage(-1.0);
            //           }
            //           if (spindexer != null) {
            //             spindexer.reverseSpindexer(250.0);
            //           }
            //         }),
            //     Commands.waitSeconds(0.2),
            //     Commands.runOnce(
            //         () -> {
            //           if (motivator != null) {
            //             motivator.stopMotivator();
            //           }
            //           if (spindexer != null) {
            //             spindexer.stopSpindexer();
            //           }
            //         })),

            // Phase 2: Wait for everything to reach setpoint (with 2s timeout)
            Commands.race(
                Commands.sequence(
                    Commands.waitUntil(
                        () -> {
                          if (motivator != null
                              && launcher.isReady()
                              && turret.atTarget()
                              && hood.atTarget()) {
                            motivator.setMotivatorVelocity(
                                getMotivatorRPM(motivatorRPMSupplier.getAsDouble()));
                          }

                          boolean launcherReady = launcher.isReady();
                          boolean motivatorReady =
                              motivator == null
                                  || motivator.getState() == Motivator.MotivatorState.READY;
                          boolean turretReady = turret.atTarget();
                          boolean hoodReady =
                              hood == null || hood.getState() == Hood.HoodState.READY;

                          return launcherReady && motivatorReady && turretReady && hoodReady;
                        })),
                Commands.sequence(
                    Commands.waitSeconds(2.0),
                    Commands.runOnce(
                        () -> {
                          // System.out.println(
                          //     "[FixedShot] WARNING: Setup timeout - continuing anyway!");
                          SmartDashboard.putString(
                              "Match/Status/State", "Fixed Launch TIMEOUT - continuing anyway");
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

                // Keep motivator running
                motivator != null
                    ? Commands.run(
                        () -> {
                          if (turret.getState() == Turret.TurretState.FLIPPING
                              || turret.getState() == Turret.TurretState.STALLED) {
                            motivator.stopMotivator();
                          } else {
                            motivator.setMotivatorVelocity(motivatorRPMSupplier.getAsDouble());
                          }
                        },
                        motivator)
                    : Commands.none(),

                // Run spindexer to feed fuel continuously (fixed shots are stationary presets)
                spindexer != null
                    ? Commands.run(
                        () -> {
                          spindexer.setSpindexerVelocity(spindexerRPMSupplier.getAsDouble());
                          if (coordinator != null) {
                            fireSimBallIfReady(coordinator, launcher);
                          }
                        },
                        spindexer)
                    : Commands.none()))
        .finallyDo(
            () -> {
              if (turret.getState() == TurretState.STALLED) {
                turret.forceTurretOutOfStallState();
              }
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
              if (hood != null) {
                hood.setHoodAngle(hood.getMinAngle());
              }
              setMode(ShootingMode.COMPETITION);
              SmartDashboard.putString("Match/Status/State", "Fixed Launch Stopped");
              // System.out.println("[FixedShot] Stopped");
            })
        .withName("FixedShot");
  }

  /**
   * Aggressive SmartLaunch variant ("dangerous"). Motivator pre-spins in parallel with launcher and
   * aiming — no reverse pulse, no waiting for FIRING. Saves ~0.3-0.5s per firing cycle. Subsystems
   * idle while auto collecting (open neutral/opponent zones). Hood clamps in trench while moving,
   * pops up below 0.6 m/s in alliance trench only.
   *
   * @param launcher The launcher subsystem
   * @param coordinator The shooting coordinator (owns the state machine)
   * @param motivator The motivator subsystem (can be null)
   * @param turret The turret subsystem
   * @param hood The hood subsystem (can be null)
   * @param spindexer The spindexer subsystem (can be null)
   * @return Command that tracks and fires based on coordinator state while held
   */
  public static Command smartLaunchDangerousCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {
    return smartLaunchDangerousCommand(
        launcher,
        coordinator,
        motivator,
        turret,
        hood,
        spindexer,
        ShootingCoordinator.ArmTrigger.IMMEDIATE);
  }

  /**
   * State-machine-driven SmartLaunch with configurable arm trigger. For sprint autos, use
   * ON_PASS_ZONE or ON_TRENCH_RETURN to prevent shooting preloads at the start.
   */
  public static Command smartLaunchDangerousCommand(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer,
      ShootingCoordinator.ArmTrigger armTrigger) {

    return Commands.parallel(
            // Launcher — track shot RPM (idles to 0 while auto collecting)
            Commands.run(
                () -> {
                  ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                  if (shot != null) {
                    double rpm = getEffectiveRPM(shot);
                    double hoodDeg = getEffectiveHoodDeg(shot);
                    launcher.setVelocity(coordinator.getEffectiveLauncherRPM(rpm));
                    coordinator.getBatchRecorder().cacheParams(rpm, hoodDeg);
                  }
                },
                launcher),

            // Turret — track shot angle, but hold position while auto collecting
            Commands.run(
                () -> {
                  if (coordinator.isAutoCollecting()) return; // hold position
                  ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                  if (shot != null) {
                    turret.setOutsideTurretAngle(shot.turretAngleDeg());
                  }
                },
                turret),

            // Hood — track shot angle; idle at min while auto collecting
            hood != null
                ? Commands.run(
                    () -> {
                      if (coordinator.isAutoCollecting()) {
                        hood.setHoodAngle(hood.getMinAngle());
                        return;
                      }
                      ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                      if (shot != null) {
                        hood.setHoodAngle(getEffectiveHoodDeg(shot));
                      }
                    },
                    hood)
                : Commands.none(),

            // Motivator — always pre-spin in teleop (ready for passing or shooting).
            // In auto, idle only while collecting (open neutral/opponent zones).
            // Reverse pulse runs on first spin-up (not on FIRING entry) to clear any
            // ball stuck at the motivator/launcher interface during free time.
            motivator != null
                ? Commands.run(
                    new Runnable() {
                      private final Timer reversePulseTimer = new Timer();
                      private boolean reversing = false;
                      private boolean wasIdle = true;
                      private static final double REVERSE_PULSE_SEC = 0.2;

                      @Override
                      public void run() {
                        boolean shouldIdle = coordinator.isAutoCollecting();
                        if (shouldIdle) {
                          motivator.stopMotivator();
                          wasIdle = true;
                          reversing = false;
                          return;
                        }

                        // Trigger reverse pulse when transitioning from idle to spinning
                        if (wasIdle) {
                          wasIdle = false;
                          reversing = true;
                          reversePulseTimer.restart();
                        }

                        if (reversing) {
                          if (reversePulseTimer.hasElapsed(REVERSE_PULSE_SEC)) {
                            reversing = false;
                          } else {
                            motivator.setMotivatorVoltage(-1.0);
                            return;
                          }
                        }

                        ShotCalculator.ShotResult s = coordinator.getCurrentShot();
                        if (s != null) {
                          double launcherRPM = getEffectiveRPM(s);
                          motivator.setMotivatorVelocity(getMotivatorRPM(launcherRPM, coordinator));
                        } else {
                          motivator.stopMotivator();
                        }
                      }
                    },
                    motivator)
                : Commands.none(),

            // Spindexer — feed only when coordinator allows AND motivator is at speed.
            spindexer != null
                ? Commands.run(
                    () -> {
                      boolean feedingAllowed = coordinator.isFeedingAllowed();
                      boolean motivatorReady =
                          motivator == null
                              || motivator.getState() == Motivator.MotivatorState.READY;
                      if (feedingAllowed && motivatorReady) {
                        launcher.setFeedingActive(true);
                        double dist = coordinator.getDistanceToTarget();
                        double spnRPM =
                            coordinator.isInPassZone()
                                ? getSpindexerPassRPM()
                                : getSpindexerRPM(dist > 0 ? dist : 1.16);
                        if (turret.getState() == Turret.TurretState.FLIPPING) {
                          spindexer.stopSpindexer();
                        } else {
                          spindexer.setSpindexerVelocity(spnRPM);
                          fireSimBallIfReady(coordinator, launcher);
                        }
                      } else if (feedingAllowed) {
                        // Motivator spinning up but not at speed — hold spindexer still
                        // to avoid pushing balls into the accelerating motivator
                        launcher.setFeedingActive(false);
                        spindexer.stopSpindexer();
                      } else {
                        // Not firing — safe to reciprocate since motivator is stopped
                        launcher.setFeedingActive(false);
                        spindexer.reciprocate();
                      }
                    },
                    spindexer)
                : Commands.none())
        .beforeStarting(
            () -> {
              setMode(ShootingMode.COMPETITION);
              coordinator.setSmartLaunchActive(true, armTrigger);

              // Start a LUT batch
              ShotVisualizer vis = coordinator.getVisualizer();
              if (vis != null) {
                coordinator.getBatchRecorder().startBatch(vis.getFuelCount());
              }

              Logger.recordOutput("SmartLaunch/Phase", "SM_ACTIVE");
              SmartDashboard.putString("Match/Status/State", "SmartLaunch 2.0 - Active");
            })
        .finallyDo(
            () -> {
              coordinator.setSmartLaunchActive(false);
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
              coordinator.clearManualShotParameters();
              setMode(ShootingMode.COMPETITION);
              Logger.recordOutput("SmartLaunch/Phase", "IDLE");
              SmartDashboard.putString("Match/Status/State", "[SmartLaunch 2.0] Stopped");
            })
        .withName("SmartLaunchDangerous");
  }

  // Old sequential smartLaunchCommandSeq removed — replaced by smartLaunchDangerousCommand.

  // ===== Shot Logging =====

  /** Log target vs actual for all subsystems when feeding starts. */
  private static void logShotStatus(
      String label, Launcher launcher, Hood hood, Motivator motivator, double targetRPM) {
    double actualRPM = launcher != null ? launcher.getVelocity() : 0;
    double hoodTarget = hood != null ? hood.getTargetAngle() : 0;
    double hoodActual = hood != null ? hood.getCurrentAngle() : 0;
    double motTarget = motivator != null ? motivator.getMotivatorTargetRPM() : 0;
    double motActual = motivator != null ? motivator.getMotivatorWheelVelocity() : 0;
    // System.out.printf(
    //     "[%s] Feeding — Launcher: %.0f/%.0f RPM | Hood: %.1f/%.1f° | Motivator: %.0f/%.0f RPM%n",
    //     label, targetRPM, actualRPM, hoodTarget, hoodActual, motTarget, motActual);
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
