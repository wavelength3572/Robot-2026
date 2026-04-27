package frc.robot.commands;

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
      new LoggedTunableNumber(
          "Shots/HubShot/LauncherRPM", Constants.getRobotConfig().getHubShotLauncherRPM());
  private static final LoggedTunableNumber hubShotHoodAngleDeg =
      new LoggedTunableNumber(
          "Shots/HubShot/HoodAngleDeg", Constants.getRobotConfig().getHubShotHoodAngleDeg());
  private static final LoggedTunableNumber hubShotTurretAngleDeg =
      new LoggedTunableNumber(
          "Shots/HubShot/TurretAngleDeg", Constants.getRobotConfig().getHubShotTurretAngleDeg());
  private static final LoggedTunableNumber hubShotMotivatorRPM =
      new LoggedTunableNumber(
          "Shots/HubShot/MotivatorRPM", Constants.getRobotConfig().getHubShotMotivatorRPM());
  private static final LoggedTunableNumber hubShotSpindexerRPM =
      new LoggedTunableNumber(
          "Shots/HubShot/SpindexerRPM", Constants.getRobotConfig().getHubShotSpindexerRPM());

  // Left trench shot
  private static final LoggedTunableNumber leftTrenchLauncherRPM =
      new LoggedTunableNumber(
          "Shots/LeftTrench/LauncherRPM", Constants.getRobotConfig().getLeftTrenchLauncherRPM());
  private static final LoggedTunableNumber leftTrenchHoodAngleDeg =
      new LoggedTunableNumber(
          "Shots/LeftTrench/HoodAngleDeg", Constants.getRobotConfig().getLeftTrenchHoodAngleDeg());
  private static final LoggedTunableNumber leftTrenchTurretAngleDeg =
      new LoggedTunableNumber(
          "Shots/LeftTrench/TurretAngleDeg",
          Constants.getRobotConfig().getLeftTrenchTurretAngleDeg());
  private static final LoggedTunableNumber leftTrenchMotivatorRPM =
      new LoggedTunableNumber(
          "Shots/LeftTrench/MotivatorRPM", Constants.getRobotConfig().getLeftTrenchMotivatorRPM());
  private static final LoggedTunableNumber leftTrenchSpindexerRPM =
      new LoggedTunableNumber(
          "Shots/LeftTrench/SpindexerRPM", Constants.getRobotConfig().getLeftTrenchSpindexerRPM());

  // Right trench shot
  private static final LoggedTunableNumber rightTrenchLauncherRPM =
      new LoggedTunableNumber(
          "Shots/RightTrench/LauncherRPM", Constants.getRobotConfig().getRightTrenchLauncherRPM());
  private static final LoggedTunableNumber rightTrenchHoodAngleDeg =
      new LoggedTunableNumber(
          "Shots/RightTrench/HoodAngleDeg",
          Constants.getRobotConfig().getRightTrenchHoodAngleDeg());
  private static final LoggedTunableNumber rightTrenchTurretAngleDeg =
      new LoggedTunableNumber(
          "Shots/RightTrench/TurretAngleDeg",
          Constants.getRobotConfig().getRightTrenchTurretAngleDeg());
  private static final LoggedTunableNumber rightTrenchMotivatorRPM =
      new LoggedTunableNumber(
          "Shots/RightTrench/MotivatorRPM",
          Constants.getRobotConfig().getRightTrenchMotivatorRPM());
  private static final LoggedTunableNumber rightTrenchSpindexerRPM =
      new LoggedTunableNumber(
          "Shots/RightTrench/SpindexerRPM",
          Constants.getRobotConfig().getRightTrenchSpindexerRPM());

  // ===== Motivator / Spindexer RPM Architecture =====
  // Each ShotStrategy (FixedHeightShotStrategy, FixedHeightPassStrategy, TwoStageShotStrategy)
  // owns its own motivator and spindexer RPM tunables and returns them in ShotResult.
  // The strategy is the single source of truth — to change values, edit the strategy.
  //
  // The getEffective*() methods below apply per-actuator dashboard overrides on top of
  // whatever the strategy returned. That's the only layer between strategy and hardware.
  //
  // To add a new pass strategy: implement ShotStrategy, return your motivator/spindexer
  // RPM in the ShotResult, and the override system works automatically.

  // ===== Per-Actuator Overrides =====
  // Each actuator can be individually overridden via a dashboard toggle + tunable value.
  // When an override is off, the calculated value is used. When on, the tunable is used.
  // Any combination of 0-4 overrides can be active at once.
  private static final LoggedTunableNumber overrideLauncherRPM =
      new LoggedTunableNumber(
          "Overrides/LauncherRPM", Constants.getRobotConfig().getOverrideLauncherRPM());
  private static final LoggedTunableNumber overrideHoodDeg =
      new LoggedTunableNumber("Overrides/HoodDeg", Constants.getRobotConfig().getOverrideHoodDeg());
  private static final LoggedTunableNumber overrideMotivatorRPM =
      new LoggedTunableNumber(
          "Overrides/MotivatorRPM", Constants.getRobotConfig().getOverrideMotivatorRPM());
  private static final LoggedTunableNumber overrideSpindexerRPM =
      new LoggedTunableNumber(
          "Overrides/SpindexerRPM", Constants.getRobotConfig().getOverrideSpindexerRPM());

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

  private ShootingCommands() {
    // Static factory class
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

    // Per-actuator override tunables and toggles (all default off)
    overrideLauncherRPM.get();
    overrideHoodDeg.get();
    overrideMotivatorRPM.get();
    overrideSpindexerRPM.get();
    SmartDashboard.putBoolean("Overrides/Launcher", false);
    SmartDashboard.putBoolean("Overrides/Hood", false);
    SmartDashboard.putBoolean("Overrides/Motivator", false);
    SmartDashboard.putBoolean("Overrides/Spindexer", false);

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
                  turret.setActivelyCommanded(true);
                  if (hood != null) {
                    hood.setActivelyCommanded(true);
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
                              && (hood == null || hood.atTarget())) {
                            motivator.setMotivatorVelocity(motivatorRPMSupplier.getAsDouble());
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
                    () -> {
                      turret.setActivelyCommanded(true);
                      turret.setOutsideTurretAngle(turretAngleDegSupplier.getAsDouble());
                    },
                    turret),

                // Keep hood positioned (reads tunable each cycle)
                hood != null
                    ? Commands.run(
                        () -> {
                          hood.setActivelyCommanded(true);
                          hood.setHoodAngle(hoodAngleDegSupplier.getAsDouble());
                        },
                        hood)
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
              turret.setActivelyCommanded(false);
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
                hood.setActivelyCommanded(false);
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
   * ON_PASS_ZONE or ON_ALLIANCE_RETURN to prevent shooting preloads at the start.
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
            // Launcher — track shot RPM; coasts when idle (0 RPM → stopMotor via kCoast)
            Commands.run(
                () -> {
                  ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                  if (shot != null) {
                    double rpm = getEffectiveRPM(shot);
                    double effectiveRPM = coordinator.getEffectiveLauncherRPM(rpm);
                    launcher.setVelocity(effectiveRPM);
                  } else {
                    launcher.setVelocity(0);
                  }
                },
                launcher),

            // Turret — track shot angle, but hold position while auto collecting.
            // Don't mark actively commanded while UNARMED so turret stays IDLE in logs
            // until the coordinator is actually driving it.
            Commands.run(
                () -> {
                  boolean unarmed =
                      coordinator.getCoordinatorState()
                          == ShootingCoordinator.CoordinatorState.UNARMED;
                  if (unarmed) return;
                  if (coordinator.isAutoCollecting()) {
                    turret.setActivelyCommanded(true);
                    return; // hold position
                  }
                  if (coordinator.getCurrentShot() != null) {
                    turret.setActivelyCommanded(true);
                    turret.setOutsideTurretAngle(coordinator.getCurrentTurretAngleDeg());
                  }
                },
                turret),

            // Hood — track shot angle; idle at min while auto collecting.
            // Same UNARMED gate as turret.
            hood != null
                ? Commands.run(
                    () -> {
                      // Always lower hood in no-fire zones or unarmed, even if not
                      // actively tracking a shot — hood safety trumps state machine.
                      ShootingCoordinator.CoordinatorState coordState =
                          coordinator.getCoordinatorState();
                      boolean forceMin =
                          coordState == ShootingCoordinator.CoordinatorState.NO_FIRE_ZONE
                              || coordState == ShootingCoordinator.CoordinatorState.UNARMED;
                      if (forceMin) {
                        hood.setActivelyCommanded(true);
                        hood.setHoodAngle(hood.getMinAngle());
                        return;
                      }
                      // Hold Fire (B11) → keep hood down; hood pops up on release.
                      // Safety net for bad odometry scenarios where zone-based clamp fails.
                      if (spindexer != null && spindexer.isFeedingSuppressed()) {
                        hood.setActivelyCommanded(true);
                        hood.setHoodAngle(hood.getMinAngle());
                        return;
                      }
                      if (coordinator.isAutoCollecting()) {
                        hood.setActivelyCommanded(true);
                        hood.setHoodAngle(hood.getMinAngle());
                        return;
                      }
                      ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                      if (shot != null) {
                        hood.setActivelyCommanded(true);
                        hood.setHoodAngle(getEffectiveHoodDeg(shot));
                      }
                    },
                    hood)
                : Commands.none(),

            // Motivator — always pre-spin in teleop (ready for passing or shooting).
            // In auto, idle only while collecting (open neutral/opponent zones).
            motivator != null
                ? Commands.run(
                    () -> {
                      if (coordinator.isAutoCollecting()) {
                        motivator.stopMotivator();
                        return;
                      }
                      // Reverse pulse on spin-up is disabled — no motor runs in reverse here.
                      // private final Timer reversePulseTimer = new Timer();
                      // private boolean reversing = false;
                      // private boolean wasIdle = true;
                      // private static final double REVERSE_PULSE_SEC = 0.2;
                      // if (wasIdle) { wasIdle = false; reversing = true;
                      // reversePulseTimer.restart(); }
                      // if (reversing) {
                      //   if (reversePulseTimer.hasElapsed(REVERSE_PULSE_SEC)) reversing = false;
                      //   else { motivator.setMotivatorVoltage(-1.0); return; }
                      // }
                      ShotCalculator.ShotResult s = coordinator.getCurrentShot();
                      if (s != null) {
                        motivator.setMotivatorVelocity(getEffectiveMotivatorRPM(s));
                      } else {
                        motivator.stopMotivator();
                      }
                    },
                    motivator)
                : Commands.none(),

            // Spindexer — feed only when coordinator allows AND motivator is at speed.
            // Once the coordinator reaches FIRING (which requires motivator READY),
            // we trust the motivator and don't re-check its state. Brief RPM dips
            // from ball loading are normal and should not interrupt feeding.
            // The gate resets automatically when the coordinator leaves FIRING.
            spindexer != null
                ? Commands.run(
                    () -> {
                      boolean feedingAllowed = coordinator.isFeedingAllowed();
                      if (feedingAllowed) {
                        launcher.setFeedingActive(true);
                        double spnRPM = getEffectiveSpindexerRPM(coordinator.getCurrentShot());
                        if (turret.getState() == Turret.TurretState.FLIPPING) {
                          spindexer.stopSpindexer();
                        } else {
                          spindexer.setSpindexerVelocity(spnRPM);
                          fireSimBallIfReady(coordinator, launcher);
                        }
                      } else if (coordinator.isAutoCollecting()) {
                        // Motivator is stopped during auto collecting — no feeding needed
                        launcher.setFeedingActive(false);
                        spindexer.stopSpindexer();
                      } else {
                        // Only back-spin while motivator is actually spinning up — keeps
                        // balls away from the launcher interface until it's at speed.
                        // Otherwise stop the spindexer; the old -150 ran unconditionally
                        // whenever not FIRING (including AIMING blocked by velcomp,
                        // SETTLING, HELD, etc.) which added ~550 RPM of reversal latency
                        // every time the coordinator reached FIRING.
                        // launcher.setFeedingActive(false);
                        // if (motivator != null
                        //     && motivator.getState()
                        //         == Motivator.MotivatorState.SPINNING_UP) {
                        //   spindexer.setSpindexerVelocity(-150);
                        // } else {
                        spindexer
                            .stopSpindexer(); // if we arent feeding and if we arent doing something
                        // special for auto, then the spindexer shoudl be off
                        // }
                      }
                    },
                    spindexer)
                : Commands.none())
        .beforeStarting(
            () -> {
              setMode(ShootingMode.COMPETITION);
              coordinator.setSmartLaunchActive(true, armTrigger);

              Logger.recordOutput("SmartLaunch/Phase", "SM_ACTIVE");
              SmartDashboard.putString("Match/Status/State", "SmartLaunch 2.0 - Active");
            })
        .finallyDo(
            () -> {
              coordinator.setSmartLaunchActive(false);
              turret.setActivelyCommanded(false);
              launcher.setFeedingActive(false);
              launcher.stop();
              if (motivator != null) {
                motivator.stopMotivator();
              }
              if (spindexer != null) {
                spindexer.stopSpindexer();
              }
              if (hood != null) {
                hood.setActivelyCommanded(false);
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

  // ===== Per-Actuator Override Helpers =====
  // Each actuator has its own Overrides/<Name> toggle. When the toggle is true,
  // the corresponding tunable value is used instead of the calculated one.
  // Override any combination of 1-4 actuators independently.

  // Safety cap: matches LauncherIOSparkFlex.MAX_VELOCITY_RPM hardware limit
  private static final double MAX_LAUNCHER_RPM = 5000.0;

  /** Get effective launcher RPM — override value when toggled, otherwise calculated + trim. */
  public static double getEffectiveRPM(ShotCalculator.ShotResult shot) {
    double rpm;
    if (SmartDashboard.getBoolean("Overrides/Launcher", false)) {
      rpm = overrideLauncherRPM.get() + launcherTrimRPM;
    } else {
      rpm = shot != null ? shot.launcherRPM() + launcherTrimRPM : 0.0;
    }
    return Math.min(rpm, MAX_LAUNCHER_RPM);
  }

  /** Get effective hood angle — override value when toggled, otherwise calculated. */
  public static double getEffectiveHoodDeg(ShotCalculator.ShotResult shot) {
    if (SmartDashboard.getBoolean("Overrides/Hood", false)) {
      return overrideHoodDeg.get();
    }
    return shot != null ? shot.hoodAngleDeg() : 0.0;
  }

  /** Get effective motivator RPM — override value when toggled, otherwise from strategy. */
  public static double getEffectiveMotivatorRPM(ShotCalculator.ShotResult shot) {
    if (SmartDashboard.getBoolean("Overrides/Motivator", false)) {
      return overrideMotivatorRPM.get();
    }
    return shot != null ? shot.motivatorRPM() : 0.0;
  }

  /** Get effective spindexer RPM — override value when toggled, otherwise from strategy. */
  public static double getEffectiveSpindexerRPM(ShotCalculator.ShotResult shot) {
    if (SmartDashboard.getBoolean("Overrides/Spindexer", false)) {
      return overrideSpindexerRPM.get();
    }
    return shot != null ? shot.spindexerRPM() : 0.0;
  }
}
