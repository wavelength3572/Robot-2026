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
import frc.robot.subsystems.shooting.ShotOverrides;
import frc.robot.subsystems.shooting.ShotVisualizer;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretState;
import frc.robot.util.FuelSim;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Factory class for shooting commands. Supports two modes:
 *
 * <ul>
 *   <li>COMPETITION: Auto-calculates optimal trajectory to hub (SmartLaunch)
 *   <li>TEST: Uses fixed preset parameters (hub shot, trench shots)
 * </ul>
 */
public class ShootingCommands {

  /** Shooting mode determines trajectory calculation and parameter source. */
  public enum ShootingMode {
    /** Auto-calculated trajectory to hub, optimized RPM/angle for distance. */
    COMPETITION,
    /** Fixed preset parameters (hub shot, trench shots). */
    TEST
  }

  private static ShootingMode currentMode = ShootingMode.COMPETITION;

  public static ShootingMode getMode() {
    return currentMode;
  }

  public static void setMode(ShootingMode mode) {
    if (currentMode != mode) {
      currentMode = mode;
      SmartDashboard.putString("Match/Status/Mode", mode.toString());
      SmartDashboard.putBoolean("Match/Status/Active", mode == ShootingMode.TEST);
    }
  }

  public static boolean isTestMode() {
    return currentMode == ShootingMode.TEST;
  }

  // ===== Fixed Shot Presets =====
  // Tunable from dashboard so values can be adjusted without redeploying.

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

  // ===== Command Behavior Constants =====

  // Minimum time between simulated ball fires (prevents multiple fires per frame)
  private static final double MIN_SHOT_INTERVAL_SECONDS = 0.14;
  private static double lastSimFireTimestamp = 0;

  private ShootingCommands() {}

  /** Initialize all tunables so they appear in the dashboard immediately on boot. */
  public static void initTunables() {
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

    ShotOverrides.initDashboard();

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

  // ===== Launcher Trim =====
  // Thin wrappers — trim state lives in ShotOverrides.

  /**
   * Set the launcher RPM trim offset. Added to all launcher RPM targets (both smart launch and
   * fixed shots). Called by button box bindings.
   */
  public static void setLauncherTrimRPM(double trimRPM) {
    ShotOverrides.setLauncherTrimRPM(trimRPM);
  }

  public static double getLauncherTrimRPM() {
    return ShotOverrides.getLauncherTrimRPM();
  }

  // ===== Per-Actuator Override Helpers =====
  // Thin wrappers over ShotOverrides — kept for backward compatibility.
  // Each returns the override value when its dashboard toggle is on, otherwise the strategy value.
  // Launcher RPM also includes the trim offset.

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

  // ===== Fixed-Position Shot Commands =====

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
   * ready (with 2s timeout), then feeds via spindexer. All parameters are suppliers so dashboard
   * tunables take effect immediately without redeploying.
   *
   * <p>Phases:
   *
   * <ol>
   *   <li>Set subsystem targets simultaneously
   *   <li>Wait for all subsystems to reach setpoint (2s timeout)
   *   <li>Feed via spindexer while keeping all subsystems running
   * </ol>
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
    DoubleSupplier trimmedLauncherRPM =
        () -> launcherRPMSupplier.getAsDouble() + ShotOverrides.getLauncherTrimRPM();

    return Commands.sequence(
            Commands.runOnce(() -> setMode(ShootingMode.TEST)),

            // Phase 1: Set all subsystem targets simultaneously
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
                  }
                  if (coordinator != null) {
                    coordinator.setManualShotParameters(launcherRPM, hoodAngle, turretAngle);
                  }
                  SmartDashboard.putString("Match/Status/State", "Fixed Launch - Positioning");
                }),

            // Phase 2: Wait for all subsystems to reach setpoint (2s timeout)
            Commands.race(
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
                      boolean hoodReady = hood == null || hood.getState() == Hood.HoodState.READY;
                      return launcherReady && motivatorReady && turretReady && hoodReady;
                    }),
                Commands.sequence(
                    Commands.waitSeconds(2.0),
                    Commands.runOnce(
                        () ->
                            SmartDashboard.putString(
                                "Match/Status/State",
                                "Fixed Launch TIMEOUT - continuing anyway")))),

            Commands.runOnce(
                () -> {
                  SmartDashboard.putString("Match/Status/State", "Ready - Feeding");
                  launcher.setFeedingActive(true);
                }),

            // Phase 3: Feed via spindexer while keeping all subsystems running
            Commands.parallel(
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
                Commands.run(
                    () -> {
                      turret.setActivelyCommanded(true);
                      turret.setOutsideTurretAngle(turretAngleDegSupplier.getAsDouble());
                    },
                    turret),
                hood != null
                    ? Commands.run(
                        () -> {
                          hood.setActivelyCommanded(true);
                          hood.setHoodAngle(hoodAngleDegSupplier.getAsDouble());
                        },
                        hood)
                    : Commands.none(),
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
            })
        .withName("FixedShot");
  }

  // ===== SmartLaunch Commands =====

  /**
   * State-machine-driven SmartLaunch (teleop default). Arms immediately; see the overload with
   * {@link ShootingCoordinator.ArmTrigger} for auto variants that delay arming.
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
   * State-machine-driven SmartLaunch with configurable arm trigger.
   *
   * <p>All subsystems run in parallel and track the coordinator's current shot. The motivator
   * pre-spins with a brief reverse pulse on each spin-up to clear any ball stuck at the
   * motivator/launcher interface. The spindexer only feeds when the coordinator reaches FIRING
   * state (all subsystems at target). The hood auto-stows when leaving firing zones.
   *
   * <p>For sprint autos use {@link ShootingCoordinator.ArmTrigger#ON_PASS_ZONE} or {@link
   * ShootingCoordinator.ArmTrigger#ON_ALLIANCE_RETURN} to prevent shooting preloads at start.
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
            // Launcher — track shot RPM; stop when no shot is available
            Commands.run(
                () -> {
                  ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
                  double rpm =
                      shot != null ? coordinator.getEffectiveLauncherRPM(getEffectiveRPM(shot)) : 0;
                  launcher.setVelocity(rpm);
                },
                launcher),

            // Turret — track shot angle; hold position during auto collecting.
            // Not marked actively commanded while UNARMED so turret stays IDLE in logs.
            Commands.run(
                () -> {
                  boolean unarmed =
                      coordinator.getCoordinatorState()
                          == ShootingCoordinator.CoordinatorState.UNARMED;
                  if (unarmed) return;
                  if (coordinator.isAutoCollecting()) {
                    turret.setActivelyCommanded(true);
                    return;
                  }
                  if (coordinator.getCurrentShot() != null) {
                    turret.setActivelyCommanded(true);
                    turret.setOutsideTurretAngle(coordinator.getCurrentTurretAngleDeg());
                  }
                },
                turret),

            // Hood — follow shot angle; force to min in no-fire zones or while collecting.
            // Hood safety (stow on zone transitions) takes priority over state machine.
            hood != null
                ? Commands.run(
                    () -> {
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
                      // Hold Fire (B11) → keep hood down; pops up when released
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

            // Motivator — pre-spins in parallel with launcher and aiming (no reverse-pulse
            // delay). A brief reverse pulse clears any ball stuck at the motivator/launcher
            // interface each time the motivator transitions from idle to spinning.
            motivator != null
                ? Commands.run(new MotivatorPreSpinner(coordinator, motivator), motivator)
                : Commands.none(),

            // Spindexer — feeds only when coordinator reaches FIRING (all subsystems ready).
            // Stops during auto collecting and between shots.
            spindexer != null
                ? Commands.run(
                    () -> {
                      if (coordinator.isFeedingAllowed()) {
                        launcher.setFeedingActive(true);
                        double spnRPM =
                            getEffectiveSpindexerRPM(coordinator.getCurrentShot());
                        if (turret.getState() == Turret.TurretState.FLIPPING) {
                          spindexer.stopSpindexer();
                        } else {
                          spindexer.setSpindexerVelocity(spnRPM);
                          fireSimBallIfReady(coordinator, launcher);
                        }
                      } else if (coordinator.isAutoCollecting()) {
                        launcher.setFeedingActive(false);
                        spindexer.stopSpindexer();
                      } else {
                        spindexer.stopSpindexer();
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

  // ===== Helpers =====

  /**
   * Fire a simulated ball if in SIM/REPLAY mode, throttled to MIN_SHOT_INTERVAL_SECONDS. Called
   * from the real feeding code path so sim behavior matches real robot gating exactly.
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

  /**
   * Runs in the motivator's {@link Commands#run} slot inside SmartLaunch. Pre-spins the motivator
   * at shot RPM and executes a brief reverse pulse each time it transitions from idle back to
   * spinning, to clear any ball that settled at the motivator/launcher interface during free time.
   */
  private static class MotivatorPreSpinner implements Runnable {
    private static final double REVERSE_PULSE_SEC = 0.2;

    private final ShootingCoordinator coordinator;
    private final Motivator motivator;
    private final Timer reversePulseTimer = new Timer();
    private boolean reversing = false;
    private boolean wasIdle = true;

    MotivatorPreSpinner(ShootingCoordinator coordinator, Motivator motivator) {
      this.coordinator = coordinator;
      this.motivator = motivator;
    }

    @Override
    public void run() {
      if (coordinator.isAutoCollecting()) {
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

      ShotCalculator.ShotResult shot = coordinator.getCurrentShot();
      if (shot != null) {
        motivator.setMotivatorVelocity(getEffectiveMotivatorRPM(shot));
      } else {
        motivator.stopMotivator();
      }
    }
  }
}
