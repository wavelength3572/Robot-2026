package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretVisualizer;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MatchPhaseTracker;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Factory class for coordinated shooting commands. Provides commands that coordinate the launcher,
 * turret, and match phase tracking to automatically shoot fuel into the active hub.
 *
 * <p>Shooting logic requires ALL conditions to be met:
 *
 * <ol>
 *   <li>Hub is active (via MatchPhaseTracker)
 *   <li>Turret is at target (within 2°)
 *   <li>Launcher is at setpoint (within 50 RPM)
 *   <li>Fuel is available (fuelCount > 0)
 *   <li>Minimum delay since last shot has elapsed
 * </ol>
 */
public class ShootingCommands {

  // Tunable shoot delay (seconds between shots)
  // Default conservative 0.25s, tune down as build team validates (target: 0.1s = 10 fuel/sec)
  private static final LoggedTunableNumber shootDelaySeconds =
      new LoggedTunableNumber("Shooting/DelaySeconds", 0.25);

  // Tunable launcher velocity for shooting (RPM)
  private static final LoggedTunableNumber launchVelocityRPM =
      new LoggedTunableNumber("Shooting/LaunchVelocityRPM", 2000.0);

  // Tolerance for turret angle (degrees)
  private static final double TURRET_ANGLE_TOLERANCE_DEG = 2.0;

  private ShootingCommands() {
    // Static factory class
  }

  /**
   * Phase 1: Basic coordinated shooting until empty. Robot stands still, launcher spins up, turret
   * aims at active hub, shoots until fuel is depleted.
   *
   * @param launcher The launcher subsystem
   * @param turret The turret subsystem
   * @param hubActiveSupplier Supplier that returns true when our hub is active
   * @return Command that shoots until empty or hub becomes inactive
   */
  public static Command shootUntilEmpty(
      Launcher launcher, Turret turret, BooleanSupplier hubActiveSupplier) {

    return Commands.sequence(
            // Log that we're starting
            Commands.runOnce(
                () -> {
                  System.out.println(
                      "[Shooting] Starting - spinning up launcher to "
                          + launchVelocityRPM.get()
                          + " RPM");
                  Logger.recordOutput("Shooting/State", "Spinning Up");
                  launcher.setVelocity(launchVelocityRPM.get());
                }),

            // Wait for launcher to reach setpoint and stabilize (debounce)
            Commands.race(
                Commands.sequence(
                    Commands.waitUntil(launcher::atSetpoint),
                    Commands.waitSeconds(0.15), // Must stay at setpoint for 150ms
                    Commands.waitUntil(launcher::atSetpoint)), // Verify still stable
                Commands.sequence(
                    Commands.waitSeconds(3.0),
                    Commands.runOnce(
                        () ->
                            System.out.println(
                                "[Shooting] Launcher spinup timeout - continuing anyway")))),
            Commands.runOnce(
                () -> {
                  System.out.println(
                      "[Shooting] Launcher ready and stable, entering shooting loop");
                  Logger.recordOutput("Shooting/State", "Launcher Ready");
                }),

            // Then repeatedly shoot while conditions are met
            createShootingLoop(launcher, turret, hubActiveSupplier))
        .finallyDo(
            () -> {
              launcher.stop();
              Logger.recordOutput("Shooting/State", "Stopped");
            })
        // Require both subsystems so the command properly interlocks
        .withName("Shoot Until Empty");
  }

  /**
   * Phase 2: Continuous shooting with intake integration. Continues shooting while picking up balls
   * during active periods; stops when hub becomes inactive.
   *
   * @param launcher The launcher subsystem
   * @param turret The turret subsystem
   * @param hubActiveSupplier Supplier that returns true when our hub is active
   * @return Command that continuously shoots while hub is active and fuel is available
   */
  public static Command continuousShooting(
      Launcher launcher, Turret turret, BooleanSupplier hubActiveSupplier) {

    return Commands.sequence(
            // Spin up launcher
            Commands.runOnce(() -> launcher.setVelocity(launchVelocityRPM.get())),

            // Continuous shooting loop - keeps running and waits for fuel
            Commands.run(
                    () -> {
                      // Keep launcher at speed
                      launcher.setVelocity(launchVelocityRPM.get());
                    })
                .until(() -> !hubActiveSupplier.getAsBoolean())
                .deadlineWith(createShootingLoop(launcher, turret, hubActiveSupplier)))
        .finallyDo(
            () -> {
              launcher.stop();
              Logger.recordOutput("Shooting/State", "Stopped");
            })
        .withName("Continuous Shooting");
  }

  /**
   * Phase 3: Shoot while moving. Driver controls the robot, turret auto-aims and shoots when
   * conditions are met.
   *
   * @param launcher The launcher subsystem
   * @param turret The turret subsystem
   * @param hubActiveSupplier Supplier that returns true when our hub is active
   * @return Command that shoots while moving (driver drives, turret auto-aims)
   */
  public static Command shootWhileMoving(
      Launcher launcher, Turret turret, BooleanSupplier hubActiveSupplier) {

    // Same as continuous shooting - turret already auto-aims in its periodic()
    // The difference is this command is designed to be bound to a trigger
    // and run while the driver is driving
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  launcher.setVelocity(launchVelocityRPM.get());
                  Logger.recordOutput("Shooting/State", "Spinning Up (Moving)");
                }),
            createShootingLoop(launcher, turret, hubActiveSupplier))
        .finallyDo(
            () -> {
              launcher.stop();
              Logger.recordOutput("Shooting/State", "Stopped");
            })
        .withName("Shoot While Moving");
  }

  /**
   * Create the core shooting loop that checks conditions and fires.
   *
   * @param launcher The launcher subsystem
   * @param turret The turret subsystem
   * @param hubActiveSupplier Supplier that returns true when our hub is active
   * @return Command that repeatedly checks conditions and shoots
   */
  private static Command createShootingLoop(
      Launcher launcher, Turret turret, BooleanSupplier hubActiveSupplier) {

    return Commands.sequence(
            // Wait for all conditions to be met
            Commands.waitUntil(
                () -> {
                  boolean hubActive = hubActiveSupplier.getAsBoolean();
                  boolean turretReady = turret.atTarget();
                  boolean launcherReady = launcher.atSetpoint();
                  TurretVisualizer visualizer = turret.getVisualizer();
                  boolean hasFuel = visualizer != null && visualizer.getFuelCount() > 0;

                  // Log current state
                  Logger.recordOutput("Shooting/Conditions/HubActive", hubActive);
                  Logger.recordOutput("Shooting/Conditions/TurretReady", turretReady);
                  Logger.recordOutput("Shooting/Conditions/LauncherReady", launcherReady);
                  Logger.recordOutput("Shooting/Conditions/HasFuel", hasFuel);

                  if (hubActive && turretReady && launcherReady && hasFuel) {
                    Logger.recordOutput("Shooting/State", "Ready to Fire");
                    return true;
                  }

                  Logger.recordOutput("Shooting/State", "Waiting for Conditions");
                  return false;
                }),

            // Fire!
            Commands.runOnce(
                () -> {
                  turret.launchFuel();
                  Logger.recordOutput("Shooting/State", "Fired!");
                  Logger.recordOutput("Shooting/LastShotTime", Timer.getFPGATimestamp());
                }),

            // Wait for tunable delay before next shot
            Commands.waitSeconds(shootDelaySeconds.get()))

        // Repeat while hub is active and we have fuel
        .repeatedly()
        .until(
            () -> {
              TurretVisualizer visualizer = turret.getVisualizer();
              boolean outOfFuel = visualizer == null || visualizer.getFuelCount() <= 0;
              boolean hubInactive = !hubActiveSupplier.getAsBoolean();

              if (outOfFuel) {
                Logger.recordOutput("Shooting/State", "Out of Fuel");
                return true;
              }
              if (hubInactive) {
                Logger.recordOutput("Shooting/State", "Hub Inactive");
                return true;
              }
              return false;
            });
  }

  /**
   * Get a supplier that checks if our hub is active using MatchPhaseTracker.
   *
   * @return BooleanSupplier that returns true when our hub is active
   */
  public static BooleanSupplier getHubActiveSupplier() {
    return () -> {
      boolean isBlue =
          DriverStation.getAlliance()
              .orElse(DriverStation.Alliance.Blue)
              .equals(DriverStation.Alliance.Blue);
      return MatchPhaseTracker.getInstance().isOurHubActive(isBlue);
    };
  }

  /**
   * Command to reset fuel count to full (25). Useful for testing.
   *
   * @param turret The turret subsystem (to access visualizer)
   * @return Command that resets fuel count
   */
  public static Command resetFuelCommand(Turret turret) {
    return Commands.runOnce(
            () -> {
              TurretVisualizer visualizer = turret.getVisualizer();
              if (visualizer != null) {
                visualizer.setFuelCount(25);
                Logger.recordOutput("Shooting/FuelReset", true);
              }
            })
        .ignoringDisable(true)
        .withName("Reset Fuel to 25");
  }

  /**
   * Command to set hub override to ON (always active).
   *
   * @return Command that sets hub override
   */
  public static Command setHubOverrideOn() {
    return Commands.runOnce(
            () -> MatchPhaseTracker.getInstance().setHubOverride(MatchPhaseTracker.HubOverride.ON))
        .ignoringDisable(true)
        .withName("Hub Override: ON");
  }

  /**
   * Command to set hub override to OFF (always inactive).
   *
   * @return Command that sets hub override
   */
  public static Command setHubOverrideOff() {
    return Commands.runOnce(
            () -> MatchPhaseTracker.getInstance().setHubOverride(MatchPhaseTracker.HubOverride.OFF))
        .ignoringDisable(true)
        .withName("Hub Override: OFF");
  }

  /**
   * Command to set hub override to AUTO (use match phase logic).
   *
   * @return Command that sets hub override
   */
  public static Command setHubOverrideAuto() {
    return Commands.runOnce(
            () ->
                MatchPhaseTracker.getInstance().setHubOverride(MatchPhaseTracker.HubOverride.AUTO))
        .ignoringDisable(true)
        .withName("Hub Override: AUTO");
  }

  /**
   * Command to toggle "we won auto" setting.
   *
   * @return Command that toggles the setting
   */
  public static Command toggleWeWonAuto() {
    return Commands.runOnce(
            () -> {
              MatchPhaseTracker tracker = MatchPhaseTracker.getInstance();
              tracker.setWeWonAuto(!tracker.getWeWonAuto());
            })
        .ignoringDisable(true)
        .withName("Toggle We Won Auto");
  }
}
