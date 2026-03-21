// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.motivator.Motivator;
import frc.robot.subsystems.shooting.ShootingCoordinator;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.FuelSim;
import java.util.ArrayList;
import java.util.List;

/**
 * Factory for composing autonomous wrapper sequences. A single unified method assembles the wrapper
 * from two strategy choices (start and path-shooting), allowing any combination without duplicating
 * setup/teardown logic.
 */
public class AutoWrapperFactory {

  private static final int AUTO_START_FUEL_COUNT = 8;

  private AutoWrapperFactory() {} // Static utility class

  /** Whether to shoot preloaded balls before running the path or sprint immediately. */
  public enum StartStrategy {
    SHOOT_PRELOADS,
    SPRINT
  }

  /** Whether to auto-shoot during the path or only fire after the path completes. */
  public enum PathShootingStrategy {
    END_OF_PATH,
    AUTO_SHOOT,
    /**
     * Always track hub (turret/launcher/motivator spin up); only fire when stationary in near
     * trench or alliance zone. Hood tracks in near trench and alliance only (not far trench).
     */
    AUTO_TRACKING_STATIONARY
  }

  // ---- Public wrapper assembler ----

  /**
   * Unified comp auto wrapper. Composes the sequence dynamically based on the two strategy choices.
   */
  public static Command compWrapped(
      Command selectedAuto,
      Pose2d startingPose,
      StartStrategy startStrategy,
      PathShootingStrategy pathStrategy,
      frc.robot.subsystems.drive.Drive drive,
      Intake intake,
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {

    List<Command> steps = new ArrayList<>();

    // Always: reset odometry, sim setup
    steps.add(resetOdometry(drive, startingPose));
    steps.add(simSetup(coordinator));

    // Start strategy
    if (startStrategy == StartStrategy.SHOOT_PRELOADS) {
      steps.add(initialSmartLaunch(launcher, coordinator, motivator, turret, hood, spindexer));
    }

    // Build the path command with shooting and intake deploy in parallel.
    // Intake deploy and hood stow (if SHOOT_PRELOADS) run alongside the path instead of
    // sequentially before it, saving ~0.25-1.0s of auto time.
    List<Command> pathParallel = new ArrayList<>();

    // SPRINT autos suppress feeding until the robot first enters a pass zone.
    // The armOnPassZone gate already prevents accidental shots at the hub.
    if (pathStrategy == PathShootingStrategy.AUTO_SHOOT) {
      boolean armOnPassZone = (startStrategy == StartStrategy.SPRINT);
      pathParallel.add(runPath(selectedAuto));
      pathParallel.add(
          ShootingCommands.continuousSmartLaunchCommand(
                  launcher, coordinator, motivator, turret, hood, spindexer, armOnPassZone)
              .asProxy());
    } else if (pathStrategy == PathShootingStrategy.AUTO_TRACKING_STATIONARY) {
      pathParallel.add(runPath(selectedAuto));
      pathParallel.add(
          ShootingCommands.autoTrackingStationaryCommand(
                  launcher, coordinator, motivator, turret, hood, spindexer)
              .asProxy());
    } else {
      // END_OF_PATH: run the path only
      pathParallel.add(runPath(selectedAuto));
    }

    // Deploy intake in parallel with path start instead of sequentially before
    pathParallel.add(deployIntake(intake));

    // Stow hood in parallel with path start (after preload shooting) instead of blocking.
    // Skip for AUTO_SHOOT and AUTO_TRACKING_STATIONARY — those commands require the hood
    // subsystem directly, so the default stow command won't run while they're active.
    //
    // Hood safety under the trench:
    //   The ShootingCoordinator's trenchModeActive flag (set when zone is TRENCH_NEAR,
    //   TRENCH_FAR, or BUMP) clamps hoodMax to Shots/TrenchMode/HoodMaxDeg (default 18°)
    //   before the shot calculator runs. So even though AUTO_SHOOT blindly tracks the shot
    //   angle and AUTO_TRACKING_STATIONARY only explicitly stows in TRENCH_FAR, the
    //   coordinator's clamp ensures neither command can raise the hood above 18° in any
    //   trench zone. The safety comes from the coordinator, not the commands themselves.
    //
    //   Remaining risk: the clamp applies when the zone *is* trench, not *before* entering
    //   it. A fast transition from open field (hood at 40°+) into a trench zone could
    //   briefly have the hood too high until the next periodic cycle stows it.
    if (startStrategy == StartStrategy.SHOOT_PRELOADS
        && pathStrategy == PathShootingStrategy.END_OF_PATH) {
      pathParallel.add(stowHood(hood));
    }

    steps.add(Commands.parallel(pathParallel.toArray(Command[]::new)));

    // Always: fire remaining balls after path (with agitation to shake loose stuck balls)
    steps.add(
        postPathSmartLaunchWithAgitation(
            launcher, coordinator, motivator, turret, hood, spindexer, intake));

    return Commands.sequence(steps.toArray(Command[]::new))
        .finallyDo(() -> teardown(launcher, motivator, intake));
  }

  // ---- Building blocks (private) ----

  private static Command resetOdometry(
      frc.robot.subsystems.drive.Drive drive, Pose2d startingPose) {
    return Commands.runOnce(
        () -> {
          if (startingPose != null) {
            drive.setPose(startingPose);
          }
        });
  }

  private static Command simSetup(ShootingCoordinator coordinator) {
    return Commands.runOnce(
        () -> {
          if (coordinator != null && coordinator.getVisualizer() != null) {
            coordinator.getVisualizer().setFuelCount(AUTO_START_FUEL_COUNT);
          }
          if (Constants.currentMode == Constants.Mode.SIM) {
            FuelSim.getInstance().clearFuel();
            FuelSim.getInstance().spawnStartingFuel();
          }
        });
  }

  private static Command deployIntake(Intake intake) {
    if (intake == null) {
      return Commands.none();
    }
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              intake.deploy();
              intake.runIntake();
            }),
        Commands.waitUntil(() -> intake.isDeployed()).withTimeout(0.25));
  }

  private static Command initialSmartLaunch(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {
    return ShootingCommands.smartLaunchCommand(
            launcher, coordinator, motivator, turret, hood, spindexer)
        .withTimeout(2.5)
        .asProxy();
  }

  private static Command stowHood(Hood hood) {
    if (hood == null) {
      return Commands.none();
    }
    return Commands.run(() -> hood.setHoodAngle(hood.getMinAngle()), hood)
        .until(() -> hood.atTarget())
        .withTimeout(0.75)
        .asProxy();
  }

  private static Command runPath(Command selectedAuto) {
    return selectedAuto.asProxy();
  }

  private static Command postPathSmartLaunchWithAgitation(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer,
      Intake intake) {
    Command smartLaunch =
        ShootingCommands.smartLaunchCommand(
                launcher, coordinator, motivator, turret, hood, spindexer)
            .withTimeout(10.0)
            .asProxy();
    if (intake != null) {
      smartLaunch = smartLaunch.alongWith(intake.agitateCommand(() -> 2000.0, () -> false));
    }
    return smartLaunch;
  }

  private static void teardown(Launcher launcher, Motivator motivator, Intake intake) {
    if (launcher != null) {
      launcher.stop();
    }
    if (motivator != null) {
      motivator.stopMotivator();
    }
    if (intake != null) {
      intake.stopRollers();
    }
  }
}
