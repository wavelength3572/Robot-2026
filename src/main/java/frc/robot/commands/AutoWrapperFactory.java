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
    AUTO_TRACKING_STATIONARY,
    /**
     * Shoot on the move in alliance zones (hub shots under speed threshold); collect only in
     * neutral zone — no passing. Identical to AUTO_SHOOT except passing is disabled.
     */
    SHOOT_ON_THE_MOVE_NO_PASS
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

    // Determine arm trigger for sprint-start protection:
    //   - SHOOT_PRELOADS: IMMEDIATE — fire as soon as subsystems are ready
    //   - SPRINT + AUTO_SHOOT: ON_PASS_ZONE — arm when first entering neutral/opponent zone
    //   - SPRINT + SotM_NO_PASS: ON_ALLIANCE_RETURN — arm when entering any alliance zone
    //   - SPRINT + others: ON_ALLIANCE_RETURN — arm when entering any alliance zone
    ShootingCoordinator.ArmTrigger trigger = ShootingCoordinator.ArmTrigger.IMMEDIATE;
    if (startStrategy == StartStrategy.SPRINT) {
      if (pathStrategy == PathShootingStrategy.AUTO_SHOOT) {
        trigger = ShootingCoordinator.ArmTrigger.ON_PASS_ZONE;
      } else if (pathStrategy == PathShootingStrategy.SHOOT_ON_THE_MOVE_NO_PASS) {
        trigger = ShootingCoordinator.ArmTrigger.ON_ALLIANCE_RETURN;
      } else {
        trigger = ShootingCoordinator.ArmTrigger.ON_ALLIANCE_RETURN;
      }
    }

    // AUTO_SHOOT fires in pass zones; other strategies only fire in alliance zones.
    coordinator.setAutoPassingEnabled(pathStrategy == PathShootingStrategy.AUTO_SHOOT);

    // Build the path + intake deploy to run together
    List<Command> pathParallel = new ArrayList<>();
    pathParallel.add(runPath(selectedAuto));
    pathParallel.add(deployIntake(intake));

    // Stow hood after preload shooting (only needed for END_OF_PATH since SmartLaunch2
    // owns the hood subsystem in the other strategies).
    if (startStrategy == StartStrategy.SHOOT_PRELOADS
        && pathStrategy == PathShootingStrategy.END_OF_PATH) {
      pathParallel.add(stowHood(hood));
    }

    Command pathWithIntake = Commands.parallel(pathParallel.toArray(Command[]::new));

    // For AUTO_SHOOT, AUTO_TRACKING_STATIONARY, and SHOOT_ON_THE_MOVE_NO_PASS: SmartLaunch2
    // runs for the ENTIRE auto (path + post-path). It doesn't die when the path ends — the
    // path is just one step in the sequence that runs underneath it. After the path, the
    // robot is stationary and SmartLaunch2 keeps firing until auto ends.
    //
    // For END_OF_PATH: SmartLaunch2 only runs after the path completes.
    if (pathStrategy != PathShootingStrategy.END_OF_PATH) {
      // Path runs as a sequence step, SmartLaunch2 wraps the whole thing in parallel.
      // When the path ends, the sequence moves to a "wait forever" that keeps SmartLaunch2
      // alive until auto ends (the overall auto timeout or disable kills everything).
      Command pathThenWait =
          Commands.sequence(
              pathWithIntake,
              // Agitate intake after path to shake loose stuck balls while still shooting
              intake != null
                  ? intake.agitateCommand(() -> 2000.0, () -> false).asProxy()
                  : Commands.idle());
      steps.add(
          Commands.parallel(
              pathThenWait,
              ShootingCommands.smartLaunchDangerousCommand(
                      launcher, coordinator, motivator, turret, hood, spindexer, trigger)
                  .asProxy()));
    } else {
      // END_OF_PATH: run path first, then shoot
      steps.add(pathWithIntake);
      steps.add(
          postPathSmartLaunchWithAgitation(
              launcher, coordinator, motivator, turret, hood, spindexer, intake));
    }

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
        Commands.waitSeconds(0.4),
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
    return ShootingCommands.smartLaunchDangerousCommand(
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
        ShootingCommands.smartLaunchDangerousCommand(
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
