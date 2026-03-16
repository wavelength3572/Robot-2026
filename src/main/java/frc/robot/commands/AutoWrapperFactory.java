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
    AUTO_SHOOT
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

    // Always: reset odometry, sim setup, deploy intake
    steps.add(resetOdometry(drive, startingPose));
    steps.add(simSetup(coordinator));
    steps.add(deployIntake(intake));

    // Start strategy
    if (startStrategy == StartStrategy.SHOOT_PRELOADS) {
      steps.add(initialSmartLaunch(launcher, coordinator, motivator, turret, hood, spindexer));
      steps.add(stowHood(hood));
    }

    // Path shooting: enable auto-shoot before path if requested
    if (pathStrategy == PathShootingStrategy.AUTO_SHOOT) {
      steps.add(enableAutoShoot(launcher, motivator, coordinator));
    }

    // Always: run the path
    steps.add(runPath(selectedAuto));

    // Path shooting: disable auto-shoot after path if it was enabled
    if (pathStrategy == PathShootingStrategy.AUTO_SHOOT) {
      steps.add(disableAutoShoot(launcher, motivator, coordinator));
    }

    // Always: fire remaining balls after path
    steps.add(postPathSmartLaunch(launcher, coordinator, motivator, turret, hood, spindexer));

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
        .withTimeout(5.0)
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

  private static Command postPathSmartLaunch(
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {
    return ShootingCommands.smartLaunchCommand(
            launcher, coordinator, motivator, turret, hood, spindexer)
        .withTimeout(10.0)
        .asProxy();
  }

  private static Command enableAutoShoot(
      Launcher launcher, Motivator motivator, ShootingCoordinator coordinator) {
    return Commands.runOnce(
        () -> {
          if (launcher != null) launcher.setVelocity(1700.0);
          if (motivator != null) motivator.setMotivatorVelocity(1000.0);
          if (coordinator != null) coordinator.enableAutoShoot();
        });
  }

  private static Command disableAutoShoot(
      Launcher launcher, Motivator motivator, ShootingCoordinator coordinator) {
    return Commands.runOnce(
        () -> {
          if (coordinator != null) coordinator.disableAutoShoot();
          if (launcher != null) launcher.stop();
          if (motivator != null) motivator.stopMotivator();
        });
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
