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

/**
 * Factory for composing autonomous wrapper sequences. Each public method assembles a complete auto
 * wrapper from reusable building blocks, allowing different wrapper strategies (shot-first, sprint,
 * etc.) without duplicating setup/teardown logic.
 */
public class AutoWrapperFactory {

  private static final int AUTO_START_FUEL_COUNT = 8;

  private AutoWrapperFactory() {} // Static utility class

  // ---- Public wrapper assemblers ----

  /**
   * Standard comp auto wrapper: shoots preloaded balls first, then runs the path, then fires
   * remaining balls. This is the existing behavior for all "Comp" folder autos.
   */
  public static Command compShotWrapped(
      Command selectedAuto,
      Pose2d startingPose,
      frc.robot.subsystems.drive.Drive drive,
      Intake intake,
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {
    return Commands.sequence(
            resetOdometry(drive, startingPose),
            simSetup(coordinator),
            deployIntake(intake),
            initialSmartLaunch(launcher, coordinator, motivator, turret, hood, spindexer),
            stowHood(hood),
            runPath(selectedAuto),
            postPathSmartLaunch(launcher, coordinator, motivator, turret, hood, spindexer))
        .finallyDo(() -> teardown(launcher, motivator, intake));
  }

  /**
   * Sprint comp auto wrapper: deploys intake and enables auto-shoot immediately, then runs the
   * path. Auto-shoot fires balls opportunistically during the path. Useful for sprint-to-neutral
   * autos where stopping to shoot at the start wastes time.
   */
  public static Command compSprintWrapped(
      Command selectedAuto,
      Pose2d startingPose,
      frc.robot.subsystems.drive.Drive drive,
      Intake intake,
      Launcher launcher,
      ShootingCoordinator coordinator,
      Motivator motivator,
      Turret turret,
      Hood hood,
      Spindexer spindexer) {
    return Commands.sequence(
            resetOdometry(drive, startingPose),
            simSetup(coordinator),
            deployIntake(intake),
            enableAutoShoot(launcher, motivator, coordinator),
            runPath(selectedAuto),
            disableAutoShoot(launcher, motivator, coordinator),
            postPathSmartLaunch(launcher, coordinator, motivator, turret, hood, spindexer))
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
