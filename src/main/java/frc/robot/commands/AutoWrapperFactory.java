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
import frc.robot.subsystems.climber.Climber;
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
    PASS_AND_SHOOT,
    /**
     * Shoot on the move in alliance zones (hub shots under speed threshold); collect only in
     * neutral zone — no passing. Identical to PASS_AND_SHOOT except passing is disabled.
     */
    NO_PASS,
    /**
     * For autos that stay in the alliance zone (e.g. depot/outpost/climb). Arms immediately so the
     * robot can shoot on the move without needing a zone transition to re-arm.
     */
    IMMEDIATE_ARM
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
      Spindexer spindexer,
      Climber climber) {

    List<Command> steps = new ArrayList<>();

    // Always: reset odometry, sim setup
    steps.add(resetOdometry(drive, startingPose));
    steps.add(simSetup(coordinator));

    // Start strategy
    if (startStrategy == StartStrategy.SHOOT_PRELOADS) {
      steps.add(initialSmartLaunch(launcher, coordinator, motivator, turret, hood, spindexer));
    }

    // Determine arm trigger for SmartLaunch2 (the path-phase shooting command).
    // Based on path strategy, not start strategy — SHOOT_PRELOADS needs the hood to stay
    // safe after preload shooting just like SPRINT needs it safe on the outbound trip.
    //   - PASS_AND_SHOOT: ON_PASS_ZONE       — arm when entering neutral/opponent zone
    //   - NO_PASS:        ON_ALLIANCE_RETURN  — arm when returning to alliance after neutral
    //   - IMMEDIATE_ARM: IMMEDIATE           — stays in alliance, no zone transition needed
    //   - END_OF_PATH:    IMMEDIATE           — no SmartLaunch2 during path anyway
    ShootingCoordinator.ArmTrigger trigger;
    if (pathStrategy == PathShootingStrategy.PASS_AND_SHOOT) {
      trigger = ShootingCoordinator.ArmTrigger.ON_PASS_ZONE;
    } else if (pathStrategy == PathShootingStrategy.NO_PASS) {
      trigger = ShootingCoordinator.ArmTrigger.ON_ALLIANCE_RETURN;
    } else {
      trigger = ShootingCoordinator.ArmTrigger.IMMEDIATE;
    }

    // PASS_AND_SHOOT fires in pass zones; other strategies only fire in alliance zones.
    // Must run inside the command sequence (not at construction time) so each auto
    // sets the flag when it actually starts, not when all autos are built at init.
    boolean passingEnabled = pathStrategy == PathShootingStrategy.PASS_AND_SHOOT;
    steps.add(Commands.runOnce(() -> coordinator.setAutoPassingEnabled(passingEnabled)));

    // Build the path + intake deploy to run together
    Command pathWithIntake = Commands.parallel(runPath(selectedAuto), deployIntake(intake));

    // For PASS_AND_SHOOT, NO_PASS, and IMMEDIATE_ARM: SmartLaunch2
    // runs for the ENTIRE auto (path + post-path). It doesn't die when the path ends — the
    // path is just one step in the sequence that runs underneath it. After the path, the
    // robot is stationary and SmartLaunch2 keeps firing until auto ends.
    //
    // For END_OF_PATH: SmartLaunch2 only runs after the path completes.
    // Climb gate for post-path agitation: suppress during CLIMBING/CLIMBED
    java.util.function.BooleanSupplier climbingOrClimbed =
        climber != null
            ? () ->
                climber.getState() == Climber.ClimberState.CLIMBING
                    || climber.getState() == Climber.ClimberState.CLIMBED
            : () -> false;

    if (pathStrategy != PathShootingStrategy.END_OF_PATH) {
      // Path runs, then post-path agitation kicks in. SmartLaunch2 runs the entire time
      // in parallel and keeps the group alive after both path and agitation finish.
      // Mid-path agitation is handled by AutoAgitate zone event markers in the paths.
      Command pathThenAgitate =
          intake != null
              ? Commands.sequence(pathWithIntake, intake.autoAgitateCommand(climbingOrClimbed))
              : pathWithIntake;
      steps.add(
          Commands.parallel(
              pathThenAgitate,
              ShootingCommands.smartLaunchDangerousCommand(
                      launcher, coordinator, motivator, turret, hood, spindexer, trigger)
                  .asProxy()));
    } else {
      // END_OF_PATH: run path first, then shoot with agitation
      steps.add(pathWithIntake);
      Command postPathShoot =
          ShootingCommands.smartLaunchDangerousCommand(
                  launcher, coordinator, motivator, turret, hood, spindexer)
              .withTimeout(10.0)
              .asProxy();
      if (intake != null) {
        postPathShoot = postPathShoot.alongWith(intake.autoAgitateCommand(climbingOrClimbed));
      }
      steps.add(postPathShoot);
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

  private static Command runPath(Command selectedAuto) {
    return selectedAuto.asProxy();
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
