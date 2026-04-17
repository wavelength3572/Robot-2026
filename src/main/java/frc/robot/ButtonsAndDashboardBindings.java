package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootingCommands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.motivator.Motivator;
import frc.robot.subsystems.shooting.ShootingCoordinator;
import frc.robot.subsystems.shooting.ShotCalculator;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FuelSim;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashSet;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class ButtonsAndDashboardBindings {

  private static OperatorInterface oi;
  private static Drive drive;
  private static Vision vision;
  private static Intake intake;
  private static Turret turret;
  private static Launcher launcher;
  private static Motivator motivator;
  private static Spindexer spindexer;
  private static Hood hood;
  private static Climber climber;
  private static ShootingCoordinator shootingCoordinator;

  // Per-subsystem tuning setpoints
  private static final LoggedTunableNumber tuningLauncherVelocity =
      new LoggedTunableNumber(
          "Tuning/Launcher/TuningVelocity", Constants.getRobotConfig().getTuningLauncherVelocity());
  private static final LoggedTunableNumber tuningMotivatorVelocity =
      new LoggedTunableNumber(
          "Tuning/Motivator/TuningVelocity",
          Constants.getRobotConfig().getTuningMotivatorVelocity());
  private static final LoggedTunableNumber tuningSpindexerVelocity =
      new LoggedTunableNumber(
          "Tuning/Spindexer/TuningVelocity",
          Constants.getRobotConfig().getTuningSpindexerVelocity());
  private static final LoggedTunableNumber tuningHoodAngle =
      new LoggedTunableNumber(
          "Tuning/Hood/TuningAngle", Constants.getRobotConfig().getTuningHoodAngle());
  private static final LoggedTunableNumber outsideTuningAngle =
      new LoggedTunableNumber(
          "Tuning/Turret/OutsideTuningAngle",
          Constants.getRobotConfig().getTuningTurretOutsideAngle());
  private static final LoggedTunableNumber tuningIntakeDeployedVelocity =
      new LoggedTunableNumber(
          "Tuning/Intake/IntakeRollers/DeployedVelocity",
          Constants.getRobotConfig().getTuningIntakeDeployedVelocity());
  private static final LoggedTunableNumber tuningIntakeRetractRollerVelocity =
      new LoggedTunableNumber(
          "Tuning/Intake/IntakeRollers/RetractRollerVelocity",
          Constants.getRobotConfig().getTuningIntakeRetractRollerVelocity());

  public ButtonsAndDashboardBindings() {}

  public static void configureTestModeButtonBindings(
      OperatorInterface operatorInterface, Drive drive) {
    oi = operatorInterface;
    ButtonsAndDashboardBindings.drive = drive;
  }

  public static void configureBindings(
      OperatorInterface operatorInterface,
      Drive drive,
      Vision vision,
      Intake intake,
      Turret turret,
      Launcher launcher,
      Motivator motivator,
      Spindexer spindexer,
      Hood hood,
      ShootingCoordinator shootingCoordinator,
      Climber climber) {
    ButtonsAndDashboardBindings.oi = operatorInterface;
    ButtonsAndDashboardBindings.drive = drive;
    ButtonsAndDashboardBindings.vision = vision;
    ButtonsAndDashboardBindings.intake = intake;
    ButtonsAndDashboardBindings.turret = turret;
    ButtonsAndDashboardBindings.launcher = launcher;
    ButtonsAndDashboardBindings.motivator = motivator;
    ButtonsAndDashboardBindings.spindexer = spindexer;
    ButtonsAndDashboardBindings.hood = hood;
    ButtonsAndDashboardBindings.shootingCoordinator = shootingCoordinator;
    ButtonsAndDashboardBindings.climber = climber;

    configureDriverButtonBindings();
    configureOperatorButtonBindings();
    configureDashboardBindings();
  }

  /****************************** */
  /*** DASHBOARD BINDINGS ****** */
  /****************************** */

  private static void configureDashboardBindings() {
    // Vision toggle on dashboard
    if (vision != null) {
      // Vision ON by default in simulation (use SmartDashboard toggle to disable if needed)
      SmartDashboard.putBoolean("Tuning/Vision/Enable", vision.isVisionOn());
      SmartDashboard.putData(
          "Tuning/Vision/Toggle",
          Commands.runOnce(
                  () -> {
                    vision.toggleVision();
                    SmartDashboard.putBoolean("Tuning/Vision/Enable", vision.isVisionOn());
                  })
              .ignoringDisable(true)
              .withName("Toggle Vision"));
    }

    // Simulation fuel management (available for any robot with a coordinator)
    if (shootingCoordinator != null) {
      SmartDashboard.putData(
          "Sim/FuelReset", ShootingCommands.resetSimulationCommand(shootingCoordinator));
      SmartDashboard.putData(
          "Sim/ResetStartOfMatch", ShootingCommands.resetStartingFieldCommand(shootingCoordinator));
      SmartDashboard.putData(
          "Sim/ToggleOutpostBarriers",
          Commands.runOnce(() -> FuelSim.getInstance().toggleOutpostBarriers())
              .ignoringDisable(true)
              .withName("Toggle Outpost Barriers"));
    }

    // Climber dashboard buttons (mirrors button box)
    if (climber != null) {
      SmartDashboard.putData(
          "Sim/ClimberExtend", Commands.runOnce(climber::extend).withName("Climber Extend"));
      SmartDashboard.putData(
          "Sim/ClimberStow", Commands.runOnce(climber::stow).withName("Climber Stow"));
      SmartDashboard.putData(
          "Sim/ClimberClimb", Commands.runOnce(climber::climb).withName("Climber Climb"));

      // Pit mode: raw voltage buttons to manually retract/extend climber, then zero encoder
      var pitRetractVolts = new LoggedTunableNumber("Pit/Climber/RetractVolts", -1.0);
      var pitExtendVolts = new LoggedTunableNumber("Pit/Climber/ExtendVolts", 1.0);
      SmartDashboard.putData(
          "Pit/Climber/RawVoltageRetract",
          Commands.startEnd(
                  () -> {
                    climber.setSoftLimitsEnabled(false);
                    climber.setVoltage(pitRetractVolts.get());
                  },
                  () -> {
                    climber.setVoltage(0.0);
                    climber.setSoftLimitsEnabled(true);
                  })
              .withName("Pit Raw Voltage Retract"));
      SmartDashboard.putData(
          "Pit/Climber/RawVoltageExtend",
          Commands.startEnd(
                  () -> {
                    climber.setSoftLimitsEnabled(false);
                    climber.setVoltage(pitExtendVolts.get());
                  },
                  () -> {
                    climber.setVoltage(0.0);
                    climber.setSoftLimitsEnabled(true);
                  })
              .withName("Pit Raw Voltage Extend"));
      SmartDashboard.putData(
          "Pit/Climber/ZeroEncoder",
          Commands.runOnce(climber::zeroEncoder)
              .ignoringDisable(true)
              .withName("Pit Climber Zero"));
      SmartDashboard.putData(
          "Pit/Climber/SetEncoderAtExtended",
          Commands.runOnce(climber::setEncoderAtExtended)
              .ignoringDisable(true)
              .withName("Pit Set Encoder At Extended"));

      // Home to hard stop: drives slowly in reverse until motor current spikes against
      // the mechanical stop, then zeros the encoder. Tune via Climber/homing* on dashboard.
      // Deferred so the debouncer and timeout pick up live tunable changes on each run.
      Set<Subsystem> homeReqs = new HashSet<>();
      homeReqs.add(climber);
      SmartDashboard.putData(
          "Pit/Climber/HomeToHardStop",
          Commands.defer(climber::homeCommand, homeReqs).withName("Pit Climber Home"));
    }

    // Launcher RPM trim buttons (mirrors button box axis knob positions)
    SmartDashboard.putData(
        "Trim/SetMinus50",
        Commands.runOnce(() -> ShootingCommands.setLauncherTrimRPM(-50.0))
            .ignoringDisable(true)
            .withName("Trim -50"));
    SmartDashboard.putData(
        "Trim/SetZero",
        Commands.runOnce(() -> ShootingCommands.setLauncherTrimRPM(0.0))
            .ignoringDisable(true)
            .withName("Trim 0"));
    SmartDashboard.putData(
        "Trim/SetPlus75",
        Commands.runOnce(() -> ShootingCommands.setLauncherTrimRPM(75.0))
            .ignoringDisable(true)
            .withName("Trim +75"));
    SmartDashboard.putData(
        "Trim/SetPlus150",
        Commands.runOnce(() -> ShootingCommands.setLauncherTrimRPM(150.0))
            .ignoringDisable(true)
            .withName("Trim +150"));

    // Turret angle trim buttons — nudge aim left/right by 0.5 deg, max +/- 3 deg
    SmartDashboard.putData(
        "Trim/TurretLeft",
        Commands.runOnce(() -> ShootingCoordinator.trimLeft())
            .ignoringDisable(true)
            .withName("Turret Trim Left"));
    SmartDashboard.putData(
        "Trim/TurretRight",
        Commands.runOnce(() -> ShootingCoordinator.trimRight())
            .ignoringDisable(true)
            .withName("Turret Trim Right"));
    SmartDashboard.putData(
        "Trim/TurretReset",
        Commands.runOnce(() -> ShootingCoordinator.resetTurretTrim())
            .ignoringDisable(true)
            .withName("Turret Trim Reset"));
    SmartDashboard.putNumber("Trim/TurretDeg", ShootingCoordinator.getTurretTrimDeg());

    // Coordinated shooting controls (requires coordinator and launcher)
    if (shootingCoordinator != null && launcher != null) {
      configureShootingControls();
    }

    // Per-subsystem tuning run buttons (always, for all robot types)
    configureTuningControls();

    // Trajectory calculators (what-if arc and distance-to-pose)
    if (shootingCoordinator != null) {
      configureShotCalculator();
    }
  }

  /** Configure dashboard controls for coordinated shooting system. */
  private static void configureShootingControls() {
    // Initialize all tunables and status values so they appear on dashboard
    // immediately
    ShootingCommands.initTunables();

    // === Shot Preset Fire Buttons (mirrors button box, usable from dashboard) ===
    if (turret != null) {
      SmartDashboard.putData(
          "Shots/HubShot/Fire",
          ShootingCommands.hubShotCommand(
              launcher, shootingCoordinator, motivator, turret, hood, spindexer));
      SmartDashboard.putData(
          "Shots/LeftTrench/Fire",
          ShootingCommands.leftTrenchShotCommand(
              launcher, shootingCoordinator, motivator, turret, hood, spindexer));
      SmartDashboard.putData(
          "Shots/RightTrench/Fire",
          ShootingCommands.rightTrenchShotCommand(
              launcher, shootingCoordinator, motivator, turret, hood, spindexer));
      {
        Set<Subsystem> dashSmartLaunchReqs = new HashSet<>();
        dashSmartLaunchReqs.add(launcher);
        dashSmartLaunchReqs.add(turret);
        if (hood != null) dashSmartLaunchReqs.add(hood);
        if (motivator != null) dashSmartLaunchReqs.add(motivator);
        if (spindexer != null) dashSmartLaunchReqs.add(spindexer);
        SmartDashboard.putData(
            "Shots/SmartLaunch/Fire",
            Commands.defer(
                () ->
                    ShootingCommands.smartLaunchDangerousCommand(
                        launcher, shootingCoordinator, motivator, turret, hood, spindexer),
                dashSmartLaunchReqs));
      }
      // Auto-track: toggle works while disabled; turret default command checks the flag
      // TODO: Only enable auto-tracking when we are in the alliance zone AND we have
      //       completed at least one smart launch. This avoids unnecessary turret movement
      //       during intake cycles across the field.
      SmartDashboard.setDefaultBoolean("Shots/AutoTrack/Enabled", false);
      SmartDashboard.putData(
          "Shots/AutoTrack/Toggle",
          Commands.runOnce(
                  () -> {
                    boolean current = SmartDashboard.getBoolean("Shots/AutoTrack/Enabled", false);
                    SmartDashboard.putBoolean("Shots/AutoTrack/Enabled", !current);
                  })
              .ignoringDisable(true)
              .withName("Toggle AutoTrack"));

      // Turret default command: auto-track when flag is enabled, otherwise idle
      turret.setDefaultCommand(
          Commands.run(
                  () -> {
                    if (SmartDashboard.getBoolean("Shots/AutoTrack/Enabled", false)) {
                      turret.setActivelyCommanded(true);
                      if (shootingCoordinator.getCurrentShot() != null) {
                        turret.setOutsideTurretAngle(
                            shootingCoordinator.getCurrentTurretAngleDeg());
                        boolean turretReady = turret.atTarget();
                        boolean achievable = shootingCoordinator.isCurrentShotAchievable();
                        boolean aimReady = turretReady && achievable;
                        Logger.recordOutput("SmartLaunch/AutoTrack/AimReady", aimReady);
                        SmartDashboard.putString(
                            "Match/Status/AutoTrackAimMode",
                            achievable ? "Tracking" : "Out of Range");
                        SmartDashboard.putBoolean("Match/Status/AutoTrackAimReady", aimReady);
                      } else {
                        Logger.recordOutput("SmartLaunch/AutoTrack/AimReady", false);
                      }
                      Logger.recordOutput("SmartLaunch/AutoTrack/Active", true);
                      SmartDashboard.putBoolean("Match/Status/AutoTracking", true);
                    } else {
                      turret.setActivelyCommanded(false);
                      Logger.recordOutput("SmartLaunch/AutoTrack/Active", false);
                      Logger.recordOutput("SmartLaunch/AutoTrack/AimReady", false);
                      SmartDashboard.putBoolean("Match/Status/AutoTracking", false);
                      SmartDashboard.putBoolean("Match/Status/AutoTrackAimReady", false);
                      SmartDashboard.putString("Match/Status/AutoTrackAimMode", "Off");
                    }
                  },
                  turret)
              .withName("AutoTrackDefault"));
    }

    // Set fuel stored to 8 (works while disabled)
    SmartDashboard.putData(
        "Match/SetFuel8",
        Commands.runOnce(
                () -> {
                  if (shootingCoordinator.getVisualizer() != null) {
                    shootingCoordinator.getVisualizer().setFuelCount(8);
                  }
                })
            .ignoringDisable(true)
            .withName("Set Fuel 8"));

    System.out.println("[Shooting] Shooting controls configured on SmartDashboard");
  }

  /** Configure per-subsystem tuning run buttons. Always called for all robot types. */
  private static void configureTuningControls() {
    // Launcher: Run button reads local TuningVelocity
    if (launcher != null) {
      SmartDashboard.putData(
          "Tuning/Launcher/RunAtTuningVelocity",
          launcher.runAtTunableVelocityCommand(tuningLauncherVelocity));
    }

    // Motivator: Run button reads local TuningVelocity
    if (motivator != null) {
      SmartDashboard.putData(
          "Tuning/Motivator/RunAtTuningVelocity",
          motivator.runMotivatorCommand(tuningMotivatorVelocity));
    }

    // Spindexer: Run button reads local TuningVelocity
    if (spindexer != null) {
      SmartDashboard.putData(
          "Tuning/Spindexer/RunAtTuningVelocity",
          spindexer.runSpindexerCommand(tuningSpindexerVelocity));
    }

    // Hood: SetAngle button reads local TuningAngle
    if (hood != null) {
      SmartDashboard.putData(
          "Tuning/Hood/SetToTuningAngle",
          Commands.run(() -> hood.setHoodAngle(tuningHoodAngle.get()), hood)
              .withName("Hood: Set Tuning Angle"));
    }

    // Turret: SetAngle button reads local TuningAngle
    if (turret != null) {
      SmartDashboard.putData(
          "Tuning/Turret/SetToTuningAngle",
          Commands.run(() -> turret.setOutsideTurretAngle(outsideTuningAngle.get()), turret)
              .withName("Turret: Set Tuning Angle"));
    }

    // Intake: Deploy/Toggle and Rollers/Run
    if (intake != null) {
      SmartDashboard.putData(
          "Tuning/Intake/IntakeDeploy/Deploy",
          Commands.runOnce(intake::deploy, intake).withName("Intake: Deploy"));
      SmartDashboard.putData(
          "Tuning/Intake/IntakeDeploy/Retract",
          Commands.runOnce(intake::retract, intake).withName("Intake: Retract"));
      SmartDashboard.putData("Tuning/Intake/STOP", intake.stopAllCommand());
      SmartDashboard.putData("Tuning/Intake/PreClimbFlush", intake.preClimbFlushCommand());
      SmartDashboard.putData(
          "Tuning/Intake/IntakeRollers/RunAtTuningVelocity",
          Commands.run(() -> intake.setRollerVelocity(tuningIntakeDeployedVelocity.get()))
              .finallyDo(intake::stopRollers)
              .withName("Intake: Run at Tuning Velocity"));
    }
  }

  /**
   * Configure trajectory calculator controls on the dashboard. Two independent tools:
   *
   * <p><b>What-If:</b> Render a hypothetical trajectory arc for user-specified RPM and hood angle,
   * using the turret's current azimuth. Logged to {@code Turret/Trajectory/WhatIf}.
   *
   * <p><b>Distance:</b> Place the robot at a given distance from the hub so the turret's periodic
   * loop auto-computes the optimized trajectory, then read back the resulting RPM and hood angle.
   */
  private static void configureShotCalculator() {
    // === Distance: move robot to a distance from hub, read back the optimized shot
    // ===
    SmartDashboard.putNumber("TrajectoryCalculators/Distance/Inches", 118.0);
    SmartDashboard.putNumber("TrajectoryCalculators/Distance/OptimalRPM", 0.0);
    SmartDashboard.putNumber("TrajectoryCalculators/Distance/OptimalHoodAngleDeg", 0.0);

    SmartDashboard.putData(
        "TrajectoryCalculators/Distance/SetPose",
        Commands.runOnce(
                () -> {
                  double inches =
                      SmartDashboard.getNumber("TrajectoryCalculators/Distance/Inches", 118.0);
                  double meters = inches * 0.0254;
                  Translation3d hub = FieldConstants.Hub.innerCenterPoint;
                  // Place robot directly in front of hub (toward blue alliance wall)
                  drive.setPose(
                      new Pose2d(hub.getX() - meters, hub.getY(), Rotation2d.fromDegrees(0)));
                },
                drive)
            .ignoringDisable(true)
            .withName("Set Pose From Distance"));

    // Read back the coordinator's optimized shot (press after SetPose to populate
    // RPM/angle)
    SmartDashboard.putData(
        "TrajectoryCalculators/Distance/ReadShot",
        Commands.runOnce(
                () -> {
                  ShotCalculator.ShotResult shot = shootingCoordinator.getCurrentShot();
                  if (shot == null) return;
                  double rpm = shot.launcherRPM();
                  SmartDashboard.putNumber(
                      "TrajectoryCalculators/Distance/OptimalRPM", Math.round(rpm * 10.0) / 10.0);
                  SmartDashboard.putNumber(
                      "TrajectoryCalculators/Distance/OptimalHoodAngleDeg",
                      Math.round(shot.hoodAngleDeg() * 10.0) / 10.0);
                })
            .ignoringDisable(true)
            .withName("Read Optimized Shot"));
  }

  /****************************** */
  /*** DRIVER BINDINGS ****** */
  /****************************** */

  private static void configureDriverButtonBindings() {
    // Enable joystick driving for all robots
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, oi::getTranslateX, oi::getTranslateY, oi::getRotate));

    // Gyro Reset (available on all robots with a gyro)
    oi.getResetGyroButton()
        .onTrue(Commands.runOnce(drive::zeroGyroscope, drive).ignoringDisable(true));

    // Snap to 90 degrees (face left side of field) - toggle on/off
    oi.getRightJoyUpButton()
        .toggleOnTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, oi::getTranslateX, oi::getTranslateY, () -> Rotation2d.fromDegrees(90.0)));

    // Pathfind to nearest tower pole — hold to follow path, release to stop.
    // Re-pressing recalculates from current position.
    // Auto-extends climber if operator hasn't already (within 1m, runs in parallel with driving).
    // After pathfind + extend complete, pauses 0.2s to let the drive settle, then fires climb
    // (same as manual Button 10). Releasing the button before climb fires cancels everything;
    // once climb fires, periodic drives it to completion regardless. Operator can still press
    // climb manually at any time — earlier press wins, later press is a harmless no-op.
    // Gated on isNearAllianceTower so it only activates within 3m of a climb pose.
    // Buttons 23 (left slider) and 24 (right slider) below the right axis, plus button 25.
    Command poleAlignWithAutoExtend =
        climber != null
            ? Commands.parallel(
                DriveCommands.pathfindToNearestPole(drive),
                Commands.waitUntil(
                        () ->
                            drive
                                    .getPose()
                                    .getTranslation()
                                    .getDistance(
                                        DriveCommands.findNearestClimbPose(drive).getTranslation())
                                <= Units.feetToMeters(
                                    Constants.getRobotConfig().getClimberAutoExtendDistanceFeet()))
                    .andThen(Commands.runOnce(climber::extend))
                    .withName("AutoExtendClimber"))
            // .andThen(Commands.waitSeconds(0.2))
            // .andThen(Commands.runOnce(climber::climb))
            : DriveCommands.pathfindToNearestPole(drive);
    oi.getRightJoyLeftButton()
        .and(() -> DriveCommands.isNearAllianceTower(drive))
        .whileTrue(poleAlignWithAutoExtend);
    oi.getRightJoyRightButton()
        .and(() -> DriveCommands.isNearAllianceTower(drive))
        .whileTrue(poleAlignWithAutoExtend);
    oi.getRightJoyDownButton()
        .and(() -> DriveCommands.isNearAllianceTower(drive))
        .whileTrue(poleAlignWithAutoExtend);

    // Sim/test dashboard button: one-shot schedule of the full pole-align + auto-climb flow.
    // Runs to completion (no whileTrue cancel-on-release behavior) — use for sim verification.
    SmartDashboard.putData(
        "Sim/PoleAlignAutoClimb", poleAlignWithAutoExtend.withName("PoleAlignAutoClimb"));

    // X-stance button (interlink button 13): while held, lock wheels in X pattern.
    // Only activates when robot speed is below 1 m/s to prevent skidding.
    oi.getLockWheels()
        .and(
            () -> {
              ChassisSpeeds speeds = drive.getChassisSpeeds();
              return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 1.0;
            })
        .whileTrue(Commands.run(() -> drive.stopWithX(), drive).withName("XStance"));
  }

  /****************************** */
  /*** BUTTON BOX BINDINGS ****** */
  /****************************** */

  private static void configureOperatorButtonBindings() {
    // Intake controls
    if (intake != null) {
      // Button 4: Deploy and run rollers while held; on release, stop rollers but stay deployed.
      // Also reciprocates the spindexer to keep fuel loose while intaking.
      // Bound as separate commands so shooting can interrupt the spindexer without killing intake.
      oi.getButtonBox1Button4()
          .whileTrue(
              Commands.startEnd(
                  () -> {
                    intake.deploy();
                    intake.setRollerVelocityWhenDeployed(tuningIntakeDeployedVelocity.get());
                  },
                  intake::stopRollers,
                  intake));
      // if (spindexer != null) {
      //   // Only reciprocate spindexer when smart launch (Button 12) is NOT held,
      //   // otherwise Button 4 steals the spindexer subsystem and cancels smart launch.
      //   oi.getButtonBox1Button4()
      //       .and(oi.getButtonBox1Button12().negate())
      //       .whileTrue(spindexer.reciprocateCommand());
      // }

      // Button 3: Retract and run rollers while held; on release, stop rollers but stay retracted.
      oi.getButtonBox1Button3()
          .whileTrue(
              Commands.startEnd(
                  () -> {
                    intake.retract();
                    intake.setRollerVelocity(tuningIntakeDeployedVelocity.get());
                  },
                  intake::stopRollers,
                  intake));
    }

    // Smart launch:Button 12— mode selected by dashboard toggle
    if (shootingCoordinator != null && launcher != null && turret != null && drive != null) {
      Set<Subsystem> smartLaunchReqs = new HashSet<>();
      smartLaunchReqs.add(launcher);
      smartLaunchReqs.add(turret);
      if (hood != null) smartLaunchReqs.add(hood);
      if (motivator != null) smartLaunchReqs.add(motivator);
      if (spindexer != null) smartLaunchReqs.add(spindexer);
      Command smartLaunchCmd =
          Commands.defer(
              () ->
                  ShootingCommands.smartLaunchDangerousCommand(
                      launcher, shootingCoordinator, motivator, turret, hood, spindexer),
              smartLaunchReqs);
      if (intake != null) {
        java.util.function.BooleanSupplier smartLaunchClimbGate =
            climber != null
                ? () ->
                    climber.getState() == Climber.ClimberState.CLIMBING
                        || climber.getState() == Climber.ClimberState.CLIMBED
                : () -> false;
        // Rollers spin in all zones when intake is deployed (safety interlock handles retracted
        // state)
        Command rollerCmd =
            intake.smartLaunchRollerCommand(
                tuningIntakeDeployedVelocity::get, oi.getButtonBox1Button4()::getAsBoolean);

        smartLaunchCmd =
            smartLaunchCmd.alongWith(intake.agitateCommand(smartLaunchClimbGate, true), rollerCmd);
      }
      oi.getButtonBox1Button12().whileTrue(smartLaunchCmd);

      // Unclog Button — flag-based so it doesn't interrupt the active shooting command.
      // While held, the spindexer reverses; on release, shooting resumes instantly.
      oi.getButtonBox1Button2().onTrue(Commands.runOnce(spindexer::activateUnclog));
      oi.getButtonBox1Button2().onFalse(Commands.runOnce(spindexer::deactivateUnclog));
    }

    // Launcher RPM trim — button box 1 axis knob (4 positions)
    // Axis values: -50 = (0,1), neutral = (0,-1), +50 = (-1,1), +100 = (1,1)
    // Three positions share Y+=1, so we use combo triggers to distinguish them.
    Trigger yPos = oi.getButtonBox1YAxisPositive(); // -50
    Trigger yNeg = oi.getButtonBox1YAxisNegative(); // +75
    Trigger xNeg = oi.getButtonBox1XAxisNegative(); // 0 spot
    Trigger xPos = oi.getButtonBox1XAxisPositive(); // +125

    // -50 RPM: Y+
    yPos.onTrue(
        Commands.runOnce(() -> ShootingCommands.setLauncherTrimRPM(-50.0)).ignoringDisable(true));
    // X-
    xNeg.onTrue(
        Commands.runOnce(() -> ShootingCommands.setLauncherTrimRPM(0.0)).ignoringDisable(true));

    // Y-
    yNeg.onTrue(
        Commands.runOnce(() -> ShootingCommands.setLauncherTrimRPM(75.0)).ignoringDisable(true));

    // X+
    xPos.onTrue(
        Commands.runOnce(() -> ShootingCommands.setLauncherTrimRPM(125.0)).ignoringDisable(true));

    // Hub shot: Button 8 — fixed position launch for close-range hub shots
    if (launcher != null && turret != null) {
      java.util.function.BooleanSupplier climbingOrClimbed =
          climber != null
              ? () ->
                  climber.getState() == Climber.ClimberState.CLIMBING
                      || climber.getState() == Climber.ClimberState.CLIMBED
              : () -> false;

      Command hubShotCmd =
          ShootingCommands.hubShotCommand(
              launcher, shootingCoordinator, motivator, turret, hood, spindexer);
      if (intake != null) {
        hubShotCmd =
            hubShotCmd.alongWith(
                intake.agitateCommand(climbingOrClimbed, true),
                intake.smartLaunchRollerCommand(
                    tuningIntakeDeployedVelocity::get, oi.getButtonBox1Button4()::getAsBoolean));
      }
      oi.getButtonBox1Button8().whileTrue(hubShotCmd);

      // Left trench shot: Button 6
      Command leftTrenchCmd =
          ShootingCommands.leftTrenchShotCommand(
              launcher, shootingCoordinator, motivator, turret, hood, spindexer);
      if (intake != null) {
        leftTrenchCmd =
            leftTrenchCmd.alongWith(
                intake.agitateCommand(climbingOrClimbed, true),
                intake.smartLaunchRollerCommand(
                    tuningIntakeDeployedVelocity::get, oi.getButtonBox1Button4()::getAsBoolean));
      }
      oi.getButtonBox1Button5().whileTrue(leftTrenchCmd);

      // Right trench shot: Button 5
      Command rightTrenchCmd =
          ShootingCommands.rightTrenchShotCommand(
              launcher, shootingCoordinator, motivator, turret, hood, spindexer);
      if (intake != null) {
        rightTrenchCmd =
            rightTrenchCmd.alongWith(
                intake.agitateCommand(climbingOrClimbed, true),
                intake.smartLaunchRollerCommand(
                    tuningIntakeDeployedVelocity::get, oi.getButtonBox1Button4()::getAsBoolean));
      }
      oi.getButtonBox1Button6().whileTrue(rightTrenchCmd);
    }

    // Turret lock — hold button 1 to lock turret in place (brake mode, no movement commands)
    // Also check initial state so a held button at boot immediately locks the turret.
    if (turret != null) {
      Trigger turretLockTrigger = oi.getButtonBox1Button1();
      turretLockTrigger.onTrue(Commands.runOnce(turret::lock));
      turretLockTrigger.onFalse(Commands.runOnce(turret::unlock));
      if (turretLockTrigger.getAsBoolean()) {
        turret.lock();
      }
    }

    // Spindexer feeding suppress - operator can hold to prevent feeding/launching.
    // Uses a flag so shooting commands keep running and resume feeding instantly on release.
    if (spindexer != null) {
      Trigger suppressTrigger = oi.getButtonBox1Button11();
      suppressTrigger.onTrue(Commands.runOnce(spindexer::suppressFeeding));
      suppressTrigger.onFalse(Commands.runOnce(spindexer::unsuppressFeeding));

      // Auto-unclog toggle — on by default, disable from dashboard if needed
      SmartDashboard.putBoolean(
          "Tuning/Spindexer/AutoUnclog/Enabled", spindexer.isAutoUnclogEnabled());
      SmartDashboard.putData(
          "Tuning/Spindexer/AutoUnclog/Toggle",
          Commands.runOnce(
                  () -> {
                    if (spindexer.isAutoUnclogEnabled()) {
                      spindexer.disableAutoUnclog();
                    } else {
                      spindexer.enableAutoUnclog();
                    }
                    SmartDashboard.putBoolean(
                        "Tuning/Spindexer/AutoUnclog/Enabled", spindexer.isAutoUnclogEnabled());
                  })
              .ignoringDisable(true)
              .withName("Toggle AutoUnclog"));
    }

    // Climber controls — no subsystem requirement to avoid canceling driver auto-align
    // B7: extend (from STOWED or CLIMBED, no-op if already extended)
    // B7 hold 2s: stow (from EXTENDED or EXTENDING, deliberate action)
    // B10: climb (from EXTENDED only)
    if (climber != null) {
      // oi.getButtonBox1Button7().onTrue(Commands.runOnce(climber::extend));
      // oi.getButtonBox1Button7()
      //     .debounce(Constants.getRobotConfig().getClimberStowHoldTimeSec())
      //     .onTrue(Commands.runOnce(climber::stow));
      // oi.getButtonBox1Button10().onTrue(Commands.runOnce(climber::climb));
    }
  }
}
