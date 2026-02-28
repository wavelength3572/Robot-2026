package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootingCommands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.motivator.Motivator;
import frc.robot.subsystems.shooting.ShootingCoordinator;
import frc.robot.subsystems.shooting.ShotCalculator;
import frc.robot.subsystems.shooting.ShotVisualizer;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.BenchTestMetrics;
import frc.robot.util.FuelSim;
import frc.robot.util.LoggedTunableNumber;

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
  private static ShootingCoordinator shootingCoordinator;

  // Intake bench test tunables
  private static final LoggedTunableNumber testIntakeSpeed =
      new LoggedTunableNumber("BenchTest/IntakePowerControl/Power", 0.8);
  private static final LoggedTunableNumber testIntakeRPM =
      new LoggedTunableNumber(
          "BenchTest/IntakeVelocityControl/RollerRPM", Intake.ROLLER_INTAKE_RPM_DEPLOYED);

  // Per-subsystem tuning setpoints
  private static final LoggedTunableNumber tuningLauncherVelocity =
      new LoggedTunableNumber("Tuning/Launcher/TuningVelocity", 1700.0);
  private static final LoggedTunableNumber tuningMotivatorVelocity =
      new LoggedTunableNumber("Tuning/Motivator/TuningVelocity", 1000.0);
  private static final LoggedTunableNumber tuningSpindexerVelocity =
      new LoggedTunableNumber("Tuning/Spindexer/TuningVelocity", 1000.0);
  private static final LoggedTunableNumber tuningHoodAngle =
      new LoggedTunableNumber("Tuning/Hood/TuningAngle", 15.0);
  private static final LoggedTunableNumber tuningTurretAngle =
      new LoggedTunableNumber("Tuning/Turret/TuningAngle", 0.0);
  private static final LoggedTunableNumber tuningIntakeDeployedVelocity =
      new LoggedTunableNumber("Tuning/Intake/IntakeRollers/DeployedVelocity", 1000.0);
  private static final LoggedTunableNumber tuningIntakeAgitationVelocity =
      new LoggedTunableNumber("Tuning/Intake/IntakeRollers/AgitationVelocity", 200.0);

  // Preset field positions for simulation (label, x_meters, y_meters, rotation_degrees)
  private static final String[][] PRESET_POSITIONS = {
    {"DepotStart", "3.50", "6.00", "180.0"},
    {"HPStart", "3.56", "0.65", "180.0"},
    {"TrenchLeft", "3.52", "7.54", "-90.28"},
    {"TrenchRight", "3.53", "0.58", "90.0"},
  };

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
      ShootingCoordinator shootingCoordinator) {
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

    // Intake bench test area
    if (intake != null) {
      configureIntakeBenchTest();
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

    // === Auto Launch Command ===
    // Auto-calculated trajectory - works for both sim and physical
    SmartDashboard.putData(
        "Match/SmartLaunch",
        ShootingCommands.launchCommand(launcher, shootingCoordinator, motivator));

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
      SmartDashboard.putData(
          "Shots/SmartLaunch/Fire",
          ShootingCommands.smartLaunchCommand(
              launcher, shootingCoordinator, motivator, turret, hood, spindexer));
      SmartDashboard.putData(
          "Shots/AutoTrack/Toggle",
          ShootingCommands.autoTrackCommand(shootingCoordinator, turret, hood));
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

    // Reset metrics button (works while disabled)
    SmartDashboard.putData(
        "BenchTest/Shooting/ResetMetrics",
        Commands.runOnce(() -> BenchTestMetrics.getInstance().reset())
            .ignoringDisable(true)
            .withName("Reset BenchTest Metrics"));

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
          Commands.run(() -> turret.setOutsideTurretAngle(tuningTurretAngle.get()), turret)
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
      SmartDashboard.putData(
          "Tuning/Intake/IntakeRollers/RunAtTuningVelocity",
          Commands.run(() -> intake.setRollerVelocity(tuningIntakeDeployedVelocity.get()))
              .finallyDo(intake::stopRollers)
              .withName("Intake: Run at Tuning Velocity"));
    }

    // Turret BenchTest buttons (kept separate from Tuning/)
    if (turret != null) {
      ShootingCommands.initTunables();
      SmartDashboard.putData(
          "BenchTest/Turret/SetOutsideAngle",
          Commands.run(
                  () ->
                      turret.setOutsideTurretAngle(
                          turret.flipOutsideAngle(
                              ShootingCommands.getOutsideTurretAngleDeg().get())),
                  turret)
              .withName("Turret Outside SetAngle"));
    }

    if (turret != null) {
      SmartDashboard.putData(
          "BenchTest/Turret/HoldOutsideAngle",
          Commands.run(
                  () ->
                      turret.holdOutsideTurretAngle(
                          ShootingCommands.getOutsideTurretAngleDeg().get(),
                          drive.getPose().getRotation().getDegrees()),
                  turret)
              .withName("Turret Outside SetAngle"));
    }

    if (turret != null) {
      SmartDashboard.putData(
          "BenchTest/Turret/SetInsideAngle",
          Commands.run(
                  () ->
                      turret.setInsideTurretAngle_ONLY_FOR_TESTING(
                          ShootingCommands.getInsideTurretAngleDeg().get()),
                  turret)
              .withName("Turret Inside SetAngle"));
    }

    if (turret != null) {
      SmartDashboard.putData(
          "BenchTest/Turret/SetVolts",
          Commands.run(
                  () -> turret.setTurretVolts(ShootingCommands.getTestTurretVolts().get()), turret)
              .withName("Turret Set Volts"));
    }
  }

  /** Configure BenchTest intake areas: deploy, power control, and velocity control. */
  private static void configureIntakeBenchTest() {
    // Deploy/Retract
    SmartDashboard.putData(
        "BenchTest/Intake/Deploy",
        Commands.runOnce(intake::deploy, intake).withName("Deploy Intake"));
    SmartDashboard.putData(
        "BenchTest/Intake/Retract",
        Commands.runOnce(intake::retract, intake).withName("Retract Intake"));
    SmartDashboard.putData("BenchTest/Intake/STOP", intake.stopAllCommand());

    // Open-loop power control (hold to run, release to stop)
    // Run button reads the Power slider live — adjust slider while running to
    // change speed
    SmartDashboard.putData(
        "BenchTest/IntakePowerControl/Run",
        Commands.run(() -> intake.setRollerSpeed(testIntakeSpeed.get()), intake)
            .finallyDo(intake::stopRollers)
            .withName("Run at Power"));
    SmartDashboard.putData(
        "BenchTest/IntakePowerControl/Run80%",
        Commands.run(() -> intake.setRollerSpeed(Intake.ROLLER_INTAKE_SPEED), intake)
            .finallyDo(intake::stopRollers)
            .withName("Run 80% Power"));
    SmartDashboard.putData(
        "BenchTest/IntakePowerControl/Reverse60%",
        Commands.run(() -> intake.setRollerSpeed(Intake.ROLLER_EJECT_SPEED), intake)
            .finallyDo(intake::stopRollers)
            .withName("Reverse 60%"));
    SmartDashboard.putData(
        "BenchTest/IntakePowerControl/HoldSlow10%",
        Commands.run(() -> intake.setRollerSpeed(Intake.ROLLER_HOLD_SPEED), intake)
            .finallyDo(intake::stopRollers)
            .withName("Hold Slow 10%"));

    // Closed-loop velocity control (hold to run, release to stop)
    SmartDashboard.putData(
        "BenchTest/IntakeVelocityControl/Run",
        Commands.run(() -> intake.setRollerVelocity(testIntakeRPM.get()), intake)
            .finallyDo(intake::stopRollers)
            .withName("Run Velocity Control"));
    SmartDashboard.putData(
        "BenchTest/IntakeVelocityControl/DeployAndRun",
        intake.deployAndRunCommand(testIntakeRPM::get));
  }

  /** Whether the what-if trajectory arc is currently displayed. */
  private static boolean whatIfVisible = false;

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
    // === What-If: hypothetical trajectory for arbitrary RPM + hood angle ===
    SmartDashboard.putNumber("TrajectoryCalculators/WhatIf/RPM", 2500.0);
    SmartDashboard.putNumber("TrajectoryCalculators/WhatIf/HoodAngleDeg", 13);

    SmartDashboard.putData(
        "TrajectoryCalculators/WhatIf/Toggle",
        Commands.runOnce(
                () -> {
                  ShotVisualizer vis = shootingCoordinator.getVisualizer();
                  if (vis == null) return;
                  whatIfVisible = !whatIfVisible;
                  if (whatIfVisible) {
                    double rpm =
                        SmartDashboard.getNumber("TrajectoryCalculators/WhatIf/RPM", 2500.0);
                    double angleDeg =
                        SmartDashboard.getNumber("TrajectoryCalculators/WhatIf/HoodAngleDeg", 45.0);
                    double exitVelocity = ShotCalculator.calculateExitVelocityFromRPM(rpm);
                    double launchAngleRad = Math.toRadians(angleDeg);
                    double azimuth = vis.getCurrentAzimuthAngle();
                    vis.updateWhatIfTrajectory(exitVelocity, launchAngleRad, azimuth);
                  } else {
                    vis.clearWhatIfTrajectory();
                  }
                })
            .ignoringDisable(true)
            .withName("Toggle What-If"));

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

    // Spindexer feeding suppress - driver can hold to prevent feeding while wiggling robot.
    // Uses a flag so shooting commands keep running and resume feeding instantly on release.
    if (spindexer != null) {
      Trigger suppressTrigger = oi.getRightJoyLeftButton().or(oi.getRightJoyRightButton());
      suppressTrigger.onTrue(Commands.runOnce(spindexer::suppressFeeding));
      suppressTrigger.onFalse(Commands.runOnce(spindexer::unsuppressFeeding));
    }

    // Snap to 90 degrees (face left side of field) - toggle on/off
    oi.getRightJoyUpButton()
        .toggleOnTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, oi::getTranslateX, oi::getTranslateY, () -> Rotation2d.fromDegrees(90.0)));
  }

  /****************************** */
  /*** BUTTON BOX BINDINGS ****** */
  /****************************** */

  private static void configureOperatorButtonBindings() {
    // Intake controls
    if (intake != null) {
      // Button 4: Deploy and run rollers (press once, stays running)
      oi.getButtonBox1Button4()
          .onTrue(
              Commands.runOnce(
                  () -> {
                    intake.deploy();
                    intake.setRollerVelocity(tuningIntakeDeployedVelocity.get());
                  },
                  intake));

      // Button 3: Retract and run rollers at agitation speed (press once, stays running)
      oi.getButtonBox1Button3()
          .onTrue(
              Commands.runOnce(
                  () -> {
                    intake.retract();
                    intake.setRollerVelocity(tuningIntakeAgitationVelocity.get());
                  },
                  intake));
    }

    // Smart launch: D-pad Up + Button 2 — odometry-based aiming (hub or pass)
    if (shootingCoordinator != null && launcher != null && turret != null) {
      oi.getButtonBox1YAxisNegative()
          .whileTrue(
              ShootingCommands.smartLaunchCommand(
                      launcher, shootingCoordinator, motivator, turret, hood, spindexer)
                  .alongWith(intakeRollersWhileShooting()));
      oi.getButtonBox1Button2()
          .whileTrue(
              ShootingCommands.smartLaunchCommand(
                      launcher, shootingCoordinator, motivator, turret, hood, spindexer)
                  .alongWith(intakeRollersWhileShooting()));
    }

    // Auto-tracking toggle: APAC right — turret/hood continuously track target
    if (shootingCoordinator != null && turret != null) {
      oi.getButtonBox1XAxisPositive()
          .toggleOnTrue(ShootingCommands.autoTrackCommand(shootingCoordinator, turret, hood));
    }

    // Hub shot: Button 8 — fixed position launch for close-range hub shots
    if (launcher != null && turret != null) {
      oi.getButtonBox1Button8()
          .whileTrue(
              ShootingCommands.hubShotCommand(
                      launcher, shootingCoordinator, motivator, turret, hood, spindexer)
                  .alongWith(intakeRollersWhileShooting()));

      // Left trench shot: Button 6
      oi.getButtonBox1Button6()
          .whileTrue(
              ShootingCommands.leftTrenchShotCommand(
                      launcher, shootingCoordinator, motivator, turret, hood, spindexer)
                  .alongWith(intakeRollersWhileShooting()));

      // Right trench shot: Button 5
      oi.getButtonBox1Button5()
          .whileTrue(
              ShootingCommands.rightTrenchShotCommand(
                      launcher, shootingCoordinator, motivator, turret, hood, spindexer)
                  .alongWith(intakeRollersWhileShooting()));
    }
  }

  /**
   * Runs intake rollers at shooting RPM while a shooting command is active, but only when the
   * intake is retracted. If the intake is deployed, does nothing so the normal roller speed is not
   * interrupted.
   */
  private static Command intakeRollersWhileShooting() {
    if (intake == null) return Commands.none();
    return Commands.run(
            () -> {
              if (!intake.isDeployed()) {
                intake.setRollerVelocity(Intake.ROLLER_SHOOTING_RPM);
              }
            })
        .finallyDo(
            () -> {
              // Restore intake rollers to normal speed instead of stopping them.
              // This prevents releasing a shooting button from killing the rollers
              // when the intake is deployed and actively collecting.
              if (intake.isDeployed()) {
                intake.runIntake();
              } else {
                intake.stopRollers();
              }
            });
  }
}
