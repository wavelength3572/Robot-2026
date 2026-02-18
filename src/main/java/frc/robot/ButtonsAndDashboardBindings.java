package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootingCommands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
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
          "BenchTest/IntakeVelocityControl/RollerRPM", IntakeConstants.ROLLER_INTAKE_RPM);

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
      SmartDashboard.putBoolean("BenchTest/Subsystems/Vision/Enable", vision.isVisionOn());
      SmartDashboard.putData(
          "BenchTest/Subsystems/Vision/Toggle",
          Commands.runOnce(
                  () -> {
                    vision.toggleVision();
                    SmartDashboard.putBoolean(
                        "BenchTest/Subsystems/Vision/Enable", vision.isVisionOn());
                  })
              .ignoringDisable(true)
              .withName("Toggle Vision"));
    }

    // Intake bench test area
    if (intake != null) {
      configureIntakeBenchTest();
    }

    // TurretBot-specific testing controls
    if (Constants.currentRobot == Constants.RobotType.TURRETBOT) {
      configureTurretBotTestControls();
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
    // Initialize all tunables and status values so they appear on dashboard immediately
    ShootingCommands.initTunables();

    // === Auto Launch Command ===
    // Auto-calculated trajectory - works for both sim and physical
    SmartDashboard.putData(
        "Match/SmartLaunch",
        ShootingCommands.launchCommand(launcher, shootingCoordinator, motivator));

    // === Manual Launch Command ===
    // Manual test mode using BenchTest/Shooting/* dashboard values for controlled testing
    SmartDashboard.putData(
        "BenchTest/Shooting/Launch",
        ShootingCommands.testLaunchCommand(launcher, shootingCoordinator, motivator, hood));

    // === Bench Test Controls ===
    // Stripped-down launch for bench testing (no turret/hood wait)
    SmartDashboard.putData(
        "BenchTest/Shooting/LauncherOnly",
        ShootingCommands.benchTestLaunchCommand(launcher, shootingCoordinator, motivator));

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

  /** Configure dashboard controls for testing TurretBot without a drive system. */
  private static void configureTurretBotTestControls() {
    // === Manual Pose Setting ===
    // Set default test position values
    SmartDashboard.putNumber("Sim/Pose/SimPoseX", 2.0);
    SmartDashboard.putNumber("Sim/Pose/SimPoseY", 4.0);
    SmartDashboard.putNumber("Sim/Pose/SimPoseRotation", 0.0);

    // Button to apply the test pose
    SmartDashboard.putData(
        "Sim/Pose/SimSetPose",
        Commands.runOnce(
                () -> {
                  double x = SmartDashboard.getNumber("Sim/Pose/SimPoseX", 0);
                  double y = SmartDashboard.getNumber("Sim/Pose/SimPoseY", 0);
                  double rotDeg = SmartDashboard.getNumber("Sim/Pose/SimPoseRotation", 0);
                  drive.setPose(new Pose2d(x, y, Rotation2d.fromDegrees(rotDeg)));
                  System.out.println(
                      "[Sim] Set pose to X=" + x + ", Y=" + y + ", Rotation=" + rotDeg + " deg");
                },
                drive)
            .ignoringDisable(true)
            .withName("SimSetPose"));

    // === Preset Field Positions (from PathPlanner auto start poses) ===
    for (String[] preset : PRESET_POSITIONS) {
      String label = preset[0];
      double x = Double.parseDouble(preset[1]);
      double y = Double.parseDouble(preset[2]);
      double rot = Double.parseDouble(preset[3]);
      SmartDashboard.putData(
          "Sim/Pose/Preset" + label,
          Commands.runOnce(
                  () -> {
                    drive.setPose(new Pose2d(x, y, Rotation2d.fromDegrees(rot)));
                  },
                  drive)
              .ignoringDisable(true)
              .withName("Preset " + label));
    }

    // === Key Field Location Poses ===
    // Robot faces 180° (intake toward alliance wall) — turret aims independently.
    // Offset = half bumper length + intake extension past bumpers (from PathPlanner features).
    double halfBumperLength = Constants.getRobotConfig().getBumperLength() / 2.0;
    double intakeExtensionPastBumper = 0.21; // meters (~8.3in)
    double robotCenterToIntakeTip = halfBumperLength + intakeExtensionPastBumper;

    // Outpost: robot centered on human player drop path, intake reaching the wall
    SmartDashboard.putData(
        "Sim/Pose/Outpost",
        Commands.runOnce(
                () -> {
                  Translation2d outpost = FieldConstants.Outpost.centerPoint;
                  drive.setPose(
                      new Pose2d(
                          robotCenterToIntakeTip, outpost.getY(), Rotation2d.fromDegrees(180.0)));
                },
                drive)
            .ignoringDisable(true)
            .withName("Pose: Outpost"));

    // Depot: robot positioned with intake reaching the depot
    SmartDashboard.putData(
        "Sim/Pose/Depot",
        Commands.runOnce(
                () -> {
                  Translation3d depot = FieldConstants.Depot.depotCenter;
                  drive.setPose(
                      new Pose2d(
                          depot.getX() + robotCenterToIntakeTip,
                          depot.getY(),
                          Rotation2d.fromDegrees(180.0)));
                },
                drive)
            .ignoringDisable(true)
            .withName("Pose: Depot"));

    // Initialize shooting velocity tunables so they appear in dashboard immediately
    // Just accessing them triggers LoggedTunableNumber to register with NetworkTables
    if (launcher != null || motivator != null) {
      ShootingCommands.initTunables();
      System.out.println("[Sim] Shooting velocity tunables initialized");
    }

    System.out.println("[Sim] Test controls configured on SmartDashboard");
  }

  /** Configure per-subsystem tuning run buttons. Always called for all robot types. */
  private static void configureTuningControls() {
    // Launcher: single Run button, reads BenchTest/Shooting/LauncherRPM
    if (launcher != null) {
      SmartDashboard.putData(
          "Tuning/Launcher/Run",
          launcher.runAtTunableVelocityCommand(ShootingCommands.getTestLauncherRPM()));
    }

    // Motivator:
    if (motivator != null) {
      SmartDashboard.putData(
          "Tuning/Motivator/RunLeadMotivator",
          motivator.runMotivatorCommand(ShootingCommands.getTestMotivatorRPM()));
    }
    // Spindexer:
    if (spindexer != null) {
      SmartDashboard.putData(
          "Tuning/Spindexer/RunSpindexer",
          spindexer.runSpindexerCommand(ShootingCommands.getTestSpindexerRPM()));
    }

    // Turret: direct angle command, reads BenchTest/Shooting/AngleDegTurret
    if (turret != null) {
      ShootingCommands.initTunables();
      SmartDashboard.putData(
          "BenchTest/Turret/SetAngle",
          Commands.run(
                  () -> turret.setTurretAngle(ShootingCommands.getTestTurretAngleDeg().get()),
                  turret)
              .withName("Turret SetAngle"));
    }

    // Intake: PID tunables are under Tuning/Intake/Roller_*, run buttons are in BenchTest

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

    // Open-loop power control (hold to run, release to stop)
    // Run button reads the Power slider live — adjust slider while running to change speed
    SmartDashboard.putData(
        "BenchTest/IntakePowerControl/Run",
        Commands.run(() -> intake.setRollerSpeed(testIntakeSpeed.get()), intake)
            .finallyDo(intake::stopRollers)
            .withName("Run at Power"));
    SmartDashboard.putData(
        "BenchTest/IntakePowerControl/Run80%",
        Commands.run(() -> intake.setRollerSpeed(IntakeConstants.ROLLER_INTAKE_SPEED), intake)
            .finallyDo(intake::stopRollers)
            .withName("Run 80% Power"));
    SmartDashboard.putData(
        "BenchTest/IntakePowerControl/Reverse60%",
        Commands.run(() -> intake.setRollerSpeed(IntakeConstants.ROLLER_EJECT_SPEED), intake)
            .finallyDo(intake::stopRollers)
            .withName("Reverse 60%"));
    SmartDashboard.putData(
        "BenchTest/IntakePowerControl/HoldSlow10%",
        Commands.run(() -> intake.setRollerSpeed(IntakeConstants.ROLLER_HOLD_SPEED), intake)
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
    SmartDashboard.putNumber("TrajectoryCalculators/WhatIf/HoodAngleDeg", 45.0);

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

    // === Distance: move robot to a distance from hub, read back the optimized shot ===
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

    // Read back the coordinator's optimized shot (press after SetPose to populate RPM/angle)
    SmartDashboard.putData(
        "TrajectoryCalculators/Distance/ReadShot",
        Commands.runOnce(
                () -> {
                  ShotCalculator.ShotResult shot = shootingCoordinator.getCurrentShot();
                  if (shot == null) return;
                  double rpm = ShotCalculator.calculateRPMForVelocity(shot.exitVelocityMps());
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
    // For TurretBot, this enables "virtual driving" to update odometry and test turret auto-aiming
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, oi::getTranslateX, oi::getTranslateY, oi::getRotate));

    // Gyro Reset (available on all robots with a gyro)
    oi.getResetGyroButton()
        .onTrue(Commands.runOnce(drive::zeroGyroscope, drive).ignoringDisable(true));

    // Vision toggle on Button H
    if (vision != null) {
      oi.getButtonH()
          .onTrue(
              Commands.runOnce(
                      () -> {
                        vision.toggleVision();
                        SmartDashboard.putBoolean(
                            "BenchTest/Subsystems/Vision/Enable", vision.isVisionOn());
                      })
                  .ignoringDisable(true));
    }

    // Turret/shooting controls (only if coordinator exists)
    if (shootingCoordinator != null) {
      // Button V: Calculate shot to hub (enables trajectory visualization)
      oi.getButtonV()
          .onTrue(
              Commands.runOnce(
                      () -> {
                        boolean isBlue =
                            DriverStation.getAlliance()
                                .orElse(DriverStation.Alliance.Blue)
                                .equals(DriverStation.Alliance.Blue);
                        shootingCoordinator.calculateShotToHub(isBlue);
                      },
                      shootingCoordinator)
                  .ignoringDisable(true));

      // Shoot button - bench test launch for TURRETBOT, normal launch for others
      if (launcher != null) {
        if (Constants.currentRobot == Constants.RobotType.TURRETBOT) {
          oi.getShootButton()
              .whileTrue(
                  ShootingCommands.benchTestLaunchCommand(
                      launcher, shootingCoordinator, motivator));
        } else {
          oi.getShootButton()
              .whileTrue(ShootingCommands.launchCommand(launcher, shootingCoordinator, motivator));
        }
      } else {
        // No launcher - just launch fuel visually
        oi.getShootButton().whileTrue(shootingCoordinator.repeatedlyLaunchFuelCommand());
      }
    }

    // Launcher button - same as shoot button (unified launch command)
    if (launcher != null && shootingCoordinator != null) {
      oi.getLauncherButton()
          .whileTrue(ShootingCommands.launchCommand(launcher, shootingCoordinator, motivator));
    }
  }

  /****************************** */
  /*** BUTTON BOX BINDINGS ****** */
  /****************************** */

  private static void configureOperatorButtonBindings() {
    // Intake controls
    if (intake != null) {
      // Button 4: Deploy intake
      oi.getButtonBox1Button4().onTrue(Commands.runOnce(intake::deploy, intake));

      // Button 3: Retract intake
      oi.getButtonBox1Button3().onTrue(Commands.runOnce(intake::retract, intake));

      // Button 6: Toggle rollers on/off
      oi.getButtonBox1Button6()
          .onTrue(
              Commands.either(
                  Commands.runOnce(intake::stopRollers, intake),
                  Commands.runOnce(intake::runIntake, intake),
                  () -> intake.getRollerVelocityRPM() > 10));
    }

    // Shooting controls - unified launch command
    if (shootingCoordinator != null && launcher != null) {
      oi.getButtonBox1Button2()
          .whileTrue(ShootingCommands.launchCommand(launcher, shootingCoordinator, motivator));
    } else if (shootingCoordinator != null) {
      // Fallback if no launcher: just launch fuel visually
      oi.getButtonBox1Button2().whileTrue(shootingCoordinator.repeatedlyLaunchFuelCommand());
    }
  }
}
