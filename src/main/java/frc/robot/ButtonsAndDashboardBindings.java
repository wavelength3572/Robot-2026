package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootingCommands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.TrajectoryOptimizer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.motivator.Motivator;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.BenchTestMetrics;
import frc.robot.util.LoggedTunableNumber;

public class ButtonsAndDashboardBindings {

  private static OperatorInterface oi;
  private static Drive drive;
  private static Vision vision;
  private static Intake intake;
  private static Turret turret;
  private static Launcher launcher;
  private static Motivator motivator;
  private static Hood hood;

  // Intake bench test tunable (duty cycle 0-1)
  private static final LoggedTunableNumber testIntakeSpeed =
      new LoggedTunableNumber("BenchTest/Intake/RollerSpeed", 0.8);

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
      Hood hood) {
    ButtonsAndDashboardBindings.oi = operatorInterface;
    ButtonsAndDashboardBindings.drive = drive;
    ButtonsAndDashboardBindings.vision = vision;
    ButtonsAndDashboardBindings.intake = intake;
    ButtonsAndDashboardBindings.turret = turret;
    ButtonsAndDashboardBindings.launcher = launcher;
    ButtonsAndDashboardBindings.motivator = motivator;
    ButtonsAndDashboardBindings.hood = hood;

    configureDriverButtonBindings();
    configureOperatorButtonBindings();
    configureDashboardBindings();
  }

  // Legacy method without vision/intake/turret/launcher/motivator/hood parameters
  public static void configureBindings(OperatorInterface operatorInterface, Drive drive) {
    configureBindings(operatorInterface, drive, null, null, null, null, null, null);
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

    // Coordinated shooting controls (requires both turret and launcher)
    if (turret != null && launcher != null) {
      configureShootingControls();
    }

    // Per-subsystem tuning run buttons (always, for all robot types)
    configureTuningControls();

    // Distance-to-shot calculator
    if (turret != null) {
      configureShotCalculator();
    }
  }

  /** Configure dashboard controls for coordinated shooting system. */
  private static void configureShootingControls() {
    // Initialize all tunables and status values so they appear on dashboard immediately
    ShootingCommands.initTunables();

    // === Simulation Reset ===
    SmartDashboard.putData("Sim/FuelReset", ShootingCommands.resetSimulationCommand(turret));
    SmartDashboard.putData(
        "Sim/ResetStartOfMatch", ShootingCommands.resetStartingFieldCommand(turret));

    // === Auto Launch Command ===
    // Auto-calculated trajectory - works for both sim and physical
    SmartDashboard.putData(
        "Match/SmartLaunch", ShootingCommands.launchCommand(launcher, turret, motivator));

    // === Manual Launch Command ===
    // Manual test mode using BenchTest/Shooting/* dashboard values for controlled testing
    SmartDashboard.putData(
        "BenchTest/Shooting/Launch",
        ShootingCommands.testLaunchCommand(launcher, turret, motivator, hood));

    // === Bench Test Controls ===
    // Stripped-down launch for bench testing (no turret/hood wait)
    SmartDashboard.putData(
        "BenchTest/Shooting/LauncherOnly",
        ShootingCommands.benchTestLaunchCommand(launcher, turret, motivator));

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

    // Motivator: 3 isolation buttons
    if (motivator != null) {
      SmartDashboard.putData(
          "Tuning/Motivator/RunLeadMotivator",
          motivator.runLeadMotivatorCommand(ShootingCommands.getTestMotivatorRPM()));
      SmartDashboard.putData(
          "Tuning/Motivator/RunBothMotivators",
          motivator.runBothMotivatorsCommand(ShootingCommands.getTestMotivatorRPM()));
      SmartDashboard.putData(
          "Tuning/Motivator/RunPrefeed",
          motivator.runPrefeedCommand(ShootingCommands.getTestPrefeedRPM()));
    }

    // Intake: Run rollers only (no deploy — deploy doesn't use PID)
    if (intake != null) {
      SmartDashboard.putData(
          "Tuning/Intake/Run",
          Commands.run(() -> intake.setRollerSpeed(testIntakeSpeed.get()), intake)
              .finallyDo(intake::stopRollers)
              .withName("Intake: Run Rollers"));
    }
  }

  /** Configure BenchTest/Intake area with deploy/retract and coordinated deploy+run. */
  private static void configureIntakeBenchTest() {
    // Deploy/Retract — simple operational buttons
    SmartDashboard.putData(
        "BenchTest/Intake/Deploy",
        Commands.runOnce(intake::deploy, intake).withName("Deploy Intake"));
    SmartDashboard.putData(
        "BenchTest/Intake/Retract",
        Commands.runOnce(intake::retract, intake).withName("Retract Intake"));

    // DeployAndRun: click to deploy + run, click again (cancel) to retract + stop
    SmartDashboard.putData(
        "BenchTest/Intake/DeployAndRun", intake.deployAndRunCommand(testIntakeSpeed::get));
  }

  /** Configure distance-to-shot calculator on dashboard. */
  private static void configureShotCalculator() {
    SmartDashboard.putNumber("BenchTest/Calculator/DistanceInches", 118.0);

    SmartDashboard.putData(
        "BenchTest/Calculator/Calculate",
        Commands.runOnce(ButtonsAndDashboardBindings::runShotCalculation)
            .ignoringDisable(true)
            .withName("Calculate Shot"));

    // CalcFromPose: read robot pose, compute distance to hub, then calculate
    SmartDashboard.putData(
        "BenchTest/Calculator/CalcFromPose",
        Commands.runOnce(
                () -> {
                  Pose2d robotPose = drive.getPose();
                  Translation3d hubTarget = FieldConstants.Hub.innerCenterPoint;
                  double distMeters =
                      robotPose.getTranslation().getDistance(hubTarget.toTranslation2d());
                  double distInches = distMeters / 0.0254;
                  SmartDashboard.putNumber(
                      "BenchTest/Calculator/DistanceInches",
                      Math.round(distInches * 100.0) / 100.0);
                  runShotCalculation();
                })
            .ignoringDisable(true)
            .withName("Calc From Pose"));

    // Calculate all presets button
    SmartDashboard.putData(
        "BenchTest/Calculator/CalculateAll",
        Commands.runOnce(ButtonsAndDashboardBindings::runAllPositionsCalculation)
            .ignoringDisable(true)
            .withName("Calculate All"));

    // Run once at init so output values appear on dashboard immediately
    runShotCalculation();
    runAllPositionsCalculation();
  }

  /** Runs the shot calculator and publishes results to SmartDashboard. */
  private static void runShotCalculation() {
    double distanceInches = SmartDashboard.getNumber("BenchTest/Calculator/DistanceInches", 118.0);
    double distanceMeters = distanceInches * 0.0254;
    double turretHeight = turret.getTurretHeightMeters();
    Translation3d turretPos = new Translation3d(0, 0, turretHeight);
    Translation3d hubTarget = new Translation3d(distanceMeters, 0, 1.43);
    TrajectoryOptimizer.OptimalShot shot =
        TrajectoryOptimizer.calculateOptimalShot(turretPos, hubTarget);
    SmartDashboard.putNumber(
        "BenchTest/Calculator/CalcLauncherRPM", Math.round(shot.rpm * 100.0) / 100.0);
    SmartDashboard.putNumber(
        "BenchTest/Calculator/CalcHoodAngleDeg", Math.round(shot.launchAngleDeg * 100.0) / 100.0);
    SmartDashboard.putNumber(
        "BenchTest/Calculator/CalcExitVelocityMps",
        Math.round(shot.exitVelocityMps * 100.0) / 100.0);
    SmartDashboard.putNumber(
        "BenchTest/Calculator/CalcExitVelocityFps",
        Math.round(shot.exitVelocityMps * 3.28084 * 100.0) / 100.0);
    SmartDashboard.putBoolean("BenchTest/Calculator/Achievable", shot.achievable);
    SmartDashboard.putString("BenchTest/Calculator/Notes", shot.notes);
  }

  /** Calculates all preset positions and publishes a pipe-separated summary. */
  private static void runAllPositionsCalculation() {
    double turretHeight = turret.getTurretHeightMeters();
    Translation3d hubTarget3d = FieldConstants.Hub.innerCenterPoint;
    StringBuilder sb = new StringBuilder();
    for (String[] preset : PRESET_POSITIONS) {
      String label = preset[0];
      double x = Double.parseDouble(preset[1]);
      double y = Double.parseDouble(preset[2]);
      // Compute horizontal distance from this position to the hub
      double distMeters =
          new edu.wpi.first.math.geometry.Translation2d(x, y)
              .getDistance(hubTarget3d.toTranslation2d());
      double distInches = distMeters / 0.0254;
      // Use simplified 1D model: turret at origin with height, hub at distance with hub height
      Translation3d turretPos = new Translation3d(0, 0, turretHeight);
      Translation3d target = new Translation3d(distMeters, 0, hubTarget3d.getZ());
      TrajectoryOptimizer.OptimalShot shot =
          TrajectoryOptimizer.calculateOptimalShot(turretPos, target);
      double rpm = Math.round(shot.rpm * 100.0) / 100.0;
      double angleDeg = Math.round(shot.launchAngleDeg * 100.0) / 100.0;
      double fps = Math.round(shot.exitVelocityMps * 3.28084 * 100.0) / 100.0;
      if (sb.length() > 0) {
        sb.append(" | ");
      }
      sb.append(label)
          .append(" ")
          .append((int) Math.round(distInches))
          .append("in: ")
          .append(rpm)
          .append("rpm ")
          .append(angleDeg)
          .append("deg ")
          .append(fps)
          .append("fps");
    }
    SmartDashboard.putString("BenchTest/Calculator/AllPositions", sb.toString());
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

    // Turret controls (only in simulation or if turret exists)
    if (turret != null) {
      // Button V: Calculate shot to hub (enables trajectory visualization)
      oi.getButtonV()
          .onTrue(
              Commands.runOnce(
                      () -> {
                        boolean isBlue =
                            DriverStation.getAlliance()
                                .orElse(DriverStation.Alliance.Blue)
                                .equals(DriverStation.Alliance.Blue);
                        turret.calculateShotToHub(isBlue);
                      },
                      turret)
                  .ignoringDisable(true));

      // Shoot button - bench test launch for TURRETBOT, normal launch for others
      if (launcher != null) {
        if (Constants.currentRobot == Constants.RobotType.TURRETBOT) {
          oi.getShootButton()
              .whileTrue(ShootingCommands.benchTestLaunchCommand(launcher, turret, motivator));
        } else {
          oi.getShootButton()
              .whileTrue(ShootingCommands.launchCommand(launcher, turret, motivator));
        }
      } else {
        // No launcher - just launch fuel visually
        oi.getShootButton().whileTrue(turret.repeatedlyLaunchFuelCommand());
      }
    }

    // Launcher button - same as shoot button (unified launch command)
    if (launcher != null) {
      oi.getLauncherButton().whileTrue(ShootingCommands.launchCommand(launcher, turret, motivator));
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
    if (turret != null && launcher != null) {
      oi.getButtonBox1Button2()
          .whileTrue(ShootingCommands.launchCommand(launcher, turret, motivator));
    } else if (turret != null) {
      // Fallback if no launcher: just launch fuel visually
      oi.getButtonBox1Button2().whileTrue(turret.repeatedlyLaunchFuelCommand());
    }
  }
}
