package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootingCommands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.motivator.Motivator;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;

public class ButtonsAndDashboardBindings {

  private static OperatorInterface oi;
  private static Drive drive;
  private static Vision vision;
  private static Intake intake;
  private static Turret turret;
  private static Launcher launcher;
  private static Motivator motivator;
  private static Hood hood;

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
      SmartDashboard.putBoolean("TestSubsystems/Vision/Enable", vision.isVisionOn());
      SmartDashboard.putData(
          "TestSubsystems/Vision/Toggle",
          Commands.runOnce(
                  () -> {
                    vision.toggleVision();
                    SmartDashboard.putBoolean("TestSubsystems/Vision/Enable", vision.isVisionOn());
                  })
              .ignoringDisable(true)
              .withName("Toggle Vision"));
    }

    // Intake controls on dashboard
    if (intake != null) {
      // Deploy/Retract buttons
      SmartDashboard.putData(
          "TestSubsystems/Intake/Deploy",
          Commands.runOnce(intake::deploy, intake).withName("Deploy Intake"));
      SmartDashboard.putData(
          "TestSubsystems/Intake/Retract",
          Commands.runOnce(intake::retract, intake).withName("Retract Intake"));

      // Roller on/off toggle
      SmartDashboard.putData(
          "TestSubsystems/Intake/Run",
          Commands.runOnce(intake::runIntake, intake).withName("Run Intake"));
      SmartDashboard.putData(
          "TestSubsystems/Intake/Stop",
          Commands.runOnce(intake::stopRollers, intake).withName("Stop Intake"));
    }

    // TurretBot-specific testing controls
    if (Constants.currentRobot == Constants.RobotType.TURRETBOT) {
      configureTurretBotTestControls();
    }

    // Coordinated shooting controls (requires both turret and launcher)
    if (turret != null && launcher != null) {
      configureShootingControls();
    }
  }

  /** Configure dashboard controls for coordinated shooting system. */
  private static void configureShootingControls() {
    // Initialize all tunables and status values so they appear on dashboard immediately
    ShootingCommands.initTunables();

    // === Simulation Reset ===
    SmartDashboard.putData("Sim/SimReset", ShootingCommands.resetSimulationCommand(turret));

    // === Auto Launch Command ===
    // Auto-calculated trajectory - works for both sim and physical
    SmartDashboard.putData(
        "Shooting/Auto/AutoLaunch", ShootingCommands.launchCommand(launcher, turret, motivator));

    // === Manual Launch Command ===
    // Manual test mode using Shooting/Test/* dashboard values for controlled testing
    SmartDashboard.putData(
        "Shooting/Test/ManualLaunch",
        ShootingCommands.testLaunchCommand(launcher, turret, motivator, hood));

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

    // === Preset Field Positions ===
    // Blue alliance zone (should aim at blue hub)
    SmartDashboard.putData(
        "Sim/Pose/SimPresetBlue",
        Commands.runOnce(
                () -> {
                  drive.setPose(new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0)));
                  System.out.println("[Sim] Set pose to Blue Alliance Zone");
                },
                drive)
            .ignoringDisable(true)
            .withName("SimPresetBlue"));

    // Red alliance zone (should aim at red hub)
    SmartDashboard.putData(
        "Sim/Pose/SimPresetRed",
        Commands.runOnce(
                () -> {
                  drive.setPose(new Pose2d(14.0, 4.0, Rotation2d.fromDegrees(180)));
                  System.out.println("[Sim] Set pose to Red Alliance Zone");
                },
                drive)
            .ignoringDisable(true)
            .withName("SimPresetRed"));

    // Field center (neutral zone)
    SmartDashboard.putData(
        "Sim/Pose/SimPresetCenter",
        Commands.runOnce(
                () -> {
                  drive.setPose(new Pose2d(8.27, 4.0, Rotation2d.fromDegrees(0)));
                  System.out.println("[Sim] Set pose to Field Center");
                },
                drive)
            .ignoringDisable(true)
            .withName("SimPresetCenter"));

    // === Launcher Test Controls ===
    if (launcher != null) {
      // Run launcher at tunable velocity (hold this button in dashboard)
      SmartDashboard.putData(
          "TestSubsystems/Launcher/Run",
          launcher.runAtTunableVelocityCommand().withName("Run Launcher"));

      // Stop launcher
      SmartDashboard.putData(
          "TestSubsystems/Launcher/Stop", launcher.stopCommand().withName("Stop Launcher"));

      // Quick test velocities
      SmartDashboard.putData(
          "TestSubsystems/Launcher/100RPM",
          launcher.runAtVelocityCommand(100).withName("Launcher 100 RPM"));

      SmartDashboard.putData(
          "TestSubsystems/Launcher/500RPM",
          launcher.runAtVelocityCommand(500).withName("Launcher 500 RPM"));

      SmartDashboard.putData(
          "TestSubsystems/Launcher/1000RPM",
          launcher.runAtVelocityCommand(1000).withName("Launcher 1000 RPM"));

      System.out.println("[TestSubsystems] Launcher test controls configured");
    }

    // === Motivator Test Controls ===
    if (motivator != null) {
      // Stop
      SmartDashboard.putData("TestSubsystems/Motivator/Stop", motivator.stopCommand());

      System.out.println("[TestSubsystems] Motivator test controls configured");
    }

    // Initialize shooting velocity tunables so they appear in dashboard immediately
    // Just accessing them triggers LoggedTunableNumber to register with NetworkTables
    if (launcher != null || motivator != null) {
      ShootingCommands.initTunables();
      System.out.println("[Sim] Shooting velocity tunables initialized");
    }

    System.out.println("[Sim] Test controls configured on SmartDashboard");
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
                            "TestSubsystems/Vision/Enable", vision.isVisionOn());
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

      // Shoot button - unified launch command
      if (launcher != null) {
        oi.getShootButton().whileTrue(ShootingCommands.launchCommand(launcher, turret, motivator));
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
