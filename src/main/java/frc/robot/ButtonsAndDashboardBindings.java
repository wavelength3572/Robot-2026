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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.motivator.Motivator;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FuelSim;

public class ButtonsAndDashboardBindings {

  private static OperatorInterface oi;
  private static Drive drive;
  private static Vision vision;
  private static Intake intake;
  private static Turret turret;
  private static Launcher launcher;
  private static Motivator motivator;

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
      Motivator motivator) {
    ButtonsAndDashboardBindings.oi = operatorInterface;
    ButtonsAndDashboardBindings.drive = drive;
    ButtonsAndDashboardBindings.vision = vision;
    ButtonsAndDashboardBindings.intake = intake;
    ButtonsAndDashboardBindings.turret = turret;
    ButtonsAndDashboardBindings.launcher = launcher;
    ButtonsAndDashboardBindings.motivator = motivator;

    configureDriverButtonBindings();
    configureOperatorButtonBindings();
    configureDashboardBindings();
  }

  // Legacy method without vision/intake/turret/launcher/motivator parameters
  public static void configureBindings(OperatorInterface operatorInterface, Drive drive) {
    configureBindings(operatorInterface, drive, null, null, null, null, null);
  }

  /****************************** */
  /*** DASHBOARD BINDINGS ****** */
  /****************************** */

  private static void configureDashboardBindings() {
    // Vision toggle on dashboard
    if (vision != null) {
      // Vision ON by default in simulation (use SmartDashboard toggle to disable if needed)
      SmartDashboard.putBoolean("Vision/Enable", vision.isVisionOn());
      SmartDashboard.putData(
          "Vision/Toggle",
          Commands.runOnce(
                  () -> {
                    vision.toggleVision();
                    SmartDashboard.putBoolean("Vision/Enable", vision.isVisionOn());
                  })
              .ignoringDisable(true)
              .withName("Toggle Vision"));
    }

    // Intake controls on dashboard
    if (intake != null) {
      // Deploy/Retract buttons
      SmartDashboard.putData(
          "Intake/Deploy", Commands.runOnce(intake::deploy, intake).withName("Deploy Intake"));
      SmartDashboard.putData(
          "Intake/Retract", Commands.runOnce(intake::retract, intake).withName("Retract Intake"));

      // Roller on/off toggle
      SmartDashboard.putData(
          "Intake/Run", Commands.runOnce(intake::runIntake, intake).withName("Run Intake"));
      SmartDashboard.putData(
          "Intake/Stop", Commands.runOnce(intake::stopRollers, intake).withName("Stop Intake"));
    }

    // Turret controls on dashboard
    if (turret != null) {
      // Calculate shot to hub (enables trajectory visualization)
      SmartDashboard.putData(
          "Turret/CalculateShot",
          Commands.runOnce(
                  () -> {
                    boolean isBlue =
                        DriverStation.getAlliance()
                            .orElse(DriverStation.Alliance.Blue)
                            .equals(DriverStation.Alliance.Blue);
                    turret.calculateShotToHub(isBlue);
                  },
                  turret)
              .ignoringDisable(true)
              .withName("Calculate Shot"));

      // Launch fuel
      SmartDashboard.putData(
          "Turret/LaunchFuel",
          Commands.runOnce(turret::launchFuel, turret).withName("Launch Fuel"));
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
    // === Shooting Commands ===
    // Start coordinated shooting (Phase 1: shoot until empty)
    SmartDashboard.putData(
        "Shooting/StartShooting",
        ShootingCommands.shootUntilEmpty(launcher, turret, ShootingCommands.getHubActiveSupplier())
            .withName("Start Shooting"));

    // Continuous shooting (Phase 2: keeps shooting as fuel is picked up)
    SmartDashboard.putData(
        "Shooting/ContinuousShooting",
        ShootingCommands.continuousShooting(
                launcher, turret, ShootingCommands.getHubActiveSupplier())
            .withName("Continuous Shooting"));

    // Shoot while moving (Phase 3: driver controls robot, auto-aims and shoots)
    SmartDashboard.putData(
        "Shooting/ShootWhileMoving",
        ShootingCommands.shootWhileMoving(launcher, turret, ShootingCommands.getHubActiveSupplier())
            .withName("Shoot While Moving"));

    // === Hub Override Controls ===
    SmartDashboard.putData("Shooting/HubOverride_ON", ShootingCommands.setHubOverrideOn());
    SmartDashboard.putData("Shooting/HubOverride_OFF", ShootingCommands.setHubOverrideOff());
    SmartDashboard.putData("Shooting/HubOverride_AUTO", ShootingCommands.setHubOverrideAuto());

    // === Match Phase Controls ===
    SmartDashboard.putData("Shooting/ToggleWeWonAuto", ShootingCommands.toggleWeWonAuto());

    // === Fuel Management ===
    SmartDashboard.putData("Shooting/ResetFuelTo25", ShootingCommands.resetFuelCommand(turret));

    System.out.println("[Shooting] Coordinated shooting controls configured on SmartDashboard");

    // FuelSim controls (simulation only)
    if (Constants.currentMode == Constants.Mode.SIM) {
      SmartDashboard.putData(
          "FuelSim/Reset100",
          Commands.runOnce(
                  () -> {
                    FuelSim.getInstance().resetWithFuel(100);
                    // Also reset turret's fuel inventory to 100
                    if (turret != null && turret.getVisualizer() != null) {
                      turret.getVisualizer().setFuelCount(100);
                    }
                  })
              .ignoringDisable(true)
              .withName("Reset 100 Fuel"));
    }
  }

  /** Configure dashboard controls for testing TurretBot without a drive system. */
  private static void configureTurretBotTestControls() {
    // === Manual Pose Setting ===
    // Set default test position values
    SmartDashboard.putNumber("TurretBot/TestPoseX", 2.0);
    SmartDashboard.putNumber("TurretBot/TestPoseY", 4.0);
    SmartDashboard.putNumber("TurretBot/TestPoseRotation", 0.0);

    // Button to apply the test pose
    SmartDashboard.putData(
        "TurretBot/SetPose",
        Commands.runOnce(
                () -> {
                  double x = SmartDashboard.getNumber("TurretBot/TestPoseX", 0);
                  double y = SmartDashboard.getNumber("TurretBot/TestPoseY", 0);
                  double rotDeg = SmartDashboard.getNumber("TurretBot/TestPoseRotation", 0);
                  drive.setPose(new Pose2d(x, y, Rotation2d.fromDegrees(rotDeg)));
                  System.out.println(
                      "[TurretBot] Set pose to X="
                          + x
                          + ", Y="
                          + y
                          + ", Rotation="
                          + rotDeg
                          + " deg");
                },
                drive)
            .ignoringDisable(true)
            .withName("Set Test Pose"));

    // === Preset Field Positions ===
    // Blue alliance zone (should aim at blue hub)
    SmartDashboard.putData(
        "TurretBot/Preset_BlueZone",
        Commands.runOnce(
                () -> {
                  drive.setPose(new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0)));
                  System.out.println("[TurretBot] Set pose to Blue Alliance Zone");
                },
                drive)
            .ignoringDisable(true)
            .withName("Blue Zone Preset"));

    // Red alliance zone (should aim at red hub)
    SmartDashboard.putData(
        "TurretBot/Preset_RedZone",
        Commands.runOnce(
                () -> {
                  drive.setPose(new Pose2d(14.0, 4.0, Rotation2d.fromDegrees(180)));
                  System.out.println("[TurretBot] Set pose to Red Alliance Zone");
                },
                drive)
            .ignoringDisable(true)
            .withName("Red Zone Preset"));

    // Field center (neutral zone)
    SmartDashboard.putData(
        "TurretBot/Preset_Center",
        Commands.runOnce(
                () -> {
                  drive.setPose(new Pose2d(8.27, 4.0, Rotation2d.fromDegrees(0)));
                  System.out.println("[TurretBot] Set pose to Field Center");
                },
                drive)
            .ignoringDisable(true)
            .withName("Center Preset"));

    // === Launcher Test Controls ===
    if (launcher != null) {
      // Run launcher at tunable velocity (hold this button in dashboard)
      SmartDashboard.putData(
          "TurretBot/Launcher_Run",
          launcher.runAtTunableVelocityCommand().withName("Run Launcher"));

      // Stop launcher
      SmartDashboard.putData(
          "TurretBot/Launcher_Stop", launcher.stopCommand().withName("Stop Launcher"));

      // Quick test velocities
      SmartDashboard.putData(
          "TurretBot/Launcher_100RPM",
          launcher.runAtVelocityCommand(100).withName("Launcher 100 RPM"));

      SmartDashboard.putData(
          "TurretBot/Launcher_500RPM",
          launcher.runAtVelocityCommand(500).withName("Launcher 500 RPM"));

      SmartDashboard.putData(
          "TurretBot/Launcher_1000RPM",
          launcher.runAtVelocityCommand(1000).withName("Launcher 1000 RPM"));

      System.out.println("[TurretBot] Launcher test controls configured");
    }

    // === Motivator Test Controls ===
    if (motivator != null) {
      // Feed (main motivator)
      SmartDashboard.putData("TurretBot/Motivator_Feed", motivator.feedCommand());

      // Prefeed (staging roller)
      SmartDashboard.putData("TurretBot/Motivator_Prefeed", motivator.prefeedCommand());

      // Feed + Prefeed together
      SmartDashboard.putData(
          "TurretBot/Motivator_FeedAndPrefeed", motivator.feedAndPrefeedCommand());

      // Eject (reverse)
      SmartDashboard.putData("TurretBot/Motivator_Eject", motivator.ejectCommand());

      // Stop
      SmartDashboard.putData("TurretBot/Motivator_Stop", motivator.stopCommand());

      System.out.println("[TurretBot] Motivator test controls configured");
    }

    // Initialize shooting velocity tunables so they appear in dashboard immediately
    // Just accessing them triggers LoggedTunableNumber to register with NetworkTables
    if (launcher != null || motivator != null) {
      ShootingCommands.initTunables();
      System.out.println("[TurretBot] Shooting velocity tunables initialized");
    }

    System.out.println("[TurretBot] Test controls configured on SmartDashboard under 'TurretBot/'");
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
                        SmartDashboard.putBoolean("Vision/Enable", vision.isVisionOn());
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

      // Shoot button binding depends on whether we have coordinated shooting
      if (launcher != null) {
        // Coordinated shooting: launcher spins up, turret aims, shoots when ready
        oi.getShootButton()
            .whileTrue(
                ShootingCommands.shootWhileMoving(
                    launcher, turret, ShootingCommands.getHubActiveSupplier()));
      } else {
        // Simple shooting: just launch fuel (no launcher coordination)
        oi.getShootButton().whileTrue(turret.repeatedlyLaunchFuelCommand());
      }
    }

    // Launcher controls (TurretBot only)
    if (launcher != null && motivator != null) {
      // Button 24: Full launch sequence - launcher + motivator at target RPMs
      oi.getLauncherButton().whileTrue(ShootingCommands.launchWithMotivator(launcher, motivator));
    } else if (launcher != null) {
      // Fallback: launcher only (no motivator available)
      oi.getLauncherButton().whileTrue(launcher.runAtTunableVelocityCommand());
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

    // Shooting controls
    if (launcher != null && motivator != null) {
      // Button 2: Full physical shooting sequence (launcher + motivator)
      // Spins up launcher, waits for speed, then feeds with motivator
      oi.getButtonBox1Button2()
          .whileTrue(ShootingCommands.launchWithMotivator(launcher, motivator));
    } else if (turret != null && launcher != null) {
      // Fallback: simulation shooting (no physical motivator)
      oi.getButtonBox1Button2()
          .whileTrue(
              ShootingCommands.shootWhileMoving(
                  launcher, turret, ShootingCommands.getHubActiveSupplier()));
    } else if (turret != null) {
      // Fallback if no launcher: just launch fuel visually
      oi.getButtonBox1Button2()
          .whileTrue(
              Commands.runOnce(
                      () -> {
                        boolean isBlue =
                            DriverStation.getAlliance()
                                .orElse(DriverStation.Alliance.Blue)
                                .equals(DriverStation.Alliance.Blue);
                        turret.calculateShotToHub(isBlue);
                      },
                      turret)
                  .andThen(turret.repeatedlyLaunchFuelCommand()));
    }
  }
}
