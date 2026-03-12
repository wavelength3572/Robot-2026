// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.revrobotics.util.StatusLogger;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.FuelSim;
import frc.robot.util.HubShiftUtil;
import java.lang.reflect.Field;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final double STARTUP_SUPPRESSION_SECONDS = 10.0;

  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private final Timer startupTimer = new Timer();
  private Watchdog watchdog;

  public Robot() {
    super(0.02);

    // Suppress loop overrun warnings during startup (re-enabled after STARTUP_SUPPRESSION_SECONDS)
    try {
      Field f = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      f.setAccessible(true);
      watchdog = (Watchdog) f.get(this);
      watchdog.setTimeout(Double.MAX_VALUE);
      System.out.println("[Robot] Suppressed loop overrun warnings for startup");
    } catch (ReflectiveOperationException e) {
      watchdog = null;
    }
    startupTimer.start();
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });
    Logger.recordMetadata("RobotType", Constants.currentRobot.toString());

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Initialize URCL
    Logger.registerURCL(URCL.startExternal());
    StatusLogger.disableAutoLogging(); // Disable REVLib's built-in logging

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    // Suppress CommandScheduler's own loop overrun warnings during startup
    CommandScheduler.getInstance().setPeriod(Double.MAX_VALUE);

    // Print all buffered startup messages together
    frc.robot.util.StartupLogger.flush();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Re-enable loop overrun warnings after startup settles
    if (watchdog != null && startupTimer.hasElapsed(STARTUP_SUPPRESSION_SECONDS)) {
      watchdog.setTimeout(0.02); // Restore IterativeRobotBase 20ms overrun detection
      CommandScheduler.getInstance().setPeriod(0.02); // Restore CommandScheduler overrun detection
      startupTimer.stop();
      watchdog = null; // Skip this check on future loops
      System.out.println("[Robot] Loop overrun warnings re-enabled");
    }

    // Refresh cached values before subsystem periodic methods run
    frc.robot.util.RobotStatus.refreshAlliance();
    frc.robot.util.LoggedTunableNumber.refreshAll();

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Update fuel simulation (only runs in SIM mode)
    robotContainer.updateFuelSim();

    // Log hub shift info
    HubShiftUtil.ShiftInfo shiftInfo = HubShiftUtil.getOfficialShiftInfo();
    Logger.recordOutput("HubShift/CurrentShift", shiftInfo.currentShift().toString());
    Logger.recordOutput("HubShift/Active", shiftInfo.active());
    Logger.recordOutput("HubShift/ElapsedTime", shiftInfo.elapsedTime());
    Logger.recordOutput("HubShift/RemainingTime", shiftInfo.remainingTime());

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    robotContainer.updateOI(); // This only matters when the joysticks change
    robotContainer.updateAutoChooserForMode(); // Rebuild chooser when Competition Mode changes
    robotContainer.updateSimulationPoseFromAuto(); // Update sim pose when auto selection changes
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Reset scores and shot counters at the start of each autonomous period
    HubShiftUtil.initialize();

    FuelSim.Hub.BLUE_HUB.resetScore();
    FuelSim.Hub.RED_HUB.resetScore();
    if (robotContainer.getShootingCoordinator() != null) {
      robotContainer.getShootingCoordinator().resetShotCounts();
    }

    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    HubShiftUtil.initialize(); // Start the match phase tracker at the beginning of teleop

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    // Disable auto-shoot when entering teleop (safety: prevent autonomous firing)
    if (robotContainer.getShootingCoordinator() != null) {
      robotContainer.getShootingCoordinator().disableAutoShoot();
    }

    // Force OI rebind on teleop init to ensure controls are bound
    // This fixes the issue where going directly to teleop without
    // being disabled first would leave controls unbound
    robotContainer.normalModeOI();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
