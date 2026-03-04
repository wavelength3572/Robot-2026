// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  // Phase delay: time between computing a shot and the ball actually leaving the turret.
  // Accounts for control loop latency, mechanical response, and ball transit time.
  // At 3 m/s this projects the aim point ~6-9 cm forward — significant at long range.
  private static final double AIM_PHASE_DELAY_SECONDS = 0.02; // One control loop cycle (20ms)

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final Turret turret;
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  // Pre-allocated arrays to reduce GC pressure in periodic()
  private final SwerveModulePosition[] odometryPositions = new SwerveModulePosition[4];
  private final SwerveModulePosition[] odometryDeltas = new SwerveModulePosition[4];
  private final SwerveModuleState[] measuredStates = new SwerveModuleState[4];
  private final SwerveModulePosition[] measuredPositions = new SwerveModulePosition[4];
  private static final ChassisSpeeds ZERO_SPEEDS = new ChassisSpeeds();
  private static final SwerveModuleState[] EMPTY_MODULE_STATES = new SwerveModuleState[] {};
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Turret turret) {
    this.gyroIO = gyroIO;
    this.turret = turret;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(10.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      Logger.recordOutput("Drive/SwerveStates/Setpoints", EMPTY_MODULE_STATES);
      Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", EMPTY_MODULE_STATES);
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module (reuse pre-allocated arrays)
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        odometryPositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        odometryDeltas[moduleIndex] =
            new SwerveModulePosition(
                odometryPositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                odometryPositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = odometryPositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(odometryDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, odometryPositions);
    }

    // Update turret to aim based on zone: shoot in alliance zone, pass otherwise.
    // Project robot position forward by phase delay to compensate for control latency.
    // TODO: Re-enable turret aiming after turret bringup is complete
    // if (turret != null) {
    //   Pose2d currentPose = getPose();
    //   Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    //
    //   // Project pose forward using Twist2d to account for phase delay.
    //   // This compensates for the time between computing the aim angle and the ball leaving.
    //   ChassisSpeeds speeds = getChassisSpeeds();
    //   Twist2d twist =
    //       new Twist2d(
    //           speeds.vxMetersPerSecond * AIM_PHASE_DELAY_SECONDS,
    //           speeds.vyMetersPerSecond * AIM_PHASE_DELAY_SECONDS,
    //           speeds.omegaRadiansPerSecond * AIM_PHASE_DELAY_SECONDS);
    //   Pose2d projectedPose = currentPose.exp(twist);
    //
    //   TurretAimingHelper.AimResult aimResult =
    //       TurretAimingHelper.getAimTarget(projectedPose.getX(), projectedPose.getY(), alliance);
    //
    //   // Log aiming data for AdvantageScope (zone-based target selection)
    //   Logger.recordOutput("Turret/Aim/Mode", aimResult.mode().toString());
    //   Logger.recordOutput(
    //       "Turret/Aim/Target",
    //       new Pose2d(aimResult.target(), new edu.wpi.first.math.geometry.Rotation2d()));
    //   Logger.recordOutput("Turret/Aim/ProjectedPose", projectedPose);
    //
    //   turret.aimAtFieldPosition(
    //       projectedPose.getX(),
    //       projectedPose.getY(),
    //       projectedPose.getRotation().getDegrees(),
    //       aimResult.target().getX(),
    //       aimResult.target().getY());
    // }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    // Log unoptimized setpoints
    Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Drive/ChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(ZERO_SPEEDS);
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = modules[i].getState();
    }
    return measuredStates;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    for (int i = 0; i < 4; i++) {
      measuredPositions[i] = modules[i].getPosition();
    }
    return measuredPositions;
  }

  /** Returns the measured chassis speeds of the robot (robot-relative). */
  @AutoLogOutput(key = "Drive/ChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Returns the measured chassis speeds of the robot in field-relative coordinates. Rotates the
   * robot-relative speeds from {@link #getChassisSpeeds()} by the current heading so that vx/vy
   * align with the field X/Y axes.
   */
  @AutoLogOutput(key = "Drive/ChassisSpeeds/FieldRelative")
  public ChassisSpeeds getFieldRelativeSpeeds() {
    ChassisSpeeds robotRelative = getChassisSpeeds();
    double heading = getRotation().getRadians();
    double cos = Math.cos(heading);
    double sin = Math.sin(heading);
    return new ChassisSpeeds(
        robotRelative.vxMetersPerSecond * cos - robotRelative.vyMetersPerSecond * sin,
        robotRelative.vxMetersPerSecond * sin + robotRelative.vyMetersPerSecond * cos,
        robotRelative.omegaRadiansPerSecond);
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Returns the pose projected forward by the aim phase delay, accounting for current velocity. Use
   * this for aiming calculations that need to compensate for control loop latency.
   */
  public Pose2d getProjectedPose() {
    Pose2d currentPose = getPose();
    ChassisSpeeds speeds = getChassisSpeeds();
    Twist2d twist =
        new Twist2d(
            speeds.vxMetersPerSecond * AIM_PHASE_DELAY_SECONDS,
            speeds.vyMetersPerSecond * AIM_PHASE_DELAY_SECONDS,
            speeds.omegaRadiansPerSecond * AIM_PHASE_DELAY_SECONDS);
    return currentPose.exp(twist);
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Zeroes the gyroscope. This sets the current rotation of the robot to zero degrees. This method
   * is intended to be invoked only when the alignment beteween the robot's rotation and the gyro is
   * sufficiently different to make field-relative driving difficult. The robot needs to be
   * positioned facing away from the driver, ideally aligned to a field wall before this method is
   * invoked.
   */
  public void zeroGyroscope() {
    setGyroOffset(0.0);
  }

  /**
   * Sets the rotation of the robot to the specified value. This method should only be invoked when
   * the rotation of the robot is known (e.g., at the start of an autonomous path). Zero degrees is
   * facing away from the driver station; CCW is positive.
   *
   * @param expectedYaw the rotation of the robot (in degrees)
   */
  public void setGyroOffset(double expectedYaw) {
    // There is a delay between setting the yaw on the Pigeon and that change
    // taking effect. As a result, it is recommended to never set the yaw and
    // adjust the local offset instead.
    this.gyroIO.resetPositionToZero();
    // if (gyroInputs.connected) {
    // this.gyroOffset = expectedYaw - gyroInputs.positionDeg;
    // } else {
    // this.gyroOffset = 0;
    // this.estimatedPoseWithoutGyro =
    // new Pose2d(
    // estimatedPoseWithoutGyro.getX(),
    // estimatedPoseWithoutGyro.getY(),
    // Rotation2d.fromDegrees(expectedYaw));
    // }
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxSpeedMetersPerSec / driveBaseRadius;
  }
}
