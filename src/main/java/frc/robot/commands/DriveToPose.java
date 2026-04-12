package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  private static final LoggedTunableNumber timeout =
      new LoggedTunableNumber("DriveToPose/timeout", 3.0);

  // PID Gains
  private static final LoggedTunableNumber kPX = new LoggedTunableNumber("DriveToPose/kPX", .9);
  private static final LoggedTunableNumber kPY = new LoggedTunableNumber("DriveToPose/kPY", .9);
  private static final LoggedTunableNumber kPTheta =
      new LoggedTunableNumber("DriveToPose/kPTheta", .9);

  // Constraints
  private static final LoggedTunableNumber maxVelXY =
      new LoggedTunableNumber("DriveToPose/MaxVelXY", 1.25);
  private static final LoggedTunableNumber maxAccelXY =
      new LoggedTunableNumber("DriveToPose/MaxAccelXY", 2.0);
  private static final LoggedTunableNumber maxVelTheta =
      new LoggedTunableNumber("DriveToPose/MaxVelThetaDeg", 270.0);
  private static final LoggedTunableNumber maxAccelTheta =
      new LoggedTunableNumber("DriveToPose/MaxAccelThetaDeg", 540.0);

  // Heading must be within this tolerance before translation begins (degrees).
  // Prevents the robot from driving toward the target while still rotating.
  private static final LoggedTunableNumber headingGateDeg =
      new LoggedTunableNumber("DriveToPose/HeadingGateDeg", 10.0);

  private final double speedScalar;
  private final Drive drivetrain;
  private final Supplier<Pose2d> poseSupplier;

  private Pose2d targetPose;
  private Pose2d currentPose;
  private Timer timeoutTimer = new Timer();
  private boolean translationStarted = false;

  private ProfiledPIDController driveControllerX, driveControllerY, thetaController;

  /** Drives to the specified pose under full software control. */
  public DriveToPose(Drive drivetrain, Supplier<Pose2d> poseSupplier, double speedScalar) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;
    this.speedScalar = MathUtil.clamp(speedScalar, 0.0, 1.0);
    addRequirements(drivetrain);
  }

  public DriveToPose(Drive drivetrain, Supplier<Pose2d> poseSupplier) {
    this(drivetrain, poseSupplier, 1.0);
  }

  @Override
  public void initialize() {
    createControllers();
    currentPose = drivetrain.getPose();

    // Log initial state
    Logger.recordOutput("DriveToPose/Init/StartPose", currentPose);
    Logger.recordOutput("DriveToPose/Init/SpeedScalar", speedScalar);

    driveControllerX.reset(currentPose.getX(), 0);
    driveControllerY.reset(currentPose.getY(), 0);
    thetaController.reset(currentPose.getRotation().getRadians(), 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    targetPose = poseSupplier.get();
    driveControllerX.setGoal(targetPose.getX());
    driveControllerY.setGoal(targetPose.getY());
    thetaController.setGoal(targetPose.getRotation().getRadians());

    // Log target pose
    Logger.recordOutput("DriveToPose/Init/TargetPose", targetPose);

    driveControllerX.setTolerance(Units.inchesToMeters(.5));
    driveControllerY.setTolerance(Units.inchesToMeters(.5));
    thetaController.setTolerance(Units.degreesToRadians(0.1));

    translationStarted = false;
    timeoutTimer.restart();
  }

  @Override
  public void execute() {
    currentPose = drivetrain.getPose();

    // Calculate errors
    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    double thetaError =
        MathUtil.angleModulus(
            targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

    Logger.recordOutput("DriveToPose/Error/XErrorMeters", xError);
    Logger.recordOutput("DriveToPose/Error/YErrorMeters", yError);
    Logger.recordOutput("DriveToPose/Error/ThetaErrorDegrees", Units.radiansToDegrees(thetaError));

    // Calculate velocities
    double driveXVelocity = driveControllerX.calculate(currentPose.getX(), targetPose.getX());
    double driveYVelocity = driveControllerY.calculate(currentPose.getY(), targetPose.getY());
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Scale velocities
    double effectiveMaxLinearSpeed = drivetrain.getMaxLinearSpeedMetersPerSec() * speedScalar;
    double scaledXVelocity = driveXVelocity * effectiveMaxLinearSpeed;
    double scaledYVelocity = driveYVelocity * effectiveMaxLinearSpeed;
    double scaledThetaVelocity = thetaVelocity * drivetrain.getMaxAngularSpeedRadPerSec();

    if (thetaController.atGoal()) scaledThetaVelocity = 0.0;

    // Suppress translation until heading is close enough to the target.
    // This ensures the robot rotates in place first, then drives straight to the goal.
    boolean headingAligned = Math.abs(Math.toDegrees(thetaError)) <= headingGateDeg.get();
    if (!headingAligned) {
      scaledXVelocity = 0.0;
      scaledYVelocity = 0.0;
      // Reset translation profiles so they don't accumulate while waiting
      driveControllerX.reset(currentPose.getX(), 0);
      driveControllerY.reset(currentPose.getY(), 0);
    } else if (!translationStarted) {
      // Restart timeout when translation actually begins, so rotation time doesn't eat into it
      translationStarted = true;
      timeoutTimer.restart();
    }

    Logger.recordOutput("DriveToPose/HeadingAligned", headingAligned);
    Logger.recordOutput("DriveToPose/VelocityCommands/X", scaledXVelocity);
    Logger.recordOutput("DriveToPose/VelocityCommands/Y", scaledYVelocity);
    Logger.recordOutput(
        "DriveToPose/VelocityCommands/Theta", Units.radiansToDegrees(scaledThetaVelocity));

    drivetrain.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            scaledXVelocity, scaledYVelocity, scaledThetaVelocity, drivetrain.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    timeoutTimer.stop();

    // Log completion state
    Logger.recordOutput("DriveToPose/End/FinalPose", drivetrain.getPose());
    Logger.recordOutput("DriveToPose/End/TotalTime", timeoutTimer.get());
    Logger.recordOutput("DriveToPose/End/WasInterrupted", interrupted);
    Logger.recordOutput("DriveToPose/End/TimedOut", timeoutTimer.hasElapsed(timeout.get()));
  }

  @Override
  public boolean isFinished() {
    boolean finished = atGoal() || timeoutTimer.hasElapsed(timeout.get());

    Logger.recordOutput("DriveToPose/IsFinished", finished);
    Logger.recordOutput("DriveToPose/CompletionReason", atGoal() ? "Reached Target" : "Timed Out");

    return finished;
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return driveControllerX.atGoal() && driveControllerY.atGoal() && thetaController.atGoal();
  }

  private void createControllers() {
    driveControllerX =
        new ProfiledPIDController(
            kPX.get(),
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(maxVelXY.get(), maxAccelXY.get()));

    driveControllerY =
        new ProfiledPIDController(
            kPY.get(),
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(maxVelXY.get(), maxAccelXY.get()));

    thetaController =
        new ProfiledPIDController(
            kPTheta.get(),
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                Math.toRadians(maxVelTheta.get()), Math.toRadians(maxAccelTheta.get())));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }
}
