package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // Tunable PID gains for deploy motor
  private static final LoggedTunableNumber deployKP =
      new LoggedTunableNumber("Tuning/Intake/IntakeDeploy/kP", IntakeConstants.DEPLOY_KP);
  private static final LoggedTunableNumber deployKI =
      new LoggedTunableNumber("Tuning/Intake/IntakeDeploy/kI", IntakeConstants.DEPLOY_KI);
  private static final LoggedTunableNumber deployKD =
      new LoggedTunableNumber("Tuning/Intake/IntakeDeploy/kD", IntakeConstants.DEPLOY_KD);

  // Velocity control toggle (default: velocity control on)
  private boolean useVelocityControl = true;

  // Tunable PID gains for roller velocity control
  private static final LoggedTunableNumber rollerKP =
      new LoggedTunableNumber("Tuning/Intake/IntakeRollers/kP", IntakeConstants.ROLLER_KP);
  private static final LoggedTunableNumber rollerKI =
      new LoggedTunableNumber("Tuning/Intake/IntakeRollers/kI", IntakeConstants.ROLLER_KI);
  private static final LoggedTunableNumber rollerKD =
      new LoggedTunableNumber("Tuning/Intake/IntakeRollers/kD", IntakeConstants.ROLLER_KD);
  private static final LoggedTunableNumber rollerKFF =
      new LoggedTunableNumber("Tuning/Intake/IntakeRollers/kFF", IntakeConstants.ROLLER_KFF);

  // Optional: supplier for robot velocity (for velocity-based roller speed)
  private DoubleSupplier robotVelocitySupplier = () -> 0.0;

  // Mechanism2d visualization (using AdvantageKit's logged version)
  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(3, 3);
  private final LoggedMechanismRoot2d root = mechanism.getRoot("intake", 1.5, 0.2);
  private final LoggedMechanismLigament2d deployArm;
  private final LoggedMechanismLigament2d roller;
  private double rollerAngle = 0.0; // Track cumulative roller angle

  /**
   * Creates a new Intake subsystem.
   *
   * @param io The IO implementation to use (real hardware or simulation)
   */
  public Intake(IntakeIO io) {
    this.io = io;

    // Create deploy arm (pivots based on deploy position)
    deployArm =
        root.append(
            new LoggedMechanismLigament2d("deployArm", 0.5, 0, 6, new Color8Bit(Color.kOrange)));

    // Create roller at end of deploy arm (rotates when running)
    roller =
        deployArm.append(
            new LoggedMechanismLigament2d("roller", 0.15, 90, 4, new Color8Bit(Color.kGreen)));
  }

  /**
   * Sets a supplier for robot velocity to adjust roller speed.
   *
   * @param velocitySupplier Supplier that returns robot linear velocity in m/s
   */
  public void setRobotVelocitySupplier(DoubleSupplier velocitySupplier) {
    this.robotVelocitySupplier = velocitySupplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Push tunable changes to IO
    if (LoggedTunableNumber.hasChanged(deployKP, deployKI, deployKD)) {
      io.configureDeployPID(deployKP.get(), deployKI.get(), deployKD.get());
    }
    if (LoggedTunableNumber.hasChanged(rollerKP, rollerKI, rollerKD, rollerKFF)) {
      io.configureRollerPID(rollerKP.get(), rollerKI.get(), rollerKD.get(), rollerKFF.get());
    }

    // Update deploy arm angle
    // Retracted (0 rotations) = 90° (pointing up)
    // Deployed (0.5 rotations) = 0° (pointing forward over bumper)
    double deployAngleDegrees = 90 - (inputs.deployPositionRotations * 180.0);
    deployArm.setAngle(deployAngleDegrees);

    // Update roller rotation (continuous spin based on velocity)
    // Visual scaling: RPM * 0.012 gives 18°/frame at 1500 RPM (smooth animation)
    rollerAngle += inputs.rollerVelocityRPM * 0.012;
    rollerAngle = rollerAngle % 360.0; // Keep in 0-360 range
    roller.setAngle(90 + rollerAngle); // 90° base offset

    // Log the mechanism to AdvantageKit (2D view)
    Logger.recordOutput("Intake/Mechanism2d", mechanism);
  }

  // ========== DEPLOY CONTROL ==========

  /** Deploy the intake (extend). */
  public void deploy() {
    io.setDeployPosition(IntakeConstants.DEPLOY_EXTENDED_POSITION);
  }

  /** Retract the intake. */
  public void retract() {
    io.setDeployPosition(IntakeConstants.DEPLOY_RETRACTED_POSITION);
  }

  /**
   * Set the deploy position directly.
   *
   * @param positionRotations Target position in rotations
   */
  public void setDeployPosition(double positionRotations) {
    io.setDeployPosition(positionRotations);
  }

  /** Check if the intake is fully deployed. */
  @AutoLogOutput(key = "Intake/IsDeployed")
  public boolean isDeployed() {
    return Math.abs(inputs.deployPositionRotations - IntakeConstants.DEPLOY_EXTENDED_POSITION)
        <= IntakeConstants.DEPLOY_POSITION_TOLERANCE;
  }

  /** Check if the intake is fully retracted. */
  @AutoLogOutput(key = "Intake/IsRetracted")
  public boolean isRetracted() {
    return Math.abs(inputs.deployPositionRotations - IntakeConstants.DEPLOY_RETRACTED_POSITION)
        <= IntakeConstants.DEPLOY_POSITION_TOLERANCE;
  }

  /** Check if the deploy mechanism is at target. */
  public boolean deployAtTarget() {
    return Math.abs(inputs.deployPositionRotations - inputs.deployTargetPosition)
        <= IntakeConstants.DEPLOY_POSITION_TOLERANCE;
  }

  /** Get the current deploy position. */
  public double getDeployPosition() {
    return inputs.deployPositionRotations;
  }

  // ========== ROLLER CONTROL ==========

  /** Returns whether velocity control is active. */
  @AutoLogOutput(key = "Intake/UseVelocityControl")
  public boolean isVelocityControlEnabled() {
    return useVelocityControl;
  }

  /** Toggle between velocity control and open-loop duty cycle control. */
  public void toggleVelocityControl() {
    useVelocityControl = !useVelocityControl;
  }

  /** Set whether to use velocity control (true) or open-loop (false). */
  public void setVelocityControlEnabled(boolean enabled) {
    useVelocityControl = enabled;
  }

  /** Command that toggles velocity control mode. */
  public Command toggleVelocityControlCommand() {
    return runOnce(this::toggleVelocityControl).withName("Intake: Toggle Velocity Control");
  }

  /** Run rollers to intake game pieces. */
  public void runIntake() {
    if (useVelocityControl) {
      io.setRollerVelocity(IntakeConstants.ROLLER_INTAKE_RPM);
    } else {
      io.setRollerDutyCycle(IntakeConstants.ROLLER_INTAKE_SPEED);
    }
  }

  /** Run rollers to eject game pieces. */
  public void runEject() {
    if (useVelocityControl) {
      io.setRollerVelocity(IntakeConstants.ROLLER_EJECT_RPM);
    } else {
      io.setRollerDutyCycle(IntakeConstants.ROLLER_EJECT_SPEED);
    }
  }

  /** Run rollers at hold speed. */
  public void runHold() {
    if (useVelocityControl) {
      io.setRollerVelocity(IntakeConstants.ROLLER_HOLD_RPM);
    } else {
      io.setRollerDutyCycle(IntakeConstants.ROLLER_HOLD_SPEED);
    }
  }

  /** Stop the rollers. */
  public void stopRollers() {
    io.setRollerDutyCycle(0.0);
  }

  /**
   * Set roller speed directly as duty cycle (always open-loop).
   *
   * @param dutyCycle Duty cycle from -1 to 1
   */
  public void setRollerSpeed(double dutyCycle) {
    io.setRollerDutyCycle(dutyCycle);
  }

  /**
   * Set roller velocity directly (closed-loop RPM).
   *
   * @param rpm Target roller velocity in RPM
   */
  public void setRollerVelocity(double rpm) {
    io.setRollerVelocity(rpm);
  }

  /**
   * Run rollers with speed adjusted for robot velocity. Faster robot = faster rollers to maintain
   * grip on game pieces.
   *
   * @param baseSpeed Base roller speed (duty cycle)
   * @param velocityFactor How much robot velocity affects roller speed (duty cycle per m/s)
   */
  public void runIntakeWithVelocityCompensation(double baseSpeed, double velocityFactor) {
    double robotVelocity = Math.abs(robotVelocitySupplier.getAsDouble());
    double compensatedSpeed = baseSpeed + (robotVelocity * velocityFactor);
    compensatedSpeed = Math.min(1.0, Math.max(-1.0, compensatedSpeed)); // Clamp to valid range
    io.setRollerDutyCycle(compensatedSpeed);
  }

  /** Get the current roller velocity in RPM. */
  public double getRollerVelocityRPM() {
    return inputs.rollerVelocityRPM;
  }

  /** Get the roller current draw (useful for game piece detection). */
  public double getRollerCurrentAmps() {
    return inputs.rollerCurrentAmps;
  }

  // ========== Commands ==========

  /**
   * Command to deploy the intake and run rollers at the supplied RPM. On cancel, stops rollers and
   * retracts.
   *
   * @param rollerRPM Supplier for roller velocity in RPM (read each cycle for live tuning)
   * @return Command that deploys + runs until interrupted
   */
  public Command deployAndRunCommand(DoubleSupplier rollerRPM) {
    return runOnce(this::deploy)
        .andThen(run(() -> io.setRollerVelocity(rollerRPM.getAsDouble())))
        .finallyDo(
            () -> {
              stopRollers();
              retract();
            })
        .withName("Intake: Deploy & Run");
  }

  // ========== GENERAL CONTROL ==========

  /** Stop all motors. */
  public void stop() {
    io.stop();
  }

  // ========== 3D VISUALIZATION ==========

  // Intake pivot point relative to robot center (meters)
  // X = forward, Y = left, Z = up
  // Adjust PIVOT_X to match your robot's front bumper location
  private static final double PIVOT_X = 0.43; // At front bumper (frame edge + bumper thickness)
  private static final double PIVOT_Y = 0.0; // Centered left-right
  private static final double PIVOT_Z = 0.20; // Pivot height above ground

  /**
   * Returns the 3D pose of the intake arm for AdvantageScope visualization. When retracted (0
   * rotations), arm points backward into robot. When deployed (0.5 rotations), arm points forward
   * over bumper.
   */
  @AutoLogOutput(key = "Odometry/Intake")
  public Pose3d getPose() {
    // Convert deploy position to pitch angle
    // 0 rotations = 90° pitch (pointing up/back into robot)
    // 0.5 rotations = -90° pitch (pointing down/forward over bumper)
    double pitchRadians = Math.PI / 2 - (inputs.deployPositionRotations * 2 * Math.PI);

    return new Pose3d(PIVOT_X, PIVOT_Y, PIVOT_Z, new Rotation3d(0, pitchRadians, 0));
  }
}
