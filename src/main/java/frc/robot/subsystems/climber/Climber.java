package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Climber subsystem with a simple state machine.
 *
 * <p>Hardware: single NEO motor through a 25:1 gearbox driving a 0.75" winch drum. The motor has a
 * brake so the mechanism holds position when the motor stops.
 *
 * <p>Match play flow: STOWED → RELEASING → RELEASED → CLIMBING → CLIMBED → EXTENDING → RELEASED →
 * STOWING → STOWED
 *
 * <p>Endgame flow: STOWED → RELEASING → RELEASED → CLIMBING → CLIMBED (stays up)
 *
 * <p>If the robot climbed in auto, teleopInit() automatically extends back down so the robot can
 * drive. The operator then presses the same button to stow.
 */
public class Climber extends SubsystemBase {

  /** Climber state machine states. */
  public enum ClimberState {
    STOWED, // Starting position, encoder zeroed
    RELEASING, // Motor extending to release rope
    RELEASED, // Rope is out, robot on ground
    CLIMBING, // Motor retracting rope, pulling robot up
    CLIMBED, // Robot is up, brake holds
    EXTENDING, // Motor extending back down after climb (putting robot back on ground)
    STOWING // Motor retracting all the way back to stowed position
  }

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private ClimberState state = ClimberState.STOWED;
  private double targetPositionRotations = 0.0;
  private boolean climbedOnce = false;

  // Tunable positions (motor rotations)
  private static final LoggedTunableNumber releaseRotations =
      new LoggedTunableNumber("Climber/releaseRotations");
  private static final LoggedTunableNumber climbRotations =
      new LoggedTunableNumber("Climber/climbRotations");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Climber/kP");

  // How close we need to be to consider a position reached (motor rotations)
  private static final LoggedTunableNumber positionTolerance =
      new LoggedTunableNumber("Climber/positionTolerance", 2.0);

  public Climber(ClimberIO io) {
    this.io = io;

    // Initialize tunables from config
    RobotConfig config = Constants.getRobotConfig();
    releaseRotations.initDefault(config.getClimberReleaseRotations());
    climbRotations.initDefault(config.getClimberClimbRotations());
    kP.initDefault(config.getClimberKp());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    // Push PID changes to motor controller
    if (LoggedTunableNumber.hasChanged(kP)) {
      io.configurePID(kP.get());
    }

    // State machine transitions — check if we've reached the target position
    switch (state) {
      case STOWED:
        break;

      case RELEASING:
        if (atPosition(targetPositionRotations)) {
          io.stop();
          state = ClimberState.RELEASED;
        }
        break;

      case RELEASED:
        break;

      case CLIMBING:
        if (atPosition(targetPositionRotations)) {
          io.stop();
          climbedOnce = true;
          state = ClimberState.CLIMBED;
        }
        break;

      case CLIMBED:
        break;

      case EXTENDING:
        if (atPosition(targetPositionRotations)) {
          io.stop();
          state = ClimberState.RELEASED;
        }
        break;

      case STOWING:
        if (atPosition(targetPositionRotations)) {
          io.stop();
          state = ClimberState.STOWED;
        }
        break;
    }

    Logger.recordOutput("Climber/State", state.name());
    Logger.recordOutput("Climber/TargetPosition", targetPositionRotations);
    Logger.recordOutput("Climber/ClimbedOnce", climbedOnce);
  }

  // ===== Actions =====

  /** Release the rope. STOWED → RELEASING */
  public void release() {
    if (state == ClimberState.STOWED) {
      targetPositionRotations = releaseRotations.get();
      io.setPosition(targetPositionRotations);
      state = ClimberState.RELEASING;
    }
  }

  /** Climb (retract rope to pull robot up). RELEASED → CLIMBING */
  public void climb() {
    if (state == ClimberState.RELEASED) {
      targetPositionRotations = releaseRotations.get() + climbRotations.get();
      io.setPosition(targetPositionRotations);
      state = ClimberState.CLIMBING;
    }
  }

  /** Extend back down after climbing (put robot back on ground). CLIMBED → EXTENDING → RELEASED */
  public void extend() {
    if (state == ClimberState.CLIMBED) {
      targetPositionRotations = releaseRotations.get();
      io.setPosition(targetPositionRotations);
      state = ClimberState.EXTENDING;
    }
  }

  /** Retract climber to stowed position. RELEASED → STOWING → STOWED */
  public void stow() {
    if (state == ClimberState.RELEASED) {
      targetPositionRotations = 0.0;
      io.setPosition(targetPositionRotations);
      state = ClimberState.STOWING;
    }
  }

  /**
   * Called on teleop init. If the robot climbed in auto, automatically extend back down so the
   * robot can drive away. The operator then presses stow when ready.
   */
  public void onTeleopInit() {
    if (state == ClimberState.CLIMBED) {
      extend();
    }
  }

  // ===== State queries =====

  public ClimberState getState() {
    return state;
  }

  /** True if the climber has completed a climb at any point this match. */
  public boolean hasClimbedOnce() {
    return climbedOnce;
  }

  public boolean isClimbed() {
    return state == ClimberState.CLIMBED;
  }

  public boolean isReleased() {
    return state == ClimberState.RELEASED;
  }

  public boolean isStowed() {
    return state == ClimberState.STOWED;
  }

  // ===== Command factories =====

  /** Command: release rope, finishes when released. */
  public Command releaseCommand() {
    return Commands.runOnce(this::release, this)
        .andThen(Commands.waitUntil(this::isReleased))
        .withName("ClimberRelease");
  }

  /** Command: climb up, finishes when climbed. */
  public Command climbCommand() {
    return Commands.runOnce(this::climb, this)
        .andThen(Commands.waitUntil(this::isClimbed))
        .withName("ClimberClimb");
  }

  /** Command: full auto climb sequence (release then climb). */
  public Command autoClimbCommand() {
    return releaseCommand().andThen(climbCommand()).withName("ClimberAutoClimb");
  }

  // ===== Helpers =====

  private boolean atPosition(double targetRotations) {
    return Math.abs(inputs.positionRotations - targetRotations) < positionTolerance.get();
  }
}
