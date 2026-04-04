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
 * <p>States:
 *
 * <ul>
 *   <li>STOWED — starting position, encoder zeroed, motor stopped
 *   <li>RELEASING — motor runs forward to release rope
 *   <li>RELEASED — rope is out, waiting for climb command
 *   <li>CLIMBING — motor retracts rope to pull robot up
 *   <li>CLIMBED — robot is up, motor stopped, brake holds
 * </ul>
 */
public class Climber extends SubsystemBase {

  /** Climber state machine states. */
  public enum ClimberState {
    STOWED,
    RELEASING,
    RELEASED,
    CLIMBING,
    CLIMBED
  }

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private ClimberState state = ClimberState.STOWED;
  private double targetPositionRotations = 0.0;

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

    // State machine transitions
    switch (state) {
      case STOWED:
        // Motor stopped, waiting for release command
        break;

      case RELEASING:
        // Check if we've reached the release position
        if (atPosition(targetPositionRotations)) {
          io.stop();
          state = ClimberState.RELEASED;
        }
        break;

      case RELEASED:
        // Rope is out, waiting for climb command
        break;

      case CLIMBING:
        // Check if we've reached the climb (or stow) position
        if (atPosition(targetPositionRotations)) {
          io.stop();
          state = ClimberState.CLIMBED;
        }
        break;

      case CLIMBED:
        // Done — brake holds the robot up
        break;
    }

    Logger.recordOutput("Climber/State", state.name());
    Logger.recordOutput("Climber/TargetRelease", releaseRotations.get());
    Logger.recordOutput("Climber/TargetClimb", releaseRotations.get() + climbRotations.get());
  }

  // ===== Commands =====

  /** Release the rope (STOWED → RELEASING → RELEASED). */
  public void release() {
    if (state == ClimberState.STOWED) {
      targetPositionRotations = releaseRotations.get();
      io.setPosition(targetPositionRotations);
      state = ClimberState.RELEASING;
    }
  }

  /** Climb (retract rope) (RELEASED → CLIMBING → CLIMBED). */
  public void climb() {
    if (state == ClimberState.RELEASED) {
      targetPositionRotations = releaseRotations.get() + climbRotations.get();
      io.setPosition(targetPositionRotations);
      state = ClimberState.CLIMBING;
    }
  }

  /** Retract fully back to stowed position (from RELEASED only — for match play). */
  public void stow() {
    if (state == ClimberState.RELEASED) {
      targetPositionRotations = 0.0;
      io.setPosition(targetPositionRotations);
      state = ClimberState.CLIMBING;
    }
  }

  /** Get current state. */
  public ClimberState getState() {
    return state;
  }

  /** Returns true when the climber has finished climbing. */
  public boolean isClimbed() {
    return state == ClimberState.CLIMBED;
  }

  /** Returns true when the rope is fully released. */
  public boolean isReleased() {
    return state == ClimberState.RELEASED;
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

  /** Command: full auto climb sequence (release → wait → climb). */
  public Command autoClimbCommand() {
    return releaseCommand().andThen(climbCommand()).withName("ClimberAutoClimb");
  }

  // ===== Helpers =====

  private boolean atPosition(double targetRotations) {
    return Math.abs(inputs.positionRotations - targetRotations) < positionTolerance.get();
  }
}
