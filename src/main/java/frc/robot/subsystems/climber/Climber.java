package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Climber subsystem — three positions, two buttons.
 *
 * <p>Hardware: single NEO motor through a 25:1 gearbox driving a 0.75" winch drum. Brake mode
 * holds position when the motor stops.
 *
 * <p>Positions:
 *
 * <ul>
 *   <li>Stowed (0) — fully retracted, put away
 *   <li>Extended (extendPosition) — arm up, lined up with the pole
 *   <li>Climbed (climbPosition) — partially retracted, robot off the ground
 * </ul>
 *
 * <p>Operator buttons:
 *
 * <ul>
 *   <li>B9 (extend): STOWED→EXTENDED, CLIMBED→EXTENDED, EXTENDED→STOWED
 *   <li>B2-1 (climb): EXTENDED→CLIMBED
 * </ul>
 */
public class Climber extends SubsystemBase {

  public enum ClimberState {
    STOWED, // Position 0 — put away
    EXTENDING, // Moving to extended position
    EXTENDED, // Arm up, lined up with pole
    CLIMBING, // Moving to climb position
    CLIMBED, // Off the ground, brake holds
    STOWING // Moving back to position 0
  }

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private ClimberState state = ClimberState.STOWED;
  private double targetPositionRotations = 0.0;

  private static final LoggedTunableNumber extendPosition =
      new LoggedTunableNumber("Climber/extendPosition");
  private static final LoggedTunableNumber climbPosition =
      new LoggedTunableNumber("Climber/climbPosition");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Climber/kP");
  private static final LoggedTunableNumber positionTolerance =
      new LoggedTunableNumber("Climber/positionTolerance", 2.0);

  public Climber(ClimberIO io) {
    this.io = io;

    RobotConfig config = Constants.getRobotConfig();
    extendPosition.initDefault(config.getClimberExtendPosition());
    climbPosition.initDefault(config.getClimberClimbPosition());
    kP.initDefault(config.getClimberKp());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    if (LoggedTunableNumber.hasChanged(kP)) {
      io.configurePID(kP.get());
    }

    switch (state) {
      case STOWED:
        break;

      case EXTENDING:
        if (atPosition(targetPositionRotations)) {
          io.stop();
          state = ClimberState.EXTENDED;
        }
        break;

      case EXTENDED:
        break;

      case CLIMBING:
        if (atPosition(targetPositionRotations)) {
          io.stop();
          state = ClimberState.CLIMBED;
        }
        break;

      case CLIMBED:
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
  }

  // ===== Actions =====

  /**
   * Extend button action. Goes to extended position from stowed or climbed. If already extended,
   * stows instead.
   */
  public void toggleExtend() {
    switch (state) {
      case STOWED:
      case CLIMBED:
        targetPositionRotations = extendPosition.get();
        io.setPosition(targetPositionRotations);
        state = ClimberState.EXTENDING;
        break;

      case EXTENDED:
        targetPositionRotations = 0.0;
        io.setPosition(targetPositionRotations);
        state = ClimberState.STOWING;
        break;

      default:
        break;
    }
  }

  /** Climb button action. From extended, retracts to climb position (off the ground). */
  public void climb() {
    if (state == ClimberState.EXTENDED) {
      targetPositionRotations = climbPosition.get();
      io.setPosition(targetPositionRotations);
      state = ClimberState.CLIMBING;
    }
  }

  // ===== State queries =====

  public ClimberState getState() {
    return state;
  }

  public boolean isExtended() {
    return state == ClimberState.EXTENDED;
  }

  public boolean isClimbed() {
    return state == ClimberState.CLIMBED;
  }

  public boolean isStowed() {
    return state == ClimberState.STOWED;
  }

  // ===== Command factories (for auto — explicit, not toggles) =====

  /** Command: extend from stowed, finishes when extended. */
  public Command extendCommand() {
    return Commands.runOnce(
            () -> {
              if (state == ClimberState.STOWED) {
                targetPositionRotations = extendPosition.get();
                io.setPosition(targetPositionRotations);
                state = ClimberState.EXTENDING;
              }
            },
            this)
        .andThen(Commands.waitUntil(this::isExtended))
        .withName("ClimberExtend");
  }

  /** Command: climb from extended, finishes when climbed. */
  public Command climbCommand() {
    return Commands.runOnce(this::climb, this)
        .andThen(Commands.waitUntil(this::isClimbed))
        .withName("ClimberClimb");
  }

  // ===== Helpers =====

  private boolean atPosition(double targetRotations) {
    return Math.abs(inputs.positionRotations - targetRotations) < positionTolerance.get();
  }
}
