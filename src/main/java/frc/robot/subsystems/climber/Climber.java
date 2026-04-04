package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Climber subsystem — two positions, two buttons.
 *
 * <p>Hardware: single NEO motor through a 25:1 gearbox driving a 0.75" winch drum. Brake mode
 * holds position when the motor stops.
 *
 * <p>Extend = rope out, arm up. Retract = rope in, arm down. "Climbing" is just retracting while
 * hooked on the bar. "Stowing" is also just retracting.
 */
public class Climber extends SubsystemBase {

  public enum ClimberState {
    RETRACTED, // At position 0 — stowed or climbed
    EXTENDING, // Moving to extended position
    EXTENDED, // Rope out, arm up
    RETRACTING // Moving back to position 0
  }

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private ClimberState state = ClimberState.RETRACTED;

  private static final LoggedTunableNumber extendPosition =
      new LoggedTunableNumber("Climber/extendPosition");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Climber/kP");
  private static final LoggedTunableNumber positionTolerance =
      new LoggedTunableNumber("Climber/positionTolerance", 2.0);

  public Climber(ClimberIO io) {
    this.io = io;

    RobotConfig config = Constants.getRobotConfig();
    extendPosition.initDefault(config.getClimberExtendPosition());
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
      case RETRACTED:
        break;

      case EXTENDING:
        if (atPosition(extendPosition.get())) {
          io.stop();
          state = ClimberState.EXTENDED;
        }
        break;

      case EXTENDED:
        break;

      case RETRACTING:
        if (atPosition(0.0)) {
          io.stop();
          state = ClimberState.RETRACTED;
        }
        break;
    }

    Logger.recordOutput("Climber/State", state.name());
  }

  // ===== Actions =====

  /** Extend the climber (rope out, arm up). */
  public void extend() {
    if (state == ClimberState.RETRACTED) {
      io.setPosition(extendPosition.get());
      state = ClimberState.EXTENDING;
    }
  }

  /** Retract the climber (rope in, arm down). Climbs if hooked, stows if not. */
  public void retract() {
    if (state == ClimberState.EXTENDED) {
      io.setPosition(0.0);
      state = ClimberState.RETRACTING;
    }
  }

  // ===== State queries =====

  public ClimberState getState() {
    return state;
  }

  public boolean isExtended() {
    return state == ClimberState.EXTENDED;
  }

  public boolean isRetracted() {
    return state == ClimberState.RETRACTED;
  }

  // ===== Command factories =====

  /** Command: extend, finishes when extended. */
  public Command extendCommand() {
    return Commands.runOnce(this::extend, this)
        .andThen(Commands.waitUntil(this::isExtended))
        .withName("ClimberExtend");
  }

  /** Command: retract, finishes when retracted. */
  public Command retractCommand() {
    return Commands.runOnce(this::retract, this)
        .andThen(Commands.waitUntil(this::isRetracted))
        .withName("ClimberRetract");
  }

  // ===== Helpers =====

  private boolean atPosition(double targetRotations) {
    return Math.abs(inputs.positionRotations - targetRotations) < positionTolerance.get();
  }
}
