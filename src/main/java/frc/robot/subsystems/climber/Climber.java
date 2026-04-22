package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Climber subsystem — despite the name, this is a pre-feed wheel that sits before the spindexer.
 * Runs at a closed-loop velocity with simple feedforward, following whatever the spindexer is
 * commanded to do (driven externally via a default command in RobotContainer).
 */
public class Climber extends SubsystemBase {

  public enum ClimberState {
    STOPPED,
    FEEDING
  }

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private ClimberState state = ClimberState.STOPPED;

  private static final LoggedTunableNumber kP;
  private static final LoggedTunableNumber kI;
  private static final LoggedTunableNumber kD;
  private static final LoggedTunableNumber kS;
  private static final LoggedTunableNumber kV;
  private static final LoggedTunableNumber toleranceRPM;
  private static final LoggedTunableNumber targetRPM;

  static {
    RobotConfig config = Constants.getRobotConfig();
    kP = new LoggedTunableNumber("Tuning/Climber/kP", config.getClimberKp());
    kI = new LoggedTunableNumber("Tuning/Climber/kI", config.getClimberKi());
    kD = new LoggedTunableNumber("Tuning/Climber/kD", config.getClimberKd());
    kS = new LoggedTunableNumber("Tuning/Climber/kS", config.getClimberKs());
    kV = new LoggedTunableNumber("Tuning/Climber/kV", config.getClimberKv());
    toleranceRPM =
        new LoggedTunableNumber(
            "Tuning/Climber/ReadyToleranceRPM", config.getClimberReadyToleranceRPM());
    targetRPM =
        new LoggedTunableNumber("Tuning/Climber/TuningVelocity", config.getTuningClimberVelocity());
  }

  /** Live-tunable target RPM for the climber (independent of spindexer speed). */
  public static LoggedTunableNumber getTuningVelocity() {
    return targetRPM;
  }

  public Climber(ClimberIO io) {
    this.io = io;
    io.setVelocityTolerance(toleranceRPM.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    Logger.recordOutput("Subsystems/ClimberState", state.name());

    if (LoggedTunableNumber.hasChanged(kP, kI, kD, kS, kV)) {
      io.configureClimberPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get());
    }
    if (LoggedTunableNumber.hasChanged(toleranceRPM)) {
      io.setVelocityTolerance(toleranceRPM.get());
    }
  }

  public void setClimberVelocity(double velocityRPM) {
    io.setClimberVelocity(velocityRPM);
    state = velocityRPM == 0.0 ? ClimberState.STOPPED : ClimberState.FEEDING;
  }

  public void stopClimber() {
    io.stopClimber();
    state = ClimberState.STOPPED;
  }

  public double getWheelRPM() {
    return inputs.wheelRPM;
  }

  public double getTargetRPM() {
    return inputs.targetRPM;
  }

  public boolean isAtSetpoint() {
    return inputs.atSetpoint;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public ClimberState getState() {
    return state;
  }

  // ========== Commands ==========

  public Command stopClimberCommand() {
    return runOnce(this::stopClimber).withName("Climber: Stop");
  }

  public Command runClimberCommand(LoggedTunableNumber rpm) {
    return run(() -> setClimberVelocity(rpm.get()))
        .finallyDo(this::stopClimber)
        .withName("Climber: Run");
  }

  public Command reverseClimberCommand(LoggedTunableNumber rpm) {
    return run(() -> setClimberVelocity(-Math.abs(rpm.get())))
        .finallyDo(this::stopClimber)
        .withName("Climber: Reverse");
  }

  /** Drive the motor in voltage mode (for SysId characterization). */
  public void runCharacterization(double output) {
    io.setClimberVoltage(output);
  }

  public double getFFCharacterizationVelocity() {
    return io.getFFCharacterizationVelocity();
  }
}
