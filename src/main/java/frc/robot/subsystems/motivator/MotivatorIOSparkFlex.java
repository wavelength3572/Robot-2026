package frc.robot.subsystems.motivator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.SparkConnection;
import java.util.function.DoubleSupplier;

/**
 * MotivatorIO implementation using a SparkFlex controller with a NEO Vortex motor.
 *
 * <p>Hardware configuration:
 *
 * <ul>
 *   <li>Motor 55: Motivator motor 1 (independent)
 * </ul>
 */
public class MotivatorIOSparkFlex implements MotivatorIO {
  private final RobotConfig config;

  // Hardware - Motivator motor 1 (CAN ID 55)
  private final SparkFlex motivator;
  private final RelativeEncoder motivatorEncoder;
  private final SparkClosedLoopController motivatorController;
  private double motivatorMotorRPM = 0.0;
  private int periodicCounter = 0;

  // Connection debouncers
  private final Debouncer motivatorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Skip CAN reads when motor is disconnected to prevent loop overruns
  private final SparkConnection motivatorConnection = new SparkConnection();

  // Velocity tolerances for at-setpoint check with hysteresis (set by subsystem via
  // setVelocityTolerance). Enter tolerance is tighter; exit tolerance is wider to
  // prevent oscillation at the boundary.
  private double motivatorEnterToleranceRPM = 100.0;
  private double motivatorExitToleranceRPM = 500.0;
  private boolean wasAtSetpoint = false;

  // Feedforward runs onboard the SparkFlex via closedLoop.feedForward.sv() at 1kHz.

  // Target tracking
  private double wheelTargetRPM = 0.0;
  private boolean motivatorVelocityMode = false;

  public MotivatorIOSparkFlex() {
    config = Constants.getRobotConfig();

    // Create SparkFlex controller
    motivator = new SparkFlex(config.getMotivatorCanId(), MotorType.kBrushless);

    // Get encoders and closed-loop controllers
    motivatorEncoder = motivator.getEncoder();
    motivatorController = motivator.getClosedLoopController();

    // Configure motivator (CAN ID 55)
    double initKp = config.getMotivatorKp();
    double initKi = config.getMotivatorKi();
    double initKd = config.getMotivatorKd();
    double initKs = config.getMotivatorKs();
    double initKv = config.getMotivatorKv();

    var motorConfig = new SparkFlexConfig();
    motorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(config.getMotivatorCurrentLimit())
        .voltageCompensation(12.0);

    // Reduce encoder velocity filter lag for faster PID response.
    // REV defaults use a 64-tap FIR filter (~164ms window, ~82ms phase lag) which
    // is far too sluggish for flywheel recovery. These settings reduce the effective
    // measurement window to ~16ms (~8ms phase lag), giving the 1kHz onboard PID
    // much fresher velocity data to work with.
    // See:
    // https://www.chiefdelphi.com/t/psa-rev-spark-default-velocity-filtering-is-still-really-bad-for-flywheels/514567
    motorConfig.encoder.uvwMeasurementPeriod(8).uvwAverageDepth(2);

    // PID + feedforward all run onboard the SparkFlex at 1kHz — no CAN latency.
    // kS/kV applied automatically in kVelocity mode (per REV 2026 API).
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(initKp, initKi, initKd);

    motorConfig.closedLoop.feedForward.sv(initKs, initKv, ClosedLoopSlot.kSlot0);

    motorConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(100)
        .busVoltagePeriodMs(100)
        .outputCurrentPeriodMs(100);

    tryUntilOk(
        motivator,
        5,
        () ->
            motivator.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Startup diagnostics
    System.out.println("[MotivatorIOSparkFlex] ========== STARTUP ==========");
    System.out.println("[MotivatorIOSparkFlex] Motivator CAN ID: " + config.getMotivatorCanId());
    System.out.println(
        "[MotivatorIOSparkFlex] Motivator Current Limit: " + config.getMotivatorCurrentLimit());
    System.out.println("[MotivatorIOSparkFlex] ==============================");
  }

  /** Convert motor RPM to wheel RPM */
  private double motorToWheelRPM(double motorRPM) {
    return motorRPM * config.getMotivatorGearRatio();
  }

  /** Convert wheel RPM to motor RPM */
  private double wheelToMotorRPM(double wheelRPM) {
    return wheelRPM / config.getMotivatorGearRatio();
  }

  @Override
  public void updateInputs(MotorInputs motor1Inputs) {
    // Update motivator inputs (skip reads if disconnected to prevent timeout delays)
    if (!motivatorConnection.isSkipping()) {
      sparkStickyFault = false;
      ifOk(motivator, motivatorEncoder::getVelocity, (value) -> motivatorMotorRPM = value);
      motor1Inputs.wheelRPM = motorToWheelRPM(motivatorMotorRPM);
      ifOk(
          motivator,
          new DoubleSupplier[] {motivator::getAppliedOutput, motivator::getBusVoltage},
          (values) -> motor1Inputs.appliedVolts = values[0] * values[1]);
      ifOk(motivator, motivator::getOutputCurrent, (value) -> motor1Inputs.currentAmps = value);

      // Throttle temperature reads to every ~1 second (50 cycles at 20ms)
      if (++periodicCounter >= 50) {
        periodicCounter = 0;
        ifOk(
            motivator, motivator::getMotorTemperature, (value) -> motor1Inputs.tempCelsius = value);
      }
      motor1Inputs.connected = motivatorConnectedDebounce.calculate(!sparkStickyFault);
      motivatorConnection.update(motor1Inputs.connected);
    } else {
      motor1Inputs.connected = false;
    }

    // Velocity control status
    motor1Inputs.targetRPM = wheelTargetRPM;

    // Hysteresis: use tighter tolerance to enter READY, wider tolerance to leave it.
    // This prevents oscillation when velocity ripples near the threshold.
    double error = Math.abs(motor1Inputs.wheelRPM - wheelTargetRPM);
    double threshold = wasAtSetpoint ? motivatorExitToleranceRPM : motivatorEnterToleranceRPM;
    wasAtSetpoint = motivatorVelocityMode && error < threshold;
    motor1Inputs.atSetpoint = wasAtSetpoint;
  }

  @Override
  public void setMotivatorVoltage(double volts) {
    // Clear velocity target when in voltage mode (for SysId characterization)
    wheelTargetRPM = 0.0;
    motivatorController.setSetpoint(volts, ControlType.kVoltage);
  }

  // ========== Velocity Control ==========

  @Override
  public void setMotivatorVelocity(double wheelVelocityRPM) {
    if (wheelVelocityRPM < 1.0) {
      stopMotivator();
    } else {
      motivatorVelocityMode = true;
      wheelTargetRPM = Math.abs(wheelVelocityRPM);
      // kVelocity with onboard kS/kV: PID + FF all run at 1kHz on SparkFlex.
      motivatorController.setSetpoint(
          wheelToMotorRPM(wheelTargetRPM), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
  }

  @Override
  public void stopMotivator() {
    motivatorVelocityMode = false;
    wheelTargetRPM = 0.0;
    motivator.stopMotor();
  }

  // ========== Configuration Methods ==========

  @Override
  public void configureMotivatorPID(double kP, double kI, double kD, double kS, double kV) {
    var pidConfig = new SparkFlexConfig();
    pidConfig.closedLoop.pid(kP, kI, kD);
    pidConfig.closedLoop.feedForward.sv(kS, kV, ClosedLoopSlot.kSlot0);
    motivator.configure(
        pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setVelocityTolerance(double enterToleranceRPM, double exitToleranceRPM) {
    this.motivatorEnterToleranceRPM = enterToleranceRPM;
    this.motivatorExitToleranceRPM = exitToleranceRPM;
  }

  @Override
  public double getFFCharacterizationVelocity() {
    return motivatorMotorRPM;
  }
}
