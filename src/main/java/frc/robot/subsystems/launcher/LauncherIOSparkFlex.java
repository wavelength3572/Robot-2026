package frc.robot.subsystems.launcher;

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
import org.littletonrobotics.junction.Logger;

/**
 * LauncherIO implementation using two SparkFlex controllers with NEO Vortex motors. The follower
 * motor uses hardware follower mode with inverted output since the motors face opposite directions
 * on the same shaft.
 *
 * <p>Gear ratio: 1 motor rotation = 1.5 wheel rotations (wheel spins faster than motor)
 *
 * <p>All encoder readings are raw motor RPM. Conversions to wheel RPM are done in software.
 */
public class LauncherIOSparkFlex implements LauncherIO {
  private final RobotConfig config;

  // Hardware
  private final SparkFlex leaderMotor;
  private final SparkFlex followerMotor;
  private final RelativeEncoder leaderEncoder;
  private final RelativeEncoder followerEncoder;
  private final SparkClosedLoopController leaderController;

  // Configuration
  private final double gearRatio; // 1 motor rotation = gearRatio wheel rotations

  // Connection debouncers
  private final Debouncer leaderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Skip CAN reads when motors are disconnected to prevent loop overruns
  private final SparkConnection leaderConnection = new SparkConnection();
  private final SparkConnection followerConnection = new SparkConnection();

  // Target tracking
  private double currentTargetWheelRPM = 0.0;

  private double leaderVelocityRPM = 0.0;
  private int periodicCounter = 0;

  // Constants
  private static final double MAX_VELOCITY_RPM = 5000.0; // Max wheel RPM (safety limit)

  // Velocity tolerance for at-setpoint check (set by subsystem via
  // setVelocityTolerance)
  private double velocityToleranceRPM = 50.0;

  public LauncherIOSparkFlex() {
    config = Constants.getRobotConfig();

    // Store gear ratio from config
    gearRatio = config.getLauncherGearRatio();

    // Create SparkFlex controllers
    leaderMotor = new SparkFlex(config.getLauncherLeaderCanId(), MotorType.kBrushless);
    followerMotor = new SparkFlex(config.getLauncherFollowerCanId(), MotorType.kBrushless);

    // Get encoders and controller
    leaderEncoder = leaderMotor.getEncoder();
    followerEncoder = followerMotor.getEncoder();
    leaderController = leaderMotor.getClosedLoopController();

    // Configure leader motor
    var leaderConfig = new SparkFlexConfig();
    leaderConfig
        .inverted(true) // Leader is inverted to match launch direction
        .idleMode(IdleMode.kCoast) // Coast for flywheels
        .smartCurrentLimit(config.getLauncherCurrentLimitAmps())
        .voltageCompensation(12.0);

    // Reduce encoder velocity filter lag for faster PID response.
    // REV defaults use a 64-tap FIR filter (~164ms window, ~82ms phase lag) which
    // is
    // far too sluggish for flywheel recovery. These settings reduce the effective
    // measurement window to ~16ms (~8ms phase lag), giving the 1kHz onboard PID
    // much fresher velocity data to work with.
    // See:
    // https://www.chiefdelphi.com/t/psa-rev-spark-default-velocity-filtering-is-still-really-bad-for-flywheels/514567
    leaderConfig.encoder.uvwMeasurementPeriod(8).uvwAverageDepth(2);

    // PID + feedforward all run onboard the SparkFlex at 1kHz — no CAN latency.
    // kS/kV applied automatically in kVelocity mode (per REV 2026 API).
    // Slot 0: Normal PID gains (gentle, for steady-state holding)
    // Slot 1: Recovery PID gains (aggressive kP + kD for fast recovery after ball impacts)
    double initKp = config.getLauncherKp();
    double initKi = config.getLauncherKi();
    double initKd = config.getLauncherKd();
    // Recovery slot: 8x kP for aggressive correction, add kD for damping
    double recoveryKp = initKp * 8.0;
    double recoveryKd = 0.0004;
    leaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(initKp, initKi, initKd, ClosedLoopSlot.kSlot0)
        .pid(recoveryKp, initKi, recoveryKd, ClosedLoopSlot.kSlot1)
        .iZone(config.getLauncherIZone());
    leaderConfig
        .closedLoop
        .feedForward
        .sv(config.getLauncherKs(), config.getLauncherKv(), ClosedLoopSlot.kSlot0)
        .sv(config.getLauncherKs(), config.getLauncherKv(), ClosedLoopSlot.kSlot1);

    // Signal update rates — 10ms velocity for tighter onboard PID recovery.
    // The 1kHz PID interpolates between updates, so faster velocity data
    // means the controller reacts to ball impacts within ~5ms instead of ~10ms.
    leaderConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(10)
        .appliedOutputPeriodMs(5)
        .busVoltagePeriodMs(5)
        .outputCurrentPeriodMs(5);

    tryUntilOk(
        leaderMotor,
        5,
        () ->
            leaderMotor.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Configure follower motor
    // In hardware follower mode, only the follow() inversion parameter matters
    // follow(leader, true) means follower inverts leader's output
    var followerConfig = new SparkFlexConfig();
    followerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(config.getLauncherCurrentLimitAmps())
        .disableVoltageCompensation()
        .follow(leaderMotor, true);

    followerConfig.encoder.uvwMeasurementPeriod(8).uvwAverageDepth(2);

    // Signal update rates for monitoring
    followerConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(100)
        .busVoltagePeriodMs(100)
        .outputCurrentPeriodMs(100);

    tryUntilOk(
        followerMotor,
        5,
        () ->
            followerMotor.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Startup diagnostics
    System.out.println("[LauncherIOSparkFlex] ========== STARTUP ==========");
    System.out.println("[LauncherIOSparkFlex] Leader CAN ID: " + config.getLauncherLeaderCanId());
    System.out.println(
        "[LauncherIOSparkFlex] Follower CAN ID: " + config.getLauncherFollowerCanId());
    System.out.println(
        "[LauncherIOSparkFlex] Gear ratio: 1 motor rotation = " + gearRatio + " wheel rotations");
    System.out.println("[LauncherIOSparkFlex] Follower inverted: true (hardware follower mode)");
    System.out.println(
        "[LauncherIOSparkFlex] Current limit: " + config.getLauncherCurrentLimitAmps() + "A");
    System.out.println("[LauncherIOSparkFlex] =============================");
  }

  /** Convert motor RPM to wheel RPM */
  private double motorToWheelRPM(double motorRPM) {
    return motorRPM * gearRatio;
  }

  /** Convert wheel RPM to motor RPM */
  private double wheelToMotorRPM(double wheelRPM) {
    return wheelRPM / gearRatio;
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    // Update leader motor inputs (skip reads if disconnected to prevent timeout
    // delays)
    if (!leaderConnection.isSkipping()) {
      sparkStickyFault = false;
      ifOk(leaderMotor, leaderEncoder::getVelocity, (value) -> leaderVelocityRPM = value);
      inputs.leaderVelocityRPM = leaderVelocityRPM;
      ifOk(
          leaderMotor,
          leaderMotor::getAppliedOutput,
          (value) -> inputs.leaderAppliedOutput = value);
      ifOk(leaderMotor, leaderMotor::getBusVoltage, (value) -> inputs.leaderBusVoltage = value);
      inputs.leaderAppliedVolts = inputs.leaderAppliedOutput * inputs.leaderBusVoltage;
      ifOk(leaderMotor, leaderMotor::getOutputCurrent, (value) -> inputs.leaderCurrentAmps = value);
      ifOk(leaderMotor, leaderController::getIAccum, (value) -> inputs.iAccum = value);
      inputs.leaderConnected = leaderConnectedDebounce.calculate(!sparkStickyFault);
      leaderConnection.update(inputs.leaderConnected);
    } else {
      inputs.leaderConnected = false;
    }

    // Update follower motor inputs (skip reads if disconnected to prevent timeout
    // delays)
    if (!followerConnection.isSkipping()) {
      sparkStickyFault = false;
      ifOk(
          followerMotor,
          followerEncoder::getVelocity,
          (value) -> inputs.followerVelocityRPM = value);
      ifOk(
          followerMotor,
          followerMotor::getAppliedOutput,
          (value) -> inputs.followerAppliedOutput = value);
      ifOk(
          followerMotor,
          followerMotor::getBusVoltage,
          (value) -> inputs.followerBusVoltage = value);
      inputs.followerAppliedVolts = inputs.followerAppliedOutput * inputs.followerBusVoltage;
      ifOk(
          followerMotor,
          followerMotor::getOutputCurrent,
          (value) -> inputs.followerCurrentAmps = value);
      inputs.followerConnected = followerConnectedDebounce.calculate(!sparkStickyFault);
      followerConnection.update(inputs.followerConnected);
    } else {
      inputs.followerConnected = false;
    }

    // Throttle temperature reads to every ~1 second (50 cycles at 20ms)
    if (++periodicCounter >= 50) {
      periodicCounter = 0;
      if (leaderConnection.isConnected()) {
        ifOk(
            leaderMotor,
            leaderMotor::getMotorTemperature,
            (value) -> inputs.leaderTempCelsius = value);
      }
      if (followerConnection.isConnected()) {
        ifOk(
            followerMotor,
            followerMotor::getMotorTemperature,
            (value) -> inputs.followerTempCelsius = value);
      }
    }

    // Wheel velocity (convert leader motor RPM to wheel RPM)
    inputs.wheelVelocityRPM = motorToWheelRPM(inputs.leaderVelocityRPM);

    // Target and at-setpoint status
    inputs.targetVelocityRPM = currentTargetWheelRPM;
    inputs.leaderTargetRPM = wheelToMotorRPM(currentTargetWheelRPM);
    inputs.atSetpoint =
        Math.abs(inputs.wheelVelocityRPM - currentTargetWheelRPM) < this.velocityToleranceRPM;
  }

  @Override
  public void setVelocity(double velocityRPM, boolean recoveryActive, double recoveryArbFF) {
    if (velocityRPM < 1.0) {
      currentTargetWheelRPM = 0.0;
      leaderMotor.stopMotor();
    }
    // Clamp to max velocity (wheel RPM)
    currentTargetWheelRPM = Math.min(Math.abs(velocityRPM), MAX_VELOCITY_RPM);

    // Convert wheel RPM to motor RPM for the SparkFlex PID controller
    double motorRPM = wheelToMotorRPM(currentTargetWheelRPM);

    // Select PID slot: Slot 1 has boosted kP for faster recovery during shooting
    ClosedLoopSlot slot = recoveryActive ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot0;
    Logger.recordOutput("Launcher/RecoveryActive", recoveryActive);
    Logger.recordOutput("Launcher/RecoveryArbFF", recoveryArbFF);

    // kVelocity with onboard kS/kV: PID + FF + arbFF all run at 1kHz on SparkFlex.
    // recoveryArbFF is pre-computed by Launcher.java proportional to target RPM.
    leaderController.setSetpoint(motorRPM, ControlType.kVelocity, slot, recoveryArbFF);
  }

  @Override
  public void setLauncherVoltage(double volts) {
    // Clear velocity target when in voltage mode (for SysId characterization)
    currentTargetWheelRPM = 0.0;

    // Command leader directly - follower follows automatically via hardware
    // follower mode
    leaderController.setSetpoint(volts, ControlType.kVoltage);
  }

  @Override
  public void stop() {
    currentTargetWheelRPM = 0.0;
    leaderMotor.stopMotor();
    // Follower stops automatically due to hardware follower mode
  }

  @Override
  public void configurePID(double kP, double kI, double kD, double iZone) {
    // Recovery slot gets 8x kP + derivative damping for fast flywheel recovery
    var pidConfig = new SparkFlexConfig();
    pidConfig
        .closedLoop
        .pid(kP, kI, kD, ClosedLoopSlot.kSlot0)
        .pid(kP * 8.0, kI, 0.0004, ClosedLoopSlot.kSlot1)
        .iZone(iZone);
    leaderMotor.configure(
        pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void configureFeedforward(double kS, double kV) {
    var ffConfig = new SparkFlexConfig();
    ffConfig.closedLoop.feedForward.sv(kS, kV, ClosedLoopSlot.kSlot0);
    ffConfig.closedLoop.feedForward.sv(kS, kV, ClosedLoopSlot.kSlot1);
    leaderMotor.configure(
        ffConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setVelocityTolerance(double toleranceRPM) {
    this.velocityToleranceRPM = toleranceRPM;
  }

  @Override
  public double getFFCharacterizationVelocity() {
    return leaderVelocityRPM;
  }
}
