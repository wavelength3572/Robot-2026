package frc.robot.subsystems.launcher;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import java.util.function.DoubleSupplier;
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
  private final PowerDistribution pdh;
  private final RelativeEncoder leaderEncoder;
  private final RelativeEncoder followerEncoder;
  private final SparkClosedLoopController leaderController;

  // Configuration
  private final double gearRatio; // 1 motor rotation = gearRatio wheel rotations

  // Feedforward controller (rebuilt when gains change via configureFeedforward)
  private SimpleMotorFeedforward feedforward;

  // Connection debouncers
  private final Debouncer leaderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Target tracking
  private double currentTargetWheelRPM = 0.0;

  private double leaderVelocityRPM = 0.0;

  // Constants
  private static final double MAX_VELOCITY_RPM = 5000.0; // Max wheel RPM (safety limit)

  // Velocity tolerance for at-setpoint check (set by subsystem via setVelocityTolerance)
  private double velocityToleranceRPM = 50.0;

  public LauncherIOSparkFlex() {
    config = Constants.getRobotConfig();

    // Store gear ratio from config
    gearRatio = config.getLauncherGearRatio();

    // Create feedforward controller with defaults (updated via configureFeedforward)
    double initKs = config.getLauncherKs();
    double initKv = config.getLauncherKv();
    feedforward = new SimpleMotorFeedforward(initKs, initKv);

    // Create PDH for independent current monitoring
    pdh = new PowerDistribution();

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

    // PID control using motor's relative encoder (units: motor RPM)
    // No conversion factors - all conversions done in software
    // Slot 0: Normal PID gains
    // Slot 1: Recovery PID gains (higher P for faster recovery during shooting)
    double initKp = config.getLauncherKp();
    double initKi = config.getLauncherKi();
    double initKd = config.getLauncherKd();
    double initRecoveryKpBoost = 0.0001;
    leaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(initKp, initKi, initKd);
    // .pid(initKp + initRecoveryKpBoost, initKi, initKd, ClosedLoopSlot.kSlot1);

    // Signal update rates
    leaderConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(10);

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
        .voltageCompensation(12.0)
        .follow(leaderMotor, true);

    // Signal update rates for monitoring
    followerConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20);

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
    // Update leader motor inputs
    sparkStickyFault = false;
    ifOk(leaderMotor, leaderEncoder::getVelocity, (value) -> leaderVelocityRPM = value);
    inputs.leaderVelocityRPM = leaderVelocityRPM;
    ifOk(
        leaderMotor,
        new DoubleSupplier[] {leaderMotor::getAppliedOutput, leaderMotor::getBusVoltage},
        (values) -> inputs.leaderAppliedVolts = values[0] * values[1]);
    ifOk(leaderMotor, leaderMotor::getOutputCurrent, (value) -> inputs.leaderCurrentAmps = value);
    inputs.leaderPdhCurrentAmps = pdh.getCurrent(0);
    ifOk(
        leaderMotor, leaderMotor::getMotorTemperature, (value) -> inputs.leaderTempCelsius = value);
    inputs.leaderConnected = leaderConnectedDebounce.calculate(!sparkStickyFault);

    // Update follower motor inputs (for monitoring)
    sparkStickyFault = false;
    ifOk(
        followerMotor, followerEncoder::getVelocity, (value) -> inputs.followerVelocityRPM = value);
    ifOk(
        followerMotor,
        new DoubleSupplier[] {followerMotor::getAppliedOutput, followerMotor::getBusVoltage},
        (values) -> inputs.followerAppliedVolts = values[0] * values[1]);
    ifOk(
        followerMotor,
        followerMotor::getOutputCurrent,
        (value) -> inputs.followerCurrentAmps = value);
    inputs.followerPdhCurrentAmps = pdh.getCurrent(1);
    ifOk(
        followerMotor,
        followerMotor::getMotorTemperature,
        (value) -> inputs.followerTempCelsius = value);
    inputs.followerConnected = followerConnectedDebounce.calculate(!sparkStickyFault);

    // PDH voltage
    inputs.pdhVoltage = pdh.getVoltage();

    // Wheel velocity (convert leader motor RPM to wheel RPM)
    inputs.wheelVelocityRPM = motorToWheelRPM(inputs.leaderVelocityRPM);

    // Target and at-setpoint status
    inputs.targetVelocityRPM = currentTargetWheelRPM;
    inputs.atSetpoint =
        Math.abs(inputs.wheelVelocityRPM - currentTargetWheelRPM) < this.velocityToleranceRPM;
  }

  @Override
  public void setVelocity(double velocityRPM) {
    setVelocityWithBoost(velocityRPM, 0.0, false);
  }

  @Override
  public void setVelocityWithBoost(double velocityRPM, double boostVolts, boolean recoveryActive) {
    // Clamp to max velocity (wheel RPM)
    currentTargetWheelRPM = Math.min(Math.abs(velocityRPM), MAX_VELOCITY_RPM);

    // Convert wheel RPM to motor RPM for the SparkFlex PID controller
    double motorRPM = wheelToMotorRPM(currentTargetWheelRPM);

    // Calculate feedforward using Motor Rotation velocity
    double arbFFVolts = feedforward.calculate(motorRPM);
    double totalFFVolts = arbFFVolts + boostVolts;
    Logger.recordOutput("Launcher/FeedforwardVolts", arbFFVolts);
    Logger.recordOutput("Launcher/TotalFeedforwardVolts", totalFFVolts);

    // Select PID slot: Slot 1 has boosted P for faster recovery during shooting
    ClosedLoopSlot slot = recoveryActive ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot0;
    Logger.recordOutput("Launcher/UsingRecoveryPID", recoveryActive);

    // Command leader with PID + feedforward - follower follows automatically in hardware
    leaderController.setSetpoint(
        motorRPM, ControlType.kVelocity, slot, totalFFVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void setLauncherVoltage(double volts) {
    // Clear velocity target when in voltage mode (for SysId characterization)
    currentTargetWheelRPM = 0.0;

    // Command leader directly - follower follows automatically via hardware follower mode
    // Note: Velocity safety limiting is handled at the command level via .until() conditions
    // rather than here, to avoid oscillations that would corrupt SysId data
    // leaderMotor.setVoltage(volts);
    leaderController.setSetpoint(volts, ControlType.kVoltage);
  }

  @Override
  public void stop() {
    currentTargetWheelRPM = 0.0;
    leaderMotor.stopMotor();
    // Follower stops automatically due to hardware follower mode
  }

  @Override
  public void configurePID(double kP, double kI, double kD, double recoveryKpBoost) {
    var pidConfig = new SparkFlexConfig();
    pidConfig.closedLoop.pid(kP, kI, kD).pid(kP + recoveryKpBoost, kI, kD, ClosedLoopSlot.kSlot1);
    leaderMotor.configure(
        pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void configureFeedforward(double kS, double kV, double kA) {
    feedforward.setKs(kS);
    feedforward.setKv(kV);
    feedforward.setKa(kA);
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
