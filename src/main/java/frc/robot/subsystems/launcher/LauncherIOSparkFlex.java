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
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
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
  private final RelativeEncoder leaderEncoder;
  private final RelativeEncoder followerEncoder;
  private final SparkClosedLoopController leaderController;

  // Configuration
  private final double gearRatio; // 1 motor rotation = gearRatio wheel rotations

  // Tunable PID gains
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  // Tunable feedforward gains (from SysId characterization)
  // kS = static friction voltage, kV = velocity voltage (V/(rad/s)), kA = acceleration voltage
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;

  // Feedforward controller (rebuilt when gains change)
  private SimpleMotorFeedforward feedforward;

  // Connection debouncers
  private final Debouncer leaderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Target tracking
  private double currentTargetWheelRPM = 0.0;

  // Constants
  private static final double MAX_VELOCITY_RPM = 7000.0; // Max wheel RPM (safety limit)

  // Shared tunable tolerance (same instance as LauncherIOSim for consistency)
  private static final LoggedTunableNumber velocityToleranceRPM =
      new LoggedTunableNumber("Tuning/Launcher/VelocityToleranceRPM", 50.0);

  // Recovery P boost - extra P gain added to Slot 1 for faster recovery during shooting
  private final LoggedTunableNumber recoveryKpBoost =
      new LoggedTunableNumber("Tuning/Launcher/RecoveryKpBoost", 0.0001);

  public LauncherIOSparkFlex() {
    config = Constants.getRobotConfig();

    // Store gear ratio from config
    gearRatio = config.getLauncherGearRatio();

    // Initialize tunable PID gains from config
    // kFF is 0 because we use SysId feedforward (kS, kV, kA) instead
    kP = new LoggedTunableNumber("Tuning/Launcher/kP", config.getLauncherKp());
    kI = new LoggedTunableNumber("Tuning/Launcher/kI", config.getLauncherKi());
    kD = new LoggedTunableNumber("Tuning/Launcher/kD", config.getLauncherKd());

    // SysId feedforward gains (from characterization)
    // Units: kS = volts, kV = volts per (rad/s), kA = volts per (rad/s^2)
    kS = new LoggedTunableNumber("Tuning/Launcher/kS", 0.29);
    kV = new LoggedTunableNumber("Tuning/Launcher/kV", 0.0165);
    kA = new LoggedTunableNumber("Tuning/Launcher/kA", 0.003);

    // Create feedforward controller
    feedforward = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());

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
    leaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP.get(), kI.get(), kD.get())
        .pid(kP.get() + recoveryKpBoost.get(), kI.get(), kD.get(), ClosedLoopSlot.kSlot1);

    // Signal update rates
    leaderConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

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
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

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
    // Check for tunable PID/FF changes and apply to both slots
    if (LoggedTunableNumber.hasChanged(kP, kI, kD, recoveryKpBoost)) {
      var pidConfig = new SparkFlexConfig();
      pidConfig
          .closedLoop
          .pid(kP.get(), kI.get(), kD.get())
          .pid(kP.get() + recoveryKpBoost.get(), kI.get(), kD.get(), ClosedLoopSlot.kSlot1);
      leaderMotor.configure(
          pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Check for tunable feedforward changes and rebuild
    if (LoggedTunableNumber.hasChanged(kS, kV, kA)) {
      feedforward = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
    }

    // Update leader motor inputs
    sparkStickyFault = false;
    ifOk(leaderMotor, leaderEncoder::getVelocity, (value) -> inputs.leaderVelocityRPM = value);
    ifOk(
        leaderMotor,
        new DoubleSupplier[] {leaderMotor::getAppliedOutput, leaderMotor::getBusVoltage},
        (values) -> inputs.leaderAppliedVolts = values[0] * values[1]);
    ifOk(leaderMotor, leaderMotor::getOutputCurrent, (value) -> inputs.leaderCurrentAmps = value);
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
    ifOk(
        followerMotor,
        followerMotor::getMotorTemperature,
        (value) -> inputs.followerTempCelsius = value);
    inputs.followerConnected = followerConnectedDebounce.calculate(!sparkStickyFault);

    // Wheel velocity (convert leader motor RPM to wheel RPM)
    inputs.wheelVelocityRPM = motorToWheelRPM(inputs.leaderVelocityRPM);

    // Target and at-setpoint status
    inputs.targetVelocityRPM = currentTargetWheelRPM;
    inputs.atSetpoint =
        Math.abs(inputs.wheelVelocityRPM - currentTargetWheelRPM) < velocityToleranceRPM.get();
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

    // Calculate feedforward using WHEEL velocity (SysId was characterized with wheel velocity)
    double wheelRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(currentTargetWheelRPM);
    double arbFFVolts = feedforward.calculate(wheelRadPerSec);
    double totalFFVolts = arbFFVolts + boostVolts;
    Logger.recordOutput("Shooter/Launcher/FeedforwardVolts", arbFFVolts);
    Logger.recordOutput("Shooter/Launcher/TotalFeedforwardVolts", totalFFVolts);

    // Select PID slot: Slot 1 has boosted P for faster recovery during shooting
    ClosedLoopSlot slot = recoveryActive ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot0;
    Logger.recordOutput("Shooter/Launcher/UsingRecoveryPID", recoveryActive);

    // Command leader with PID + feedforward - follower follows automatically in hardware
    leaderController.setSetpoint(
        motorRPM, ControlType.kVelocity, slot, totalFFVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void setVoltage(double volts) {
    // Clear velocity target when in voltage mode (for SysId characterization)
    currentTargetWheelRPM = 0.0;

    // Command leader directly - follower follows automatically via hardware follower mode
    // Note: Velocity safety limiting is handled at the command level via .until() conditions
    // rather than here, to avoid oscillations that would corrupt SysId data
    leaderMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    currentTargetWheelRPM = 0.0;
    leaderMotor.stopMotor();
    // Follower stops automatically due to hardware follower mode
  }
}
