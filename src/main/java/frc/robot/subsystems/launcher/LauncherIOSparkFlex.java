package frc.robot.subsystems.launcher;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

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
  private final LoggedTunableNumber kFF;

  // Connection debouncers
  private final Debouncer leaderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Target tracking
  private double currentTargetWheelRPM = 0.0;

  // Constants
  private static final double MAX_VELOCITY_RPM = 6000.0; // Max wheel RPM
  private static final double VELOCITY_TOLERANCE_RPM = 50.0; // At-setpoint tolerance

  public LauncherIOSparkFlex() {
    config = Constants.getRobotConfig();

    // Store gear ratio from config
    gearRatio = config.getLauncherGearRatio();

    // Initialize tunable numbers from config
    kP = new LoggedTunableNumber("Launcher/kP", config.getLauncherKp());
    kI = new LoggedTunableNumber("Launcher/kI", config.getLauncherKi());
    kD = new LoggedTunableNumber("Launcher/kD", config.getLauncherKd());
    kFF = new LoggedTunableNumber("Launcher/kFF", config.getLauncherKff());

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
    leaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kP.get(), kI.get(), kD.get(), kFF.get());

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
    // Leader is inverted(true), follower is inverted(false) due to opposite physical orientation
    // Follower inverts leader's output to match directions
    var followerConfig = new SparkFlexConfig();
    followerConfig
        .inverted(false)
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
    // Check for tunable PID changes and apply
    if (LoggedTunableNumber.hasChanged(kP, kI, kD, kFF)) {
      var pidConfig = new SparkFlexConfig();
      pidConfig.closedLoop.pidf(kP.get(), kI.get(), kD.get(), kFF.get());
      leaderMotor.configure(
          pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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
        Math.abs(inputs.wheelVelocityRPM - currentTargetWheelRPM) < VELOCITY_TOLERANCE_RPM;
  }

  @Override
  public void setVelocity(double velocityRPM) {
    // Clamp to max velocity (wheel RPM)
    currentTargetWheelRPM = Math.min(Math.abs(velocityRPM), MAX_VELOCITY_RPM);

    // Convert wheel RPM to motor RPM for the controller
    double motorRPM = wheelToMotorRPM(currentTargetWheelRPM);

    // Command leader only - follower follows automatically in hardware
    leaderController.setReference(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void stop() {
    currentTargetWheelRPM = 0.0;
    leaderMotor.stopMotor();
    // Follower stops automatically due to hardware follower mode
  }
}
