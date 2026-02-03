package frc.robot.subsystems.motivator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import java.util.function.DoubleSupplier;

/**
 * MotivatorIO implementation using three SparkFlex controllers with NEO Vortex motors. The
 * motivator feeds balls to the launcher.
 *
 * <p>Hardware configuration:
 *
 * <ul>
 *   <li>Motor 55: Main motivator leader
 *   <li>Motor 56: Main motivator follower (inverted, follows leader)
 *   <li>Motor 57: Prefeed roller (independent)
 * </ul>
 */
public class MotivatorIOSparkFlex implements MotivatorIO {
  private final RobotConfig config;

  // Hardware - Main motivator (leader/follower pair)
  private final SparkFlex motivatorLeader;
  private final SparkFlex motivatorFollower;
  private final RelativeEncoder motivatorLeaderEncoder;
  private final RelativeEncoder motivatorFollowerEncoder;

  // Hardware - Prefeed roller (independent)
  private final SparkFlex prefeedMotor;
  private final RelativeEncoder prefeedEncoder;

  // Connection debouncers
  private final Debouncer motivatorLeaderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer motivatorFollowerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer prefeedConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public MotivatorIOSparkFlex() {
    config = Constants.getRobotConfig();

    // Create SparkFlex controllers
    motivatorLeader = new SparkFlex(config.getMotivatorLeaderCanId(), MotorType.kBrushless);
    motivatorFollower = new SparkFlex(config.getMotivatorFollowerCanId(), MotorType.kBrushless);
    prefeedMotor = new SparkFlex(config.getPrefeedCanId(), MotorType.kBrushless);

    // Get encoders
    motivatorLeaderEncoder = motivatorLeader.getEncoder();
    motivatorFollowerEncoder = motivatorFollower.getEncoder();
    prefeedEncoder = prefeedMotor.getEncoder();

    // Configure motivator leader
    var leaderConfig = new SparkFlexConfig();
    leaderConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(config.getMotivatorCurrentLimit())
        .voltageCompensation(12.0);

    leaderConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        motivatorLeader,
        5,
        () ->
            motivatorLeader.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Configure motivator follower (inverted, follows leader)
    var followerConfig = new SparkFlexConfig();
    followerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(config.getMotivatorCurrentLimit())
        .voltageCompensation(12.0)
        .follow(motivatorLeader, true); // Inverted follower

    followerConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        motivatorFollower,
        5,
        () ->
            motivatorFollower.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Configure prefeed motor (independent)
    var prefeedConfig = new SparkFlexConfig();
    prefeedConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(config.getPrefeedCurrentLimit())
        .voltageCompensation(12.0);

    prefeedConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        prefeedMotor,
        5,
        () ->
            prefeedMotor.configure(
                prefeedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Startup diagnostics
    System.out.println("[MotivatorIOSparkFlex] ========== STARTUP ==========");
    System.out.println(
        "[MotivatorIOSparkFlex] Motivator Leader CAN ID: " + config.getMotivatorLeaderCanId());
    System.out.println(
        "[MotivatorIOSparkFlex] Motivator Follower CAN ID: " + config.getMotivatorFollowerCanId());
    System.out.println("[MotivatorIOSparkFlex] Prefeed CAN ID: " + config.getPrefeedCanId());
    System.out.println(
        "[MotivatorIOSparkFlex] Motivator Current Limit: " + config.getMotivatorCurrentLimit());
    System.out.println(
        "[MotivatorIOSparkFlex] Prefeed Current Limit: " + config.getPrefeedCurrentLimit());
    System.out.println("[MotivatorIOSparkFlex] ==============================");
  }

  @Override
  public void updateInputs(MotivatorIOInputs inputs) {
    // Update motivator leader inputs
    sparkStickyFault = false;
    ifOk(
        motivatorLeader,
        motivatorLeaderEncoder::getVelocity,
        (value) -> inputs.motivatorLeaderVelocityRPM = value);
    ifOk(
        motivatorLeader,
        new DoubleSupplier[] {motivatorLeader::getAppliedOutput, motivatorLeader::getBusVoltage},
        (values) -> inputs.motivatorLeaderAppliedVolts = values[0] * values[1]);
    ifOk(
        motivatorLeader,
        motivatorLeader::getOutputCurrent,
        (value) -> inputs.motivatorLeaderCurrentAmps = value);
    ifOk(
        motivatorLeader,
        motivatorLeader::getMotorTemperature,
        (value) -> inputs.motivatorLeaderTempCelsius = value);
    inputs.motivatorLeaderConnected = motivatorLeaderConnectedDebounce.calculate(!sparkStickyFault);

    // Update motivator follower inputs
    sparkStickyFault = false;
    ifOk(
        motivatorFollower,
        motivatorFollowerEncoder::getVelocity,
        (value) -> inputs.motivatorFollowerVelocityRPM = value);
    ifOk(
        motivatorFollower,
        new DoubleSupplier[] {
          motivatorFollower::getAppliedOutput, motivatorFollower::getBusVoltage
        },
        (values) -> inputs.motivatorFollowerAppliedVolts = values[0] * values[1]);
    ifOk(
        motivatorFollower,
        motivatorFollower::getOutputCurrent,
        (value) -> inputs.motivatorFollowerCurrentAmps = value);
    ifOk(
        motivatorFollower,
        motivatorFollower::getMotorTemperature,
        (value) -> inputs.motivatorFollowerTempCelsius = value);
    inputs.motivatorFollowerConnected =
        motivatorFollowerConnectedDebounce.calculate(!sparkStickyFault);

    // Update prefeed inputs
    sparkStickyFault = false;
    ifOk(prefeedMotor, prefeedEncoder::getVelocity, (value) -> inputs.prefeedVelocityRPM = value);
    ifOk(
        prefeedMotor,
        new DoubleSupplier[] {prefeedMotor::getAppliedOutput, prefeedMotor::getBusVoltage},
        (values) -> inputs.prefeedAppliedVolts = values[0] * values[1]);
    ifOk(
        prefeedMotor, prefeedMotor::getOutputCurrent, (value) -> inputs.prefeedCurrentAmps = value);
    ifOk(
        prefeedMotor,
        prefeedMotor::getMotorTemperature,
        (value) -> inputs.prefeedTempCelsius = value);
    inputs.prefeedConnected = prefeedConnectedDebounce.calculate(!sparkStickyFault);

    // Future: ball detection sensor
    inputs.ballDetected = false;
  }

  @Override
  public void setMotivatorDutyCycle(double dutyCycle) {
    motivatorLeader.set(dutyCycle);
    // Follower follows automatically in hardware
  }

  @Override
  public void setPrefeedDutyCycle(double dutyCycle) {
    prefeedMotor.set(dutyCycle);
  }

  @Override
  public void stop() {
    motivatorLeader.stopMotor();
    prefeedMotor.stopMotor();
  }

  @Override
  public void stopMotivator() {
    motivatorLeader.stopMotor();
  }

  @Override
  public void stopPrefeed() {
    prefeedMotor.stopMotor();
  }
}
