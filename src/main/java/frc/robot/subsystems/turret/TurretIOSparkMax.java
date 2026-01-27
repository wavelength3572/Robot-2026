package frc.robot.subsystems.turret;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

/**
 * TurretIO implementation for TurretBot using NEO 550 motor with Spark Max and REV Through Bore
 * Encoder for absolute position. Configuration is loaded from the current RobotConfig.
 *
 * <p>Hardware setup:
 *
 * <ul>
 *   <li>Single Spark Max with NEO 550 motor + Through Bore Encoder via data port (Gadgeteer)
 * </ul>
 *
 * <p>The absolute encoder sits AFTER the motor gearbox but BEFORE the external gearing to the
 * turret.
 */
public class TurretIOSparkMax implements TurretIO {
  private final RobotConfig config;

  // Hardware - single Spark Max with motor and absolute encoder on data port
  private final SparkMax motorSpark;
  private final RelativeEncoder motorEncoder;
  private final AbsoluteEncoder absoluteEncoder;
  private final SparkClosedLoopController motorController;

  // Tunable PID gains - initialized from RobotConfig
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;

  // Configuration values from RobotConfig
  private final double maxAngleDegrees;
  private final double minAngleDegrees;
  private final double degreesPerMotorRotation;

  // Connection debouncer
  private final Debouncer motorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Target tracking
  private Rotation2d targetRotation = new Rotation2d();

  // Diagnostic logging
  private int diagnosticCounter = 0;
  private static final int DIAGNOSTIC_INTERVAL = 50; // Log every 50 cycles (1 second at 50Hz)

  public TurretIOSparkMax() {
    config = Constants.getRobotConfig();

    // Initialize tunable numbers from config
    kP = new LoggedTunableNumber("Turret/kP", config.getTurretKp());
    kI = new LoggedTunableNumber("Turret/kI", config.getTurretKi());
    kD = new LoggedTunableNumber("Turret/kD", config.getTurretKd());

    // Store angle limits
    maxAngleDegrees = config.getTurretMaxAngleDegrees();
    minAngleDegrees = config.getTurretMinAngleDegrees();

    // Position conversion: degrees per motor rotation
    // Motor rotation * (360 degrees / gear ratio) = turret degrees
    degreesPerMotorRotation = 360.0 / config.getTurretGearRatio();

    // Create Spark Max controller
    motorSpark = new SparkMax(config.getTurretMotorCanId(), MotorType.kBrushless);

    // Get encoders and controller
    // The absolute encoder is connected via the data port (Gadgeteer) on the same Spark Max
    motorEncoder = motorSpark.getEncoder();
    absoluteEncoder = motorSpark.getAbsoluteEncoder();
    motorController = motorSpark.getClosedLoopController();

    // Configure Spark Max
    var motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(config.getTurretMotorInverted())
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(config.getTurretCurrentLimitAmps())
        .voltageCompensation(12.0);

    // Configure motor's relative encoder
    motorConfig
        .encoder
        // Position in degrees: motor rotations * (360 / gear ratio)
        .positionConversionFactor(degreesPerMotorRotation)
        // Velocity in degrees per second: motor RPM * (360 / gear ratio) / 60
        .velocityConversionFactor(degreesPerMotorRotation / 60.0);

    // Configure absolute encoder (Through Bore Encoder on data port)
    // The encoder is after the motor gearbox, so encoder rotations map to degrees via external
    // ratio
    // Encoder rotation * (360 / external_ratio) = turret degrees
    double externalRatio = config.getTurretExternalGearRatio();
    motorConfig
        .absoluteEncoder
        .positionConversionFactor(360.0 / externalRatio)
        .velocityConversionFactor(360.0 / externalRatio / 60.0)
        .inverted(config.getTurretEncoderInverted())
        .zeroOffset(0.0); // Calibrate this to set the zero position

    // Configure closed-loop control using motor's relative encoder
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kP.get(), kI.get(), kD.get(), 0.0);

    // Soft limits in degrees
    motorConfig
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit((float) maxAngleDegrees)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit((float) minAngleDegrees);

    // Configure signal update rates
    motorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        motorSpark,
        5,
        () ->
            motorSpark.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Seed the motor's relative encoder from the absolute encoder
    // The absolute encoder reads in degrees (after our conversion factor)
    // The motor encoder also reads in degrees (after our conversion factor)
    double absolutePositionDegrees = absoluteEncoder.getPosition();
    tryUntilOk(motorSpark, 5, () -> motorEncoder.setPosition(absolutePositionDegrees));

    double externalRatioForLog = config.getTurretExternalGearRatio();
    System.out.println("[TurretIOSparkMax] ========== STARTUP DIAGNOSTICS ==========");
    System.out.println("[TurretIOSparkMax] CAN ID: " + config.getTurretMotorCanId());
    System.out.println(
        "[TurretIOSparkMax] Total gear ratio (motor to turret): "
            + config.getTurretGearRatio()
            + ":1");
    System.out.println(
        "[TurretIOSparkMax] External gear ratio (encoder to turret): "
            + externalRatioForLog
            + ":1");
    System.out.println(
        "[TurretIOSparkMax] Encoder wraps every: "
            + String.format("%.1f", 360.0 / externalRatioForLog)
            + "° of turret rotation");
    System.out.println(
        "[TurretIOSparkMax] Absolute encoder position (converted): "
            + String.format("%.2f", absolutePositionDegrees)
            + "°");
    System.out.println(
        "[TurretIOSparkMax] Motor encoder seeded to: "
            + String.format("%.2f", absolutePositionDegrees)
            + "°");
    System.out.println(
        "[TurretIOSparkMax] Travel limits: " + minAngleDegrees + "° to " + maxAngleDegrees + "°");
    System.out.println("[TurretIOSparkMax] Current zeroOffset: 0.0 (set in config)");
    System.out.println("[TurretIOSparkMax] ===========================================");
    System.out.println(
        "[TurretIOSparkMax] To calibrate: Move turret to physical 0°, note the AbsEnc value,");
    System.out.println(
        "[TurretIOSparkMax] then set .zeroOffset() to that value in TurretIOSparkMax.java line 113");
    System.out.println("[TurretIOSparkMax] ===========================================");
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Check if PID values have changed and apply new configuration
    if (LoggedTunableNumber.hasChanged(kP, kI, kD)) {
      var pidConfig = new SparkMaxConfig();
      pidConfig.closedLoop.pidf(kP.get(), kI.get(), kD.get(), 0.0);
      motorSpark.configure(
          pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Update motor inputs
    sparkStickyFault = false;
    ifOk(motorSpark, motorEncoder::getPosition, (value) -> inputs.currentAngleDegrees = value);
    inputs.currentAngleRadians = Math.toRadians(inputs.currentAngleDegrees);
    ifOk(motorSpark, motorEncoder::getVelocity, (value) -> inputs.velocityDegreesPerSec = value);
    ifOk(
        motorSpark,
        new DoubleSupplier[] {motorSpark::getAppliedOutput, motorSpark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(motorSpark, motorSpark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    boolean motorOk = motorConnectedDebounce.calculate(!sparkStickyFault);

    // Update target angle
    inputs.targetAngleDegrees = targetRotation.getDegrees();
    inputs.targetAngleRadians = targetRotation.getRadians();

    // Log connection status
    if (!motorOk) {
      System.err.println("[TurretIOSparkMax] Spark Max disconnected!");
    }

    // Periodic diagnostic logging (every ~1 second)
    diagnosticCounter++;
    if (diagnosticCounter >= DIAGNOSTIC_INTERVAL) {
      diagnosticCounter = 0;
      double rawAbsolutePos = absoluteEncoder.getPosition();
      double motorEncoderPos = motorEncoder.getPosition();
      System.out.println(
          "[TurretDiag] AbsEnc(converted): "
              + String.format("%.2f", rawAbsolutePos)
              + "° | MotorEnc: "
              + String.format("%.2f", motorEncoderPos)
              + "° | Target: "
              + String.format("%.2f", targetRotation.getDegrees())
              + "°");
    }
  }

  @Override
  public void setTargetAngle(Rotation2d rotation) {
    // Clamp the target angle to valid range
    double clampedDegrees =
        Math.max(minAngleDegrees, Math.min(maxAngleDegrees, rotation.getDegrees()));
    targetRotation = Rotation2d.fromDegrees(clampedDegrees);

    // Set position reference in degrees (motor encoder is configured for degrees)
    motorController.setReference(clampedDegrees, ControlType.kPosition);
  }
}
