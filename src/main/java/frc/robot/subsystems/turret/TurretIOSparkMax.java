package frc.robot.subsystems.turret;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * TurretIO implementation for TurretBot using NEO 550 motor with Spark Max and REV Through Bore
 * Encoder for absolute position. Configuration is loaded from the current RobotConfig.
 *
 * <p>NO conversion factors are set on the SparkMax. All encoder readings are raw:
 *
 * <ul>
 *   <li>Motor encoder: position in motor rotations, velocity in RPM
 *   <li>Absolute encoder: position in raw rotations (0.0 to 1.0)
 * </ul>
 *
 * <p>All conversions to turret degrees are done in software for full transparency.
 *
 * <p>The absolute encoder sits AFTER the motor gearbox but BEFORE the external gearing to the
 * turret.
 */
public class TurretIOSparkMax implements TurretIO {
  private final RobotConfig config;

  // Hardware
  private final SparkMax motorSpark;
  private final RelativeEncoder motorEncoder;
  private final AbsoluteEncoder absoluteEncoder;
  private final SparkClosedLoopController motorController;

  // Tunable PID gains
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kD;

  // Configuration values
  private final double maxAngleDegrees;
  private final double minAngleDegrees;
  private final double totalGearRatio;
  private final double externalGearRatio;
  private final double absoluteEncoderOffset;
  private final boolean motorInverted;

  // Connection debouncer
  private final Debouncer motorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Target tracking
  private Rotation2d targetRotation = new Rotation2d();

  // --- Conversion helpers ---
  // All conversions between raw motor rotations and turret degrees happen here.

  /** Convert turret degrees to motor rotations (applies software inversion if configured). */
  private double degreesToMotorRotations(double degrees) {
    double rotations = degrees * totalGearRatio / 360.0;
    return motorInverted ? -rotations : rotations;
  }

  /** Convert motor rotations to turret degrees (applies software inversion if configured). */
  private double motorRotationsToDegrees(double rotations) {
    double degrees = rotations * 360.0 / totalGearRatio;
    return motorInverted ? -degrees : degrees;
  }

  /**
   * Convert raw absolute encoder (0.0-1.0) to turret degrees, applying the zero offset and
   * wrap-around correction. Assumes turret is within ±(wrapDegrees/2) of 0° at startup.
   */
  private double absoluteEncoderToTurretDegrees(double rawAbsValue) {
    // Apply zero offset in software (keep in 0-1 range)
    double adjusted = rawAbsValue - absoluteEncoderOffset;
    adjusted = ((adjusted % 1.0) + 1.0) % 1.0; // Wrap to 0.0-1.0

    // Convert to turret degrees via external gear ratio
    // One full encoder rotation = (360 / externalGearRatio) turret degrees
    double wrapDegrees = 360.0 / externalGearRatio;
    double turretDegrees = adjusted * wrapDegrees;

    // Wrap-around correction: if > half the wrap range, turret is at a negative angle
    if (turretDegrees > wrapDegrees / 2.0) {
      turretDegrees -= wrapDegrees;
    }
    return turretDegrees;
  }

  public TurretIOSparkMax() {
    config = Constants.getRobotConfig();

    // Initialize tunable numbers from config
    kP = new LoggedTunableNumber("Tuning/Turret/kP", config.getTurretKp());
    kD = new LoggedTunableNumber("Tuning/Turret/kD", config.getTurretKd());

    // Store config values
    maxAngleDegrees = config.getTurretMaxAngleDegrees();
    minAngleDegrees = config.getTurretMinAngleDegrees();
    totalGearRatio = config.getTurretGearRatio();
    externalGearRatio = config.getTurretExternalGearRatio();
    absoluteEncoderOffset = config.getTurretAbsoluteEncoderOffset();
    motorInverted = config.getTurretMotorInverted();

    // Create Spark Max controller
    motorSpark = new SparkMax(config.getTurretMotorCanId(), MotorType.kBrushless);

    // Get encoders and controller
    motorEncoder = motorSpark.getEncoder();
    absoluteEncoder = motorSpark.getAbsoluteEncoder();
    motorController = motorSpark.getClosedLoopController();

    // Configure Spark Max - minimal settings only, NO encoder conversion factors
    // Motor inversion handled in software (degreesToMotorRotations / motorRotationsToDegrees)
    // Idle mode set to BRAKE for safety when stopping
    var motorConfig = new SparkMaxConfig();
    motorConfig
        .smartCurrentLimit(config.getTurretCurrentLimitAmps())
        .voltageCompensation(12.0)
        .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    // PID control using motor's relative encoder (units: motor rotations)
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP.get(), 0.0, kD.get());

    // ========== HARDWARE SOFT LIMITS (Critical Safety Feature) ==========
    // These limits are enforced by the SparkMax itself, providing protection even if
    // software bugs out. The motor will refuse to drive past these positions.
    // Units: motor rotations (must convert from turret degrees)
    double forwardLimitMotorRot = degreesToMotorRotations(maxAngleDegrees);
    double reverseLimitMotorRot = degreesToMotorRotations(minAngleDegrees);

    // Ensure forward > reverse (swap if motor is inverted and values got flipped)
    double actualForward = Math.max(forwardLimitMotorRot, reverseLimitMotorRot);
    double actualReverse = Math.min(forwardLimitMotorRot, reverseLimitMotorRot);

    motorConfig
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit((float) actualForward)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit((float) actualReverse);

    // Signal update rates
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

    // --- Seed motor encoder from absolute encoder ---
    // Read raw absolute encoder (0.0-1.0 rotations, no firmware conversions applied)
    double rawAbsValue = absoluteEncoder.getPosition();
    double turretPositionDegrees = absoluteEncoderToTurretDegrees(rawAbsValue);
    double seedMotorRotations = degreesToMotorRotations(turretPositionDegrees);
    tryUntilOk(motorSpark, 5, () -> motorEncoder.setPosition(seedMotorRotations));

    // --- Startup diagnostics ---
    double wrapDegrees = 360.0 / externalGearRatio;
    double roomCW = maxAngleDegrees - turretPositionDegrees;
    double roomCCW = turretPositionDegrees - minAngleDegrees;

    System.out.println();
    System.out.println(
        "+=========================================================================+");
    System.out.println(
        "|              TURRET HARDWARE INITIALIZATION                             |");
    System.out.println(
        "+=========================================================================+");
    System.out.println(
        "|  HARDWARE CONFIG                                                        |");
    System.out.printf(
        "|    CAN ID: %d  |  Gear Ratio: %.1f:1  |  External: %.2f:1            %n",
        config.getTurretMotorCanId(), totalGearRatio, externalGearRatio);
    System.out.printf(
        "|    Encoder wraps every: %.1f deg of turret rotation                   %n", wrapDegrees);
    System.out.println(
        "|                                                                         |");
    System.out.println(
        "|  ENCODER READINGS                                                       |");
    System.out.printf(
        "|    Absolute encoder RAW: %.6f (0.0-1.0)                            %n", rawAbsValue);
    System.out.printf(
        "|    Configured zero offset: %.6f                                     %n",
        absoluteEncoderOffset);
    System.out.println(
        "|                                                                         |");
    System.out.println(
        "|  +---------------------------------------------------------------------+|");
    System.out.printf(
        "|  |  BELIEVED POSITION: %+7.1f deg                                   ||%n",
        turretPositionDegrees);
    System.out.println(
        "|  +---------------------------------------------------------------------+|");
    System.out.println(
        "|                                                                         |");
    System.out.println(
        "|  ROOM TO ROTATE                                                         |");
    System.out.printf(
        "|    -> CW  (positive): %6.1f deg until %+.1f deg limit                %n",
        roomCW, maxAngleDegrees);
    System.out.printf(
        "|    <- CCW (negative): %6.1f deg until %+.1f deg limit               %n",
        roomCCW, minAngleDegrees);
    System.out.println(
        "|                                                                         |");
    System.out.println(
        "|  HARDWARE SOFT LIMITS (SparkMax enforced)                               |");
    System.out.printf(
        "|    Reverse: %.2f motor rot (%.1f deg)                                %n",
        actualReverse, minAngleDegrees);
    System.out.printf(
        "|    Forward: %.2f motor rot (%.1f deg)                                 %n",
        actualForward, maxAngleDegrees);
    System.out.println(
        "|                                                                         |");
    System.out.println(
        "|  *** WARNING: Encoder wraps within travel range!                        |");
    System.out.printf(
        "|      If turret is >%.1f deg from center at startup, position is WRONG %n",
        wrapDegrees / 2.0);
    System.out.println(
        "|      VERIFY physical position matches believed position above!          |");
    System.out.println(
        "+=========================================================================+");
    System.out.println();
    System.out.println("[TurretIOSparkMax] CALIBRATION TIP: To set zero offset, position turret");
    System.out.println("  at physical 0 deg, note the AbsRaw value above, and set that as");
    System.out.println("  turretAbsoluteEncoderOffset in TurretBotConfig.java");
    System.out.println();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Check if PID values have changed and apply new configuration
    if (LoggedTunableNumber.hasChanged(kP, kD)) {
      var pidConfig = new SparkMaxConfig();
      pidConfig.closedLoop.pid(kP.get(), 0.0, kD.get());
      motorSpark.configure(
          pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Read raw motor encoder (motor rotations) and convert to turret degrees in software
    sparkStickyFault = false;
    ifOk(
        motorSpark,
        motorEncoder::getPosition,
        (value) -> inputs.currentAngleDegrees = motorRotationsToDegrees(value));
    inputs.currentAngleRadians = Math.toRadians(inputs.currentAngleDegrees);
    ifOk(
        motorSpark,
        motorEncoder::getVelocity,
        (value) -> inputs.velocityDegreesPerSec = value * 360.0 / totalGearRatio / 60.0);
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

    // Log encoder diagnostics for AdvantageScope (all values shown)
    double rawAbsPos = absoluteEncoder.getPosition();
    double absTurretDegrees = absoluteEncoderToTurretDegrees(rawAbsPos);
    double rawMotorRotations = motorEncoder.getPosition();
    Logger.recordOutput("Turret/Diag/AbsRaw", rawAbsPos);
    Logger.recordOutput("Turret/Diag/AbsTurretDeg", absTurretDegrees);
    Logger.recordOutput("Turret/Diag/MotorRaw", rawMotorRotations);
    Logger.recordOutput("Turret/Diag/MotorTurretDeg", motorRotationsToDegrees(rawMotorRotations));
  }

  @Override
  public void setTargetAngle(Rotation2d rotation) {
    // Clamp the target angle to valid range
    double clampedDegrees =
        Math.max(minAngleDegrees, Math.min(maxAngleDegrees, rotation.getDegrees()));
    targetRotation = Rotation2d.fromDegrees(clampedDegrees);

    // Convert degrees to motor rotations for the PID controller
    motorController.setSetpoint(degreesToMotorRotations(clampedDegrees), ControlType.kPosition);
  }
}
