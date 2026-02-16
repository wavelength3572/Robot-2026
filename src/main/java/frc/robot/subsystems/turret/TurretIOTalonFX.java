package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;

/**
 * TurretIO implementation for TalonFX motor controller. Used by SquareBot. Configuration is loaded
 * from the current RobotConfig.
 */
public class TurretIOTalonFX implements TurretIO {
  private final TalonFX turretMotor;
  private final RobotConfig config;

  // Tunable PID gains - initialized from RobotConfig
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;

  // Angle limits from config
  private final double maxAngleDegrees;
  private final double minAngleDegrees;

  // Status signals (cached for efficiency)
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> statorCurrent;

  // Control request
  private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(true);
  private Rotation2d targetRotation = new Rotation2d();

  public TurretIOTalonFX() {
    config = Constants.getRobotConfig();

    // Initialize tunable numbers from config
    kP = new LoggedTunableNumber("Tuning/Turret/kP", config.getTurretKp());
    kI = new LoggedTunableNumber("Tuning/Turret/kI", config.getTurretKi());
    kD = new LoggedTunableNumber("Tuning/Turret/kD", config.getTurretKd());
    kS = new LoggedTunableNumber("Tuning/Turret/kS", 0.0);
    kV = new LoggedTunableNumber("Tuning/Turret/kV", config.getTurretKff());

    // Store angle limits
    maxAngleDegrees = config.getTurretMaxAngleDegrees();
    minAngleDegrees = config.getTurretMinAngleDegrees();

    // Create motor with CAN ID from config
    turretMotor = new TalonFX(config.getTurretMotorCanId());

    // Configure the motor
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // Motor output configuration
    motorConfig.MotorOutput.Inverted =
        config.getTurretMotorInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Feedback configuration - gear ratio converts motor rotations to mechanism rotations
    motorConfig.Feedback.SensorToMechanismRatio = config.getTurretGearRatio();

    // PID configuration (Slot 0) - uses tunable values
    motorConfig.Slot0.kP = kP.get();
    motorConfig.Slot0.kI = kI.get();
    motorConfig.Slot0.kD = kD.get();
    motorConfig.Slot0.kS = kS.get();
    motorConfig.Slot0.kV = kV.get();

    // IMPORTANT: Disable continuous wrap since turret has limited travel
    // With ContinuousWrap=false, the controller won't try to wrap through ±180°
    motorConfig.ClosedLoopGeneral.ContinuousWrap = false;

    // Software soft limits - prevent turret from exceeding physical travel limits
    // These values are in mechanism rotations (after SensorToMechanismRatio is applied)
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxAngleDegrees / 360.0;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minAngleDegrees / 360.0;

    // Current limits
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = config.getTurretCurrentLimitAmps();
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = config.getTurretCurrentLimitAmps() + 10;

    // Apply configuration
    turretMotor.getConfigurator().apply(motorConfig);

    // Cache status signals
    position = turretMotor.getPosition();
    velocity = turretMotor.getVelocity();
    appliedVoltage = turretMotor.getMotorVoltage();
    statorCurrent = turretMotor.getStatorCurrent();

    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVoltage, statorCurrent);
    turretMotor.optimizeBusUtilization();

    System.out.println(
        "[TurretIOTalonFX] Initialized with CAN ID "
            + config.getTurretMotorCanId()
            + ", gear ratio "
            + config.getTurretGearRatio());
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Check if PID values have changed and apply new configuration
    if (LoggedTunableNumber.hasChanged(kP, kI, kD, kS, kV)) {
      Slot0Configs slot0 = new Slot0Configs();
      slot0.kP = kP.get();
      slot0.kI = kI.get();
      slot0.kD = kD.get();
      slot0.kS = kS.get();
      slot0.kV = kV.get();
      turretMotor.getConfigurator().apply(slot0);
    }

    // Refresh all status signals
    BaseStatusSignal.refreshAll(position, velocity, appliedVoltage, statorCurrent);

    // Position is in rotations, convert to degrees and radians
    double positionRotations = position.getValueAsDouble();
    inputs.currentAngleDegrees = positionRotations * 360.0;
    inputs.currentAngleRadians = positionRotations * 2.0 * Math.PI;

    // Target angle
    inputs.targetAngleDegrees = targetRotation.getDegrees();
    inputs.targetAngleRadians = targetRotation.getRadians();

    // Velocity is in rotations per second, convert to degrees per second
    inputs.velocityDegreesPerSec = velocity.getValueAsDouble() * 360.0;

    // Voltage and current
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.currentAmps = statorCurrent.getValueAsDouble();
  }

  @Override
  public void setTurretAngle(Rotation2d rotation) {
    // Clamp the target angle to valid range before commanding
    double clampedDegrees =
        Math.max(minAngleDegrees, Math.min(maxAngleDegrees, rotation.getDegrees()));
    targetRotation = Rotation2d.fromDegrees(clampedDegrees);

    // Convert rotation to rotations (TalonFX position unit)
    double targetRotations = targetRotation.getRotations();
    turretMotor.setControl(positionRequest.withPosition(targetRotations));
  }
}
