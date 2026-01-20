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
import frc.robot.util.LoggedTunableNumber;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX turretMotor;

  // Tunable PID gains - adjust these from /Tuning/ in NetworkTables
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Turret/kP", TurretConstants.kP);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", 0.0);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Turret/kD", TurretConstants.kD);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", 0.0);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Turret/kV", TurretConstants.kFF);

  // Status signals (cached for efficiency)
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> statorCurrent;

  // Control request
  private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(true);
  private Rotation2d targetRotation = new Rotation2d();

  public TurretIOTalonFX() {
    turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_CAN_ID);

    // Configure the motor
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor output configuration
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Feedback configuration - gear ratio converts motor rotations to mechanism rotations
    config.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;

    // PID configuration (Slot 0) - uses tunable values
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();
    config.Slot0.kS = kS.get();
    config.Slot0.kV = kV.get();

    // IMPORTANT: Disable continuous wrap since turret has limited travel (400° total)
    // With ContinuousWrap=false, the controller won't try to wrap through ±180°
    config.ClosedLoopGeneral.ContinuousWrap = false;

    // Software soft limits - prevent turret from exceeding physical travel limits
    // These values are in mechanism rotations (after SensorToMechanismRatio is applied)
    // The motor will automatically stop when reaching these limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        TurretConstants.FORWARD_SOFT_LIMIT_ROTATIONS;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        TurretConstants.REVERSE_SOFT_LIMIT_ROTATIONS;

    // Current limits
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = TurretConstants.CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = TurretConstants.CURRENT_THRESHOLD_AMPS;

    // Apply configuration
    turretMotor.getConfigurator().apply(config);

    // Cache status signals
    position = turretMotor.getPosition();
    velocity = turretMotor.getVelocity();
    appliedVoltage = turretMotor.getMotorVoltage();
    statorCurrent = turretMotor.getStatorCurrent();

    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVoltage, statorCurrent);
    turretMotor.optimizeBusUtilization();
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
  public void setTargetAngle(Rotation2d rotation) {
    // Clamp the target angle to valid range before commanding
    double clampedDegrees =
        Math.max(
            TurretConstants.MIN_ANGLE_DEGREES,
            Math.min(TurretConstants.MAX_ANGLE_DEGREES, rotation.getDegrees()));
    targetRotation = Rotation2d.fromDegrees(clampedDegrees);

    // Convert rotation to rotations (TalonFX position unit)
    double targetRotations = targetRotation.getRotations();
    turretMotor.setControl(positionRequest.withPosition(targetRotations));
  }
}
