// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.SparkConnection;
import java.util.Arrays;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for a Kraken X60 drive motor (TalonFX) and SparkMax turn motor, with a
 * CANcoder absolute encoder for initial turn position. Functionally equivalent to {@link
 * ModuleIOSpark}, which uses a SparkMax for both drive and turn.
 *
 * <p>Unlike SparkMax, TalonFX status signals are read from an auto-updating cache and never block
 * on a disconnected device, so the drive motor does not need the read-skipping behavior ({@code
 * SparkConnection}) that protects against SparkMax CAN timeouts.
 *
 * <p>The drive TalonFX is configured with {@code Feedback.SensorToMechanismRatio} so that its
 * reported position/velocity (and its internal closed loop) operate directly in wheel rotations,
 * matching the wheel-radian domain that {@code driveKp}/{@code driveKd} were tuned in on the Spark
 * implementation (hence the {@code 2 * PI} scale factor applied to those gains below).
 */
public class ModuleIOSparkKraken implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final CANBus kCANBus = CANBus.roboRIO();
  private final TalonFX driveKraken;
  private final TalonFXConfiguration driveKrakenConfig;
  private final SparkBase turnSpark;
  private final RelativeEncoder turnEncoder;
  private final CANcoder cancoder; // null if no CANcoder (e.g., RectangleBot)

  // Drive status signals (auto-updating cache; never block on disconnect)
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVoltsSignal;
  private final StatusSignal<Current> driveCurrentSignal;

  // Reused control requests (avoid allocating every loop)
  private final VoltageOut driveVoltageRequest = new VoltageOut(0.0);
  private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0.0);

  // Inputs from CANcoder (null if no CANcoder)
  private final StatusSignal<Angle> turnAbsolutePosition;

  // Closed loop controller for turn only (drive closed loop runs on the TalonFX itself)
  private final SparkClosedLoopController turnController;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Pre-allocated arrays for odometry drain (avoids stream boxing + allocation each cycle)
  private static final int ODOMETRY_QUEUE_CAPACITY = 20;
  private double[] odometryTimestampsBuf = new double[ODOMETRY_QUEUE_CAPACITY];
  private double[] odometryDrivePosBuf = new double[ODOMETRY_QUEUE_CAPACITY];
  private Rotation2d[] odometryTurnPosBuf = new Rotation2d[ODOMETRY_QUEUE_CAPACITY];

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnEncoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Skip CAN reads when the turn motor is disconnected to prevent loop overruns.
  // Not needed for the drive TalonFX: Phoenix6 status signals read from a cache and
  // never block, so there is no timeout to protect against.
  private final SparkConnection turnConnection = new SparkConnection();

  public ModuleIOSparkKraken(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> Rotation2d.kZero;
        };

    // Create TalonFX for drive motor
    driveKraken =
        new TalonFX(
            switch (module) {
              case 0 -> frontLeftDriveCanId;
              case 1 -> frontRightDriveCanId;
              case 2 -> backLeftDriveCanId;
              case 3 -> backRightDriveCanId;
              default -> 0;
            },
            kCANBus);

    // Create SparkMax for turn motor
    turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> frontLeftTurnCanId;
              case 1 -> frontRightTurnCanId;
              case 2 -> backLeftTurnCanId;
              case 3 -> backRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);

    // Create CANcoder for absolute position (if available)
    int cancoderCanId =
        switch (module) {
          case 0 -> frontLeftCANCoderCanId;
          case 1 -> frontRightCANCoderCanId;
          case 2 -> backLeftCANCoderCanId;
          case 3 -> backRightCANCoderCanId;
          default -> 0;
        };
    cancoder = cancoderCanId > 0 ? new CANcoder(cancoderCanId) : null;

    turnEncoder = turnSpark.getEncoder();
    turnController = turnSpark.getClosedLoopController();

    // Configure drive motor. SensorToMechanismRatio makes getPosition()/getVelocity() (and the
    // TalonFX's own closed loop) report/operate directly in wheel rotations, so driveKp/driveKd
    // (tuned in volts per wheel-rad/sec on the Spark implementation) are scaled by 2*PI here.
    driveKrakenConfig = new TalonFXConfiguration();
    driveKrakenConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    driveKrakenConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveKrakenConfig.CurrentLimits.SupplyCurrentLimit = driveMotorCurrentLimit;
    driveKrakenConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveKrakenConfig.CurrentLimits.StatorCurrentLimit = driveMotorCurrentLimit;
    driveKrakenConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveKrakenConfig.Voltage.PeakForwardVoltage = 12.0;
    driveKrakenConfig.Voltage.PeakReverseVoltage = -12.0;
    driveKrakenConfig.Feedback.SensorToMechanismRatio = driveMotorReduction;
    driveKrakenConfig.Slot0.kP = driveKp * 2.0 * Math.PI;
    driveKrakenConfig.Slot0.kD = driveKd * 2.0 * Math.PI;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = driveKraken.getConfigurator().apply(driveKrakenConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    // Zero the drive position at startup (mirrors driveEncoder.setPosition(0.0) on Spark)
    StatusCode zeroStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      zeroStatus = driveKraken.setPosition(0.0);
      if (zeroStatus.isOK()) break;
    }

    drivePosition = driveKraken.getPosition();
    driveVelocity = driveKraken.getVelocity();
    driveAppliedVoltsSignal = driveKraken.getMotorVoltage();
    driveCurrentSignal = driveKraken.getStatorCurrent();
    drivePosition.setUpdateFrequency(odometryFrequency);
    driveVelocity.setUpdateFrequency(50.0);
    driveAppliedVoltsSignal.setUpdateFrequency(10.0);
    driveCurrentSignal.setUpdateFrequency(10.0);
    driveKraken.optimizeBusUtilization();

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .encoder
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .uvwAverageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pid(turnKp, 0.0, turnKd);
    turnConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(100)
        .busVoltagePeriodMs(100)
        .outputCurrentPeriodMs(100);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig,
                ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters));

    // Configure CANcoder and seed turn encoder (if CANcoder available)
    if (cancoder != null) {
      CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
      cancoder.getConfigurator().apply(cancoderConfig);
      turnAbsolutePosition = cancoder.getAbsolutePosition();
      BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnAbsolutePosition);

      // Seed the turn encoder with the CANcoder absolute position
      double initialTurnPosition =
          Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
              .minus(zeroRotation)
              .getRadians();
      tryUntilOk(turnSpark, 5, () -> turnEncoder.setPosition(initialTurnPosition));
    } else {
      // No CANcoder - initialize turn encoder to 0 (wheels must be aligned on startup)
      turnAbsolutePosition = null;
      tryUntilOk(turnSpark, 5, () -> turnEncoder.setPosition(0.0));
    }

    // Create odometry queues. The drive signal is cloned because StatusSignal objects are not
    // thread-safe; the clone is refreshed from the odometry (Notifier) thread while the fields
    // above are read from the main thread off the auto-updating cache.
    var drivePositionOdometry = drivePosition.clone();
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(() -> drivePositionOdometry.refresh().getValueAsDouble());
    turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Read CANcoder from auto-updated cache (no blocking CAN refresh needed).
    // The CANcoder updates at 50Hz automatically; refreshAll() would force a
    // blocking round-trip that adds ~5-10ms per module on a congested bus.
    boolean cancoderOk = false;
    if (turnAbsolutePosition != null) {
      cancoderOk = turnAbsolutePosition.getStatus().isOK();
    }

    // Update drive inputs. TalonFX status signals read from an auto-updating cache and never
    // block on disconnect, so there is no need to skip reads like the SparkConnection does below.
    boolean driveSignalsOk =
        drivePosition.getStatus().isOK()
            && driveVelocity.getStatus().isOK()
            && driveAppliedVoltsSignal.getStatus().isOK()
            && driveCurrentSignal.getStatus().isOK();
    inputs.drivePositionRad = drivePosition.refresh().getValueAsDouble() * 2.0 * Math.PI;
    inputs.driveVelocityRadPerSec = driveVelocity.refresh().getValueAsDouble() * 2.0 * Math.PI;
    inputs.driveAppliedVolts = driveAppliedVoltsSignal.refresh().getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrentSignal.refresh().getValueAsDouble();
    inputs.driveConnected = driveConnectedDebounce.calculate(driveSignalsOk);

    // Update turn inputs (skip reads if motor is disconnected to prevent timeout delays)
    if (!turnConnection.isSkipping()) {
      sparkStickyFault = false;
      ifOk(
          turnSpark,
          turnEncoder::getPosition,
          (value) -> inputs.turnPosition = Rotation2d.fromRadians(value));
      inputs.CANCoderPosition =
          turnAbsolutePosition != null
              ? Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
              : Rotation2d.kZero;
      inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);
      inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(cancoderOk);
      ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
      ifOk(
          turnSpark,
          new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
          (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
      ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
      turnConnection.update(inputs.turnConnected);
    } else {
      inputs.turnConnected = false;
    }

    // Update odometry inputs — drain queues into pre-allocated arrays (no boxing/streams)
    int size = timestampQueue.size();
    if (size > odometryTimestampsBuf.length) {
      odometryTimestampsBuf = new double[size];
      odometryDrivePosBuf = new double[size];
      odometryTurnPosBuf = new Rotation2d[size];
    }
    int idx = 0;
    for (Double v : timestampQueue) {
      odometryTimestampsBuf[idx++] = v;
    }
    inputs.odometryTimestamps =
        (idx == odometryTimestampsBuf.length)
            ? odometryTimestampsBuf
            : Arrays.copyOf(odometryTimestampsBuf, idx);

    idx = 0;
    for (Double v : drivePositionQueue) {
      odometryDrivePosBuf[idx++] = v * 2.0 * Math.PI;
    }
    inputs.odometryDrivePositionsRad =
        (idx == odometryDrivePosBuf.length)
            ? odometryDrivePosBuf
            : Arrays.copyOf(odometryDrivePosBuf, idx);

    idx = 0;
    for (Double v : turnPositionQueue) {
      odometryTurnPosBuf[idx++] = new Rotation2d(v);
    }
    inputs.odometryTurnPositions =
        (idx == odometryTurnPosBuf.length)
            ? odometryTurnPosBuf
            : Arrays.copyOf(odometryTurnPosBuf, idx);

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveKraken.setControl(driveVoltageRequest.withOutput(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = velocityRadPerSec / (2.0 * Math.PI);
    double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
    driveKraken.setControl(
        driveVelocityRequest.withVelocity(velocityRotPerSec).withFeedForward(ffVolts));
  }

  @Override
  public void setDriveMotorCurrentLimits(double currentLimit) {
    var currentLimitsConfig =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(driveMotorCurrentLimit)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(driveMotorCurrentLimit)
            .withStatorCurrentLimitEnable(true);
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = driveKraken.getConfigurator().apply(currentLimitsConfig);
      if (status.isOK()) break;
    }
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnController.setSetpoint(rotation.getRadians(), ControlType.kPosition);
  }
}
