// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(pigeonCanId);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(odometryFrequency);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    var yawClone = yaw.clone(); // Status signals are not thread-safe
    yawPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(() -> yawClone.refresh().getValueAsDouble());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Read from auto-updated cache instead of blocking refreshAll().
    // The yaw updates at odometryFrequency (250Hz) and yawVelocity at 50Hz automatically.
    // Blocking refreshAll() adds ~5-10ms on a congested CAN bus for no benefit,
    // since the odometry thread already reads gyro independently at 250Hz.
    inputs.connected = yaw.getStatus().isOK() && yawVelocity.getStatus().isOK();
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public void resetPositionToZero() {
    pigeon.getConfigurator().setYaw(0.0);
  }
}
