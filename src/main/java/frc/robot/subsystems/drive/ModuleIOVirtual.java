// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Virtual module IO implementation that simulates wheel motion by integrating velocity commands.
 * Used for TurretBot to enable "virtual driving" via joystick - the robot pose updates in
 * AdvantageScope based on joystick input, allowing turret auto-aiming to be tested without a
 * physical drive base.
 *
 * <p>This implementation:
 *
 * <ul>
 *   <li>Stores commanded drive velocity and turn angle
 *   <li>Integrates velocity over time to update wheel position
 *   <li>Returns simulated odometry positions to the Drive subsystem
 *   <li>Does not simulate any motor physics - just direct integration
 * </ul>
 */
public class ModuleIOVirtual implements ModuleIO {
  private double drivePositionRad = 0.0;
  private double driveVelocityRadPerSec = 0.0;
  private double turnPositionRad = 0.0;

  public ModuleIOVirtual() {}

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Integrate velocity to update position (0.02s = 50Hz loop period)
    drivePositionRad += driveVelocityRadPerSec * 0.02;

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = drivePositionRad;
    inputs.driveVelocityRadPerSec = driveVelocityRadPerSec;
    inputs.driveAppliedVolts = 0.0;
    inputs.driveCurrentAmps = 0.0;

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnPosition = Rotation2d.fromRadians(turnPositionRad);
    inputs.CANCoderPosition = Rotation2d.fromRadians(turnPositionRad);
    inputs.turnVelocityRadPerSec = 0.0;
    inputs.turnAppliedVolts = 0.0;
    inputs.turnCurrentAmps = 0.0;

    // Update odometry inputs (50Hz - single sample per cycle)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    // Convert voltage output to approximate velocity (rough approximation)
    // This allows open-loop commands to work, though closed-loop is preferred
    driveVelocityRadPerSec = output * 10.0; // Rough scaling factor
  }

  @Override
  public void setTurnOpenLoop(double output) {
    // For virtual driving, we don't need open-loop turn control
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    // Directly set the velocity - no motor simulation, instant response
    driveVelocityRadPerSec = velocityRadPerSec;
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    // Directly set the turn angle - instant response
    turnPositionRad = rotation.getRadians();
  }
}
