package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Read-only snapshot of shooting state passed from ShootingCoordinator (control) to ShotVisualizer
 * (display). Decouples visualization from control so the visualizer is a passive observer.
 */
public record ShotSnapshot(
    Pose2d robotPose,
    ChassisSpeeds fieldSpeeds,
    boolean isBlueAlliance,
    double currentAngleDeg,
    double targetAngleDeg,
    double effectiveMinAngleDeg,
    double effectiveMaxAngleDeg,
    double centerOffsetDeg,
    double warningZoneDeg,
    ShotCalculator.ShotResult currentShot,
    double turretHeightMeters,
    double turretXOffset,
    double turretYOffset) {}
