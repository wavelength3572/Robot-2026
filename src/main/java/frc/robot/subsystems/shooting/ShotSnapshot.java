package frc.robot.subsystems.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.ZoneDetector;

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
    Translation3d compensatedAimTarget,
    double exitVelocityMps,
    double turretHeightMeters,
    double turretXOffset,
    double turretYOffset,
    TrajectoryReadiness trajectoryReadiness,
    ZoneDetector.Zone currentZone) {

  /** Trajectory color based on actual shot readiness gating. */
  public enum TrajectoryReadiness {
    NOT_ACTIVE, // Red — no shot calculated
    NOT_READY, // Yellow — shot exists but one or more subsystems not ready
    READY // Green — all subsystems ready, would fire if triggered
  }
}
