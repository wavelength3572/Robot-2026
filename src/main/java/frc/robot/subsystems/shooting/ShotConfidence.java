package frc.robot.subsystems.shooting;

import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Computes a graduated shot confidence score (0–100) using a weighted geometric mean of 5 factors.
 * Replaces binary auto-shoot gating with a nuanced measure of shot quality.
 *
 * <p>If any single factor is zero, the entire score is zero — this is intentional. A shot with zero
 * vision confidence or a completely off-target turret should never fire, regardless of how good the
 * other factors are.
 *
 * <p>Inspired by FRC 5962's fire control system.
 */
public class ShotConfidence {

  // Factor weights — higher weight means more influence on the composite score.
  // Turret aim accuracy dominates (1.5) because a misaimed shot is always a miss.
  private final LoggedTunableNumber wVelocityStability =
      new LoggedTunableNumber("Shots/Confidence/Weight/VelocityStability", 0.8);
  private final LoggedTunableNumber wVisionConfidence =
      new LoggedTunableNumber("Shots/Confidence/Weight/VisionConfidence", 1.2);
  private final LoggedTunableNumber wAimAccuracy =
      new LoggedTunableNumber("Shots/Confidence/Weight/AimAccuracy", 1.5);
  private final LoggedTunableNumber wDistanceQuality =
      new LoggedTunableNumber("Shots/Confidence/Weight/DistanceQuality", 0.5);
  private final LoggedTunableNumber wLauncherReadiness =
      new LoggedTunableNumber("Shots/Confidence/Weight/LauncherReadiness", 1.0);

  // Factor parameters
  private final LoggedTunableNumber velocityChangeThreshold =
      new LoggedTunableNumber("Shots/Confidence/VelocityChangeThreshold", 0.5);
  private final LoggedTunableNumber aimMaxErrorDeg =
      new LoggedTunableNumber("Shots/Confidence/AimMaxErrorDeg", 15.0);
  private final LoggedTunableNumber aimReferenceDistM =
      new LoggedTunableNumber("Shots/Confidence/AimReferenceDistM", 2.5);
  private final LoggedTunableNumber launcherToleranceRPM =
      new LoggedTunableNumber("Shots/Confidence/LauncherToleranceRPM", 100.0);

  // Scoring range for distance quality factor
  private final LoggedTunableNumber minScoringDistM =
      new LoggedTunableNumber("Shots/Confidence/MinScoringDistM", 0.5);
  private final LoggedTunableNumber maxScoringDistM =
      new LoggedTunableNumber("Shots/Confidence/MaxScoringDistM", 5.0);

  // Previous speed for velocity stability calculation
  private double prevSpeedMps = 0.0;

  /**
   * Calculate shot confidence score.
   *
   * @param robotSpeedMps Current robot speed in m/s
   * @param visionConfidence Vision subsystem confidence [0, 1]
   * @param turretErrorDeg Turret aim error in degrees (absolute)
   * @param distanceM Distance to target in meters
   * @param rpmError Launcher RPM error (absolute)
   * @return Confidence score 0–100
   */
  public double calculate(
      double robotSpeedMps,
      double visionConfidence,
      double turretErrorDeg,
      double distanceM,
      double rpmError) {

    // Factor 1: Velocity stability — penalizes acceleration/deceleration transients
    double speedChange = Math.abs(robotSpeedMps - prevSpeedMps);
    double f1 = clamp(1.0 - speedChange / velocityChangeThreshold.get(), 0, 1);
    prevSpeedMps = robotSpeedMps;

    // Factor 2: Vision confidence — passthrough from vision subsystem
    double f2 = clamp(visionConfidence, 0, 1);

    // Factor 3: Turret aim accuracy — scaled by distance (tighter at long range)
    double scaledMaxError =
        aimMaxErrorDeg.get()
            * clamp(aimReferenceDistM.get() / Math.max(distanceM, 0.5), 0.5, 2.0)
            * (1.0 / (1.0 + robotSpeedMps));
    double f3 = clamp(1.0 - Math.abs(turretErrorDeg) / scaledMaxError, 0, 1);

    // Factor 4: Distance quality — triangular, peaks at mid-range
    double range = maxScoringDistM.get() - minScoringDistM.get();
    double rangeFraction =
        range > 0 ? (distanceM - minScoringDistM.get()) / range : 0.5;
    rangeFraction = clamp(rangeFraction, 0, 1);
    double f4 = 1.0 - 2.0 * Math.abs(rangeFraction - 0.5);

    // Factor 5: Launcher readiness — graduated instead of binary at-setpoint
    double f5 = clamp(1.0 - rpmError / launcherToleranceRPM.get(), 0, 1);

    // Weighted geometric mean: exp(Σ(w_i * ln(c_i)) / Σw_i) * 100
    double[] factors = {f1, f2, f3, f4, f5};
    double[] weights = {
      wVelocityStability.get(),
      wVisionConfidence.get(),
      wAimAccuracy.get(),
      wDistanceQuality.get(),
      wLauncherReadiness.get()
    };

    double logSum = 0;
    double weightSum = 0;
    for (int i = 0; i < factors.length; i++) {
      if (factors[i] <= 0) {
        logFactors(f1, f2, f3, f4, f5, 0);
        return 0; // Any zero factor kills the score
      }
      logSum += weights[i] * Math.log(factors[i]);
      weightSum += weights[i];
    }

    double composite = weightSum > 0 ? Math.exp(logSum / weightSum) * 100.0 : 0;
    composite = clamp(composite, 0, 100);

    logFactors(f1, f2, f3, f4, f5, composite);
    return composite;
  }

  private void logFactors(
      double f1, double f2, double f3, double f4, double f5, double composite) {
    Logger.recordOutput("Shots/Confidence/Factor/VelocityStability", f1);
    Logger.recordOutput("Shots/Confidence/Factor/VisionConfidence", f2);
    Logger.recordOutput("Shots/Confidence/Factor/AimAccuracy", f3);
    Logger.recordOutput("Shots/Confidence/Factor/DistanceQuality", f4);
    Logger.recordOutput("Shots/Confidence/Factor/LauncherReadiness", f5);
    Logger.recordOutput("Shots/Confidence/Score", composite);
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
