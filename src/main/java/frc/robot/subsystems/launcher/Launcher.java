package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.turret.TurretCalculator;
import frc.robot.util.BenchTestMetrics;
import frc.robot.util.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Launcher subsystem for controlling two coupled Vortex motors. Uses velocity control with hardware
 * follower mode to ensure motors run in sync.
 */
public class Launcher extends SubsystemBase {
  private final LauncherIO io;
  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  // Recovery boost tunables
  private static final LoggedTunableNumber recoveryBoostThresholdRPM =
      new LoggedTunableNumber("Tuning/Launcher/RecoveryBoostThresholdRPM", 50.0);
  private static final LoggedTunableNumber recoveryBoostVolts =
      new LoggedTunableNumber("Tuning/Launcher/RecoveryBoostVolts", 0.0);

  // Feeding flag - set by ShootingCommands when prefeed starts
  private boolean feedingActive = false;

  // Tracks whether recovery mode is currently active (for hysteresis)
  private boolean recoveryActive = false;

  // Safety threshold for velocity mismatch between motors
  private static final double VELOCITY_MISMATCH_THRESHOLD_RPM = 200.0;

  // SysId safety: end test when velocity reaches this threshold
  private static final double SYSID_MAX_VELOCITY_RPM = 3000.0;

  private final List<Double> sysIdVoltages = new ArrayList<>();
  private final List<Double> sysIdVelocities = new ArrayList<>();
  private boolean collectingSysIdData = false;

  public Launcher(LauncherIO io) {
    this.io = io;

    // Configure SysId routine for flywheel characterization
    // Using Volts.per(Second) for ramp rate (0.5 V/s) and Volts for step voltage (4V)
    // Step voltage kept moderate to respect 3000 RPM max velocity limit
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(0.5), // Ramp rate: 0.5 V/s for quasistatic
                Volts.of(4), // Step voltage: 4V for dynamic (limited for safety)
                Seconds.of(10), // Timeout: 10 seconds
                (state) -> Logger.recordOutput("Shooter/Launcher/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                (log) -> {
                  double velocityRadPerSec =
                      Units.rotationsPerMinuteToRadiansPerSecond(inputs.wheelVelocityRPM);

                  // Log data for SysId tool analysis
                  log.motor("launcher")
                      .voltage(Volts.of(inputs.leaderAppliedVolts))
                      .angularVelocity(RadiansPerSecond.of(velocityRadPerSec));

                  // Collect data for automatic kS/kV calculation
                  if (collectingSysIdData && inputs.wheelVelocityRPM > 1000.0) {
                    sysIdVoltages.add(inputs.leaderAppliedVolts);
                    sysIdVelocities.add(velocityRadPerSec);
                  }
                },
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Launcher", inputs);

    // Update TurretCalculator with current wheel RPM for trajectory calculations
    TurretCalculator.setLauncherRPM(inputs.wheelVelocityRPM);

    // Log velocity error
    double velocityError = inputs.targetVelocityRPM - inputs.wheelVelocityRPM;
    Logger.recordOutput("Shooter/Launcher/velocityError", velocityError);

    // Safety: detect velocity mismatch between motors
    // Both encoders read motor RPM, so compare directly
    double velocityMismatch =
        Math.abs(Math.abs(inputs.leaderVelocityRPM) - Math.abs(inputs.followerVelocityRPM));
    Logger.recordOutput("Shooter/Launcher/velocityMismatch", velocityMismatch);

    // Alert if mismatch exceeds threshold while running
    boolean mismatchAlert =
        velocityMismatch > VELOCITY_MISMATCH_THRESHOLD_RPM && inputs.targetVelocityRPM > 100;
    Logger.recordOutput("Shooter/Launcher/velocityMismatchAlert", mismatchAlert);

    if (mismatchAlert) {
      System.err.println(
          "[Launcher] WARNING: Velocity mismatch detected! Leader: "
              + inputs.leaderVelocityRPM
              + " RPM, Follower: "
              + inputs.followerVelocityRPM
              + " RPM");
    }

    // Update bench test metrics with current state
    BenchTestMetrics.getInstance()
        .periodic(
            inputs.wheelVelocityRPM,
            inputs.targetVelocityRPM,
            inputs.leaderTempCelsius,
            inputs.followerTempCelsius);
  }

  /**
   * Set the launcher wheel velocity. Computes recovery boost if feeding is active and RPM has
   * dipped below threshold.
   *
   * @param velocityRPM Target velocity in wheel RPM
   */
  public void setVelocity(double velocityRPM) {
    // Determine if recovery is active with hysteresis:
    // - Activate when error exceeds threshold
    // - Deactivate only when error drops below half the threshold
    double boost = 0.0;
    if (feedingActive && velocityRPM > 100.0) {
      double error = velocityRPM - inputs.wheelVelocityRPM;
      double threshold = recoveryBoostThresholdRPM.get();
      if (!recoveryActive && error > threshold) {
        recoveryActive = true;
      } else if (recoveryActive && error < threshold * 0.5) {
        recoveryActive = false;
      }
      if (recoveryActive) {
        boost = recoveryBoostVolts.get();
      }
    } else {
      recoveryActive = false;
    }
    Logger.recordOutput("Shooter/Launcher/RecoveryBoostActive", recoveryActive);
    Logger.recordOutput("Shooter/Launcher/RecoveryBoostVolts", boost);

    io.setVelocityWithBoost(velocityRPM, boost, recoveryActive);
    // Update TurretCalculator with target RPM for setpoint trajectory
    TurretCalculator.setTargetLauncherRPM(velocityRPM);
  }

  /**
   * Set whether feeding is actively pushing balls into the launcher. Used by ShootingCommands to
   * enable recovery boost only during actual shooting.
   *
   * @param active True when prefeed is running
   */
  public void setFeedingActive(boolean active) {
    this.feedingActive = active;
  }

  /** Stop the launcher. */
  public void stop() {
    feedingActive = false;
    io.stop();
    TurretCalculator.setTargetLauncherRPM(0.0);
  }

  /**
   * Signal that a ball was fired (simulation only). In sim, this triggers a recovery period where
   * atSetpoint returns false until the configured recovery time elapses. Does nothing on real
   * hardware.
   */
  public void notifyBallFired() {
    io.notifyBallFired();
  }

  /**
   * Get the current wheel velocity.
   *
   * @return Current velocity in wheel RPM
   */
  @AutoLogOutput(key = "Shooter/Launcher/currentVelocity")
  public double getVelocity() {
    return inputs.wheelVelocityRPM;
  }

  /**
   * Get the target velocity.
   *
   * @return Target velocity in wheel RPM
   */
  public double getTargetVelocity() {
    return inputs.targetVelocityRPM;
  }

  /**
   * Check if the launcher is at the target velocity.
   *
   * @return True if at setpoint within tolerance
   */
  public boolean atSetpoint() {
    return inputs.atSetpoint;
  }

  /**
   * Check if both motors are connected.
   *
   * @return True if both motors are responding
   */
  public boolean isConnected() {
    return inputs.leaderConnected && inputs.followerConnected;
  }

  // ========== SysId Commands ==========

  /** Returns true when velocity exceeds safe threshold for SysId testing. */
  private boolean sysIdVelocityLimitReached() {
    return inputs.wheelVelocityRPM >= SYSID_MAX_VELOCITY_RPM;
  }

  /** Clear collected SysId data and start collecting. */
  private void startSysIdCollection() {
    sysIdVoltages.clear();
    sysIdVelocities.clear();
    collectingSysIdData = true;
  }

  /** Stop collecting and calculate kS/kV from quasistatic data using linear regression. */
  private void finishSysIdCollection() {
    collectingSysIdData = false;

    int n = sysIdVoltages.size();
    if (n < 10) {
      System.out.println("[Launcher SysId] Not enough data points (" + n + ") to calculate gains");
      return;
    }

    // Linear regression: V = kS + kV * ω
    // Using least squares: kV = (n*Σ(ωV) - Σω*ΣV) / (n*Σ(ω²) - (Σω)²)
    //                      kS = (ΣV - kV*Σω) / n
    double sumV = 0, sumW = 0, sumVW = 0, sumW2 = 0;
    double minW = Double.MAX_VALUE, maxW = Double.MIN_VALUE;
    for (int i = 0; i < n; i++) {
      double v = sysIdVoltages.get(i);
      double w = sysIdVelocities.get(i);
      sumV += v;
      sumW += w;
      sumVW += v * w;
      sumW2 += w * w;
      minW = Math.min(minW, w);
      maxW = Math.max(maxW, w);
    }

    double denominator = n * sumW2 - sumW * sumW;
    if (Math.abs(denominator) < 1e-6) {
      System.out.println(
          "[Launcher SysId] Cannot calculate gains - insufficient velocity variation");
      return;
    }

    double kV = (n * sumVW - sumW * sumV) / denominator;
    double kS = (sumV - kV * sumW) / n;

    // Convert velocity range to RPM for readability
    double minRPM = Units.radiansPerSecondToRotationsPerMinute(minW);
    double maxRPM = Units.radiansPerSecondToRotationsPerMinute(maxW);

    // Output results
    System.out.println("[Launcher SysId] ========== RESULTS ==========");
    System.out.printf("[Launcher SysId] Velocity range: %.0f - %.0f RPM%n", minRPM, maxRPM);
    System.out.println("[Launcher SysId] Data points: " + n);
    System.out.printf("[Launcher SysId] kS = %.4f V (static friction)%n", kS);
    System.out.printf("[Launcher SysId] kV = %.6f V/(rad/s) (velocity gain)%n", kV);
    System.out.println("[Launcher SysId] ==============================");
    System.out.println("[Launcher SysId] To use these values, update Launcher/kS and Launcher/kV");

    // Also log to AdvantageKit for dashboard viewing
    Logger.recordOutput("Shooter/Launcher/SysId/CalculatedKs", kS);
    Logger.recordOutput("Shooter/Launcher/SysId/CalculatedKv", kV);
    Logger.recordOutput("Shooter/Launcher/SysId/DataPoints", n);
    Logger.recordOutput("Shooter/Launcher/SysId/MinVelocityRPM", minRPM);
    Logger.recordOutput("Shooter/Launcher/SysId/MaxVelocityRPM", maxRPM);
  }

  /**
   * SysId quasistatic characterization command (slow voltage ramp). Automatically ends when
   * velocity reaches the safety threshold. Calculates and outputs kS/kV when complete.
   *
   * @return Command to run quasistatic characterization
   */
  public Command launcherSysIdQuasistatic() {
    return sysId
        .quasistatic(SysIdRoutine.Direction.kForward)
        .until(this::sysIdVelocityLimitReached)
        .beforeStarting(this::startSysIdCollection)
        .finallyDo(this::finishSysIdCollection)
        .withName("Launcher SysId Quasistatic");
  }

  /**
   * SysId dynamic characterization command (step voltage). Automatically ends when velocity reaches
   * the safety threshold. Useful for determining kA (acceleration gain).
   *
   * @return Command to run dynamic characterization
   */
  public Command launcherSysIdDynamic() {
    return sysId
        .dynamic(SysIdRoutine.Direction.kForward)
        .until(this::sysIdVelocityLimitReached)
        .withName("Launcher SysId Dynamic");
  }

  // ========== Commands ==========

  /**
   * Command to run the launcher at a specific velocity.
   *
   * @param velocityRPM Target velocity in wheel RPM
   * @return Command that runs until interrupted
   */
  public Command runAtVelocityCommand(double velocityRPM) {
    return run(() -> setVelocity(velocityRPM))
        .finallyDo(this::stop)
        .withName("Launcher: Run at " + velocityRPM + " RPM");
  }

  /**
   * Command to spin up and wait until at setpoint.
   *
   * @param velocityRPM Target velocity in wheel RPM
   * @return Command that completes when at setpoint
   */
  public Command spinUpCommand(double velocityRPM) {
    return runOnce(() -> setVelocity(velocityRPM))
        .andThen(run(() -> {}).until(this::atSetpoint))
        .withName("Launcher: Spin Up to " + velocityRPM + " RPM");
  }

  /**
   * Command to stop the launcher.
   *
   * @return Instant command that stops motors
   */
  public Command stopCommand() {
    return runOnce(this::stop).withName("Launcher: Stop");
  }
}
