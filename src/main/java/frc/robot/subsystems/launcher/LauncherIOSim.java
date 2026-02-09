package frc.robot.subsystems.launcher;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.util.LoggedTunableNumber;
import java.util.Random;

/**
 * Simulation implementation of LauncherIO using tunable timing parameters. Simulates realistic
 * spinup and recovery behavior based on real-world measurements.
 *
 * <p>Key parameters (all tunable via dashboard):
 *
 * <ul>
 *   <li>SpinupTimeSeconds: Time to reach target from rest (e.g., 1.5s)
 *   <li>RecoveryTimeSeconds: Time to recover from a shot's RPM drop (e.g., 0.19s)
 *   <li>Variation parameters add randomness for more realistic testing
 * </ul>
 */
public class LauncherIOSim implements LauncherIO {
  private final FlywheelSim sim;
  private final double gearRatio;
  private final Random random = new Random();

  private double targetWheelRPM = 0.0;
  private double currentWheelRPM = 0.0;
  private double appliedVolts = 0.0;
  private boolean voltageMode = false; // True when using direct voltage control (SysId)

  // Transition tracking for realistic timing
  private double transitionStartRPM = 0.0;
  private double transitionStartTime = 0.0;
  private double currentTransitionDuration = 0.0; // How long this transition should take
  private boolean inTransition = false;

  // Simulation constants
  private static final double FLYWHEEL_MOI = 0.005; // kg*m^2 moment of inertia

  // Velocity tolerance for atSetpoint (set by subsystem via setVelocityTolerance)
  private double velocityToleranceRPM = 50.0;

  // Tunable timing parameters - set these based on real robot observations
  // Prefixed with Sim to clearly indicate these only affect simulation
  private static final LoggedTunableNumber spinupTimeSeconds =
      new LoggedTunableNumber("Sim/Launcher/SimSpinupTime", 1.5);

  private static final LoggedTunableNumber recoveryTimeSeconds =
      new LoggedTunableNumber("Sim/Launcher/SimRecoveryTime", 0.19);

  // Optional variation for more realistic testing (set to 0 to disable)
  private static final LoggedTunableNumber spinupTimeVariation =
      new LoggedTunableNumber("Sim/Launcher/SimSpinupVariation", 0.1);

  private static final LoggedTunableNumber recoveryTimeVariation =
      new LoggedTunableNumber("Sim/Launcher/SimRecoveryVariation", 0.03);

  // Threshold to determine if we're "spinning up from rest" vs "recovering from shot"
  // If current RPM is below this fraction of target, use spinup time
  private static final double SPINUP_THRESHOLD_FRACTION = 0.5;

  public LauncherIOSim() {
    RobotConfig config = Constants.getRobotConfig();
    gearRatio = config.getLauncherGearRatio();

    // FlywheelSim kept for SysId voltage mode only
    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), FLYWHEEL_MOI, 1.0 / gearRatio),
            DCMotor.getNEO(2));
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    if (!voltageMode) {
      updateVelocityMode();
    } else {
      // Voltage mode for SysId - use physics sim
      sim.update(0.02);
      currentWheelRPM = sim.getAngularVelocityRPM();
    }

    double motorRPM = currentWheelRPM / gearRatio;

    // Both motors report same values in simulation
    inputs.leaderConnected = true;
    inputs.followerConnected = true;
    inputs.leaderVelocityRPM = motorRPM;
    inputs.followerVelocityRPM = motorRPM;
    inputs.wheelVelocityRPM = currentWheelRPM;
    inputs.leaderAppliedVolts = appliedVolts;
    inputs.followerAppliedVolts = appliedVolts;
    inputs.leaderCurrentAmps = Math.abs(currentWheelRPM) * 0.01;
    inputs.leaderPdhCurrentAmps = Math.abs(currentWheelRPM) * 0.01;
    inputs.followerCurrentAmps = Math.abs(currentWheelRPM) * 0.01;
    inputs.followerPdhCurrentAmps = Math.abs(currentWheelRPM) * 0.01;
    inputs.leaderTempCelsius = 25.0;
    inputs.followerTempCelsius = 25.0;
    inputs.targetVelocityRPM = targetWheelRPM;
    inputs.atSetpoint = Math.abs(currentWheelRPM - targetWheelRPM) < velocityToleranceRPM;
  }

  /** Update velocity using time-based transitions for realistic behavior. */
  private void updateVelocityMode() {
    if (!inTransition || currentTransitionDuration <= 0) {
      // No active transition, snap to target (or stay at current if no target change)
      currentWheelRPM = targetWheelRPM;
      appliedVolts = currentWheelRPM * 0.004;
      return;
    }

    double elapsed = Timer.getFPGATimestamp() - transitionStartTime;
    double progress = Math.min(1.0, elapsed / currentTransitionDuration);

    // Linear interpolation from start RPM to target
    currentWheelRPM = transitionStartRPM + (targetWheelRPM - transitionStartRPM) * progress;

    // End transition when complete
    if (progress >= 1.0) {
      inTransition = false;
      currentWheelRPM = targetWheelRPM;
    }

    appliedVolts = currentWheelRPM * 0.004;
  }

  @Override
  public void setVelocity(double velocityRPM) {
    voltageMode = false;
    double newTarget = Math.abs(velocityRPM);

    // Only start a new transition if target actually changed significantly
    if (Math.abs(newTarget - targetWheelRPM) > 10) {
      startTransition(newTarget, false);
    }
    targetWheelRPM = newTarget;
  }

  @Override
  public void setVelocityWithBoost(double velocityRPM, double boostVolts, boolean recoveryActive) {
    // Sim doesn't need boost - delegate to normal setVelocity
    setVelocity(velocityRPM);
  }

  @Override
  public void setVoltage(double volts) {
    voltageMode = true;
    targetWheelRPM = 0.0;
    inTransition = false;

    // Safety: limit voltage when approaching max velocity
    double maxVelocityRPM = 3500.0;
    if (Math.abs(currentWheelRPM) >= maxVelocityRPM * 0.95) {
      volts = 0.0;
    } else if (Math.abs(currentWheelRPM) >= maxVelocityRPM * 0.85) {
      double scale = (maxVelocityRPM * 0.95 - Math.abs(currentWheelRPM)) / (maxVelocityRPM * 0.1);
      volts = volts * scale;
    }

    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    voltageMode = false;
    targetWheelRPM = 0.0;
    currentWheelRPM = 0.0;
    appliedVolts = 0.0;
    inTransition = false;
    sim.setInputVoltage(0.0);
  }

  @Override
  public void notifyBallFired() {
    // Start recovery transition - RPM drops briefly then recovers
    // The drop amount is internal (just enough for visual feedback)
    double visualRpmDrop = targetWheelRPM * 0.3; // 30% drop for visual effect
    currentWheelRPM = Math.max(0, currentWheelRPM - visualRpmDrop);

    // Start recovery transition back to target
    startTransition(targetWheelRPM, true);
  }

  /**
   * Start a transition to a new target RPM.
   *
   * @param newTarget The target RPM to reach
   * @param isRecovery True if recovering from a shot (use recovery time), false for spinup
   */
  private void startTransition(double newTarget, boolean isRecovery) {
    transitionStartRPM = currentWheelRPM;
    transitionStartTime = Timer.getFPGATimestamp();

    // Determine transition duration based on context
    double baseTime;
    double variation;

    if (isRecovery) {
      // Recovering from a shot - use recovery time
      baseTime = recoveryTimeSeconds.get();
      variation = recoveryTimeVariation.get();
    } else if (currentWheelRPM < newTarget * SPINUP_THRESHOLD_FRACTION) {
      // Starting from rest/low RPM - use full spinup time
      baseTime = spinupTimeSeconds.get();
      variation = spinupTimeVariation.get();
    } else {
      // Small velocity adjustment - scale spinup time proportionally
      double rpmDelta = Math.abs(newTarget - currentWheelRPM);
      double fullRpmRange = newTarget; // Approximate full range
      double fraction = Math.min(1.0, rpmDelta / Math.max(1, fullRpmRange));
      baseTime = spinupTimeSeconds.get() * fraction;
      variation = spinupTimeVariation.get() * fraction;
    }

    // Add random variation if enabled
    if (variation > 0) {
      // Random value between -variation and +variation
      double randomOffset = (random.nextDouble() * 2 - 1) * variation;
      currentTransitionDuration = Math.max(0.01, baseTime + randomOffset);
    } else {
      currentTransitionDuration = baseTime;
    }

    inTransition = true;
  }

  @Override
  public void setVelocityTolerance(double toleranceRPM) {
    this.velocityToleranceRPM = toleranceRPM;
  }
}
