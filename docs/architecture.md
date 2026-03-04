# Robot-2026 Architecture Reference

## Project Overview
- Java FRC robot codebase using WPILib 2026.2.1 + AdvantageKit + PathPlanner
- Multi-robot support: SquareBot, MainBot, TurretBot, RectangleBot
- Swerve drive with turret, launcher, hood, intake, motivator subsystems
- IO interface pattern for hardware abstraction (real/sim/replay)

## Build System
- Gradle with GradleRIO plugin
- Spotless for code formatting (Google Java Format)

## Key Architecture Patterns
- IO interfaces with @AutoLog for input classes
- LoggedTunableNumber for runtime PID/constant tuning
- SparkOdometryThread for high-freq encoder reads (250Hz)
- FuelSim for 3D ball physics in simulation
- VisionIOPhotonVisionSim for camera simulation

## Key Files
- `Turret.java`: calculateTurretAngle() uses closest-offset unwrapping strategy
- `Drive.java`: AIM_PHASE_DELAY_SECONDS for Twist2d forward projection
- `Vision.java`: Adaptive std devs (ambiguity + speed scaling)
- `TurretCalculator.java`: Iterative moving-shot ballistics
- `TrajectoryOptimizer.java`: Two-point parabolic trajectory solver

## Subsystem Summary

### Drive (Swerve)
- 4 modules: REV NEO/Vortex via SparkMax + CTRE CANcoder
- Gyro: CTRE Pigeon 2.0
- SwerveDrivePoseEstimator with vision integration
- High-freq odometry via SparkOdometryThread (250Hz)
- Twist2d forward projection compensates for control loop phase delay

### Turret
- TalonFX or SparkMax depending on robot
- Closest-offset unwrapping: avoids unnecessary 340+ degree sweeps
- Dual-mode range: tracking (narrow) vs launch (full)
- Soft limits with tunable flip angle and center offset
- Integration with TurretCalculator for ballistics

### Vision
- Up to 5 PhotonVision cameras (4 corners + 1 front-center on MainBot)
- Adaptive std devs: distance^2/tagCount * ambiguity scale * speed scale
- Ambiguity scaling: up to 3x at max ambiguity
- Speed scaling: up to 1.5x at 3+ m/s
- Per-camera multipliers for differential trust

### Launcher
- Two SparkFlex Vortex motors in follower mode
- PID velocity control + SysId-based feedforward (kS + kV)
- Recovery boost with hysteresis (slot switching to higher kP)

### Hood
- Adjusts launch angle via TrajectoryOptimizer
- Two-point parabolic trajectory solver (hub edge + hub center)
- Descent angle constraint for consistent entry

### Intake
- Deploy arm (position control) + roller (duty cycle)
- Velocity-based roller speed adjustment from robot speed

### Motivator
- Three independent motors (Motor 1, Motor 2, Prefeed)
- Feeds balls into launcher

## Simulation
- ModuleIOSim: DCMotorSim with realistic motor models (HIGH fidelity)
- TurretIOSim: TrapezoidProfile (MEDIUM fidelity, kinematic only)
- LauncherIOSim: Time-based spinup/recovery (MEDIUM fidelity)
- VisionIOPhotonVisionSim: PhotonVision detection sim (HIGH fidelity)
- FuelSim: Full 3D ball physics with collisions (HIGH fidelity)
- HoodIOSim: First-order filter (LOW fidelity)

## Improvements (2026-02-06, inspired by FRC 6328)
1. Turret closest-offset unwrapping: tries +/-360 offsets, picks closest legal angle
2. Dual-mode turret range: tracking (10deg narrower) vs launch (full range)
3. Twist2d forward projection: projects pose forward by 20ms before aiming
4. Adaptive vision std devs: ambiguity (up to 3x) + speed (up to 1.5x at 3m/s)
