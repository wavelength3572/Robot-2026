# Robot Systems Overview

This document explains how all the robot's systems work together, with extra detail on the launching flow.

---

## High-Level Architecture

The robot uses WPILib's **command-based** framework with the **AdvantageKit** logging layer. Every piece of hardware is wrapped behind an IO interface so the same code can run on the real robot, in physics simulation, or in log replay mode.

**Startup chain:**
```
Main.java → Robot.java (lifecycle) → RobotContainer.java (creates all subsystems + binds buttons)
```

**Data flow:**
```
Joystick inputs → OperatorInterface → Commands → Subsystems → IO layer → Motors/Sensors
```

---

## Subsystems

### 1. Drive (Swerve Drivetrain)
The foundation of the robot. Four swerve modules (front-left, front-right, back-left, back-right), each with a NEO Vortex drive motor and a NEO 550 turning motor plus a CANCoder absolute encoder. A Pigeon 2 IMU provides heading.

- Max speed ~5.2 m/s
- Uses `SwerveDrivePoseEstimator` to fuse wheel odometry with vision measurements
- PathPlanner handles autonomous path following
- Field-relative driving is the default in teleop

### 2. Vision (AprilTag Localization)
Four PhotonVision cameras detect AprilTags on the field to correct the robot's estimated position. Measurements are filtered (rejects outliers, scales uncertainty by robot speed) and fed into the drive's pose estimator.

### 3. Turret
A NEO 550 motor on a 55:1 gear reduction rotates the entire launcher assembly. Range is -180 to +180 degrees relative to the robot's forward direction. Uses PD control (no integral term to avoid friction windup). The turret only handles physical rotation — the ShootingCoordinator tells it *where* to aim.

### 4. Hood (Launch Angle)
A SparkMax motor adjusts the launch angle between 13 and 46 degrees. Works with the TrajectoryOptimizer to find the best angle for any given distance. When idle, it stows to the minimum angle for clearance under trenches.

### 5. Launcher (Flywheel)
Two NEO Vortex motors in leader-follower mode spin the main launch wheel (3" diameter) through a 1.5:1 gear ratio. A smaller 2" hood roller is chained to the main wheel. Together they accelerate the fuel ball.

- Closed-loop velocity control via SparkFlex MAXMotion
- **Recovery mode**: When a ball feeds into the spinning wheel, it causes an RPM dip. The launcher detects this and temporarily boosts its PID gain to recover faster.
- Tunable feedforward: kS=0.31, kV=0.00174

### 6. Motivator (Ball Feeder)
A single SparkFlex motor (1:3 reduction) sits between the spindexer and the launcher. Its job is to push individual fuel balls up into the spinning flywheel at a controlled rate. Its RPM is set as a ratio of the launcher RPM (~56.5%).

### 7. Spindexer (Ball Indexer)
A SparkMax motor (1:3 reduction) that holds a stack of fuel balls and rotates to feed them one at a time into the motivator. Has a sophisticated state machine:

- **FEEDING**: Normal operation, pushing fuel toward the motivator
- **RECIPROCATING**: Gentle back-and-forth when idle to keep fuel loose
- **AUTO_UNCLOGGING**: Detects jams (high current + low velocity) and reverses briefly
- **SUPPRESSED**: Operator can pause feeding without interrupting an active shot

### 8. Intake
A deploy motor extends/retracts the intake mechanism, and a roller motor spins to grab fuel balls off the ground. The deploy goes through states: RETRACTED → DEPLOYING → DEPLOY_SETTLING → DEPLOYED, and back. The roller speed can increase with robot speed.

### 9. LED Indicator Lights
42 addressable RGB LEDs on PWM 0. Used to show robot status (ready to fire, vision lock, etc.) to the driver.

---

## The Launching Flow (Detailed)

This is the core of the robot. Here's exactly how a ball goes from "sitting in the robot" to "flying through the air and scoring."

### The Chain of Mechanisms

```
[Fuel balls on field]
        ↓  (intake rollers grab them)
   [ INTAKE ]
        ↓  (balls fall into the indexer)
  [ SPINDEXER ]  ← rotates to queue balls one at a time
        ↓
  [ MOTIVATOR ]  ← pushes individual balls up into the launcher
        ↓
  [ LAUNCHER ]   ← spinning flywheel accelerates the ball
        ↓
    [ HOOD ]     ← adjusts the exit angle
        ↓
   Ball exits at calculated velocity + angle
```

### Smart Launch Command — Step by Step

The `smartLaunchCommand` is the main firing sequence. Here's what happens when the driver holds the shoot button:

#### Phase 1: Spin-Up (Instant)
1. Mode is set to COMPETITION (auto-calculated trajectory)
2. The **ShootingCoordinator** reads the robot's current position from odometry
3. It calculates: *"I'm X meters from the hub, at this angle"*
4. From that, it determines the optimal **launcher RPM**, **hood angle**, and **turret angle**
5. All subsystems start moving to their targets simultaneously:
   - Launcher starts spinning up to target RPM
   - Turret rotates to aim at the hub
   - Hood tilts to the calculated angle
   - Motivator starts spinning (at 56.5% of launcher RPM)

#### Phase 2: Wait for Ready (Up to 5 seconds)
The system continuously re-calculates the shot as the robot moves (shoot-on-the-move). It waits until ALL of these are true:
- **Launcher** is at target RPM (within tolerance)
- **Motivator** is at target RPM
- **Turret** is aimed at the target (within angle tolerance)
- **Hood** is at the calculated angle
- The shot is **achievable** (physics says we can reach the target)
- The robot is **slow enough** to feed (under 1.75 m/s, configurable)

If everything isn't ready within 5 seconds, it times out and proceeds anyway (better to try than wait forever in a match).

There's also a 100ms settling delay after all conditions are first met — this prevents firing on a momentary flicker of "ready."

#### Phase 3: Firing (Continuous until button released)
Once ready, everything runs in parallel:
- **Launcher**: Keeps tracking the latest calculated RPM (updates every loop as robot moves)
- **Turret**: Keeps tracking the target angle
- **Hood**: Keeps tracking the optimal angle
- **Motivator**: Runs at the calculated RPM ratio
- **Spindexer**: Feeds fuel into the motivator, BUT only when the turret is on-target and robot speed is acceptable. When those conditions aren't met, it reciprocates (gentle back-and-forth) instead of feeding.
- **Sim firing loop**: In simulation, virtual fuel balls are spawned with the calculated trajectory

#### Cleanup (When button released)
- Launcher stops spinning
- Motivator stops
- Spindexer stops
- Hood returns to stow angle (minimum)
- Manual shot parameters cleared
- Mode returns to COMPETITION

### Shot Calculation — How the Math Works

The **ShotCalculator** is the physics engine behind every shot:

1. **Get turret's field position**: Transforms turret offset from robot-relative to field coordinates using the robot's heading
2. **Calculate distance**: Straight-line distance from turret to hub target
3. **Choose strategy** (selectable from dashboard):
   - **LUT (Lookup Table)**: Uses empirically recorded RPM + hood angle pairs indexed by distance. Pure "we tested this and it works" data.
   - **Parametric**: Physics-based calculation using a two-roller model with distance-dependent efficiency curve
4. **Calculate exit velocity**: Uses a two-roller model:
   - Main wheel surface speed = RPM × wheel circumference
   - Hood roller surface speed = main wheel × 0.709 (geared slower)
   - Exit velocity = average of both surface speeds × efficiency factor
   - Efficiency varies with distance (quadratic curve, peaks ~0.785 at mid-range)
5. **Calculate turret angle**: Robot-relative angle to point at the (possibly velocity-compensated) aim target, wrapped to minimize rotation from current position
6. **Velocity compensation** (shoot-on-the-move): If the robot is moving, the aim point is shifted to lead the target. The time-of-flight is estimated, and the aim target is adjusted to account for where the robot will have moved by the time the ball arrives. This is refined iteratively (3 passes).

### Recovery Mode

When a ball feeds into the spinning flywheel, it steals energy and the RPM dips. The launcher detects this velocity drop and switches to a higher PID gain (recovery mode) to spin back up faster. Once it recovers, it returns to normal gains. This is critical for rapid-fire accuracy.

### Speed-Limited Launch Variant

There's also `smartLaunchWithSpeedLimitCommand` — same as above, but it caps the robot's drive speed while firing (default 0.5 m/s). This lets the spindexer feed without the speed gate since the driver physically can't go too fast. When released, speed ramps back up smoothly to avoid jerk.

---

## Autonomous System

Autonomous routines are PathPlanner paths wrapped with shooting sequences:

### Comp Shot Wrapper (Standard)
```
1. Reset odometry to starting pose
2. Set up fuel simulation (if in sim)
3. Deploy intake + start rollers
4. Smart launch preloaded balls (up to 5s timeout)
5. Stow hood
6. Run the PathPlanner auto path (driving + collecting fuel)
7. Smart launch collected balls (up to 10s timeout)
8. Teardown (stop launcher, motivator, intake)
```

### Comp Sprint Wrapper (Aggressive)
```
1. Reset odometry to starting pose
2. Set up fuel simulation
3. Deploy intake + start rollers
4. Enable auto-shoot + spin launcher to 1700 RPM
5. Run the PathPlanner path (auto-shoot fires opportunistically during path)
6. Disable auto-shoot
7. Smart launch remaining balls
8. Teardown
```

**Auto-shoot** fires automatically whenever: launcher is at setpoint, turret is aimed, the shot is achievable, at least 150ms has passed since the last shot, and robot speed is acceptable.

### Named Commands (PathPlanner Event Markers)
PathPlanner paths can trigger named commands at waypoints:
- `enableAutoShoot` / `disableAutoShoot`
- `RunIntake` / `RetractIntake`
- `SmartLaunch`
- `holdFire` / `releaseFire` (suppress/allow feeding)
- `StowHood`

---

## Passing System

When the robot is outside its alliance zone, the ShootingCoordinator switches from hub shots to pass shots — launching fuel across the field to teammates.

**Two pass strategies:**
- **Symmetric**: Picks left or right trench target based on robot's Y position on the field
- **Driver Station**: Picks target based on FMS station number, uses a lob trajectory that clears the hub net

Pass shots use two-point trajectory math: the system solves for the unique parabola that passes through a clearance point (e.g., above the net) and lands at the target.

---

## Configuration System

The robot supports two physical configurations via `RobotConfig`:
- **MainBot**: 31"×23.5" chassis, NEO Vortex drive, full shooter system
- **SquareBot**: 21.25" chassis, NEO drive, testing platform

All PID gains, feedforward values, shot presets, and physical dimensions are tunable live from the SmartDashboard. The `LoggedTunableNumber` system detects changes and applies them without redeploying code.

---

## Summary Data Flow Diagram

```
            INPUTS                    BRAIN                     OUTPUTS
   ┌─────────────────┐      ┌───────────────────┐      ┌──────────────────┐
   │  Driver Joystick │─────→│                   │─────→│  Drive Motors    │
   │  Button Box      │      │   RobotContainer  │      │  (4 swerve)      │
   │  Vision Cameras  │─────→│        +          │─────→│  Turret Motor    │
   │  Encoders        │      │   Command         │      │  Launcher Motors │
   │  Pigeon IMU      │─────→│   Scheduler       │─────→│  Hood Motor      │
   │  AprilTags       │      │        +          │      │  Motivator Motor │
   │  Dashboard       │─────→│   Shooting        │─────→│  Spindexer Motor │
   │                  │      │   Coordinator     │      │  Intake Motors   │
   │                  │      │        +          │      │  LEDs            │
   │                  │      │   ShotCalculator  │      │                  │
   └─────────────────┘      └───────────────────┘      └──────────────────┘
```
