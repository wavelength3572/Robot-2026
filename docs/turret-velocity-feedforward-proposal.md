# Turret Velocity Feedforward - Discussion Document

**Status:** Proposal for mentor review. No code changes until approved.

## Problem

Our turret uses pure PD position control (kP=0.12, kD=2.0) on a NEO 550 through
a 55:1 gear reduction. When the robot rotates or translates, the turret must
continuously adjust to track the target. PD control reacts to position error,
which means the turret always lags behind the target during motion. The faster
the robot spins, the larger the tracking lag.

## What Competitive Teams Do

### Team 254 (2020, 2022)
Feeds a velocity feedforward to the turret motor:
```
ffDegPerSec = -(robotOmegaDegPerSec + tangentialVelocityDegPerSec)
```
The turret motor preemptively counters the robot's rotation instead of waiting
for position error to build up.

### Teams 386 and Falcons (2026)
Compute turret angular velocity derivatives with moving-average filters
(window=5 samples). The filtered derivative is fed as velocity feedforward
alongside their PID position controller.

## Our Hardware

- **Motor:** NEO 550 (SparkMAX)
- **Gear ratio:** 55:1 total (10:1 internal NEO 550 + 5.5:1 external 66:12)
- **Controller:** SparkMAX PID, slot 0, PD only (kP=0.12, kD=2.0, kI=0)
- **Encoder:** REV Through Bore on turret shaft (absolute reference), motor
  encoder for closed-loop control
- **Range:** -180 to +180 degrees

## Proposed Implementation

Add a velocity feedforward term to the turret position command:

```java
// In ShootingCoordinator, after computing currentTurretAngleDeg:
double robotOmegaDegPerSec = Math.toDegrees(fieldSpeeds.omegaRadiansPerSecond);

// Tangential velocity component: how fast the aim point moves due to
// translational velocity at the current turret-to-target distance
double tangentialDegPerSec = computeTangentialVelocity(turretX, turretY,
    compensatedAimTarget, fieldSpeeds);

// Feedforward: turret must counter both rotation and translation
double ffDegPerSec = -(robotOmegaDegPerSec + tangentialDegPerSec);

// Clamp to safe range
double maxTurretVelDegPerSec = 360.0; // TBD based on motor limits
ffDegPerSec = Math.max(-maxTurretVelDegPerSec,
    Math.min(maxTurretVelDegPerSec, ffDegPerSec));

turret.setPosition(currentTurretAngleDeg, ffDegPerSec);
```

The `TurretIOSparkMax` would pass `ffDegPerSec` through the SparkMAX arbitrary
feedforward slot, converting deg/sec to motor voltage using the kV constant.

## Questions for Mentor

1. **SparkMAX arbitrary feedforward:** Is the arbitrary FF slot available on our
   turret PID controller? It needs to accept a voltage alongside the position
   setpoint.

2. **Max safe turret velocity:** What's the mechanical limit? We need to clamp
   the FF to avoid commanding velocities the turret can't physically achieve
   (belt skip, mechanical stress).

3. **Prototype path:** Should we prototype in TurretIOSim first? The sim uses
   a DCMotorSim with the correct 55:1 ratio, so we could validate FF behavior
   before touching hardware.

4. **Does PD already track well enough?** With our 55:1 ratio, the turret has
   high torque and fast response. If PD tracking error is small during normal
   driving, FF may not be worth the complexity. We should measure actual tracking
   error in AdvantageKit logs during practice to quantify the problem.

5. **Moving-average filter:** 386/Falcons filter the angular velocity derivative
   over 5 samples to smooth noise. Should we use the same approach, or is raw
   omega from the gyro clean enough?

## Estimated Effort

If approved: ~30 lines across `TurretIO`, `TurretIOSparkMax`, `TurretIOSim`,
and `ShootingCoordinator`.

## Risk Assessment

- **Low risk if clamped properly.** The FF only adds a small voltage on top of
  PD control. If the FF is wrong, PD still corrects the position.
- **Medium risk if unclamped.** Runaway FF could command full-speed turret
  rotation and stress mechanical components.
- **Zero risk to test in sim first.** TurretIOSim already models the motor and
  gear ratio accurately.
