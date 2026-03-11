# Firmware Rev 26 Turret Breakage Analysis

**Date**: 2026-03-11
**Issue**: Turret broken after firmware update — PID/feedforward behavior changed

## Current Software Versions

- **REVLib**: `2026.0.4` (vendordeps/REVLib.json)
- **SPARK MAX firmware**: 25.0.0+ (required by REVLib 2026)
- **Phoenix 6**: `26.1.1` (drive motors only, not turret)

## Root Cause: PIDF → New Feedforward Model

SPARK firmware 25+ (which REVLib 2026 requires) **replaced the old PIDF closed-loop model** with a new physics-based feedforward system.

### Old System (Firmware 24 / REVLib 2024)

```
output = kP*error + kI*integral + kD*derivative + kF*setpoint
```

- `kF` was a simple multiplier applied to the **setpoint**, producing a **duty cycle** contribution
- Units: duty cycle per unit of setpoint
- Single feedforward term built into the PIDF loop

### New System (Firmware 25+ / REVLib 2025-2026)

```
output = PID_output + kS*sign(velocity) + kV*velocity + kA*acceleration + [kG or kCos*cos(pos)]
```

- `kV` is in **volts per RPM** (not duty cycle)
- `kS` overcomes static friction (volts)
- `kA` tracks acceleration (volts per RPM/s)
- `kG`/`kCos` handle gravity for elevators/arms
- Arbitrary feedforward (4th arg to `setSetpoint()`) added **on top** as voltage
- All feedforward is now voltage-based, not duty-cycle-based

## Turret-Specific Issues

### Current turret configuration

From `MainBotConfig.java`:
- `turretKp = 0.15`
- `turretKd = 0.0`
- No kI, no firmware-side feedforward (kS/kV/kA/kG all defaulting to 0)
- Current limit: 10A
- Motor: NEO 550 via SparkMax (CAN ID 50)
- Gear ratio: 55:1 total

### Problem 1: Hardcoded 0.13V Arbitrary Feedforward

In `TurretIOSparkMax.java:242-246`:
```java
motorController.setSetpoint(
    degreesToMotorRotations(targetInsideDeg),
    ControlType.kPosition,
    ClosedLoopSlot.kSlot0,
    0.13);  // Always 0.13V regardless of direction
```

This constant 0.13V is applied in **every** setpoint command regardless of error direction. Under the old firmware, this likely interacted differently with the PIDF loop. Under the new firmware, it's a constant voltage bias that:
- Helps in one direction but **fights** the motor in the other direction
- May cause asymmetric turret behavior (fast one way, sluggish the other)

### Problem 2: PID Gain Scaling Changed

The old firmware's PIDF operated primarily in duty cycle space. The new firmware's PID output combines with feedforward in voltage space (with 12V voltage compensation enabled). The same `kP = 0.15` produces a different effective response under the new math.

### Problem 3: No Static Friction Compensation

The turret has no `kS` configured. The old `kF * setpoint` term in PIDF would scale with the setpoint and help push through friction. With the new firmware, there's no equivalent unless you explicitly configure `kS`.

The TODO in `Turret.java:38-39` acknowledges this:
> "Turret uses PD-only control (no kI, no feedforward). This causes steady-state error where the motor can't push through friction at small errors."

## Why Launcher/Motivator/Spindexer Are Probably Fine

These subsystems use a different pattern:
- WPILib `SimpleMotorFeedforward` computes feedforward voltages on the roboRIO
- Voltage passed as arbitrary feedforward via `ArbFFUnits.kVoltage` to `setSetpoint()`
- On-controller PID gains are tiny (e.g., launcher `kP = 0.00004`) — just error correction
- Feedforward does the heavy lifting externally

This pattern doesn't rely on the firmware's internal feedforward model, so it's unaffected by the change.

## Recommended Fix Plan

### Priority 1: Remove hardcoded 0.13V feedforward
**File**: `TurretIOSparkMax.java:246`
Change the 4th argument from `0.13` to `0.0`, or remove it entirely. This constant voltage bias is likely the primary cause of erratic behavior.

### Priority 2: Re-tune turret kP
**File**: `MainBotConfig.java:116`
Start fresh with tuning. Under the new firmware with 12V voltage compensation, try starting at `kP = 0.1` and increase until response is adequate without oscillation.

### Priority 3: Add kS for friction compensation
**File**: `TurretIOSparkMax.java` (motor configuration section)
Add firmware-side static friction compensation:
```java
motorConfig.closedLoop.feedForward.kS(value);
```
Start around 0.05-0.15V. This replaces what the old kF partially handled.

### Priority 4: Add kD for damping
**File**: `MainBotConfig.java:117`
Currently `0.0`. Adding a small kD will help dampen oscillation as you increase kP.

### Priority 5: Make arbitrary FF directional (if needed)
**File**: `TurretIOSparkMax.java:242-246`
If friction compensation via firmware kS isn't sufficient, compute directional feedforward:
```java
double arbFF = Math.signum(error) * staticFrictionVolts;
motorController.setSetpoint(targetRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFF);
```

## References

- [REVLib Changelog](https://docs.revrobotics.com/revlib/install/changelog)
- [REV Feed Forward Control Docs](https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control)
- [REV PID Tuning Guide](https://docs.revrobotics.com/revlib/spark/closed-loop/getting-started-with-pid-tuning)
- [REV Closed Loop Getting Started](https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started)
