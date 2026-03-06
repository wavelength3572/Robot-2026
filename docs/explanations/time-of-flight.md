# Time of Flight (ToF) — What It Means in This Project

## Overview

In Robot-2026, "time of flight" (ToF) refers to the **time a launched ball
spends traveling through the air** before reaching its target (typically the
hub).  It is *not* a distance sensor — it is a calculated or empirically
measured duration used by the shooting subsystem.

## Why It Matters

When the robot shoots **while moving**, the ball takes a non-trivial amount of
time to reach the target.  During that flight time the robot continues to move,
so the aiming system must "lead" the target — that is, aim slightly offset from
the true target position to compensate for the robot's velocity.  ToF is the
key variable that determines *how much* to lead.

Without ToF-based velocity compensation, shots consistently miss when the robot
is in motion.

## How ToF Is Determined

There are two approaches in the codebase:

### 1. Parametric (Physics-Based)

Defined in `ShotCalculator.calculateTimeOfFlight()`:

```
ToF = distance / (exitVelocity × cos(launchAngle))
```

This is simple projectile-motion physics: horizontal distance divided by
horizontal velocity.  It is easy to compute but ignores air resistance.

### 2. Lookup Table (Empirical)

Measured ToF values at known distances are stored in `shot-data/lut-data.json`
and loaded via `ShotLookupTable`.  Intermediate distances are interpolated.
These empirical values capture real-world drag that the parametric model
ignores, making them more accurate.

## Velocity Compensation Loop

Regardless of which ToF source is used, the compensation algorithm is the same
and runs **3 iterations** to converge:

1. Estimate ToF for the current distance to the target.
2. Predict an adjusted aim point: `target − (robot velocity × ToF)`.
3. Clamp the aim offset to a maximum of 1.5 m so it cannot drift off-field.
4. Recalculate the distance to the new aim point and re-estimate ToF.
5. Repeat from step 2.

This logic appears in:

- `ShotCalculator.calculateHubShot()` — parametric strategy
- `LUTShotStrategy` — pure lookup-table strategy
- `HybridShotStrategy` — LUT ToF with parametric shot parameters

## Shot Strategies and ToF Sources

| Strategy      | Shot params from | ToF from     | Best for                          |
|---------------|------------------|--------------|-----------------------------------|
| Parametric    | Physics solver   | Physics calc | No empirical data available yet   |
| LUT           | Lookup table     | Lookup table | Full empirical dataset            |
| Hybrid        | Physics solver   | Lookup table | Sparse data + better move-accuracy|

## Key Files

| File | Role |
|------|------|
| `ShotCalculator.java` | Physics-based ToF calculation, velocity compensation helpers |
| `ShotLookupTable.java` | Empirical ToF storage and interpolation |
| `LUTShotStrategy.java` | Shot strategy using LUT ToF for compensation |
| `HybridShotStrategy.java` | Hybrid strategy (LUT ToF + parametric solver) |
| `StationaryShotBatchRecorder.java` | Records measured ToF during practice |
| `shot-data/lut-data.json` | Stored empirical ToF data |
| `docs/lessons/shoot-on-the-move-lesson-plan.md` | In-depth lesson plan |

## Safety Bounds

- **Max aim offset**: 1.5 m (`ShotCalculator.MAX_AIM_OFFSET_METERS`) prevents
  the aim point from drifting off-field when ToF is large.
- **Unachievable-shot fallback**: If the optimizer cannot find a valid shot
  (exit velocity = 0), ToF becomes `Double.MAX_VALUE`.  The system detects this
  and disables velocity compensation rather than aiming at garbage coordinates.
