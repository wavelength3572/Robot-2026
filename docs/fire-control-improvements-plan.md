# Fire Control Improvements Plan
## Inspired by FRC 5962's frc-fire-control, adapted for Team 3572

---

## Change 1: Second-Order Latency Compensation

**Problem:** Your velocity compensation currently uses first-order pose projection
(Twist2d at 20ms). During acceleration and deceleration — exactly when SOTM
matters most — this under/overshoots the predicted pose.

**What 5962 does:** They compute acceleration via finite-difference of velocity
between cycles (`a = (v_now - v_prev) / 0.02`), then apply a second-order
Taylor expansion through the Lie group exponential:
```
displacement = v*dt + 0.5*a*dt²
```
They also separate **vision pipeline latency** (30ms, used for pose prediction)
from **mechanism latency** (20ms, added to solved TOF).

**Proposed change in `ShotCalculator.java`:**

1. Store `prevFieldVx`, `prevFieldVy`, `prevOmega` from the previous cycle
2. Compute acceleration: `ax = (vx - prevVx) / 0.020`
3. Replace the current pose used for distance calculation with:
   ```java
   double dt = VISION_LATENCY_SEC; // new constant, ~0.030s
   Pose2d compensatedPose = rawPose.exp(new Twist2d(
       vx*dt + 0.5*ax*dt*dt,
       vy*dt + 0.5*ay*dt*dt,
       omega*dt + 0.5*alpha*dt*dt));
   ```
4. Add a separate `MECHANISM_LATENCY_SEC` (~0.020s) constant that gets added
   to the TOF used for velocity compensation (but NOT for LUT/parametric lookup)

**Files touched:** `ShotCalculator.java`
**Risk:** Low — purely additive, falls back to first-order when acceleration is zero.
**Tuning:** Two new `LoggedTunableNumber` constants for the two latency values.

---

## Change 2: Shot Confidence Scoring

**Problem:** Your auto-shoot gating is binary — launcher at setpoint, turret aimed,
valid shot exists. There's no graduated measure of "how good is this shot?" This
means you either shoot marginal shots or miss good windows due to rigid thresholds.

**What 5962 does:** A 5-factor weighted geometric mean producing a 0–100 score.
Any single factor at zero kills the entire score. Heading accuracy is weighted
highest (1.5x), meaning aim alignment dominates.

**Proposed change — new class `ShotConfidence.java`:**

```java
public class ShotConfidence {
    // Factor 1: Velocity stability (weight 0.8)
    //   clamp(1.0 - |currentSpeed - prevSpeed| / 0.5, 0, 1)
    //   Penalizes acceleration/deceleration transients

    // Factor 2: Vision confidence (weight 1.2)
    //   Passthrough from Vision subsystem, clamped [0,1]
    //   Zero vision = zero confidence (no blind shots)

    // Factor 3: Turret aim accuracy (weight 1.5)
    //   clamp(1.0 - |aimError| / scaledMaxError, 0, 1)
    //   scaledMaxError shrinks with distance, grows with speed

    // Factor 4: Distance quality (weight 0.5)
    //   Triangular: 1.0 at mid-range, 0.0 at range edges
    //   Penalizes extreme close/far shots

    // Factor 5: Launcher readiness (weight 1.0)
    //   clamp(1.0 - |rpmError| / toleranceRPM, 0, 1)
    //   Graduated instead of binary at-setpoint check

    public double calculate() {
        // Weighted geometric mean: exp(Σ(w_i * ln(c_i)) / Σw_i) * 100
    }
}
```

**Integration into `ShootingCoordinator.runAutoShoot()`:**

Replace the current binary readiness checks with:
```java
double confidence = shotConfidence.calculate();
boolean shouldShoot = confidence >= minShootConfidence; // tunable threshold, e.g. 60
```

Log the confidence score and all 5 component factors to AdvantageKit for
post-match analysis. This lets you tune the threshold empirically by reviewing
match logs: "we had a 45-confidence shot at 3:22 — should we have taken it?"

**Files touched:** New `ShotConfidence.java`, modify `ShootingCoordinator.java`
**Risk:** Low — existing binary checks become a special case (threshold = 0).
**Tuning:** 5 weights + 1 threshold, all `LoggedTunableNumber`.

---

## Change 3: Newton-Raphson SOTM Solver (replaces iterative refinement)

**Problem:** Your current velocity compensation uses a 3-iteration refinement loop
(Oblarg/CD approach): compute static distance → look up TOF → offset aim point
by `v*TOF` → repeat. This converges slowly and doesn't account for air drag on
the inherited robot velocity. At 3 m/s robot speed, the aim-point error after
3 iterations can still be centimeters off.

**What 5962 does:** A proper Newton-Raphson root-finding solver with:
- Exponential drag model: displacement = `v * (1 - e^(-c*tof)) / c` instead of `v * tof`
- Analytic derivative via chain rule for fast convergence (1-2 iterations with warm start)
- Warm-starting from previous cycle's solved TOF
- Divergence guards and TOF clamping

**Proposed change — refactor velocity compensation in `ShotCalculator`:**

```
// Current approach (3 fixed iterations, no drag):
for i in 0..2:
    tof = strategy.lookupTOF(distance)
    aimPoint = target - velocity * tof
    distance = |aimPoint - launcher|

// Proposed approach (Newton solver with drag):
tof = warmStartTOF > 0 ? warmStartTOF : initialLookupTOF
for i in 0..maxIter:
    dragDrift = (1 - exp(-c * tof)) / c
    projectedDistance = |target - velocity * dragDrift - launcher|
    lookupTOF = strategy.lookupTOF(projectedDistance)
    residual = lookupTOF - tof
    if |residual| < 0.001: break
    derivative = d(lookupTOF)/d(tof)  // chain rule through drag and LUT
    tof -= residual / derivative
    tof = clamp(tof, 0.05, 5.0)
warmStartTOF = tof
```

This works with BOTH your parametric and LUT strategies — the solver is agnostic
to how TOF is computed from distance.

**Key detail — the derivative:**
```
d(projDist)/d(tof) = -e^(-c*tof) * (rx*vx + ry*vy) / projDist
d(lookupTOF)/d(projDist) = central difference with h=0.01m
f'(tof) = d(lookupTOF)/d(projDist) * d(projDist)/d(tof) - 1.0
```

**Files touched:** `ShotCalculator.java` (velocity compensation method)
**Risk:** Medium — changes core shot math. Requires careful testing. Keep old
3-iteration method behind a strategy toggle during development.
**Tuning:** `sotmDragCoeff` (start at 0.47), `maxIterations` (25), `convergenceTolerance` (0.001s).
All `LoggedTunableNumber`.

---

## Change 4: Tilt Gate (Bump/Ramp Rejection)

**Problem:** If the robot drives over a bump or field element, the launcher tilts
and shots go wild. Your code has no protection against this.

**What 5962 does:** Reads pitch and roll from the gyro. If either exceeds 5°,
the shot is invalidated entirely.

**Proposed change in `ShootingCoordinator`:**

1. Read pitch/roll from the Pigeon 2 gyro (already available via `GyroIO`)
2. Add a tilt check before auto-shoot:
   ```java
   double pitch = Math.abs(gyro.getPitch());
   double roll  = Math.abs(gyro.getRoll());
   boolean tiltSafe = pitch < maxTiltDeg && roll < maxTiltDeg;
   ```
3. When tilt exceeds threshold: suppress feeding, log a warning, continue
   tracking (turret/hood/launcher stay active for instant recovery)

**Files touched:** `ShootingCoordinator.java`, possibly `GyroIO.java` if pitch/roll
aren't already exposed in inputs.
**Risk:** Very low — purely additive safety gate.
**Tuning:** `maxTiltDeg` as `LoggedTunableNumber`, default 5.0°.

---

## ~~Change 5: Launcher Position Transform~~ — ALREADY IMPLEMENTED

**Status:** Your code already does this. `ShotCalculator.getTurretFieldPosition()`
(line 267) and `Turret.calculateOutsideTurretAngleFromTurret()` (line 233) both
rotate `xOffset/yOffset` by robot heading. The `TurretConfig` record carries
`heightMeters`, `xOffset`, and `yOffset`, and every distance calculation in
`ShotCalculator` uses the transformed turret position, not robot center.

**One minor gap:** 5962 also adds the rotational velocity component to the field
velocity (`vx += -offsetY_field * omega`). This is a small effect at typical
offsets and will be naturally incorporated as part of the Newton solver work
(Change 3) if you choose to include it there. Not worth a separate change.

---

## Change 5: Speed Gate for Shot Validity

**Problem:** Your auto-shoot has a single speed gate (`maxFeedSpeedMps = 1.75 m/s`)
that suppresses feeding. But there's no upper bound that invalidates the shot
entirely. At very high speeds (3+ m/s), the velocity compensation becomes
unreliable and shots should be refused, not just delayed.

**What 5962 does:** Two-tier speed gating:
- Below 0.1 m/s → static shot (skip SOTM solver entirely, direct LUT lookup)
- Above 3.0 m/s → `INVALID` (refuse to shoot)

**Proposed change:**

1. Add `maxSOTMSpeed` constant (default 3.0 m/s) — above this, `currentShot`
   is set to null / non-achievable
2. Add `minSOTMSpeed` constant (default 0.1 m/s) — below this, skip velocity
   compensation entirely for a cleaner static shot
3. Keep existing `maxFeedSpeedMps` as the feed-suppression gate within the valid range

This creates three zones:
```
[0, 0.1) m/s  → static shot, no velocity comp
[0.1, 1.75) m/s → SOTM with auto-feed enabled
[1.75, 3.0) m/s → SOTM with feed suppressed (wait for decel)
[3.0, ∞) m/s  → shot invalid
```

**Files touched:** `ShootingCoordinator.java`, `ShotCalculator.java`
**Risk:** Very low — adds safety, never degrades existing behavior.
**Tuning:** Two new `LoggedTunableNumber` constants.

---

## ~~Change 7: Angular Velocity Feedforward for Turret~~ — DROPPED (Safety Concern)

**Why it's dropped:** On closer inspection, this is the riskiest proposed change:

1. **5962 doesn't actually do turret FF either.** Their `driveAngularVelocityRadPerSec`
   is a feedforward for the *swerve heading controller* (whole robot rotation),
   not an independent turret motor. Their turret aiming is position-only, same as yours.

2. **Turret FF failure modes are dangerous.** Your `setOutsideTurretAngle()` sends
   position commands with soft-limit clamping. A velocity feedforward bypasses that
   protection — if the FF calculation spikes (noisy velocity data, sign error, NaN),
   it commands the turret to spin into the hard stops, potentially damaging wiring,
   the slip ring, or the frame.

3. **The real gains come from better target prediction.** Changes 1 (second-order
   latency comp) and 3 (Newton solver) reduce the *error* the PD loop needs to
   chase. Your PD loop tracks position well enough — the problem is predicting the
   *right* position, not tracking faster.

**If you revisit this later:** The safe way would be to compute the expected
turret angular rate and feed it as an *additive voltage* (not velocity command)
with a hard clamp (e.g., ±1V max). But that's a future experiment, not something
to ship alongside the other changes.

---

## Implementation Priority (Revised)

| Priority | Change | Impact | Risk | Effort |
|----------|--------|--------|------|--------|
| **1** | Tilt gate (#4) | Safety | Very low | Small |
| **2** | Speed gate (#5) | Safety + reliability | Very low | Small |
| **3** | Confidence scoring (#2) | Shot selection quality | Low | Medium |
| **4** | Second-order latency comp (#1) | Accuracy during accel/decel | Low | Small |
| **5** | Newton-Raphson solver (#3) | SOTM convergence + drag modeling | Medium | Large |

~~#5 (Launcher position transform)~~ — Already implemented in your codebase.
~~#7 (Turret angular velocity FF)~~ — Dropped due to safety risk.

Recommended approach: Ship changes 1–2 immediately (safety nets, minimal risk).
Develop 3–4 together (they complement each other — confidence scoring uses the
improved pose prediction). Change 5 (Newton solver) is the big accuracy win and
should be developed with the old 3-iteration solver available as a fallback toggle.

---

## What NOT to adopt from 5962

- **Their LUT generator** (RK4 sim → 91-point table): Your empirical approach
  with `StationaryShotBatchRecorder` captures real-world effects that simulation
  misses. Keep it.

- **Their FuelPhysicsSim**: You already have `FuelSim`. Theirs is more elaborate
  (CCD, spatial hashing, Gauss-Seidel) but yours is integrated with your
  AdvantageKit logging. Not worth the migration cost.

- **Their behind-hub dot-product check**: Your field geometry and aiming helper
  already handle zone-based mode switching (SHOOT vs PASS). Their check is
  simpler but less flexible.
