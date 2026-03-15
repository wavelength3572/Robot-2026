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

## Change 5: Launcher Position Transform

**Problem:** Your shot calculator computes distance from the robot center to the
hub. The ball actually launches from the turret/launcher, which is offset from
center and moves with turret rotation. At close range this offset matters —
it's potentially 8+ inches of error.

**What 5962 does:** Rotates the launcher offset by robot heading and adds to pose:
```java
launcherX = robotX + offsetX*cos(h) - offsetY*sin(h);
launcherY = robotY + offsetX*sin(h) + offsetY*cos(h);
```
They also compute the launcher's velocity including the rotational component:
```java
vx += (-offsetY_field) * omega;
vy += (+offsetX_field) * omega;
```

**Proposed change in `ShotCalculator`:**

1. Add launcher offset constants to `RobotConfig`:
   ```java
   double getLauncherOffsetX(); // meters forward of center
   double getLauncherOffsetY(); // meters left of center
   ```
2. In `calculateShotToHub()`, transform robot pose to launcher pose using
   heading + turret angle
3. Add rotational velocity component to the field velocity used for compensation
4. Use the transformed position for distance calculation and aim angle

**Files touched:** `ShotCalculator.java`, `RobotConfig.java`, `MainBotConfig.java`
**Risk:** Low-medium — changes distance calculation. Easy to A/B test by toggling
the offset to zero.
**Tuning:** Two constants from CAD measurements. Not runtime-tunable (fixed geometry).

---

## Change 6: Speed Gate for Shot Validity

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

## Change 7: Angular Velocity Feedforward for Drive Heading

**Problem:** When shooting on the move, the robot needs to maintain a specific
field-relative heading (aim angle). Your swerve heading controller reacts to
error, but doesn't anticipate the rate at which the aim angle changes as the
robot moves past the hub.

**What 5962 does:** Computes the angular rate of the aim angle geometrically:
```java
// tangentialVel = cross product of relative position and velocity, divided by distance
tangentialVel = (ry*vx - rx*vy) / distance;
aimAngularRate = tangentialVel / distance;
```
This is output alongside the aim angle so the swerve heading controller can
use it as a feedforward term.

**Proposed change:**

1. Compute `aimAngularVelocityRadPerSec` in `ShotCalculator` alongside the
   turret angle
2. Pass it through `ShotResult` to `ShootingCoordinator`
3. Feed it to the turret as a velocity feedforward term (supplement PD control)

This is especially valuable for your turret — PD-only control (no integral,
no feedforward) means you're always chasing the target. A feedforward term
lets the turret lead the target during SOTM.

**Files touched:** `ShotCalculator.java`, `ShootingCoordinator.java`, `Turret.java`
**Risk:** Low — feedforward is additive to existing PD. Set coefficient to 0 to disable.
**Tuning:** Feedforward gain as `LoggedTunableNumber`.

---

## Implementation Priority

| Priority | Change | Impact | Risk | Effort |
|----------|--------|--------|------|--------|
| **1** | Tilt gate (#4) | Safety | Very low | Small |
| **2** | Speed gate (#6) | Safety + reliability | Very low | Small |
| **3** | Confidence scoring (#2) | Shot selection quality | Low | Medium |
| **4** | Second-order latency comp (#1) | Accuracy during accel/decel | Low | Small |
| **5** | Launcher position transform (#5) | Close-range accuracy | Low-med | Small |
| **6** | Angular velocity feedforward (#7) | Turret tracking during SOTM | Low | Medium |
| **7** | Newton-Raphson solver (#3) | SOTM convergence + drag modeling | Medium | Large |

Recommended approach: Ship changes 1–2 immediately (safety nets, minimal risk).
Develop 3–4 together (they complement each other — confidence scoring uses the
improved pose prediction). Changes 5–7 are the SOTM accuracy package and should
be developed and tested as a group, ideally with the old 3-iteration solver
available as a fallback toggle.

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
