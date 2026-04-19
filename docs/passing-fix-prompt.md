# Prompt: Prototype a geo-fenced passing mode

Paste this into a fresh Claude Code conversation opened in the Robot-2026 repo.

---

## Context

I'm working on the 2026 FRC robot in this repo. It's command-based, Java,
AdvantageKit-logged. The shooting subsystem aims in three modes — `HUB`,
`PASS`, `LONG_PASS` — via a `ShootingCoordinator` and `ShotCalculator`.
HUB shots work great. **PASS shots fail about 50% of the time**, and I want
to prototype a replacement aiming path for passes that ignores the current
precision-aim math.

## What I learned from replaying 8 match logs (in `StatesLogs/`)

- 28 of 57 `PASS`/`LONG_PASS` held-fire intervals produced **zero balls**.
- Root cause: velcomp **divergence gate at 20°** blocks feeding. Observed
  mean divergence across PASS intervals is 20–40°.
- `PassContractionRate` is near zero — the 3-pass iterative solver finds
  a fixed point fast, but at a high-residual one for pass geometry.
  Adding iterations won't help (already contracting).
- TOF for passes is 1.2–1.8 s (vs ~0.4 s for hub). That's the fundamental
  reason the iterative lead-angle solver's initial guess is far from the
  fixed point — long flight time amplifies any state error.

### Ruled out as causes
- Launcher tracking is fine (<3% wheel-RPM error during PASS intervals).
- Applied output never saturates (0 of 125 intervals pegged at 1.0).
- Battery sag is present but not the main blocker.
- Motivator / spindexer / launcher-ready: 0 failures — downstream actuators
  are healthy.
- Driver speed is only weakly correlated with divergence. Mean PASS speed
  is 1.4 m/s; 9% of firing time is over the 1.75 m/s zone gate.

## What I actually want from passing

I do **not** need precision aim. The target is a ~4 m × 4 m alliance zone,
not a point. My two requirements:

1. **Fire reliably** — don't sit blocked for 5+ seconds while I'm holding
   the button.
2. **Don't shoot off the field** — safety gate must be strict about that.

Landing within the alliance zone is good enough; anywhere inside counts.

## Proposed approach

Replace the precision-aim gate with a **predicted-landing-point gate**.

1. For `AimMode == PASS` (and `LONG_PASS`), bypass the iterative velcomp
   solver. I believe I'm already using `Pass Waypoint` strategy but it's
   being routed through the same divergence gate.
2. Aim at the waypoint's fixed zone-center target.
3. Optional: apply a **single-pass** open-loop lead correction
   (`aim += robot_velocity × tof_estimate`). Do not iterate.
4. Replace the 20° divergence gate with a **geo-fence**: compute the
   predicted ball landing point from current aim + robot state, and
   approve the shot iff the landing point is inside a safe polygon
   (the alliance zone shrunk inward by ~0.5 m margin).
5. Keep the existing speed cap (~1.5 m/s) for passes.
6. **Do not change HUB logic.**

## Files to read first (before changing anything)

- `src/main/java/frc/robot/subsystems/shooting/ShootingCoordinator.java`
- `src/main/java/frc/robot/subsystems/shooting/ShotCalculator.java`
- `src/main/java/frc/robot/subsystems/shooting/WaypointPassStrategy.java`
- `src/main/java/frc/robot/subsystems/shooting/FixedHeightPassStrategy.java`
- `src/main/java/frc/robot/FieldConstants.java`
- `src/main/java/frc/robot/util/ZoneDetector.java`
- `src/main/java/frc/robot/commands/ShootingCommands.java` (especially
  the feed-gate logic around line 560–780 where motivator/spindexer are
  gated on aim readiness)
- `src/main/java/frc/robot/MainBotConfig.java` (for tunables)

## Logged signals you can verify against in `StatesLogs/*.wpilog`

```
/RealOutputs/SmartLaunch/Status/AimMode            string  HUB | PASS | LONG_PASS | NONE
/RealOutputs/SmartLaunch/Status/Strategy           string  "Pass FixedHeight" | "Pass Waypoint" | ...
/RealOutputs/SmartLaunch/Status/Zone               string
/RealOutputs/SmartLaunch/CoordinatorState          string  INACTIVE | AIMING | FIRING | ...
/RealOutputs/SmartLaunch/Blocking                  string  freshest reason a shot is blocked
/RealOutputs/SmartLaunch/VelocityComp/DivergenceDeg        double
/RealOutputs/SmartLaunch/VelocityComp/PassContractionRate  double
/RealOutputs/SmartLaunch/Distance/TOF              double  (seconds)
/RealOutputs/SmartLaunch/Target/Achievable         boolean
/RealOutputs/SmartLaunch/Pass/Waypoint             struct:Pose3d
/Launcher/WheelVelocityRPM                         double
/Launcher/TargetVelocityRPM                        double
/Turret/CurrentInsideAngleDeg                      double
/Turret/TargetInsideAngleDeg                       double
/RealOutputs/Drive/RobotSpeedMps                   double
/RealOutputs/Launcher/BallImpact/Count             int64
```

A Python tool that ingests wpilogs and writes per-match JSON already
exists at `tools/analyze_wpilog.py` with analyzers in `tools/analyzers/`.
You can replay the 8 existing match logs without a robot.

## What I want you to deliver

1. **First read the files above and confirm how a PASS shot flows today.**
   Tell me, with `file:line` citations:
   - Which pass strategy is being selected for neutral-zone shots
     (likely `WaypointPassStrategy` based on what I saw).
   - Where `velcomp` is applied to that strategy's output.
   - Where the 20° divergence gate blocks firing.
   - Whether `Target/Achievable = false` is driven by the divergence gate
     or by something else.

2. **Prototype the new path as a feature-flagged alternative:**
   - Add `kUseGeoFencePass` (boolean) in `MainBotConfig` or equivalent.
   - When true, `AimMode == PASS | LONG_PASS` skips iterative velcomp,
     uses the waypoint strategy's static target, and replaces the
     divergence gate with a landing-point check.
   - Landing-point prediction: simple ballistic model from
     `{turret angle, hood angle, launcher wheel RPM, robot pose, robot
     velocity}`. If a drag coefficient is already defined for the ball,
     use it; otherwise start with vacuum ballistics and note the
     simplification. Any existing ballistic-model code in
     `ShotCalculator` should be reused.
   - Geo-fence polygon: defined in `FieldConstants` as a list of
     `Translation2d` vertices for the alliance zone, shrunk inward by
     a tunable margin (default 0.5 m). Point-in-polygon test each loop.
   - HUB logic is **unchanged**.

3. **Add new logged outputs** so we can verify the fix from replays:
   ```
   /RealOutputs/SmartLaunch/Pass/PredictedLandingX   double
   /RealOutputs/SmartLaunch/Pass/PredictedLandingY   double
   /RealOutputs/SmartLaunch/Pass/GeoFencePass        boolean
   /RealOutputs/SmartLaunch/Pass/FenceReason         string
   ```

4. **Write a short `docs/geo-fence-passing.md`** covering the flag,
   how the polygon is defined, how to tune the margin, and a one-line
   rollback instruction.

5. **Do not commit or push.** I want to inspect the diff first and
   test in simulation.

## Constraints

- Do **not** widen the existing 20° divergence gate. It's correct for
  hub shots and would cause balls off the field for passes.
- With `kUseGeoFencePass = false`, behavior must be identical to today.
- Preserve all existing NetworkTables log signal names/types.

## Nice-to-haves (only if time)

- A unit test in the existing test layout that feeds a few synthetic
  `(pose, velocity, aim)` tuples and asserts the geo-fence decision.
- A CLI or note showing how to replay a log through the new logic
  offline (we may add this analyzer to `tools/analyzers/` after you
  wire the Java side).
