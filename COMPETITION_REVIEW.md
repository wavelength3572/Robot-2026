# GVSU Week 0 Competition Review

**Event:** GVSU Week 0 — February 21, 2026
**Branch:** GVSUCompetitionWeek0

---

## What We Fixed During Competition

- **Intake motor inversion** — motors were spinning the wrong direction; inverted to correct
- **Intake position tuning** — reworked to a 3-state system (stow / deploy / score) with adjusted setpoints
- **Drive axis/motor inversions** — drive felt wrong; corrected axis mapping and motor inversion flags
- **Auto odometry reset** — added heading reset at the start of auto paths (commit `700c8ac`); **untested on real hardware**
- **Intake roller behavior during shooting** — rollers were running when they shouldn't; fixed sequencing

---

## What Worked

- **Depot auto** — 8-ball burst, consistent ~3100 RPM, performed well in both logged matches
- **SmartLaunch / odometry-based aiming** — worked reliably in teleop
- **Turret and hood PID** — tracked targets well, no instability observed
- **Teleop drive** — drove well after the inversion fixes

---

## Open Issues

Each issue below is scoped so it can be tackled independently in a future session.

---

### 1. [QUICK FIX] Leading space in `' AutoAimTurret'` path event trigger

**What:** Named command has a leading space (` AutoAimTurret` instead of `AutoAimTurret`) in one or more path files. PathPlanner silently fails to match it — the command never runs.

**Impact:** Any auto that relies on this event trigger to aim the turret mid-path is firing a no-op. This is the leading theory for the bad turret heading in the human player auto.

**Files:** `src/main/deploy/pathplanner/paths/` — search all `.path` files for `" AutoAimTurret"` (with leading space)

**Fix:** Remove the leading space from the event trigger name in every affected path file. No Java changes needed.

---

### 2. [INVESTIGATE] PhotonVision TimeSyncServer not configured

**What:** Both coprocessors report `Long.MAX_VALUE` since last pong — they have never successfully synced time with the roboRIO.

**Impact:** Vision pose corrections are being applied with wrong timestamps. This corrupts pose estimation and can cause the robot to drive to the wrong location when relying on vision.

**Files:** `Robot.java` (TimeSyncServer initialization), PhotonVision network settings on each Pi

**Fix:**
1. Enable `TimeSyncServer` on the roboRIO in `Robot.java`
2. Point each Pi's PhotonVision config to the roboRIO TimeSyncServer address
3. Verify both cameras show a valid last-pong time after enabling

---

### 3. [INVESTIGATE] ShootingCoordinator.periodic() loop overruns

**What:** `ShootingCoordinator.periodic()` is consistently taking 20–45 ms every cycle in both match logs. This is the largest steady-state loop time offender.

**Impact:** Loop overruns delay all other periodic tasks and can cause control instability.

**Files:** `ShootingCoordinator.java`

**Fix:**
1. Profile what is running inside `periodic()` — look for expensive calculations, repeated trigonometry, or excessive logging calls
2. Cache computed values that don't change every cycle
3. Move any non-time-critical work out of the 20 ms loop

---

### 4. [TEST NEEDED] Human player auto — turret aimed at ~-120° instead of ~+30°

**What:** During the human player auto run, the turret aimed badly (reported ~-120°). No log is available for this run.

**Leading theory:** Issue #1 (leading space no-op) caused `AutoAimTurret` to never fire, leaving the turret at its default/previous position.

**Secondary theory:** The new auto odometry reset (commit `700c8ac`) may have set a wrong initial heading in this specific path.

**Fix:** First apply fix #1, then re-run the human player auto in testing. If the problem persists, compare the expected vs. actual odometry heading at path start and check the reset logic in `700c8ac`.

---

### 5. [HARDWARE] BackRight camera unreliable

**What:** The BackRight camera was completely non-functional in the 4 PM match. It came back in the 6 PM match. Intermittent failure pattern.

**Impact:** Reduced vision coverage; localization degrades when this camera is missing.

**Fix:**
1. Inspect the USB/ethernet cable and connector on the BackRight Pi — reseat or replace
2. Confirm the Pi boots reliably and shows up on the network at startup
3. Consider adding a network watchdog or connection indicator to AdvantageScope logging

---

### 6. [CODE QUALITY] Camera names logged as camera0/1/2/3

**What:** Vision cameras appear in AdvantageScope logs as `camera0`, `camera1`, etc. instead of descriptive names like `BackLeft`, `BackRight`, `FrontLeft`, `FrontRight`.

**Impact:** Debugging and post-match analysis is much harder — you can't tell at a glance which camera dropped.

**Files:** `Vision.java` or `VisionConstants.java` — wherever camera logging keys are defined

**Fix:** Change the logging key strings to use the camera's descriptive name (e.g., `"Vision/BackLeft/..."` instead of `"Vision/camera1/..."`).

---

### 7. [RETHINK] Intake deploy arm — violent/jerky movement

**What:** The arm requires a high P gain to deploy in time, but this causes violent, jerky movement throughout the travel range.

**Root cause:** The effective load on the arm motor changes significantly through the range of motion (nonlinear plant due to gravity and linkage geometry). A fixed P gain that works at one point in travel will be too aggressive or too slow at others.

**Impact:** Mechanism stress, potential for missed positions, and possible alliance partner/field hazard during matches.

**Fix options (needs architecture discussion):**
- **Motion profile (trapezoidal):** Limit velocity and acceleration rather than going full-speed to setpoint. WPILib `TrapezoidProfile` or PathPlanner on-the-fly
- **Gravity feedforward:** Add a feedforward term that compensates for the arm's weight as a function of angle — reduces the work the P term has to do
- **Both:** A motion profile with feedforward is the standard FRC approach for arm control and would likely eliminate the violence entirely

This one needs design discussion before coding — do not just tune P down without addressing root cause.

---

### 8. [MONITOR] Launcher follower velocity mismatch

**What:** During one bench test burst in the logs, the follower motor ran approximately 33% faster than the leader.

**Current assessment:** This may be a spin-up transient (follower responds faster before back-EMF builds). Has not been confirmed as a persistent issue.

**Fix:** Monitor across future test sessions. If it recurs during steady-state operation (not just spin-up), investigate:
- Motor controller follower configuration (is it truly in follower mode, or just matching a setpoint?)
- Whether the two motors are mechanically coupled correctly
- Possible encoder scaling issue on one side

---

## What We DON'T Have

- **No log from the human player auto run** — can't confirm turret behavior from data; must reproduce in testing
- **Auto odometry reset is untested on real hardware** — commit `700c8ac` was written day-of and never validated on the field
