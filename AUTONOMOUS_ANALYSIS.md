# Autonomous Routines Analysis & Gap Identification

## Overview

The autonomous system uses PathPlanner autos with a strategy-based wrapper architecture
(`AutoWrapperFactory`). Two configurable axes — **StartStrategy** and **PathShootingStrategy** —
combine to produce the full autonomous sequence. This is a well-designed, modular system.

## Current Autonomous Routines (21 autos)

| Category | Autos |
|----------|-------|
| **Comp (standard)** | TrenchRightSafe, TrenchLeftSafe, TrenchRightShotOnly, TrenchLeftShotOnly |
| **CompSafe (conservative)** | TrenchRightSnowblowSafe, TrenchLeftSnowblowSafe |
| **CompSprint (aggressive)** | TrenchRightSprintSafe, TrenchLeftSprintSafe |
| **CompStationaryShoot** | TrenchRight2Cycles, TrenchLeft2Cycles |
| **Specialty** | TrenchRight+Climb, TrenchRight+HumanPlayer, Depot, Depot2.0, Human Player |
| **Test/Utility** | Basic Loop 1/2, Basic Sweep Score On Left/Right, Angle Unwrapping Tests |

## Identified Gaps

### 1. Teardown Does Not Stop Turret, Hood, or Spindexer

**File:** `AutoWrapperFactory.java:212-222`

The `teardown()` method stops the launcher, motivator, and intake — but **not** the turret, hood,
or spindexer. If auto ends abruptly (e.g. disabled mid-auto, or the `finallyDo` fires during an
unexpected interruption), these subsystems may continue running their last commanded state into
teleop init.

**Recommendation:** Add `turret.stop()`, `hood.stop()` (or stow), and `spindexer.stop()` to the
teardown block.

### 2. No Null-Check on Turret, Hood, Spindexer, or Coordinator in Teardown

**File:** `AutoWrapperFactory.java:212-222`

Launcher, motivator, and intake are null-checked before stopping. If any of the other subsystems
were ever null (e.g. on SquareBot which may not have all subsystems), the teardown would throw a
NullPointerException. This is a minor concern since those subsystems are currently always present,
but inconsistent with the existing null-guard pattern.

### 3. No Timeout on the Overall Autonomous Sequence

**File:** `AutoWrapperFactory.java:58-126`

The individual phases have timeouts (initial launch: 3.5s, post-path: 10s), but there is no
overall timeout on the composed sequence. If PathPlanner hangs or a subsystem command never
finishes, the auto could run indefinitely until the FMS transitions to teleop. WPILib will cancel
on mode switch, but a safety timeout (e.g. 14.5s for a 15s auto period) would be defensive.

### 4. No Left-Side Equivalents for Specialty Autos

Only `TrenchRight+Climb` and `TrenchRight+HumanPlayer` exist — there are no `TrenchLeft+Climb` or
`TrenchLeft+HumanPlayer` variants. If the robot starts on the left side of the field, these
strategies are unavailable.

**Recommendation:** Create mirrored left-side versions, or confirm that PathPlanner's alliance
flipping handles this automatically (it handles Red/Blue flipping, but not left/right within the
same alliance).

### 5. Depot Autos Lack a "Safe" or "Sprint" Variant

The Depot autos (`Depot`, `Depot2.0`) exist but have no sprint or safe variants. If depot-side
starts are used in competition, the team has no fallback if the primary Depot auto fails or
conflicts arise.

### 6. No "Do Nothing" / Mobility-Only Auto

There is no minimal auto that just drives forward for mobility points without shooting. This is a
common fallback at competition when something is broken (launcher, turret, vision, etc.) and the
team just wants the mobility bonus.

**Recommendation:** Add a simple "MobilityOnly" auto that drives forward ~1.5m and stops.

### 7. Hold/Release Fire Named Commands Have No Auto-Reset

**File:** `RobotContainer.java:391-393`

The `autoFeedingSuppressed` flag is toggled by `holdFire`/`releaseFire` named commands but is never
explicitly reset at auto start. If auto is interrupted between `holdFire` and `releaseFire`, the
flag stays true, potentially suppressing feeding in the next auto run.

**Recommendation:** Reset `autoFeedingSuppressed = false` in `autonomousInit()` or at the start of
`compWrapped()`.

### 8. Starting Pose Resolution Silently Falls Through on Null

**File:** `AutoWrapperFactory.java:133-137`, `RobotContainer.java:795`

If `resolveStartingPose()` returns null (e.g. a misconfigured auto file), the odometry reset is
silently skipped. The robot then runs the entire auto with stale pose data, causing every
vision-less shot calculation to be wrong. There's no warning logged.

**Recommendation:** Log a warning when `startingPose` is null so the drive team notices on the
dashboard.

### 9. No Autonomous Self-Test or Pre-Match Validation

There is no pre-match check that validates subsystem readiness before auto starts (e.g. "is the
turret homed?", "is vision getting AprilTag locks?", "is the launcher motor responding?"). A
pre-match checklist command could catch hardware issues before the match begins.

### 10. Post-Path Shoot Phase Has a Long 10s Timeout

**File:** `AutoWrapperFactory.java:203-205`

The post-path smart launch has a 10-second timeout. In a 15-second auto period, if the path takes
10+ seconds, the post-path phase could extend well beyond the auto period. While the FMS mode
switch will cancel it, this means the robot may still be shooting as teleop begins rather than
transitioning to a ready state.

**Recommendation:** Consider calculating a dynamic timeout based on elapsed auto time, or reduce
the static timeout to 5-6 seconds.

## Strengths

- **Strategy composition is clean** — the `AutoWrapperFactory` pattern avoids code duplication
- **Dashboard-configurable strategies** allow on-the-fly adjustment without redeployment
- **Zone-based speed gating** prevents shots at unsafe speeds
- **Folder-based auto categorization** with competition mode filtering is well thought out
- **Named commands** (`holdFire`/`releaseFire`, `RunIntake`) provide mid-path control
- **`.asProxy()` usage** correctly avoids subsystem requirement conflicts in parallel groups
- **`finallyDo` teardown** ensures cleanup on interruption

## Summary

The autonomous system is architecturally solid. The most impactful gaps to address are:

1. **Incomplete teardown** (turret/hood/spindexer not stopped) — easy fix, prevents stale state
2. **Missing `autoFeedingSuppressed` reset** — could cause silent failures between auto runs
3. **No mobility-only fallback auto** — important competition safety net
4. **Silent null starting pose** — could cause an entire auto to shoot incorrectly with no warning
