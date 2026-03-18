# Autonomous Routines Analysis & Gap Identification

## Overview

The autonomous system uses PathPlanner autos with a strategy-based wrapper architecture
(`AutoWrapperFactory`). Three configurable axes — **StartStrategy**, **PathShootingStrategy**, and
**Pass Time Budget** — combine to produce the full autonomous sequence.

## Current Autonomous Routines (20 autos)

| Folder | Right | Left | Notes |
|--------|-------|------|-------|
| **Comp** | TrenchRightSafe, TrenchRight+HumanPlayer | — | **TrenchLeftSafe missing** |
| **CompSprint** | TrenchRightSprintSafe, TrenchRightSnowblowAggressive | TrenchLeftSprintSafe, TrenchLeftSnowblowSafe | Naming mismatch: Aggressive vs Safe |
| **CompStationaryShoot** | TrenchRight2Cycles | TrenchLeft2Cycles | Matched |
| **Old Autos** | TrenchRightShotOnly, TrenchRight+Climb | TrenchLeftShotOnly | +Climb is old/unused |
| **Other** | Depot, Depot2.0, Human Player | — | Right-side only (by design) |
| **Test** | Basic Loop 1/2, Basic Sweep L/R, Angle Unwrap Tests | — | Not competition |

### Auto Mirroring Gaps

| Missing Auto | Based On | Priority |
|-------------|----------|----------|
| **TrenchLeftSafe** | TrenchRightSafe (Comp) | **High** — only Comp-folder right-side auto with no left mirror |
| TrenchLeftSnowblowAggressive | TrenchRightSnowblowAggressive (CompSprint) | Medium — left side has "Safe" variant but not "Aggressive" |

Human Player autos are right-side only by field design. Climb autos are in Old Autos folder.

## Resolved Gaps (code changes made)

### 1. Teardown now stops spindexer (was missing)
**`AutoWrapperFactory.java`** — Added `spindexer.stopSpindexer()` with null check to `teardown()`.
Previously only stopped launcher, motivator, intake.

### 2. `autoFeedingSuppressed` reset at auto start
**`RobotContainer.java`** — `getAutonomousCommand()` now resets `autoFeedingSuppressed = false`
before building the command. Prevents stale holdFire state from a previous interrupted auto.

### 3. Warning on null starting pose
**`AutoWrapperFactory.java`** — `resetOdometry()` now calls `DriverStation.reportWarning()` when
starting pose is null, so the drive team sees it on the dashboard instead of silent bad shots.

### 4. Overall auto safety timeout
**`AutoWrapperFactory.java`** — Added 14.5s overall timeout on the composed sequence (15s auto
period minus 0.5s margin). Catches hung paths or stuck subsystem commands.

### 5. Pass Time Budget (new dashboard control)
**`ShootingCommands.java` / `AutoWrapperFactory.java` / `RobotContainer.java`**

New dashboard chooser "Auton Pass Time Budget" controls how long the spindexer feeds in neutral
(pass) zones per visit:
- **Pass All (no limit)** — current behavior, feed everything
- **Pass 2s then Hold** — feed for 2s in neutral, then suppress and reciprocate
- **Pass 4s then Hold** — feed for 4s in neutral, then suppress and reciprocate
- **Hold All in Neutral** — never feed in neutral zone, keep all balls for alliance scoring

The timer resets each time the robot leaves the pass zone, so multi-cycle autos (path → shoot →
path → shoot) get a fresh budget per neutral zone visit.

Logged to AdvantageKit: `ContinuousSmartLaunch/Gate/PassBudgetExhausted`,
`ContinuousSmartLaunch/Gate/PassSuppressed`, `ContinuousSmartLaunch/PassTimeRemainingSec`.

### 6. Turret pre-match validation (already existed)
Turret subsystem already has encoder error detection with dashboard warning and auto-lock.

## Remaining Items

- **TrenchLeftSafe** auto needs to be created in PathPlanner (mirror of TrenchRightSafe)
- Consider whether TrenchLeftSnowblowAggressive is needed alongside TrenchLeftSnowblowSafe
