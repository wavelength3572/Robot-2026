# TODO Review & Code Cleanup Plan

Reviewed: 2026-03-10

This document consolidates all TODOs, FIXMEs, and code cleanup opportunities across
the codebase, organized by priority and theme. Each section explains *what* the
issue is, *why* it matters, and *what to do about it*.

---

## 1. Critical Bugs (Fix Before Next Match)

These are broken behaviors in the current code that directly affect robot functionality.

### 1.1 HoodIOSparkMax: `connected` hardcoded to `false`
- **File:** `src/main/java/frc/robot/subsystems/hood/HoodIOSparkMax.java` (~line 100)
- **Problem:** `inputs.connected = false;` — the hood always appears disconnected in
  logs and dashboard, making debugging impossible.
- **Fix:** Change to `inputs.connected = true;` (or use actual connection status).

### 1.2 HoodIOSparkMax: `atTarget` hardcoded to `false`
- **File:** `src/main/java/frc/robot/subsystems/hood/HoodIOSparkMax.java` (~line 112)
- **Problem:** `inputs.atTarget = false;` — any command waiting for hood readiness
  (including smartLaunchCommand) will never proceed. This silently breaks all
  shooting sequences that check `hood.atTarget()`.
- **Fix:** `inputs.atTarget = Math.abs(inputs.currentAngleDeg - targetAngle) < 1.0;`

### 1.3 ShootingCommands: `getInsideTurretAngleDeg()` returns wrong tunable
- **File:** `src/main/java/frc/robot/commands/ShootingCommands.java` (~line 170)
- **Problem:** Returns `testOutsideTurretAngleDeg` instead of `testInsideTurretAngleDeg`.
- **Impact:** Inside-angle bench testing is broken — you think you're testing inside
  angle but you're actually reading the outside angle tunable.
- **Fix:** Return `testInsideTurretAngleDeg` instead.

### 1.4 SquareBotConfig missing `getTurretZeroOffset()` override
- **File:** `SquareBotConfig.java`
- **Problem:** Defines `turretZeroOffset = 63.873` but never overrides the interface
  method. The default returns `0.0`, so `TurretIOSparkMax` gets the wrong offset.
- **Impact:** Incorrect inside/outside angle conversions and wrong soft limits.
- **Fix:** Add `@Override public double getTurretZeroOffset() { return turretZeroOffset; }`

### 1.5 TurretIOSim: missing outside angle support
- **File:** `src/main/java/frc/robot/subsystems/turret/TurretIOSim.java`
- **Problem:** Never implements `getOutsideCurrentAngle()` / `getOutsideTargetAngle()`,
  never populates `inputs.currentOutsideAngleDeg` or `inputs.targetOutsideAngleDeg`.
- **Impact:** Sim turret always reports 0 degrees. Breaks shot calculations, `atTarget()`
  checks, and all turret logging in simulation. This makes sim-based development
  unreliable for any shooting-related work.
- **Fix:** Override both methods and populate outside angle inputs in `updateInputs()`.

### 1.6 IntakeIOSparkMax: deploy motor workaround
- **File:** `src/main/java/frc/robot/subsystems/intake/IntakeIOSparkMax.java` (line 171)
- **TODO:** `// TODO: Revert when deploy motor is reconnected.`
- **Problem:** When deploy motor is disconnected, it fakes `deployPositionRotations`
  so `isDeployed()` returns true. This is a temporary workaround — if the deploy
  motor is reconnected but this code isn't reverted, the intake will report a
  false position.
- **Fix:** Check if the deploy motor is actually reconnected on the robot. If so,
  revert the workaround. If not, add a comment explaining when this should be reverted.

---

## 2. Feature Gaps (Port from ShootingCoordinatorUpdate)

These items from `TODO-from-SCU.md` represent functional improvements that should
be ported. They are listed in suggested implementation order.

### 2.1 CoordinatorMode state machine (IDLE / TRACKING / AUTO_SHOOT)
- Replaces the simple `autoShootEnabled` boolean.
- **Why:** The current boolean doesn't distinguish between "calculate but don't
  actuate" (useful for dashboard visualization during disable) and "track the target"
  (turret moves but doesn't fire). A 3-state machine makes intent explicit.
- **Priority:** High — this is the foundation for items 2.2-2.4.

### 2.2 Un-comment turret aiming in ShootingCoordinator
- `turret.setOutsideTurretAngle(...)` is commented out in both
  `calculateAndCommandHubShot()` and `calculateAndCommandPassShot()`.
- **Why:** Without this, the coordinator calculates shots but never moves the turret.
  This is the single biggest functional gap.

### 2.3 Expanded ShootingCoordinator constructor (add Spindexer, Motivator)
- Allows full pipeline orchestration from the coordinator.
- **Why:** Currently the coordinator can't control the full firing pipeline.

### 2.4 Unified `launchCommand`
- Merge the 3 separate commands (`launchCommand`, `testLaunchCommand`,
  `benchTestLaunchCommand`) into one that checks `ShootingMode` at startup.
- **Why:** Three nearly-identical commands is confusing, error-prone, and makes
  the button bindings harder to understand.

### 2.5 Remove turret dependency from Drive
- **File:** `src/main/java/frc/robot/subsystems/drive/Drive.java` (line 187)
- **TODO:** `// TODO: Re-enable turret aiming after turret bringup is complete`
- Turret aiming belongs in ShootingCoordinator, not Drive. The Drive subsystem
  currently takes a `Turret turret` parameter and has commented-out aiming code.
- **Why:** This violates separation of concerns. Drive should only handle driving.
  Moving aiming to ShootingCoordinator simplifies Drive and makes the aiming logic
  testable independently.

### 2.6 Rename COMPETITION/TEST to AUTO/MANUAL
- **File:** `src/main/java/frc/robot/commands/ShootingCommands.java` (lines 46-51)
- More intuitive naming. `isTestMode()` becomes `isManualMode()`.
- **Why:** "TEST" implies something temporary or broken. "MANUAL" clearly communicates
  that the operator is directly controlling parameters.

### 2.7 Default mode to MANUAL
- **File:** `src/main/java/frc/robot/commands/ShootingCommands.java` (line 54)
- Safer for bringup — robot starts in manual mode, preventing unexpected turret
  movement on boot.

---

## 3. Control System TODOs (Tuning & Performance)

### 3.1 Turret: PD-only control causes steady-state error
- **File:** `src/main/java/frc/robot/subsystems/turret/Turret.java` (lines 38-39)
- **TODO:** Consider adding kI and/or kS (static friction feedforward).
- **Why:** Without integral gain or feedforward, the turret can't push through
  friction at small errors. This means the turret "almost" reaches the target but
  never quite gets there — which directly affects shot accuracy.
- **Recommendation:** Start with a small kS (static friction feedforward) rather
  than kI. kI can cause windup and overshoot; kS provides a constant "push"
  proportional to the direction of error. If kS alone isn't enough, add a small kI
  with a reasonable iZone to limit windup.

### 3.2 Hood: PD-only control causes steady-state error
- **File:** `src/main/java/frc/robot/subsystems/hood/Hood.java` (lines 27-29)
- **TODO:** Same issue as turret — no kI or kS.
- **Why:** The hood fights gravity in addition to friction. A gravity feedforward
  term (kG, proportional to cos(angle)) would be even more effective here than kS.
- **Recommendation:** Add kG (gravity compensation) + small kS. Consider using
  WPILib's `ArmFeedforward` which already handles the cos(angle) term.

### 3.3 HubShiftUtil: hardcoded time-of-flight values
- **File:** `src/main/java/frc/robot/util/HubShiftUtil.java` (lines 41, 43)
- **FIXME:** `minTimeOfFlight = 1.0` and `maxTimeOfFlight = 1.5` are hardcoded
  instead of computed from `ShotCalculator`.
- **Why:** As shot parameters are tuned, these constants become stale. If the actual
  TOF is outside the hardcoded range, hub shift timing will be wrong.
- **Fix:** Wire these to `ShotCalculator` or at minimum make them tunables so they
  can be adjusted from the dashboard.

---

## 4. Hardware Configuration TODOs (Pre-Competition)

### 4.1 Motor inversion: 3 files need robot-specific settings
- `SpindexerIOSparkMax.java` (line 86): `.inverted(false)`
- `MotivatorIOSparkMax.java` (line 87): `.inverted(true)`
- `MotivatorIOSparkFlex.java` (line 81): `.inverted(false)`
- All have: `// TODO: Make robot-specific when MainBot is ready`
- **Fix:** Move the inversion boolean into `RobotConfig` interface (e.g.,
  `isMotivatorInverted()`, `isSpindexerInverted()`). Each robot config
  (MainBotConfig, SquareBotConfig) provides the correct value.
- **Why:** Hardcoded inversion that's wrong for one robot means things spin
  backwards — this can jam mechanisms or eject balls the wrong direction.

### 4.2 Hood CAN ID confirmation
- **File:** `src/main/java/frc/robot/MainBotConfig.java` (line 660)
- **TODO:** `// TODO: Confirm CAN ID when hardware is ready`
- **Fix:** Verify on the physical robot and update. Consider adding a CAN ID
  validation check at startup that logs warnings if devices don't respond.

### 4.3 VisionConstants: camera yaw angle discrepancy
- **File:** `src/main/java/frc/robot/subsystems/vision/VisionConstants.java` (line 126)
- **TODO:** Right Front camera yaw was changed from PDF spec (-83 deg) to -97 deg
  to mirror LeftRear's offset. Needs CAD confirmation.
- **Why:** Wrong camera transforms = wrong pose estimation = missed shots.
- **Fix:** Get the actual mount angle from CAD and update. Document the source of
  truth (CAD model revision) in a comment.

---

## 5. Code Readability & Organization Improvements

These aren't bugs, but they'd make the codebase significantly easier to navigate,
understand, and maintain.

### 5.1 ShootingCommands.java is too large (~54KB, ~1400 lines)
This is the single biggest readability issue in the codebase. The file contains:
- Shot presets (hub, left trench, right trench, pass)
- Bench test tunables
- Mode management (COMPETITION/TEST)
- `smartLaunchCommand` (the main shooting sequence)
- `fixedPositionLaunchCommand` (preset-based shots)
- `launchCommand` / `testLaunchCommand` / `benchTestLaunchCommand`
- Simulation helpers
- Turret angle getters

**Recommendation:** Split into:
1. **`ShotPresets.java`** — All the `LoggedTunableNumber` presets (hub shot, left
   trench, right trench, pass target). These are pure data, not commands.
2. **`ShootingMode.java`** — The mode enum, `getMode()`/`setMode()`/`isTestMode()`.
   This is global state, not a command.
3. **`ShootingCommands.java`** — Just the command factory methods, now much shorter
   and focused.

### 5.2 RobotContainer constructor is very long (~270 lines)
The constructor handles: subsystem creation, default commands, RobotStatus init,
vision wiring, intake wiring, ShootingCoordinator creation, LED creation, FuelSim
init, named commands, dashboard toggles, alliance chooser, launcher default command,
auto chooser, and OI update.

**Recommendation:** Extract helper methods:
- `createSubsystems()` — the big switch statement
- `wireSubsystemDependencies()` — vision speed supplier, intake speed supplier
- `configureDashboard()` — SmartDashboard puts, alliance chooser
- The constructor then reads as a high-level narrative.

### 5.3 Inconsistent Javadoc on accessor methods
Several `RobotContainer` accessor methods have wrong Javadoc. For example:
- `getTurret()` says "SquareBot only" but turret is actually MainBot only
- `getVision()` says "SquareBot only" but vision is MainBot only
- `getIntake()` says "SquareBot only" but intake is MainBot only

These should just say "if it exists" without referencing a specific robot, since
the config system handles that.

### 5.4 Add section comments to large files
Files like `Drive.java`, `ShootingCoordinator.java`, and `RobotContainer.java`
would benefit from clear section dividers:
```java
// ==================== Odometry ====================
// ==================== Vision Fusion ====================
// ==================== Trajectory Following ====================
```

### 5.5 Intake.java class-level comment
- **File:** `src/main/java/frc/robot/subsystems/intake/Intake.java` (lines 20-21)
- **TODO:** `// TODO might need some sort of deploy sequence or intelligence to fix
  the backlash with respect to deploy`
- This reads more like a design note than a TODO. It should either be converted into
  a concrete task (with specific behavior to implement) or moved to a design
  document. As-is, it's unclear what "fix the backlash" means or when it should be
  done.
- **Recommendation:** If backlash is currently causing issues, create a specific
  issue. If it's a "maybe someday" thought, move it to a design notes section in
  `docs/` and remove the TODO from the code.

### 5.6 Documentation TODOs in lesson plans
- `docs/lessons/shoot-on-the-move-lesson-plan.md` has two TODOs for populating
  lookup tables with measured data.
- These are in documentation (example code), not production code, so they're low
  priority. But they should be either populated with real data to make the lessons
  useful, or marked as `// Example — replace with your team's data` to make it
  clear they're intentionally blank.

---

## 6. Suggested Implementation Order

For maximum impact with minimum risk:

| Phase | Items | Why This Order |
|-------|-------|----------------|
| **Phase 1: Critical fixes** | 1.1, 1.2, 1.3, 1.4, 1.5 | These are bugs. Fix them first. |
| **Phase 2: Hardware config** | 4.1, 4.2, 4.3, 1.6 | Needed before the robot runs correctly. |
| **Phase 3: Naming & readability** | 2.6, 2.7, 5.1, 5.3 | Low risk, high readability payoff. |
| **Phase 4: Architecture** | 2.1, 2.2, 2.5, 5.2 | Bigger changes, do after critical path is clear. |
| **Phase 5: Control tuning** | 3.1, 3.2, 3.3 | Requires robot access and testing time. |
| **Phase 6: Feature completion** | 2.3, 2.4 | Build on Phase 4 architecture. |
| **Phase 7: Polish** | 5.4, 5.5, 5.6 | Nice-to-haves when there's time. |

---

## Summary

| Category | Count | Severity |
|----------|-------|----------|
| Critical bugs | 6 | Must fix |
| Feature gaps (from SCU) | 7 | Important |
| Control tuning | 3 | Important |
| Hardware config | 3 | Pre-competition |
| Code readability | 6 | Quality of life |
| **Total** | **25** | |

The most impactful single change is **fixing items 1.1 and 1.2** (HoodIOSparkMax
hardcoded values) because they silently break the entire shooting pipeline. The most
impactful readability change is **splitting ShootingCommands.java** (5.1) because
at 54KB it's the hardest file to navigate and the most frequently edited.
