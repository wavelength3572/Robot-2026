# TODO: Items to Port from ShootingCoordinatorUpdate

Compiled from the full history of the ShootingCoordinatorUpdate branch. These are ideas, fixes, and improvements to apply on Reorganization5.

---

## Critical Bugs (broken in Reorg5 right now)

### 1. HoodIOSparkMax `connected` hardcoded to `false`
- **File:** `src/main/java/frc/robot/subsystems/hood/HoodIOSparkMax.java` (~line 100)
- **Bug:** `inputs.connected = false;` is hardcoded.
- **Fix:** Change to `inputs.connected = true;`
- **Impact:** Hood always appears disconnected in logs/dashboard.

### 2. HoodIOSparkMax `atTarget` hardcoded to `false`
- **File:** `src/main/java/frc/robot/subsystems/hood/HoodIOSparkMax.java` (~line 112)
- **Bug:** `inputs.atTarget = false;` is hardcoded.
- **Fix:** Change to `inputs.atTarget = Math.abs(inputs.currentAngleDeg - targetAngle) < 1.0;`
- **Impact:** Any command waiting for hood readiness will never proceed.

### 3. `getInsideTurretAngleDeg()` returns the wrong tunable
- **File:** `src/main/java/frc/robot/commands/ShootingCommands.java` (~line 170)
- **Bug:** Returns `testOutsideTurretAngleDeg` instead of `testInsideTurretAngleDeg`.
- **Fix:** Change return to `testInsideTurretAngleDeg`.
- **Impact:** Inside-angle bench testing is broken.

### 4. SquareBotConfig & TurretBotConfig missing `getTurretZeroOffset()` override
- **Files:** `SquareBotConfig.java`, `TurretBotConfig.java`
- **Bug:** Both define `turretZeroOffset = 63.873` but never override the interface method. Default returns `0.0`.
- **Fix:** Add `@Override public double getTurretZeroOffset() { return turretZeroOffset; }` to both.
- **Impact:** `TurretIOSparkMax` gets wrong offset, causing incorrect inside/outside angle conversions and wrong soft limits.

### 5. TurretIOSim missing outside angle support
- **File:** `src/main/java/frc/robot/subsystems/turret/TurretIOSim.java`
- **Bug:** Never implements `getOutsideCurrentAngle()` / `getOutsideTargetAngle()`, never populates `inputs.currentOutsideAngleDeg` or `inputs.targetOutsideAngleDeg`.
- **Fix:** Override both methods, populate outside angle inputs in `updateInputs()`.
- **Impact:** Sim turret always reports 0 degrees. Breaks shot calculations, `atTarget()` checks, and all turret logging in simulation.

### 6. ShootingCoordinator is nulled out in RobotContainer
- **File:** `src/main/java/frc/robot/RobotContainer.java` (~line 461)
- **Bug:** Construction is commented out: `shootingCoordinator = null;` with a TODO.
- **Fix:** Uncomment and wire up properly.
- **Impact:** No turret aiming, no shot calculation, no auto-shoot. Shooting is completely non-functional.

---

## Feature Changes (functional improvements to port)

### 7. CoordinatorMode state machine (IDLE / TRACKING / AUTO_SHOOT)
- Replaces the simple `autoShootEnabled` boolean in ShootingCoordinator.
- **IDLE:** Calculations run (logging/viz) but NO actuators commanded.
- **TRACKING:** Turret + hood track the target, no auto-firing.
- **AUTO_SHOOT:** Full tracking + automatic firing when conditions met.
- Key method: `isTrackingOrHigher()` gates whether turret/hood are actually commanded.
- Integration: `enableLaunchMode()` sets TRACKING, `disableLaunchMode()` returns to IDLE, `enableAutoShoot()` sets AUTO_SHOOT.

### 8. Expanded ShootingCoordinator constructor
- Takes `Spindexer` and `Motivator` in addition to Turret/Hood/Launcher.
- Adds `getSpindexer()` / `getMotivator()` accessors.
- Allows full pipeline orchestration from the coordinator.

### 9. Un-comment turret aiming in ShootingCoordinator
- Reorg5 has `turret.setOutsideTurretAngle(...)` commented out in both `calculateAndCommandHubShot()` and `calculateAndCommandPassShot()`, plus the `setTurretAngle()` facade is a no-op.
- Without this, the coordinator calculates shots but never moves the turret.
- **IMPORTANT:** Use `turret.setOutsideTurretAngle()` (not `setTurretAngle()` which doesn't exist).

### 10. Unified `launchCommand`
- Merges Reorg5's 3 separate commands (`launchCommand`, `testLaunchCommand`, `benchTestLaunchCommand`) into one that checks `ShootingMode` at startup.
- New signature: `launchCommand(Launcher, ShootingCoordinator, Motivator, Hood, Spindexer)`
- In MANUAL mode: positions turret + hood from dashboard, spins up, waits for readiness, feeds, fires.
- In AUTO mode: lets coordinator handle aiming, spins up, fires.
- Simplifies bindings and prevents mode confusion.

### 11. Spindexer integration in launch command
- Activates spindexer in firing phase: `spindexer.setSpindexerVelocity(testSpindexerRPM.get())`
- Stops on cleanup: `spindexer.stopSpindexer()`

### 12. Turret + Hood readiness checking before firing
- Wait-for-setpoint checks 4 conditions: launcher, motivator, turret, hood.
- Dashboard indicators: `Match/Status/ReadyTurret` and `Match/Status/ReadyHood`.
- Timeout increased from 3s to 5s to allow turret/hood positioning.

### 13. Remove turret dependency from Drive
- Turret aiming belongs in ShootingCoordinator (gets pose via supplier), not in Drive.
- Remove `Turret turret` param from Drive constructor, remove `AIM_PHASE_DELAY_SECONDS`, remove commented-out aiming code and `getProjectedPose()`.

### 14. BenchTestMetrics on every shot
- Adds `BenchTestMetrics.getInstance().recordShot()` to the firing loop.
- Adds `BenchTestMetrics.getInstance().reset()` at launch start.

---

## Refactor / Quality of Life

### 15. Rename COMPETITION/TEST to AUTO/MANUAL
- More intuitive naming.
- `isTestMode()` becomes `isManualMode()`.
- Update all references in ShootingCommands and ShootingCoordinator.

### 16. Default mode to MANUAL
- Safer for bringup. Robot starts in manual mode, preventing unexpected turret movement on boot.
- Change: `private static ShootingMode currentMode = ShootingMode.MANUAL;`

### 17. Updated default tunable values
- Values from actual bench testing sessions:
  - `BenchTest/Shooting/LauncherRPM`: 1700 -> **2500**
  - `BenchTest/Shooting/MotivatorRPM`: 1000 -> **1500**
  - `BenchTest/Shooting/AngleDegHood`: 45.0 -> **16.0**
  - `BenchTest/Shooting/SpindexerRPM`: 1000 -> **750**

### 18. Remove separate Tuning/Shooting/* tunables
- `launchVelocityRPM` and `motivatorVelocityRPM` are redundant once the command is unified.
- Both modes use BenchTest/Shooting/* tunables. (AUTO mode has a TODO to get RPMs from the coordinator's calculated shot.)

### 19. Remove `Match/Status/Active` dashboard boolean
- No longer needed after mode rename/unification.

### 20. Simplify shoot button binding
- Remove TURRETBOT special-case that uses `benchTestLaunchCommand`.
- Unified command handles all robot types.
