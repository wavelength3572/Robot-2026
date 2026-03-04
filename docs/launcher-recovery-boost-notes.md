# Launcher Recovery Boost - Testing Notes (2/5/2026)

## What It Does
Detects RPM dips during active shooting (prefeed running) and temporarily switches to a more aggressive PID configuration on the SparkFlex to help the flywheel recover faster.

## How Detection Works
1. `ShootingCommands` sets `feedingActive = true` when Phase 3 (prefeed) starts
2. Every 20ms, `Launcher.setVelocity()` checks: is feeding active AND is the velocity error above the threshold?
3. If yes, switches to recovery mode (Slot 1 PID + optional FF boost)
4. **Hysteresis** prevents overshoot: recovery turns ON at the threshold (default 50 RPM error) but only turns OFF at half the threshold (25 RPM error). This prevents the aggressive gains from cutting out abruptly while the motor still has momentum.

## Two Recovery Mechanisms
- **PID Slot Switching (Slot 0 → Slot 1)**: Slot 1 has a higher P gain (`kP + RecoveryKpBoost`). This runs at 1kHz on the SparkFlex, so it reacts much faster than anything in robot code. This is the primary recovery tool.
- **FF Voltage Boost**: Adds a flat voltage on top of the normal feedforward. Less useful than P boost because it doesn't scale with error and can cause overshoot. Can be left at 0.

## Dashboard Tunables
| Tunable | Default | What it does |
|---|---|---|
| `Tuning/Launcher/RecoveryKpBoost` | 0.0001 | Extra P gain added to Slot 1 |
| `Tuning/Launcher/RecoveryBoostVolts` | 0.0 | Extra FF voltage during recovery (optional) |
| `Tuning/Launcher/RecoveryBoostThresholdRPM` | 50.0 | Error that triggers recovery mode |

## AdvantageScope Signals
- `Launcher/RecoveryBoostActive` - boolean, true when recovery mode is on
- `Launcher/RecoveryBoostVolts` - the FF boost being applied (0 if disabled)
- `Launcher/UsingRecoveryPID` - boolean, true when using Slot 1
- `Launcher/TotalFeedforwardVolts` - base FF + boost
- `Launcher/FeedforwardVolts` - base FF alone (compare with Total to see boost)

## Testing Results (2/5/2026)
- **Baseline recovery**: ~0.20s with no boost
- **FF boost only** (0.5V): Got to ~0.16s, but diminishing returns - adding more voltage didn't help
- **Why FF boost plateaued**: Motor was hitting the 60A current limit. No amount of extra voltage helps once the controller is clamped.
- **P boost (0.0012) + FF boost (0.5V)**: Got to ~0.12s
- **Overshoot observed**: When boost cut off abruptly, RPM would overshoot the target. Added hysteresis to fix this.
- **Current limit raised**: 60A → 80A. NEO Vortex handles brief bursts fine. This gives the controller more headroom during recovery transients.

## Key Insight: Why P Boost > FF Boost
- P runs at **1kHz** on the SparkFlex and **scales with error** (bigger dip = more correction, naturally tapers as it recovers)
- FF boost is a **flat voltage** updated at 20ms that doesn't adapt to the actual error
- Making FF proportional to error would just be a slower, worse version of P

## Testing Stopped
Physical issue - rubbing detected, needs mechanical investigation before continuing.

## Next Steps for Testing
1. Try P boost alone (set RecoveryBoostVolts to 0) with the 80A current limit - may be enough on its own
2. Check if current still hits 80A ceiling during recovery - if so, that's the new limit
3. Watch for overshoot with hysteresis enabled - should be much better
4. If overshoot persists, can increase the hysteresis band (currently deactivates at 50% of threshold)

## Files Modified
- `LauncherIO.java` - `setVelocityWithBoost(rpm, boostVolts, recoveryActive)` interface method
- `LauncherIOSparkFlex.java` - Slot 0/1 config, slot switching, RecoveryKpBoost tunable
- `LauncherIOSim.java` - Stub (delegates to setVelocity)
- `Launcher.java` - Detection logic, hysteresis, feedingActive flag
- `ShootingCommands.java` - Sets feedingActive true/false around Phase 3
- `MainBotConfig.java` / `TurretBotConfig.java` - Current limit 60→80A

## Other Default Changes Made
- `Shooting/Auto/LaunchVelocityRPM`: 2000 → 1700
- `Shooting/Auto/PrefeedVelocityRPM`: 500 → 1000
- `Shooting/Test/LauncherRPM`: 2000 → 1700
- `Shooting/Test/PrefeedRPM`: 500 → 1000
- Removed unused `Tuning/Launcher/TargetVelocityRPM` and `Tuning/Launcher/SysIdMinVelocityRPM`
