# Robot 2026 — Log Analysis Findings
*Generated 2026-04-20T12:36:04 from 8 match logs in `StatesLogs/`.*

## Headline numbers

- **Truly-active BPS (fleet): 1.91** — balls divided by time the spindexer was actually feeding. Target is 12.0.
- **HUB BPS: 2.15** (548 balls across hub-scoring intervals)
- **PASS BPS: 0.91** (57 balls across passing intervals) — notably worse than HUB
- **131 firing intervals** across 8 matches, 605 total balls impacted the flywheel
- **80 intervals fired zero balls** (61% of all held-fire)

## Top findings, ranked

### 1. PASS shots fail about half the time — this is the #1 opportunity

45 of the 80 zero-ball intervals were in PASS/LONG_PASS mode; only 35 were HUB. PASS BPS (0.91) is less than half of HUB BPS (2.15).

Longest pass attempts that fired nothing:
- **q36** @ 239.9s — 9.8s held, 0 balls, cause `other`, blocking: `launcher turret hood speed`
- **q49** @ 147.8s — 8.5s held, 0 balls, cause `other`, blocking: `turret velcomp(27.4/20deg) speed`
- **q69** @ 196.7s — 6.3s held, 0 balls, cause `turret_not_aimed`, blocking: `turret velcomp(29.0/20deg) speed`
- **q32** @ 215.2s — 6.0s held, 0 balls, cause `zone_exit`, blocking: `launcher motivator turret hood`
- **q32** @ 264.9s — 5.8s held, 0 balls, cause `other`, blocking: `launcher turret`

**Root cause (from `velcomp_diagnostic`):** The velocity-compensation solver's residual divergence sits at 20-40° for the long time-of-flight passes (TOF 1.2-1.8s vs 0.4s for hub). The feed-gate requires <20°. The solver iterates correctly — it just finds a fixed point above the gate threshold. More iterations won't help.

**Not the cause:** The downstream actuators are fine — 0 intervals failed due to launcher-never-ready, motivator-never-spun, or spindexer-never-fed.

**Suggested fix:** Before writing new code, try switching the dashboard chooser from `Pass Symmetric` to `Pass Waypoint` at a practice — `WaypointPassStrategy.java` adds a 3D apex constraint that may give the solver a better-conditioned problem. If that doesn't help, see `docs/passing-fix-prompt.md` for the proposed geo-fence replacement — aim at a fixed zone-based target and gate firing on "predicted landing inside allowed polygon" instead of aim precision.

### 2. Bug: coord stays in FIRING for seconds with all motor targets at 0

6 intervals across 4 matches, totalling **45.8s**, where `CoordinatorState` reports FIRING but motivator and spindexer targets have dropped to 0. Each time the robot was driving out of the alliance zone toward a no-fire zone. The shot went null but the state machine kept reporting FIRING for 3-10s before actually transitioning.

Worst examples:
- **q69** @ 125.6s — 10.2s FIRING with zero targets, zone at start: `ALLIANCE_TRENCH`
- **q32** @ 168.4s — 9.6s FIRING with zero targets, zone at start: `ALLIANCE_TRENCH`
- **q74** @ 94.3s — 8.9s FIRING with zero targets, zone at start: `ALLIANCE_TRENCH`
- **q49** @ 104.1s — 6.8s FIRING with zero targets, zone at start: `ALLIANCE_TRENCH`

**Suggested fix:** In `ShootingCoordinator.java`, find the FIRING→AIMING transition condition and add an early exit when `currentShot == null` for N consecutive loops.

### 3. 28 button taps shorter than 500ms

Of all zero-ball intervals, these were driver button-taps too brief for the ready-check + feed chain to complete a shot. Not a software bug per se, but consider a "pulse fire" mode where a tap stays armed for 500-800ms to forgive the quick taps.

### 4. Auto-unclog: 6/57 (11%) trigger then immediately clear in <100ms

57 total auto-unclog events across the 8 matches. 6 last less than 100ms — likely borderline stall triggers that clear themselves. Worth bumping `autoUnclogStallDurationSec` slightly to kill those without missing real jams.

### 5. Battery brownout on 29/125 firing intervals

Bus voltage sags below 9V for at least 25% of the interval in 29 cases out of 125 total firing intervals examined. Fleet minimum bus voltage was **5.7V**, peak launcher current **150A**. Not currently blocking feeding (launcher still tracks), but indicates heavy battery load during shooting. Worth keeping an eye on but not the top priority.

## Things the data ruled out

- **Launcher tracking**: mean RPM error is -2% to -3% across the fleet — the launcher controller is healthy.
- **Motor voltage saturation**: **0 of 125** firing intervals pegged applied-output near 1.0. The controller has headroom; it's not being commanded to full power.
- **Turret flipping during firing**: small total time, not a meaningful blocker in these matches.
- **Hold-fire button misuse**: 0 seconds of firing-attempt time spent with spindexer `SUPPRESSED`.
- **Spindexer / motivator feed chain**: when the gate allows, they work. 0 "spindexer never fed" or "motivator never spun" failures.

## Per-match snapshot

| Match | Balls | Effective BPS | HUB BPS | PASS BPS | PASS intervals | 0-ball | Jams | Worst dry stretch |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| q32 | 100 | 2.35 | 2.64 | 0.66 | 8 | 12 | 17 | 17.0s |
| q36 | 79 | 1.50 | 1.96 | 0.44 | 8 | 9 | 41 | 13.3s |
| q43 | 91 | 1.93 | 2.19 | 0.00 | 5 | 7 | 34 | 13.9s |
| q49 | 65 | 2.02 | 2.32 | 0.81 | 4 | 15 | 31 | 10.9s |
| q55 | 96 | 2.28 | 2.45 | 1.33 | 10 | 12 | 33 | 13.2s |
| q65 | 2 | 1.79 | 1.79 | 0.00 | 0 | 0 | 0 | 1.0s |
| q69 | 95 | 1.77 | 1.77 | 1.80 | 9 | 12 | 36 | 11.7s |
| q74 | 77 | 1.67 | 1.90 | 0.90 | 8 | 13 | 33 | 16.9s |

## Where to look next

1. **Cheapest experiment — no code change:** at your next practice, flip the dashboard's pass-strategy chooser from `Pass Symmetric` to `Pass Waypoint`. If passes start firing more reliably, you're done.

2. **If waypoint doesn't fix it:** hand `docs/passing-fix-prompt.md` to a fresh coding session to prototype the geo-fence gate. It's scoped behind a feature flag so it can't break HUB shots.

3. **Small code fix worth doing anyway:** the coord-stays-in-FIRING bug (Finding 2). Check `ShootingCoordinator.java` for the FIRING→AIMING transition condition.

---
*This report is auto-generated by `tools/generate_report.py`. Re-run `tools/analyze_wpilog.py` after adding new wpilogs to refresh it. Open `tools/log-analysis/dashboard.html` for the interactive explorer.*
