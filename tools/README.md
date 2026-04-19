# WPILog analysis tools

Offline analysis of AdvantageKit `.wpilog` files from `StatesLogs/`. Python
extracts per-match JSON; a static web dashboard renders it.

```
StatesLogs/*.wpilog
        │
        ▼
tools/analyze_wpilog.py   (parse + run every analyzer → per-match JSON)
        │
        ▼
tools/log-analysis/data/  (summary.json + match_<stem>.json)
        │
        ▼
tools/log-analysis/index.html   (fleet + match views)
```

## Install

```
pip install -r tools/requirements.txt
```

The only dependency is `robotpy-wpiutil`, which ships `wpiutil.log.DataLogReader`.

## Extract

```
python3 tools/analyze_wpilog.py StatesLogs/*.wpilog --out tools/log-analysis/data
```

On the current 8-match set this takes ~15 seconds end-to-end. Each analyzer
runs in <0.05 s once the log is parsed.

## View

```
python3 -m http.server 8080 -d tools/log-analysis
```

Open <http://localhost:8080>. The dashboard has two views:

- **Fleet** — table of matches, headline BPS/jam/flip counts, bar charts.
- **Match** — pick a match from the dropdown (or click a row in the fleet
  table) to see an interactive timeline with event markers and a searchable
  event list. Clicking an event zooms the timeline to ±8 seconds around it.

## Adding a new analyzer

Every analyzer lives in `tools/analyzers/` as a single module. To add one:

```python
# tools/analyzers/my_new_thing.py
from log_context import AnalyzerResult, LogContext, register_analyzer

@register_analyzer(id="my_new_thing", title="My new thing")
def analyze(ctx: LogContext) -> AnalyzerResult:
    sig = ctx.signal("/SomePath/IWantToInspect")
    events = []
    summary = {"count_of_interesting_thing": 0}
    for ts, v in zip(sig.timestamps_us, sig.values):
        if looks_interesting(v):
            events.append({"t_s": ctx.rel_s(ts), "detail": v})
            summary["count_of_interesting_thing"] += 1
    return AnalyzerResult(
        id="my_new_thing",
        title="My new thing",
        events=events,
        summary=summary,
    )
```

Re-run the extractor. The fleet table automatically picks up new
`summary` keys as columns, and events appear in the match-view event list.

### What `LogContext` gives you

- `ctx.signal(name)` → `TimeSeries(timestamps_us, values, dtype)` — empty if missing
- `series.value_at(ts_us, default)` — last value at or before a timestamp
- `series.iter_between(t0, t1)` — (ts, value) pairs in a window
- `series.sampled(step_us)` — decimated copy for the dashboard timeline
- `ctx.rel_s(ts_us)` — convert microsecond log time to match-relative seconds
- `ctx.duration_s()`, `ctx.t0_us`, `ctx.t1_us`, `ctx.record_count`

## Current analyzers

| id | Output |
|---|---|
| `effective_bps` | **Main BPS analyzer.** Partitions every `FIRING` interval into 9 time buckets (feeding/flipping/suppressed/unclogging/auto-unclogging/jammed/reciprocating/stopped/not-ready). Emits `truly_active_bps = balls / feeding_s` — the honest number — and per-interval event with `aim_mode` (HUB/PASS/LONG_PASS/NONE) |
| `bps_by_mode` | Same partition logic, grouped by `SmartLaunch/Status/AimMode`. Produces `hub_bps`, `pass_bps`, `long_pass_bps` separately so alliance-zone hub shooting and neutral-zone passing don't get averaged together |
| `firing_intervals` | Contiguous `SmartLaunch/CoordinatorState == FIRING` spans with balls-per-interval |
| `bps` | Legacy per-firing-interval BPS (balls ÷ FIRING duration). Kept for comparison |
| `slugs` | Ball-impact clustering (gaps < 300 ms). Diagnostic only — inter-ball gap percentiles, best slug |
| `jams` | Windows where coordinator is `FIRING` + spindexer is `FEEDING` but no ball impact for ≥0.5 s |
| `turret_flips` | `Subsystems/TurretState == FLIPPING` entries, flagged when during/near firing |
| `spindexer_events` | Time in `SUPPRESSED` (operator hold-fire), `UNCLOGGING` (manual), `AUTO_UNCLOGGING`, `JAMMED`, `RECIPROCATING` during any firing attempt |
| `motivator_zero_cause` | `/Motivator/TargetRPM` falling edges, classified against the three real causes from `ShootingCommands.java:569/749/775` |
| `match_summary` | Duration, shot counts, plus downsampled signal series for the timeline |

The "12 BPS" target comes from the competitive baseline (83 ms between balls).
`bps_at_p10_gap` from `slugs` is the honest mechanism ceiling — it's the BPS
implied by the fastest 10 % of inter-ball gaps observed.

## Reference: signal paths used

```
/RealOutputs/SmartLaunch/CoordinatorState         string enum
/RealOutputs/SmartLaunch/StateTransition          string w/ reason
/RealOutputs/SmartLaunch/Blocking                 string
/RealOutputs/SmartLaunch/Target/Achievable        boolean
/RealOutputs/Subsystems/TurretState               string enum
/RealOutputs/Subsystems/SpindexerState            string enum
/RealOutputs/Launcher/BallImpact/Count            int64 (monotonic)
/RealOutputs/Launcher/BallImpact/DipRPM           double
/Launcher/LeaderVelocityRPM                       double
/Launcher/TargetVelocityRPM                       double
/Launcher/AtSetpoint                              boolean
/Motivator/WheelRPM                               double
/Motivator/TargetRPM                              double
/Motivator/AtSetpoint                             boolean
/Turret/CurrentInsideAngleDeg                     double
/Turret/TargetInsideAngleDeg                      double
```

## Decompiled predecessor

`analyze_wpilog.recovered.py` is the decompilation of the
`__pycache__/analyze_wpilog.cpython-312.pyc` whose `.py` source was lost.
Decompilation is incomplete (the file was Python 3.12 bytecode decompiled
with `pycdc`), so it's kept purely as a reference for which signals and
analyses the prior script cared about (shots, recovery events, turret big
moves, fuel events, cycle-time overruns). Re-implemented bits live in the
modular `analyzers/` directory.
