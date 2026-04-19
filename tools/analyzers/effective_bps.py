"""Effective BPS — the honest driver metric.

For each `CoordinatorState == FIRING` interval:
    fire_down_ts = moment coord entered FIRING (button pressed, system armed)
    ready_ts     = first moment at/after fire_down when Launcher/AtSetpoint is true
                   (the launcher is finally spun up and actually able to shoot)
    fire_up_ts   = moment coord left FIRING (button released)

    effective_bps = balls_fired / (fire_up_ts - ready_ts)

This excludes the spin-up latency (which you can't help) and answers "once I'm
actually ready to shoot, how fast do balls come out until I release the
button?"

If the launcher never reaches AtSetpoint during the interval, it's counted
separately as `intervals_never_ready` and excluded from the ratio.

NOTE on out-of-balls vs jammed: the per-interval data exposes
`dry_seconds_after_ready` (time with launcher ready but no ball impacts) so
you can eyeball in the timeline whether that dry time was a jam or an empty
indexer. We don't attempt to classify automatically — that's a judgment call
you'd have to make by watching the match video.

Per-interval events:
    {start_s, ready_s, end_s, spinup_s, active_s, balls, effective_bps,
     longest_dry_s, phase}
"""

from __future__ import annotations

from log_context import AnalyzerResult, LogContext, register_analyzer

COORDINATOR_STATE = "/RealOutputs/SmartLaunch/CoordinatorState"
LAUNCHER_AT_SETPOINT = "/Launcher/AtSetpoint"
BALL_IMPACT_COUNT = "/RealOutputs/Launcher/BallImpact/Count"
DS_AUTONOMOUS = "/DriverStation/Autonomous"

TARGET_BPS = 12.0
DRY_MIN_SEC = 0.3  # only dry stretches >= this count toward longest_dry


@register_analyzer(id="effective_bps", title="Effective BPS")
def analyze(ctx: LogContext) -> AnalyzerResult:
    coord = ctx.signal(COORDINATOR_STATE)
    ready = ctx.signal(LAUNCHER_AT_SETPOINT)
    ball_count = ctx.signal(BALL_IMPACT_COUNT)
    auto = ctx.signal(DS_AUTONOMOUS)

    # Firing intervals
    firing: list[tuple[int, int]] = []
    start = None
    for ts, v in zip(coord.timestamps_us, coord.values):
        if v == "FIRING" and start is None:
            start = ts
        elif v != "FIRING" and start is not None:
            firing.append((start, ts))
            start = None
    if start is not None and coord.timestamps_us:
        firing.append((start, coord.timestamps_us[-1]))

    # Precompute ball-impact timestamps
    impacts: list[int] = []
    prev = 0
    for ts, v in zip(ball_count.timestamps_us, ball_count.values):
        c = int(v or 0)
        if c > prev:
            for _ in range(c - prev):
                impacts.append(ts)
            prev = c

    import bisect

    events = []
    total_balls = 0
    total_effective_s = 0.0
    total_spinup_s = 0.0
    never_ready = 0

    for s, e in firing:
        # First moment launcher is at setpoint within [s, e].
        first_ready = _first_true_at(ready, s, e)
        dur_firing = (e - s) / 1e6
        balls_before = int(ball_count.value_at(s - 1, 0) or 0)
        balls_after = int(ball_count.value_at(e, balls_before) or balls_before)
        balls = max(0, balls_after - balls_before)
        phase = "auto" if auto.value_at(s, False) else "teleop"

        if first_ready is None:
            never_ready += 1
            events.append({
                "start_s": ctx.rel_s(s),
                "ready_s": None,
                "end_s": ctx.rel_s(e),
                "spinup_s": round(dur_firing, 3),
                "active_s": 0.0,
                "balls": balls,
                "effective_bps": 0.0,
                "longest_dry_s": 0.0,
                "phase": phase,
                "note": "launcher never reached setpoint during firing",
            })
            total_spinup_s += dur_firing
            continue

        spinup = (first_ready - s) / 1e6
        active = (e - first_ready) / 1e6
        ebps = balls / active if active > 0.05 else 0.0
        # Longest dry stretch (no ball impacts) inside [first_ready, e] — helps
        # eyeball jam vs empty. We include the gap from first_ready to first
        # impact, and from last impact to e.
        lo = bisect.bisect_left(impacts, first_ready)
        hi = bisect.bisect_right(impacts, e)
        interval_impacts = impacts[lo:hi]
        edges = [first_ready] + interval_impacts + [e]
        gaps = [(edges[i + 1] - edges[i]) / 1e6 for i in range(len(edges) - 1)]
        longest_dry = max((g for g in gaps if g >= DRY_MIN_SEC), default=0.0)

        total_balls += balls
        total_effective_s += active
        total_spinup_s += spinup

        events.append({
            "start_s": ctx.rel_s(s),
            "ready_s": ctx.rel_s(first_ready),
            "end_s": ctx.rel_s(e),
            "spinup_s": round(spinup, 3),
            "active_s": round(active, 3),
            "balls": balls,
            "effective_bps": round(ebps, 3),
            "longest_dry_s": round(longest_dry, 3),
            "phase": phase,
        })

    effective_bps = total_balls / total_effective_s if total_effective_s > 0.1 else 0.0

    summary = {
        "effective_bps": round(effective_bps, 3),
        "target_bps": TARGET_BPS,
        "target_gap_bps": round(max(0.0, TARGET_BPS - effective_bps), 3),
        "balls_during_firing": total_balls,
        "active_seconds_total": round(total_effective_s, 2),
        "spinup_seconds_total": round(total_spinup_s, 2),
        "firing_intervals_total": len(firing),
        "intervals_never_ready": never_ready,
        "longest_dry_s": round(max((e.get("longest_dry_s", 0) for e in events), default=0.0), 3),
    }
    return AnalyzerResult(id="effective_bps", title="Effective BPS", events=events, summary=summary)


def _first_true_at(signal, t0_us: int, t1_us: int):
    # If already true at t0, return t0.
    if bool(signal.value_at(t0_us, False)):
        return t0_us
    for ts, v in signal.iter_between(t0_us, t1_us):
        if bool(v):
            return ts
    return None
