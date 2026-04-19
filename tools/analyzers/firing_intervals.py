"""Firing-state intervals.

Contiguous spans where `SmartLaunch/CoordinatorState == "FIRING"`. This is
the driver's-seat "fire button held" window. Each interval has the number
of ball impacts that occurred inside it and whether it was auto or teleop.

Event output:
    {start_s, end_s, duration_s, phase, balls_fired}
"""

from __future__ import annotations

from log_context import AnalyzerResult, LogContext, register_analyzer

COORDINATOR_STATE_KEY = "/RealOutputs/SmartLaunch/CoordinatorState"
DS_AUTONOMOUS_KEY = "/DriverStation/Autonomous"
BALL_IMPACT_COUNT = "/RealOutputs/Launcher/BallImpact/Count"


@register_analyzer(id="firing_intervals", title="Firing intervals")
def analyze(ctx: LogContext) -> AnalyzerResult:
    coord = ctx.signal(COORDINATOR_STATE_KEY)
    auto = ctx.signal(DS_AUTONOMOUS_KEY)
    ball_count = ctx.signal(BALL_IMPACT_COUNT)

    intervals: list[tuple[int, int]] = []
    start = None
    for ts, v in zip(coord.timestamps_us, coord.values):
        if v == "FIRING" and start is None:
            start = ts
        elif v != "FIRING" and start is not None:
            intervals.append((start, ts))
            start = None
    if start is not None and coord.timestamps_us:
        intervals.append((start, coord.timestamps_us[-1]))

    events = []
    total_firing_s = 0.0
    total_balls = 0
    for s, e in intervals:
        dur = (e - s) / 1e6
        total_firing_s += dur
        phase = "auto" if auto.value_at(s, False) else "teleop"
        balls_before = int(ball_count.value_at(s - 1, 0) or 0)
        balls_after = int(ball_count.value_at(e, balls_before) or balls_before)
        balls = max(0, balls_after - balls_before)
        total_balls += balls
        events.append(
            {
                "start_s": ctx.rel_s(s),
                "end_s": ctx.rel_s(e),
                "duration_s": round(dur, 3),
                "phase": phase,
                "balls_fired": balls,
            }
        )

    summary = {
        "firing_intervals": len(events),
        "firing_seconds_total": round(total_firing_s, 2),
        "balls_during_firing": total_balls,
    }
    return AnalyzerResult(id="firing_intervals", title="Firing intervals", events=events, summary=summary)
