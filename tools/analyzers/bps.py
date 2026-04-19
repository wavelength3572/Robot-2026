"""Per-firing-interval BPS — driver's-seat view.

This is the metric that matters competitively: from pulling the fire button
to releasing it, how many balls came out?

(The previous "active BPS" metric was dropped. It tried to credit only the
time when Launcher and Motivator were simultaneously AtSetpoint, but the
robot code doesn't actually stop the motivator between balls during FIRING,
so that math was measuring `AtSetpoint` flicker noise rather than anything
meaningful. See the `slugs` analyzer for the mechanism-capability ceiling.)

Summary:
    firing_bps         — total balls during FIRING ÷ total FIRING seconds
    peak_2s_bps        — highest BPS over any 2 s window within any firing interval
    target_bps         — 12.0 (competitive baseline)
    target_gap_bps     — 12.0 - firing_bps (how short we are)
    balls_during_firing, firing_seconds
"""

from __future__ import annotations

from log_context import AnalyzerResult, LogContext, register_analyzer

COORDINATOR_STATE = "/RealOutputs/SmartLaunch/CoordinatorState"
BALL_IMPACT_COUNT = "/RealOutputs/Launcher/BallImpact/Count"

TARGET_BPS = 12.0


@register_analyzer(id="bps", title="Firing BPS")
def analyze(ctx: LogContext) -> AnalyzerResult:
    coord = ctx.signal(COORDINATOR_STATE)
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

    events: list[dict] = []
    total_firing_s = 0.0
    total_balls = 0
    peak_burst = 0.0

    for s, e in intervals:
        dur = (e - s) / 1e6
        total_firing_s += dur
        balls_before = int(ball_count.value_at(s - 1, 0) or 0)
        balls_after = int(ball_count.value_at(e, balls_before) or balls_before)
        delta = max(0, balls_after - balls_before)
        total_balls += delta

        interval_bps = delta / dur if dur > 0 else 0.0
        burst = _peak_burst(s, e, ball_count, window_us=2_000_000)
        peak_burst = max(peak_burst, burst)

        events.append(
            {
                "start_s": ctx.rel_s(s),
                "end_s": ctx.rel_s(e),
                "duration_s": round(dur, 3),
                "balls_fired": delta,
                "bps": round(interval_bps, 3),
                "peak_2s_bps": round(burst, 3),
            }
        )

    firing_bps = total_balls / total_firing_s if total_firing_s > 0.1 else 0.0

    summary = {
        "firing_bps": round(firing_bps, 3),
        "peak_2s_bps": round(peak_burst, 3),
        "target_bps": TARGET_BPS,
        "target_gap_bps": round(max(0.0, TARGET_BPS - firing_bps), 3),
        "balls_during_firing": total_balls,
        "firing_seconds": round(total_firing_s, 2),
    }
    return AnalyzerResult(id="bps", title="Firing BPS", events=events, summary=summary)


def _peak_burst(t0_us: int, t1_us: int, ball_count, window_us: int) -> float:
    if not ball_count.timestamps_us:
        return 0.0
    peak = 0.0
    step = 200_000
    t = t0_us
    while t + window_us <= t1_us:
        a = int(ball_count.value_at(t, 0) or 0)
        b = int(ball_count.value_at(t + window_us, a) or a)
        burst = (b - a) / (window_us / 1e6)
        if burst > peak:
            peak = burst
        t += step
    return peak
