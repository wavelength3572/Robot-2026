"""Balls-per-second (BPS) metrics.

Three headline numbers per match:
    active_bps  = ball impacts / seconds in FIRING & launcher & motivator at setpoint
    firing_bps  = ball impacts / seconds in FIRING
    best_burst  = peak BPS over any 2s window inside a FIRING interval

Plus per-firing-interval BPS so the dashboard can highlight good/bad bursts.
"""

from __future__ import annotations

from log_context import AnalyzerResult, LogContext, register_analyzer

COORDINATOR_STATE = "/RealOutputs/SmartLaunch/CoordinatorState"
LAUNCHER_AT_SETPOINT = "/Launcher/AtSetpoint"
MOTIVATOR_AT_SETPOINT = "/Motivator/AtSetpoint"
BALL_IMPACT_COUNT = "/RealOutputs/Launcher/BallImpact/Count"


@register_analyzer(id="bps", title="Balls per second")
def analyze(ctx: LogContext) -> AnalyzerResult:
    coord = ctx.signal(COORDINATOR_STATE)
    launcher_ready = ctx.signal(LAUNCHER_AT_SETPOINT)
    motivator_ready = ctx.signal(MOTIVATOR_AT_SETPOINT)
    ball_count = ctx.signal(BALL_IMPACT_COUNT)

    # Find firing intervals (duplicates firing_intervals analyzer logic — cheap
    # enough and keeps analyzers independent).
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
    total_active_s = 0.0
    total_balls_in_firing = 0
    peak_burst = 0.0

    for s, e in intervals:
        dur = (e - s) / 1e6
        total_firing_s += dur
        active_s = _active_seconds(s, e, launcher_ready, motivator_ready)
        total_active_s += active_s

        balls_before = int(ball_count.value_at(s - 1, 0) or 0)
        balls_after = int(ball_count.value_at(e, balls_before) or balls_before)
        delta = max(0, balls_after - balls_before)
        total_balls_in_firing += delta

        interval_bps = delta / dur if dur > 0 else 0.0
        interval_active_bps = delta / active_s if active_s > 0.15 else 0.0

        burst = _peak_burst(s, e, ball_count, window_us=2_000_000)
        peak_burst = max(peak_burst, burst)

        events.append(
            {
                "start_s": ctx.rel_s(s),
                "end_s": ctx.rel_s(e),
                "duration_s": round(dur, 3),
                "active_duration_s": round(active_s, 3),
                "balls_fired": delta,
                "bps": round(interval_bps, 3),
                "active_bps": round(interval_active_bps, 3),
                "peak_2s_bps": round(burst, 3),
            }
        )

    active_bps = total_balls_in_firing / total_active_s if total_active_s > 0.1 else 0.0
    firing_bps = total_balls_in_firing / total_firing_s if total_firing_s > 0.1 else 0.0

    summary = {
        "active_bps": round(active_bps, 3),
        "firing_bps": round(firing_bps, 3),
        "peak_2s_bps": round(peak_burst, 3),
        "balls_during_firing": total_balls_in_firing,
        "firing_seconds": round(total_firing_s, 2),
        "active_firing_seconds": round(total_active_s, 2),
    }
    return AnalyzerResult(id="bps", title="Balls per second", events=events, summary=summary)


def _active_seconds(t0_us: int, t1_us: int, launcher_ready, motivator_ready) -> float:
    edges = {t0_us, t1_us}
    for ts, _ in launcher_ready.iter_between(t0_us, t1_us):
        edges.add(ts)
    for ts, _ in motivator_ready.iter_between(t0_us, t1_us):
        edges.add(ts)
    sorted_edges = sorted(edges)
    total_us = 0
    for a, b in zip(sorted_edges[:-1], sorted_edges[1:]):
        if bool(launcher_ready.value_at(a, False)) and bool(motivator_ready.value_at(a, False)):
            total_us += b - a
    return total_us / 1e6


def _peak_burst(t0_us: int, t1_us: int, ball_count, window_us: int) -> float:
    """Max BPS over any `window_us`-long window inside [t0, t1]."""
    ts_list = ball_count.timestamps_us
    val_list = ball_count.values
    if not ts_list:
        return 0.0
    peak = 0.0
    step = 200_000  # 0.2s sliding step
    t = t0_us
    while t + window_us <= t1_us:
        a = int(ball_count.value_at(t, 0) or 0)
        b = int(ball_count.value_at(t + window_us, a) or a)
        burst = (b - a) / (window_us / 1e6)
        if burst > peak:
            peak = burst
        t += step
    return peak
