"""Firing-state interval detection.

Produces two interval sets:

- `firing` — contiguous spans where `SmartLaunch/CoordinatorState == "FIRING"`.
- `active_firing` — subset where `Launcher/AtSetpoint` and `Motivator/AtSetpoint`
  are both true at the same moment. This is the denominator for BPS and the
  basis for jam detection (we only call a jam a jam once we're *trying* to feed).

Event output has:
    {start, end, duration_s, active_duration_s, phase, shots_fired}
"""

from __future__ import annotations

from log_context import AnalyzerResult, LogContext, register_analyzer

COORDINATOR_STATE_KEY = "/RealOutputs/SmartLaunch/CoordinatorState"
LAUNCHER_AT_SETPOINT_KEY = "/Launcher/AtSetpoint"
MOTIVATOR_AT_SETPOINT_KEY = "/Motivator/AtSetpoint"
DS_AUTONOMOUS_KEY = "/DriverStation/Autonomous"
SHOT_COUNTER_KEY = "/RealOutputs/ShotLog/TotalShots"


def _contiguous(state_ts: list[int], state_val: list[str], match: str) -> list[tuple[int, int]]:
    """Return [(start_us, end_us), ...] where state == match."""
    intervals: list[tuple[int, int]] = []
    start = None
    for ts, v in zip(state_ts, state_val):
        if v == match and start is None:
            start = ts
        elif v != match and start is not None:
            intervals.append((start, ts))
            start = None
    if start is not None and state_ts:
        intervals.append((start, state_ts[-1]))
    return intervals


@register_analyzer(id="firing_intervals", title="Firing intervals")
def analyze(ctx: LogContext) -> AnalyzerResult:
    coord = ctx.signal(COORDINATOR_STATE_KEY)
    launcher_ready = ctx.signal(LAUNCHER_AT_SETPOINT_KEY)
    motivator_ready = ctx.signal(MOTIVATOR_AT_SETPOINT_KEY)
    auto = ctx.signal(DS_AUTONOMOUS_KEY)
    shots = ctx.signal(SHOT_COUNTER_KEY)

    firing_intervals = _contiguous(coord.timestamps_us, coord.values, "FIRING")

    events = []
    total_firing_s = 0.0
    total_active_s = 0.0
    total_shots = 0
    for start, end in firing_intervals:
        dur = (end - start) / 1e6
        total_firing_s += dur

        # Active firing time inside this span: walk the launcher ready signal.
        active_s = _active_seconds(start, end, launcher_ready, motivator_ready)
        total_active_s += active_s

        phase = "auto" if auto.value_at(start, False) else "teleop"
        shots_before = shots.value_at(start - 1, 0) or 0
        shots_after = shots.value_at(end, shots_before) or shots_before
        fired = int(shots_after) - int(shots_before)
        total_shots += fired

        events.append(
            {
                "start_s": ctx.rel_s(start),
                "end_s": ctx.rel_s(end),
                "duration_s": round(dur, 3),
                "active_duration_s": round(active_s, 3),
                "phase": phase,
                "shots_fired": max(0, fired),
            }
        )

    summary = {
        "firing_intervals": len(events),
        "firing_seconds_total": round(total_firing_s, 2),
        "active_firing_seconds_total": round(total_active_s, 2),
        "shots_during_firing": total_shots,
    }
    return AnalyzerResult(id="firing_intervals", title="Firing intervals", events=events, summary=summary)


def _active_seconds(t0_us: int, t1_us: int, launcher_ready, motivator_ready) -> float:
    """Integrate the time during [t0, t1] where both 'at setpoint' signals are true."""
    # Merge event times from both signals plus the endpoints.
    edges = [t0_us, t1_us]
    for ts, _ in launcher_ready.iter_between(t0_us, t1_us):
        edges.append(ts)
    for ts, _ in motivator_ready.iter_between(t0_us, t1_us):
        edges.append(ts)
    edges = sorted(set(e for e in edges if t0_us <= e <= t1_us))
    if len(edges) < 2:
        return 0.0
    total_us = 0
    for a, b in zip(edges[:-1], edges[1:]):
        la = bool(launcher_ready.value_at(a, False))
        ma = bool(motivator_ready.value_at(a, False))
        if la and ma:
            total_us += b - a
    return total_us / 1e6
