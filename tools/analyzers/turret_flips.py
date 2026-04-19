"""Turret-flip impact analysis.

Counts entries into `Subsystems/TurretState == "FLIPPING"`. For each flip,
reports duration, entry angle, exit angle, and whether it overlapped any
firing interval (or occurred within ±0.5 s of one).

Event output:
    {start_s, end_s, duration_s, entry_angle, exit_angle, during_firing}
"""

from __future__ import annotations

from log_context import AnalyzerResult, LogContext, register_analyzer

TURRET_STATE = "/RealOutputs/Subsystems/TurretState"
TURRET_ANGLE = "/Turret/CurrentInsideAngleDeg"
COORDINATOR_STATE = "/RealOutputs/SmartLaunch/CoordinatorState"

FIRING_PROXIMITY_S = 0.5


@register_analyzer(id="turret_flips", title="Turret flips")
def analyze(ctx: LogContext) -> AnalyzerResult:
    state = ctx.signal(TURRET_STATE)
    angle = ctx.signal(TURRET_ANGLE)
    coord = ctx.signal(COORDINATOR_STATE)

    events: list[dict] = []
    total_flip_s = 0.0
    during_firing_count = 0
    during_firing_s = 0.0

    in_flip = False
    flip_start = 0
    flip_entry_angle = 0.0

    for ts, v in zip(state.timestamps_us, state.values):
        if v == "FLIPPING" and not in_flip:
            in_flip = True
            flip_start = ts
            flip_entry_angle = angle.value_at(ts, 0.0) or 0.0
        elif v != "FLIPPING" and in_flip:
            in_flip = False
            dur = (ts - flip_start) / 1e6
            total_flip_s += dur
            # Was this during firing (or within ±0.5s of a firing span)?
            overlap = _near_firing(flip_start, ts, coord, FIRING_PROXIMITY_S)
            if overlap:
                during_firing_count += 1
                during_firing_s += dur
            events.append(
                {
                    "start_s": ctx.rel_s(flip_start),
                    "end_s": ctx.rel_s(ts),
                    "duration_s": round(dur, 3),
                    "entry_angle": round(flip_entry_angle, 1),
                    "exit_angle": round(angle.value_at(ts, 0.0) or 0.0, 1),
                    "during_firing": overlap,
                }
            )

    summary = {
        "flip_count": len(events),
        "flips_during_firing": during_firing_count,
        "flip_seconds_total": round(total_flip_s, 2),
        "flip_seconds_during_firing": round(during_firing_s, 2),
        "longest_flip_s": round(max((e["duration_s"] for e in events), default=0.0), 3),
    }
    return AnalyzerResult(id="turret_flips", title="Turret flips", events=events, summary=summary)


def _near_firing(t0_us: int, t1_us: int, coord, pad_s: float) -> bool:
    pad_us = int(pad_s * 1e6)
    lo = t0_us - pad_us
    hi = t1_us + pad_us
    # Quick scan: any FIRING sample inside [lo, hi]?
    for _, v in coord.iter_between(lo, hi):
        if v == "FIRING":
            return True
    # Edge case: sparse coord state, check current state at t0 and t1.
    if coord.value_at(t0_us) == "FIRING" or coord.value_at(t1_us) == "FIRING":
        return True
    return False
