"""BPS split by aim mode — HUB vs PASS vs LONG_PASS.

Same partition logic as `effective_bps` but re-bucketed by
`/RealOutputs/SmartLaunch/Status/AimMode`. The coordinator picks the mode
per-zone (`ShootingCoordinator.java:682-755`):
    ALLIANCE_CLOSE/MID/FAR, ALLIANCE_TRENCH     → HUB
    NEUTRAL                                     → PASS
    OPPONENT                                    → LONG_PASS
    BUMP, DANGER_TRENCH, NEUTRAL_TRENCH          → NONE (firing suppressed)

Summary per match:
    hub_bps, hub_balls, hub_feeding_s, hub_intervals
    pass_bps, pass_balls, pass_feeding_s, pass_intervals
    long_pass_bps, long_pass_balls, long_pass_feeding_s, long_pass_intervals
    none_intervals

Events: one per firing interval, with an `aim_mode` field so the dashboard
can filter.
"""

from __future__ import annotations

from log_context import AnalyzerResult, LogContext, register_analyzer

COORDINATOR_STATE = "/RealOutputs/SmartLaunch/CoordinatorState"
LAUNCHER_AT_SETPOINT = "/Launcher/AtSetpoint"
TURRET_STATE = "/RealOutputs/Subsystems/TurretState"
SPINDEXER_STATE = "/RealOutputs/Subsystems/SpindexerState"
BALL_IMPACT_COUNT = "/RealOutputs/Launcher/BallImpact/Count"
AIM_MODE = "/RealOutputs/SmartLaunch/Status/AimMode"

TARGET_BPS = 12.0
TURRET_BLOCKING = {"FLIPPING", "STALLED"}
MODES = ("HUB", "PASS", "LONG_PASS", "NONE")


@register_analyzer(id="bps_by_mode", title="BPS by aim mode")
def analyze(ctx: LogContext) -> AnalyzerResult:
    coord = ctx.signal(COORDINATOR_STATE)
    launcher_ready = ctx.signal(LAUNCHER_AT_SETPOINT)
    turret = ctx.signal(TURRET_STATE)
    spindexer = ctx.signal(SPINDEXER_STATE)
    ball_count = ctx.signal(BALL_IMPACT_COUNT)
    aim_mode = ctx.signal(AIM_MODE)

    firing = []
    start = None
    for ts, v in zip(coord.timestamps_us, coord.values):
        if v == "FIRING" and start is None:
            start = ts
        elif v != "FIRING" and start is not None:
            firing.append((start, ts))
            start = None
    if start is not None and coord.timestamps_us:
        firing.append((start, coord.timestamps_us[-1]))

    by_mode = {
        m: {"balls": 0, "feeding_s": 0.0, "intervals": 0, "firing_s": 0.0}
        for m in MODES
    }
    events = []

    for s, e in firing:
        dur = (e - s) / 1e6
        balls_before = int(ball_count.value_at(s - 1, 0) or 0)
        balls_after = int(ball_count.value_at(e, balls_before) or balls_before)
        balls = max(0, balls_after - balls_before)

        feeding_s = _feeding_seconds(s, e, launcher_ready, turret, spindexer)

        mode = aim_mode.value_at(s, "") or aim_mode.value_at(e - 1, "") or "NONE"
        if mode not in by_mode:
            mode = "NONE"
        by_mode[mode]["balls"] += balls
        by_mode[mode]["feeding_s"] += feeding_s
        by_mode[mode]["firing_s"] += dur
        by_mode[mode]["intervals"] += 1

        events.append({
            "start_s": ctx.rel_s(s),
            "end_s": ctx.rel_s(e),
            "duration_s": round(dur, 3),
            "feeding_s": round(feeding_s, 3),
            "balls": balls,
            "bps": round(balls / feeding_s, 3) if feeding_s > 0.05 else 0.0,
            "aim_mode": mode,
        })

    summary = {}
    for m in MODES:
        d = by_mode[m]
        prefix = m.lower()
        summary[f"{prefix}_balls"] = d["balls"]
        summary[f"{prefix}_feeding_s"] = round(d["feeding_s"], 2)
        summary[f"{prefix}_firing_s"] = round(d["firing_s"], 2)
        summary[f"{prefix}_intervals"] = d["intervals"]
        summary[f"{prefix}_bps"] = (
            round(d["balls"] / d["feeding_s"], 3) if d["feeding_s"] > 0.05 else 0.0
        )
    summary["target_bps"] = TARGET_BPS
    return AnalyzerResult(id="bps_by_mode", title="BPS by aim mode", events=events, summary=summary)


def _feeding_seconds(t0_us, t1_us, launcher_ready, turret, spindexer) -> float:
    edges = {t0_us, t1_us}
    for ts, _ in launcher_ready.iter_between(t0_us, t1_us):
        edges.add(ts)
    for ts, _ in turret.iter_between(t0_us, t1_us):
        edges.add(ts)
    for ts, _ in spindexer.iter_between(t0_us, t1_us):
        edges.add(ts)
    edges = sorted(e for e in edges if t0_us <= e <= t1_us)
    total = 0
    for a, b in zip(edges[:-1], edges[1:]):
        if not bool(launcher_ready.value_at(a, False)):
            continue
        if turret.value_at(a, "") in TURRET_BLOCKING:
            continue
        if spindexer.value_at(a, "") != "FEEDING":
            continue
        total += b - a
    return total / 1e6
