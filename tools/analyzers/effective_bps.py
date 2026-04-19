"""Effective BPS with time-bucket accounting + aim-mode tag.

For each `CoordinatorState == FIRING` interval we walk the union of edge
timestamps from coord / launcher-ready / turret state / spindexer state and
classify each sub-slice into one of nine buckets. The buckets sum to the
full firing interval:

    feeding_s          coord=FIRING, launcher AtSetpoint, turret not FLIPPING/STALLED,
                       spindexer=FEEDING. This is the *real* denominator for BPS.
    flipping_s         turret FLIPPING or STALLED (motivator stopped per
                       ShootingCommands.java:569 — we can't feed regardless
                       of spindexer state)
    suppressed_s       spindexer SUPPRESSED (operator hold-fire, button 11)
    unclogging_s       spindexer UNCLOGGING (manual, button 2)
    auto_unclogging_s  spindexer AUTO_UNCLOGGING
    jammed_s           spindexer JAMMED
    reciprocating_s    spindexer RECIPROCATING during FIRING
    stopped_s          spindexer STOPPED during FIRING
    not_ready_s        launcher not yet AtSetpoint (spin-up latency)

Headline:  truly_active_bps = balls ÷ feeding_s

Per-interval event fields mirror the buckets plus `aim_mode` sampled at
interval start (HUB / PASS / LONG_PASS / NONE).
"""

from __future__ import annotations

import bisect

from log_context import AnalyzerResult, LogContext, register_analyzer

COORDINATOR_STATE = "/RealOutputs/SmartLaunch/CoordinatorState"
LAUNCHER_AT_SETPOINT = "/Launcher/AtSetpoint"
TURRET_STATE = "/RealOutputs/Subsystems/TurretState"
SPINDEXER_STATE = "/RealOutputs/Subsystems/SpindexerState"
BALL_IMPACT_COUNT = "/RealOutputs/Launcher/BallImpact/Count"
DS_AUTONOMOUS = "/DriverStation/Autonomous"
AIM_MODE = "/RealOutputs/SmartLaunch/Status/AimMode"
ZONE = "/RealOutputs/SmartLaunch/Status/Zone"

TARGET_BPS = 12.0
DRY_MIN_SEC = 0.3

TURRET_BLOCKING = {"FLIPPING", "STALLED"}

# Buckets in the order we report them — also defines the stacked-bar order.
BUCKETS = [
    "feeding_s",
    "flipping_s",
    "suppressed_s",
    "unclogging_s",
    "auto_unclogging_s",
    "jammed_s",
    "reciprocating_s",
    "stopped_s",
    "not_ready_s",
]


@register_analyzer(id="effective_bps", title="Effective BPS")
def analyze(ctx: LogContext) -> AnalyzerResult:
    coord = ctx.signal(COORDINATOR_STATE)
    launcher_ready = ctx.signal(LAUNCHER_AT_SETPOINT)
    turret = ctx.signal(TURRET_STATE)
    spindexer = ctx.signal(SPINDEXER_STATE)
    ball_count = ctx.signal(BALL_IMPACT_COUNT)
    auto = ctx.signal(DS_AUTONOMOUS)
    aim_mode = ctx.signal(AIM_MODE)
    zone = ctx.signal(ZONE)

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

    # Pre-extract ball impact timestamps from the monotonic counter.
    impacts: list[int] = []
    prev = 0
    for ts, v in zip(ball_count.timestamps_us, ball_count.values):
        c = int(v or 0)
        if c > prev:
            for _ in range(c - prev):
                impacts.append(ts)
            prev = c

    events = []
    totals = {k: 0.0 for k in BUCKETS}
    total_balls = 0
    total_firing_s = 0.0
    longest_dry_overall = 0.0
    never_ready_count = 0

    for s, e in firing:
        dur = (e - s) / 1e6
        total_firing_s += dur

        buckets = _bucket_firing_interval(s, e, launcher_ready, turret, spindexer)
        for k, v in buckets.items():
            totals[k] += v

        balls_before = int(ball_count.value_at(s - 1, 0) or 0)
        balls_after = int(ball_count.value_at(e, balls_before) or balls_before)
        balls = max(0, balls_after - balls_before)
        total_balls += balls

        first_ready = _first_true_at(launcher_ready, s, e)
        if first_ready is None:
            never_ready_count += 1

        # Dry-stretch detection: longest gap between impacts inside feeding
        # slices. Simpler approximation: longest gap inside [first_ready, e]
        # (same as before) — feeding-only version would need the merged slice
        # list; this is close enough for the timeline-scrub use case.
        longest_dry = _longest_dry(first_ready or s, e, impacts)
        longest_dry_overall = max(longest_dry_overall, longest_dry)

        feeding_s = buckets["feeding_s"]
        truly_bps = balls / feeding_s if feeding_s > 0.05 else 0.0

        phase = "auto" if auto.value_at(s, False) else "teleop"
        mode = aim_mode.value_at(s, "") or aim_mode.value_at(e - 1, "") or "NONE"
        zone_val = zone.value_at(s, "") or ""

        events.append({
            "start_s": ctx.rel_s(s),
            "end_s": ctx.rel_s(e),
            "duration_s": round(dur, 3),
            "ready_s": ctx.rel_s(first_ready) if first_ready is not None else None,
            "balls": balls,
            "truly_active_bps": round(truly_bps, 3),
            "longest_dry_s": round(longest_dry, 3),
            "phase": phase,
            "aim_mode": mode,
            "zone_at_start": zone_val,
            **{k: round(v, 3) for k, v in buckets.items()},
        })

    feeding_total = totals["feeding_s"]
    truly_active_bps = total_balls / feeding_total if feeding_total > 0.1 else 0.0

    # Retained: "effective_bps" = balls / (all firing time where launcher was
    # ever ready), i.e. firing time minus not-ready. Useful as a comparison.
    effective_denom = total_firing_s - totals["not_ready_s"]
    effective_bps = total_balls / effective_denom if effective_denom > 0.1 else 0.0

    summary = {
        "truly_active_bps": round(truly_active_bps, 3),
        "effective_bps": round(effective_bps, 3),
        "target_bps": TARGET_BPS,
        "target_gap_bps": round(max(0.0, TARGET_BPS - truly_active_bps), 3),
        "balls_during_firing": total_balls,
        "firing_seconds_total": round(total_firing_s, 2),
        "firing_intervals_total": len(firing),
        "intervals_never_ready": never_ready_count,
        "longest_dry_s": round(longest_dry_overall, 3),
        **{k: round(v, 2) for k, v in totals.items()},
    }
    return AnalyzerResult(id="effective_bps", title="Effective BPS", events=events, summary=summary)


def _bucket_firing_interval(t0_us, t1_us, launcher_ready, turret, spindexer):
    """Walk [t0, t1] and classify each sub-slice into a bucket."""
    edges = {t0_us, t1_us}
    for ts, _ in launcher_ready.iter_between(t0_us, t1_us):
        edges.add(ts)
    for ts, _ in turret.iter_between(t0_us, t1_us):
        edges.add(ts)
    for ts, _ in spindexer.iter_between(t0_us, t1_us):
        edges.add(ts)
    edges = sorted(e for e in edges if t0_us <= e <= t1_us)

    out = {k: 0.0 for k in BUCKETS}
    for a, b in zip(edges[:-1], edges[1:]):
        secs = (b - a) / 1e6
        bucket = _classify_slice(a, launcher_ready, turret, spindexer)
        out[bucket] += secs
    return out


def _classify_slice(t_us, launcher_ready, turret, spindexer) -> str:
    # Order matters — check blocking conditions before feeding.
    if not bool(launcher_ready.value_at(t_us, False)):
        return "not_ready_s"
    tstate = turret.value_at(t_us, "")
    if tstate in TURRET_BLOCKING:
        return "flipping_s"
    sstate = spindexer.value_at(t_us, "")
    if sstate == "FEEDING":
        return "feeding_s"
    if sstate == "SUPPRESSED":
        return "suppressed_s"
    if sstate == "UNCLOGGING":
        return "unclogging_s"
    if sstate == "AUTO_UNCLOGGING":
        return "auto_unclogging_s"
    if sstate == "JAMMED":
        return "jammed_s"
    if sstate == "RECIPROCATING":
        return "reciprocating_s"
    if sstate == "STOPPED":
        return "stopped_s"
    # Unknown/missing spindexer state — count as stopped rather than drop.
    return "stopped_s"


def _first_true_at(signal, t0_us, t1_us):
    if bool(signal.value_at(t0_us, False)):
        return t0_us
    for ts, v in signal.iter_between(t0_us, t1_us):
        if bool(v):
            return ts
    return None


def _longest_dry(t0_us, t1_us, impacts):
    lo = bisect.bisect_left(impacts, t0_us)
    hi = bisect.bisect_right(impacts, t1_us)
    between = impacts[lo:hi]
    edges = [t0_us] + between + [t1_us]
    gaps = [(edges[i + 1] - edges[i]) / 1e6 for i in range(len(edges) - 1)]
    return max((g for g in gaps if g >= DRY_MIN_SEC), default=0.0)
