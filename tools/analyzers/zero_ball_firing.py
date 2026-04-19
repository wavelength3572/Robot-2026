"""Zero-ball firing intervals — classify why they fired nothing.

Scans every `CoordinatorState == FIRING` interval that produced zero ball
impacts and inspects the context to classify the likely cause.

Signals inspected during each zero-ball interval:
    /RealOutputs/SmartLaunch/Target/Achievable
    /RealOutputs/SmartLaunch/Blocking          (freshest reason string)
    /RealOutputs/SmartLaunch/Distance/RawDistanceM
    /Motivator/TargetRPM, /Motivator/WheelRPM
    /RealOutputs/Subsystems/SpindexerState
    /Launcher/AtSetpoint
    /Launcher/WheelVelocityRPM, /Launcher/TargetVelocityRPM
    /Turret/TargetInsideAngleDeg, /Turret/CurrentInsideAngleDeg

Classification (first match wins):
    too_short              duration < 0.5s (button tapped, can't feed that fast)
    zone_exit              motivator target dropped to 0 during the interval
                           (the FIRING-but-zero-targets bug we've seen)
    unachievable           Target/Achievable was false for > 30% of interval
    turret_not_aimed       |turret angle - target| > 5 deg for > 30% of interval
    spindexer_never_fed    SpindexerState was never FEEDING during the interval
    motivator_never_spun   Motivator wheelRPM never exceeded 100 RPM
    launcher_never_ready   Launcher AtSetpoint was never true during the interval
    other                  none of the above

Per-interval event:
    {start_s, end_s, duration_s, aim_mode, zone_at_start, distance_m_at_start,
     cause, achievable_frac_false, turret_off_aim_frac, feeding_frac,
     launcher_ready_frac, motivator_peak_rpm, blocking_sample}
"""

from __future__ import annotations

from log_context import AnalyzerResult, LogContext, register_analyzer

COORDINATOR_STATE = "/RealOutputs/SmartLaunch/CoordinatorState"
AIM_MODE = "/RealOutputs/SmartLaunch/Status/AimMode"
ZONE = "/RealOutputs/SmartLaunch/Status/Zone"
ACHIEVABLE = "/RealOutputs/SmartLaunch/Target/Achievable"
BLOCKING = "/RealOutputs/SmartLaunch/Blocking"
DISTANCE = "/RealOutputs/SmartLaunch/Distance/RawDistanceM"
MOTIVATOR_TARGET = "/Motivator/TargetRPM"
MOTIVATOR_ACTUAL = "/Motivator/WheelRPM"
SPINDEXER_STATE = "/RealOutputs/Subsystems/SpindexerState"
LAUNCHER_READY = "/Launcher/AtSetpoint"
LAUNCHER_ACTUAL = "/Launcher/WheelVelocityRPM"
LAUNCHER_TARGET = "/Launcher/TargetVelocityRPM"
TURRET_CURRENT = "/Turret/CurrentInsideAngleDeg"
TURRET_TARGET = "/Turret/TargetInsideAngleDeg"
BALL_IMPACT_COUNT = "/RealOutputs/Launcher/BallImpact/Count"

MIN_DUR_S = 0.5
MIN_MOTIVATOR_RPM = 100.0
TURRET_OFF_AIM_DEG = 5.0
HIGH_FRAC = 0.30


@register_analyzer(id="zero_ball_firing", title="Zero-ball firing intervals")
def analyze(ctx: LogContext) -> AnalyzerResult:
    coord = ctx.signal(COORDINATOR_STATE)
    aim = ctx.signal(AIM_MODE)
    zone = ctx.signal(ZONE)
    achievable = ctx.signal(ACHIEVABLE)
    blocking = ctx.signal(BLOCKING)
    distance = ctx.signal(DISTANCE)
    mot_tgt = ctx.signal(MOTIVATOR_TARGET)
    mot_act = ctx.signal(MOTIVATOR_ACTUAL)
    spin = ctx.signal(SPINDEXER_STATE)
    l_ready = ctx.signal(LAUNCHER_READY)
    t_cur = ctx.signal(TURRET_CURRENT)
    t_tgt = ctx.signal(TURRET_TARGET)
    ball_count = ctx.signal(BALL_IMPACT_COUNT)

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

    events: list[dict] = []
    counts: dict[str, int] = {}

    for s, e in firing:
        balls_before = int(ball_count.value_at(s - 1, 0) or 0)
        balls_after = int(ball_count.value_at(e, balls_before) or balls_before)
        if balls_after > balls_before:
            continue  # had balls, not a zero-ball interval
        dur = (e - s) / 1e6

        # Sample every 50ms
        samples = max(4, int(dur / 0.05))
        achievable_falses = 0
        turret_off = 0
        feeding = 0
        launcher_ready_cnt = 0
        motivator_peak = 0.0
        motivator_target_went_zero = False
        motivator_target_was_nonzero = False

        for i in range(samples):
            frac = i / (samples - 1) if samples > 1 else 0.5
            t_us = s + int((e - s) * frac)
            if not bool(achievable.value_at(t_us, True)):
                achievable_falses += 1
            tc = float(t_cur.value_at(t_us, 0.0) or 0.0)
            tt = float(t_tgt.value_at(t_us, 0.0) or 0.0)
            if abs(tc - tt) > TURRET_OFF_AIM_DEG:
                turret_off += 1
            if spin.value_at(t_us, "") == "FEEDING":
                feeding += 1
            if bool(l_ready.value_at(t_us, False)):
                launcher_ready_cnt += 1
            ma = float(mot_act.value_at(t_us, 0.0) or 0.0)
            motivator_peak = max(motivator_peak, abs(ma))
            mt = float(mot_tgt.value_at(t_us, 0.0) or 0.0)
            if mt > 10:
                motivator_target_was_nonzero = True
            if motivator_target_was_nonzero and mt <= 1:
                motivator_target_went_zero = True

        achievable_frac = achievable_falses / samples
        turret_off_frac = turret_off / samples
        feeding_frac = feeding / samples
        launcher_ready_frac = launcher_ready_cnt / samples

        # Classify
        if dur < MIN_DUR_S:
            cause = "too_short"
        elif motivator_target_went_zero:
            cause = "zone_exit"
        elif achievable_frac > HIGH_FRAC:
            cause = "unachievable"
        elif turret_off_frac > HIGH_FRAC:
            cause = "turret_not_aimed"
        elif feeding_frac == 0:
            cause = "spindexer_never_fed"
        elif motivator_peak < MIN_MOTIVATOR_RPM:
            cause = "motivator_never_spun"
        elif launcher_ready_frac == 0:
            cause = "launcher_never_ready"
        else:
            cause = "other"

        counts[cause] = counts.get(cause, 0) + 1

        # Freshest blocking message inside the interval
        blocking_msg = ""
        for ts, v in blocking.iter_between(s, e):
            blocking_msg = str(v)  # keep the last one
        events.append({
            "start_s": ctx.rel_s(s),
            "end_s": ctx.rel_s(e),
            "duration_s": round(dur, 3),
            "aim_mode": aim.value_at(s, "") or aim.value_at(e - 1, "") or "NONE",
            "zone_at_start": zone.value_at(s, "") or "",
            "distance_m_at_start": round(float(distance.value_at(s, 0.0) or 0.0), 2),
            "cause": cause,
            "achievable_frac_false": round(achievable_frac, 3),
            "turret_off_aim_frac": round(turret_off_frac, 3),
            "feeding_frac": round(feeding_frac, 3),
            "launcher_ready_frac": round(launcher_ready_frac, 3),
            "motivator_peak_rpm": round(motivator_peak, 0),
            "blocking_sample": blocking_msg[:80],
        })

    summary = {
        "zero_ball_intervals": len(events),
    }
    for c in ("too_short", "zone_exit", "unachievable", "turret_not_aimed",
              "spindexer_never_fed", "motivator_never_spun", "launcher_never_ready", "other"):
        summary[f"cause_{c}"] = counts.get(c, 0)
    return AnalyzerResult(id="zero_ball_firing", title="Zero-ball firing intervals", events=events, summary=summary)
