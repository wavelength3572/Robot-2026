"""Classify every time `/Motivator/TargetRPM` drops to zero during firing.

Per `ShootingCommands.java:773` the motivator is commanded every loop while
the coordinator has a valid shot. It only gets zeroed while *attempting to
fire* under three conditions (from the Java code):

1. `ShootingCommands.java:569` — turret is `FLIPPING` or `STALLED`
2. `ShootingCommands.java:749` — auto-collect idle phase
3. `ShootingCommands.java:775` — currentShot == null (unachievable, zone, etc.)

We classify each falling edge accordingly. The operator releasing the button
is *not* captured here — it exits `FIRING` entirely, so it's already handled
by the `firing_intervals` analyzer.

Event output:
    {t_s, cause, coord_state, turret_state, transition_reason, achievable}
"""

from __future__ import annotations

from log_context import AnalyzerResult, LogContext, register_analyzer

COORDINATOR_STATE = "/RealOutputs/SmartLaunch/CoordinatorState"
STATE_TRANSITION = "/RealOutputs/SmartLaunch/StateTransition"
BLOCKING = "/RealOutputs/SmartLaunch/Blocking"
MOTIVATOR_TARGET = "/Motivator/TargetRPM"
ACHIEVABLE = "/RealOutputs/SmartLaunch/Target/Achievable"
TURRET_STATE = "/RealOutputs/Subsystems/TurretState"

# Any firing-attempt state. If the motivator drops to 0 while the coordinator
# is *not* in one of these, we just ignore it — that's a normal end-of-shot.
FIRING_ATTEMPT = {"AIMING", "FIRING", "SETTLING", "HELD"}

TRANSITION_WINDOW_US = 1_000_000
CAUSES = (
    "turret_flipping_or_stalled",
    "unachievable",
    "zone_change",
    "coord_left_firing",
    "unknown",
)


@register_analyzer(id="motivator_zero_cause", title="Motivator → 0 causes")
def analyze(ctx: LogContext) -> AnalyzerResult:
    coord = ctx.signal(COORDINATOR_STATE)
    transition = ctx.signal(STATE_TRANSITION)
    blocking = ctx.signal(BLOCKING)
    motivator_target = ctx.signal(MOTIVATOR_TARGET)
    achievable = ctx.signal(ACHIEVABLE)
    turret = ctx.signal(TURRET_STATE)

    events: list[dict] = []
    counts: dict[str, int] = {}

    prev_val = 0.0
    for ts, v in zip(motivator_target.timestamps_us, motivator_target.values):
        val = float(v) if v is not None else 0.0
        if prev_val > 1e-3 and val <= 1e-3:
            coord_before = coord.value_at(ts - 1, "INACTIVE")
            # We only care about drops that happened while *attempting to fire*.
            if coord_before not in FIRING_ATTEMPT:
                prev_val = val
                continue
            cause, reason = _classify(ts, coord, transition, blocking, turret, achievable, coord_before)
            counts[cause] = counts.get(cause, 0) + 1
            events.append(
                {
                    "t_s": ctx.rel_s(ts),
                    "cause": cause,
                    "coord_before": coord_before,
                    "coord_after": coord.value_at(ts, ""),
                    "transition_reason": reason,
                    "turret_state": turret.value_at(ts, ""),
                    "achievable": bool(achievable.value_at(ts, False)),
                }
            )
        prev_val = val

    summary = {"total_drops": len(events)}
    for c in CAUSES:
        summary[f"cause_{c}"] = counts.get(c, 0)
    return AnalyzerResult(id="motivator_zero_cause", title="Motivator → 0 causes", events=events, summary=summary)


def _classify(ts, coord, transition, blocking, turret, achievable, coord_before):
    recent = _recent(ts, transition, TRANSITION_WINDOW_US)
    coord_after = coord.value_at(ts, "")

    # Cause 1: turret flipping / stalled (ShootingCommands.java:569)
    tstate = turret.value_at(ts, "")
    if tstate in {"FLIPPING", "STALLED"}:
        return "turret_flipping_or_stalled", recent
    if recent and ("turret_flip" in recent.lower()):
        return "turret_flipping_or_stalled", recent

    # Cause 2: the coordinator left any firing-attempt state between
    # coord_before and now (coord cleared the shot → motivator zeroed via s==null)
    if coord_after not in FIRING_ATTEMPT:
        if coord_after == "NO_FIRE_ZONE" or (recent and "no_fire_zone" in recent.lower()):
            return "zone_change", recent
        return "coord_left_firing", recent

    # Cause 3: shot unachievable (currentShot becomes null)
    if not bool(achievable.value_at(ts, True)):
        return "unachievable", recent

    return "unknown", recent or _recent(ts, blocking, TRANSITION_WINDOW_US)


def _recent(ts, sig, window_us) -> str:
    if not sig.timestamps_us:
        return ""
    for i in range(len(sig.timestamps_us) - 1, -1, -1):
        tts = sig.timestamps_us[i]
        if tts > ts:
            continue
        if ts - tts > window_us:
            return ""
        return str(sig.values[i])
    return ""
