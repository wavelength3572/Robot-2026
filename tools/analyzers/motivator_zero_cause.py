"""Classify every time `SmartLaunch/Target/MotivatorRPM` drops to zero.

During any firing attempt (CoordinatorState ∈ {AIMING, FIRING, SETTLING}) we
watch the commanded motivator RPM. Each falling edge > 0 → 0 is labeled with
one of:
    operator_release   — coordinator exited to INACTIVE/UNARMED/HELD
    turret_flipping    — TurretState was FLIPPING at the moment of drop
    speed_gate         — StateTransition message mentioned 'too fast'/'speed'
    not_achievable     — Target/Achievable was false
    spindexer_suppressed — SpindexerState == SUPPRESSED
    unknown            — none of the above matched

Event output:
    {t_s, cause, coord_before, coord_after, transition_reason,
     turret_state, spindexer_state, achievable}
"""

from __future__ import annotations

from log_context import AnalyzerResult, LogContext, register_analyzer

COORDINATOR_STATE = "/RealOutputs/SmartLaunch/CoordinatorState"
STATE_TRANSITION = "/RealOutputs/SmartLaunch/StateTransition"
BLOCKING = "/RealOutputs/SmartLaunch/Blocking"
# `/Motivator/TargetRPM` is the actual motor-command-level setpoint (written
# every time the Motivator subsystem's target changes). The SmartLaunch
# coordinator's `Target/MotivatorRPM` is only logged sparsely.
MOTIVATOR_TARGET = "/Motivator/TargetRPM"
ACHIEVABLE = "/RealOutputs/SmartLaunch/Target/Achievable"
TURRET_STATE = "/RealOutputs/Subsystems/TurretState"
SPINDEXER_STATE = "/RealOutputs/Subsystems/SpindexerState"

FIRING_ATTEMPT_STATES = {"AIMING", "FIRING", "SETTLING", "HELD"}
TRANSITION_WINDOW_US = 1_000_000  # 1 second


@register_analyzer(id="motivator_zero_cause", title="Motivator → 0 causes")
def analyze(ctx: LogContext) -> AnalyzerResult:
    coord = ctx.signal(COORDINATOR_STATE)
    transition = ctx.signal(STATE_TRANSITION)
    blocking = ctx.signal(BLOCKING)
    motivator_target = ctx.signal(MOTIVATOR_TARGET)
    achievable = ctx.signal(ACHIEVABLE)
    turret = ctx.signal(TURRET_STATE)
    spindexer = ctx.signal(SPINDEXER_STATE)

    events: list[dict] = []
    counts: dict[str, int] = {}

    prev_val = 0.0
    for ts, v in zip(motivator_target.timestamps_us, motivator_target.values):
        val = float(v) if v is not None else 0.0
        if prev_val > 1e-3 and val <= 1e-3:
            current_coord = coord.value_at(ts, "INACTIVE")
            coord_before = coord.value_at(ts - 1, "INACTIVE")
            if current_coord in FIRING_ATTEMPT_STATES or coord_before in FIRING_ATTEMPT_STATES:
                cause, reason = _classify(ts, coord, transition, blocking, turret, spindexer, achievable)
                counts[cause] = counts.get(cause, 0) + 1
                events.append(
                    {
                        "t_s": ctx.rel_s(ts),
                        "cause": cause,
                        "coord_before": coord_before,
                        "coord_after": current_coord,
                        "transition_reason": reason or "",
                        "turret_state": turret.value_at(ts, ""),
                        "spindexer_state": spindexer.value_at(ts, ""),
                        "achievable": bool(achievable.value_at(ts, False)),
                    }
                )
        prev_val = val

    summary = {"total_drops": len(events)}
    # Make sure every known category is present (so the dashboard shows 0s).
    for k in ("operator_release", "turret_flipping", "speed_gate",
              "not_achievable", "spindexer_suppressed", "spindexer_stopped",
              "no_fire_zone", "settling", "unknown"):
        summary[f"cause_{k}"] = counts.get(k, 0)
    return AnalyzerResult(id="motivator_zero_cause", title="Motivator → 0 causes", events=events, summary=summary)


def _classify(ts, coord, transition, blocking, turret, spindexer, achievable) -> tuple[str, str]:
    recent_reason = _recent_transition(ts, transition, window_us=TRANSITION_WINDOW_US)
    recent_block = _recent_transition(ts, blocking, window_us=TRANSITION_WINDOW_US)

    coord_after = coord.value_at(ts, "")
    coord_before = coord.value_at(ts - 1, "")

    # Operator released the fire button: coord exited firing-attempt states.
    if coord_before in FIRING_ATTEMPT_STATES and coord_after in {"INACTIVE", "UNARMED"}:
        return "operator_release", recent_reason

    # Turret flip gate (either FLIPPING now or SETTLING w/ turret_flip reason).
    if turret.value_at(ts, "") == "FLIPPING":
        return "turret_flipping", recent_reason
    if recent_reason and "turret_flip" in recent_reason.lower():
        return "turret_flipping", recent_reason

    # Speed / zone gate per coordinator transition messages.
    if recent_reason:
        lower = recent_reason.lower()
        if "too fast" in lower or "speed" in lower:
            return "speed_gate", recent_reason
        if "no_fire_zone" in lower or "neutral_trench" in lower or "bump" in lower:
            return "no_fire_zone", recent_reason
        if "settling" in lower or coord_after == "SETTLING":
            return "settling", recent_reason

    if not bool(achievable.value_at(ts, True)):
        return "not_achievable", recent_reason or recent_block

    # Spindexer variants:
    spin = spindexer.value_at(ts, "")
    if spin == "SUPPRESSED":
        return "spindexer_suppressed", recent_reason
    if spin in {"STOPPED", "RECIPROCATING"}:
        # Motivator is gated on the spindexer's feeding state in ShootingCommands;
        # when the spindexer isn't feeding, the motivator target is zeroed even
        # though the coordinator is still "FIRING".
        return "spindexer_stopped", recent_reason or recent_block

    return "unknown", recent_reason or recent_block


def _recent_transition(ts: int, sig, window_us: int) -> str:
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


def _recent_transition(ts: int, transition, window_us: int) -> str:
    if not transition.timestamps_us:
        return ""
    # Find the last transition <= ts within window_us.
    for i in range(len(transition.timestamps_us) - 1, -1, -1):
        tts = transition.timestamps_us[i]
        if tts > ts:
            continue
        if ts - tts > window_us:
            return ""
        return str(transition.values[i])
    return ""
