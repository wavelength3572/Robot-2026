"""Spindexer-state events during firing.

Tracks time spent (and entry events) in each non-FEEDING `SpindexerState`
while the coordinator is in any firing-attempt state. Lets us answer "how
often did the operator hold-fire?", "how often did we unclog/auto-unclog?",
"did we ever JAM?".

Signal: `/RealOutputs/Subsystems/SpindexerState`. Values:
    STOPPED, FEEDING, SUPPRESSED, UNCLOGGING, AUTO_UNCLOGGING, JAMMED,
    RECIPROCATING.

Events (one per entry into a non-feeding state while attempting to fire):
    {start_s, end_s, duration_s, state, coord_state_at_start}
"""

from __future__ import annotations

from log_context import AnalyzerResult, LogContext, register_analyzer

SPINDEXER_STATE = "/RealOutputs/Subsystems/SpindexerState"
COORDINATOR_STATE = "/RealOutputs/SmartLaunch/CoordinatorState"

FIRING_ATTEMPT = {"AIMING", "FIRING", "SETTLING", "HELD"}
# The states we care about — FEEDING and STOPPED are "normal"; these indicate
# something interrupting feeding.
INTERESTING = {"SUPPRESSED", "UNCLOGGING", "AUTO_UNCLOGGING", "JAMMED", "RECIPROCATING"}


@register_analyzer(id="spindexer_events", title="Spindexer events during firing")
def analyze(ctx: LogContext) -> AnalyzerResult:
    state = ctx.signal(SPINDEXER_STATE)
    coord = ctx.signal(COORDINATOR_STATE)

    events: list[dict] = []
    seconds_by_state: dict[str, float] = {}
    events_by_state: dict[str, int] = {}

    current_state = None
    state_started_us = 0

    for ts, v in zip(state.timestamps_us, state.values):
        if current_state is not None and v != current_state:
            _close(events, seconds_by_state, events_by_state, current_state, state_started_us, ts, coord, ctx)
        if v != current_state:
            current_state = v
            state_started_us = ts

    # Close the final open interval.
    if current_state is not None and state.timestamps_us:
        _close(events, seconds_by_state, events_by_state, current_state, state_started_us, state.timestamps_us[-1], coord, ctx)

    # Summary counts every interesting state, even if zero, so the dashboard shows 0s.
    summary = {}
    for s in ("SUPPRESSED", "UNCLOGGING", "AUTO_UNCLOGGING", "JAMMED", "RECIPROCATING"):
        summary[f"{s.lower()}_events"] = events_by_state.get(s, 0)
        summary[f"{s.lower()}_seconds"] = round(seconds_by_state.get(s, 0.0), 2)

    # Specifically: how much firing time was spent held-fire or unclogging?
    total_interrupted = sum(seconds_by_state.get(s, 0.0) for s in INTERESTING)
    summary["total_interrupted_seconds"] = round(total_interrupted, 2)
    summary["total_interesting_events"] = sum(events_by_state.get(s, 0) for s in INTERESTING)

    return AnalyzerResult(id="spindexer_events", title="Spindexer events during firing", events=events, summary=summary)


def _close(events, seconds_by_state, events_by_state, state_name, start_us, end_us, coord, ctx):
    if state_name not in INTERESTING:
        return
    # Only count events that overlap any firing-attempt window.
    if not _overlaps_firing(start_us, end_us, coord):
        return
    dur = (end_us - start_us) / 1e6
    seconds_by_state[state_name] = seconds_by_state.get(state_name, 0.0) + dur
    events_by_state[state_name] = events_by_state.get(state_name, 0) + 1
    events.append(
        {
            "start_s": ctx.rel_s(start_us),
            "end_s": ctx.rel_s(end_us),
            "duration_s": round(dur, 3),
            "state": state_name,
            "coord_state_at_start": coord.value_at(start_us, ""),
        }
    )


def _overlaps_firing(t0_us: int, t1_us: int, coord) -> bool:
    if coord.value_at(t0_us) in FIRING_ATTEMPT or coord.value_at(t1_us) in FIRING_ATTEMPT:
        return True
    for _, v in coord.iter_between(t0_us, t1_us):
        if v in FIRING_ATTEMPT:
            return True
    return False
