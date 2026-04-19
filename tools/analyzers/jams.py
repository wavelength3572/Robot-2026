"""Jam detection.

A jam is a window inside a firing interval where we're actively trying to shoot
(launcher & motivator at setpoint, commanded motivator RPM > 0) but no ball
impacts are being detected on the flywheel. `Launcher/BallImpact/Count` is a
monotonically increasing counter — if it doesn't tick for more than
`JAM_NO_IMPACT_SEC`, we flag a jam.

Event output:
    {start_s, end_s, duration_s, launcher_target_rpm, motivator_target_rpm,
     coordinator_state, spindexer_state}
"""

from __future__ import annotations

from log_context import AnalyzerResult, LogContext, register_analyzer

COORD_STATE = "/RealOutputs/SmartLaunch/CoordinatorState"
LAUNCHER_AT_SETPOINT = "/Launcher/AtSetpoint"
MOTIVATOR_AT_SETPOINT = "/Motivator/AtSetpoint"
MOTIVATOR_TARGET_KEY = "/Motivator/TargetRPM"
LAUNCHER_TARGET_KEY = "/Launcher/TargetVelocityRPM"
BALL_IMPACT_COUNT = "/RealOutputs/Launcher/BallImpact/Count"
BALL_TIME_SINCE_LAST_MS = "/RealOutputs/Launcher/BallImpact/TimeSinceLastMs"
SPINDEXER_STATE = "/RealOutputs/Subsystems/SpindexerState"

JAM_NO_IMPACT_SEC = 0.75
MIN_MOTIVATOR_RPM = 50.0


@register_analyzer(id="jams", title="Jams")
def analyze(ctx: LogContext) -> AnalyzerResult:
    coord = ctx.signal(COORD_STATE)
    launcher_ready = ctx.signal(LAUNCHER_AT_SETPOINT)
    motivator_ready = ctx.signal(MOTIVATOR_AT_SETPOINT)
    motivator_target = ctx.signal(MOTIVATOR_TARGET_KEY)
    launcher_target = ctx.signal(LAUNCHER_TARGET_KEY)
    ball_count = ctx.signal(BALL_IMPACT_COUNT)
    spindexer = ctx.signal(SPINDEXER_STATE)

    events: list[dict] = []
    total_jam_s = 0.0

    # For each FIRING interval, scan forward: when conditions to "actively feed"
    # first become true, mark that moment as the start of a potential jam window.
    # The window ends either when ball count increments or when we stop trying.
    in_window = False
    window_start = 0
    window_start_ball_count = 0

    # Merge relevant event timestamps inside the log so we advance one step at a time.
    step_times = sorted({
        *coord.timestamps_us,
        *launcher_ready.timestamps_us,
        *motivator_ready.timestamps_us,
        *motivator_target.timestamps_us,
        *ball_count.timestamps_us,
    })

    def trying_to_feed(t_us: int) -> bool:
        if coord.value_at(t_us) != "FIRING":
            return False
        if not launcher_ready.value_at(t_us, False):
            return False
        if not motivator_ready.value_at(t_us, False):
            return False
        tgt = motivator_target.value_at(t_us, 0.0) or 0.0
        return tgt > MIN_MOTIVATOR_RPM

    for t in step_times:
        trying = trying_to_feed(t)
        current_ball_count = int(ball_count.value_at(t, 0) or 0)
        if trying:
            if not in_window:
                in_window = True
                window_start = t
                window_start_ball_count = current_ball_count
            else:
                # If a ball impact lands, reset the window.
                if current_ball_count > window_start_ball_count:
                    window_start = t
                    window_start_ball_count = current_ball_count
                elif (t - window_start) / 1e6 >= JAM_NO_IMPACT_SEC:
                    dur = (t - window_start) / 1e6
                    events.append(
                        {
                            "start_s": ctx.rel_s(window_start),
                            "end_s": ctx.rel_s(t),
                            "duration_s": round(dur, 3),
                            "launcher_target_rpm": round(launcher_target.value_at(window_start, 0.0) or 0.0, 0),
                            "motivator_target_rpm": round(motivator_target.value_at(window_start, 0.0) or 0.0, 0),
                            "coordinator_state": "FIRING",
                            "spindexer_state": spindexer.value_at(window_start, ""),
                        }
                    )
                    total_jam_s += dur
                    # Re-anchor so we don't re-emit the same event every step.
                    window_start = t
        else:
            in_window = False

    summary = {
        "jam_events": len(events),
        "jam_seconds_total": round(total_jam_s, 2),
        "longest_jam_s": round(max((e["duration_s"] for e in events), default=0.0), 3),
    }
    return AnalyzerResult(id="jams", title="Jams", events=events, summary=summary)
