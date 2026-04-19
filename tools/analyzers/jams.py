"""Jam detection.

A jam is a window where the robot is trying to feed — coordinator in `FIRING`,
spindexer in `FEEDING` — but no ball impacts land for `JAM_NO_IMPACT_SEC`.
This is the "we're pushing but nothing is coming out" case. Mechanical stall,
ball stuck in the chute, or the motivator can't pull it through.

(We deliberately don't gate on `Launcher/AtSetpoint` or `Motivator/AtSetpoint`
anymore — those flicker false as the launcher chases a moving target during
shoot-on-the-move, which made jams look rarer than they are.)

Event output:
    {start_s, end_s, duration_s, launcher_target_rpm, motivator_target_rpm,
     spindexer_state_at_start}
"""

from __future__ import annotations

from log_context import AnalyzerResult, LogContext, register_analyzer

COORD_STATE = "/RealOutputs/SmartLaunch/CoordinatorState"
SPINDEXER_STATE = "/RealOutputs/Subsystems/SpindexerState"
MOTIVATOR_TARGET_KEY = "/Motivator/TargetRPM"
LAUNCHER_TARGET_KEY = "/Launcher/TargetVelocityRPM"
BALL_IMPACT_COUNT = "/RealOutputs/Launcher/BallImpact/Count"

JAM_NO_IMPACT_SEC = 0.5  # 0.5s without a ball = probably jammed
MIN_MOTIVATOR_RPM = 50.0


@register_analyzer(id="jams", title="Jams")
def analyze(ctx: LogContext) -> AnalyzerResult:
    coord = ctx.signal(COORD_STATE)
    spindexer = ctx.signal(SPINDEXER_STATE)
    motivator_target = ctx.signal(MOTIVATOR_TARGET_KEY)
    launcher_target = ctx.signal(LAUNCHER_TARGET_KEY)
    ball_count = ctx.signal(BALL_IMPACT_COUNT)

    events: list[dict] = []
    total_jam_s = 0.0

    step_times = sorted({
        *coord.timestamps_us,
        *spindexer.timestamps_us,
        *motivator_target.timestamps_us,
        *ball_count.timestamps_us,
    })

    def trying_to_feed(t_us: int) -> bool:
        if coord.value_at(t_us) != "FIRING":
            return False
        if spindexer.value_at(t_us) != "FEEDING":
            return False
        if (motivator_target.value_at(t_us, 0.0) or 0.0) <= MIN_MOTIVATOR_RPM:
            return False
        return True

    in_window = False
    window_start = 0
    window_start_count = 0
    for t in step_times:
        trying = trying_to_feed(t)
        current = int(ball_count.value_at(t, 0) or 0)
        if trying:
            if not in_window:
                in_window = True
                window_start = t
                window_start_count = current
            else:
                if current > window_start_count:
                    # Ball fed — reset window.
                    window_start = t
                    window_start_count = current
                elif (t - window_start) / 1e6 >= JAM_NO_IMPACT_SEC:
                    dur = (t - window_start) / 1e6
                    events.append(
                        {
                            "start_s": ctx.rel_s(window_start),
                            "end_s": ctx.rel_s(t),
                            "duration_s": round(dur, 3),
                            "launcher_target_rpm": round(launcher_target.value_at(window_start, 0.0) or 0.0, 0),
                            "motivator_target_rpm": round(motivator_target.value_at(window_start, 0.0) or 0.0, 0),
                            "spindexer_state_at_start": spindexer.value_at(window_start, ""),
                        }
                    )
                    total_jam_s += dur
                    window_start = t
        else:
            in_window = False

    summary = {
        "jam_events": len(events),
        "jam_seconds_total": round(total_jam_s, 2),
        "longest_jam_s": round(max((e["duration_s"] for e in events), default=0.0), 3),
    }
    return AnalyzerResult(id="jams", title="Jams", events=events, summary=summary)
