"""Match-level headline numbers + compact sampled series for the dashboard.

This analyzer doesn't emit "events" — it writes the per-match context the UI
needs: duration, shot counts, and downsampled signal traces for the timeline.
Downsampling to 10 ms (~100 Hz) keeps match JSON under a few MB.
"""

from __future__ import annotations

import statistics

from log_context import AnalyzerResult, LogContext, register_analyzer

TOTAL_SHOTS = "/RealOutputs/ShotLog/TotalShots"
AUTO_SHOTS = "/RealOutputs/ShotLog/AutoShots"
TELEOP_SHOTS = "/RealOutputs/ShotLog/TeleopShots"
DISTANCE = "/RealOutputs/SmartLaunch/Distance/RawDistanceM"

SERIES_STEP_US = 20_000  # 50 Hz — same as robot periodic loop

SAMPLED_SIGNALS: list[tuple[str, str]] = [
    ("launcher_actual_rpm", "/Launcher/LeaderVelocityRPM"),
    ("launcher_target_rpm", "/Launcher/TargetVelocityRPM"),
    ("motivator_actual_rpm", "/Motivator/WheelRPM"),
    ("motivator_target_rpm", "/Motivator/TargetRPM"),
    ("turret_angle_deg", "/Turret/CurrentInsideAngleDeg"),
    ("turret_target_deg", "/Turret/TargetInsideAngleDeg"),
    ("coord_state", "/RealOutputs/SmartLaunch/CoordinatorState"),
    ("turret_state", "/RealOutputs/Subsystems/TurretState"),
    ("spindexer_state", "/RealOutputs/Subsystems/SpindexerState"),
    ("launcher_state", "/RealOutputs/Subsystems/LauncherState"),
    ("motivator_state", "/RealOutputs/Subsystems/MotivatorState"),
    ("ball_impact_count", "/RealOutputs/Launcher/BallImpact/Count"),
    ("ball_impact_dip_rpm", "/RealOutputs/Launcher/BallImpact/DipRPM"),
    ("launcher_at_setpoint", "/Launcher/AtSetpoint"),
    ("motivator_at_setpoint", "/Motivator/AtSetpoint"),
    ("target_achievable", "/RealOutputs/SmartLaunch/Target/Achievable"),
    ("distance_m", "/RealOutputs/SmartLaunch/Distance/RawDistanceM"),
    ("dist_vel_comp_m", "/RealOutputs/SmartLaunch/Distance/VelocityCompensatedDistanceM"),
    ("state_transition", "/RealOutputs/SmartLaunch/StateTransition"),
]


@register_analyzer(id="match_summary", title="Match summary")
def analyze(ctx: LogContext) -> AnalyzerResult:
    summary = {
        "duration_s": round(ctx.duration_s(), 2),
        "record_count": ctx.record_count,
    }

    for key, path in [("total_shots", TOTAL_SHOTS), ("auto_shots", AUTO_SHOTS), ("teleop_shots", TELEOP_SHOTS)]:
        s = ctx.signal(path)
        if s.timestamps_us:
            summary[key] = int(s.values[-1])
        else:
            summary[key] = 0

    dist = ctx.signal(DISTANCE)
    if dist.values:
        vals = [v for v in dist.values if isinstance(v, (int, float))]
        if vals:
            summary["median_distance_m"] = round(statistics.median(vals), 2)
            summary["max_distance_m"] = round(max(vals), 2)

    series = {}
    for key, path in SAMPLED_SIGNALS:
        s = ctx.signal(path)
        if not s.timestamps_us:
            continue
        ts_us, vals = s.sampled(SERIES_STEP_US)
        series[key] = {
            "t_s": [round((t - ctx.t0_us) / 1e6, 3) for t in ts_us],
            "v": vals,
            "dtype": s.dtype,
        }

    return AnalyzerResult(id="match_summary", title="Match summary", summary=summary, series=series)
