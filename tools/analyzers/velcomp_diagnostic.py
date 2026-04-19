"""Velocity-compensation diagnostic for PASS / LONG_PASS intervals.

For each firing interval in PASS / LONG_PASS mode, samples:
    /RealOutputs/SmartLaunch/VelocityComp/DivergenceDeg
    /RealOutputs/SmartLaunch/VelocityComp/ContractionRate
    /RealOutputs/SmartLaunch/VelocityComp/PassContractionRate
    /RealOutputs/SmartLaunch/Distance/TOF
    /RealOutputs/SmartLaunch/Distance/VelocityCompensatedDistanceM
    /RealOutputs/Drive/RobotSpeedMps

and computes:
    mean / p10 / p90 divergence
    mean PassContractionRate (1.0 = solver squeezes error to zero each pass,
                              0 = no progress)
    oscillation score — coefficient of variation of divergence
                        (high = bouncing around, low = steady)
    linear slope of divergence over the interval (is it coming down?)
    mean TOF, mean robot speed

Classification (first match wins):
    not_converging   mean PassContractionRate < 0.3 — solver isn't progressing
    stuck_residual   low variance, high mean divergence — converged but to a
                     non-zero fixed residual (geometry of pass problem is
                     preventing 3 iterations from squeezing it out)
    oscillating      high coefficient of variation, mean divergence > 15°
                     — solver jumps around, unstable
    converging       mean divergence < 10° or trend strongly negative
                     — healthy
    high_tof         TOF > 1.0s — ball flight is so long that any target
                     motion swamps the lead calc
    mixed            none of the above clearly

Per-interval event includes all signal stats + the verdict.
"""

from __future__ import annotations

import statistics

from log_context import AnalyzerResult, LogContext, register_analyzer

COORDINATOR_STATE = "/RealOutputs/SmartLaunch/CoordinatorState"
AIM_MODE = "/RealOutputs/SmartLaunch/Status/AimMode"
DIVERGENCE = "/RealOutputs/SmartLaunch/VelocityComp/DivergenceDeg"
CONTRACTION = "/RealOutputs/SmartLaunch/VelocityComp/ContractionRate"
PASS_CONTRACTION = "/RealOutputs/SmartLaunch/VelocityComp/PassContractionRate"
TOF = "/RealOutputs/SmartLaunch/Distance/TOF"
DIST_VC = "/RealOutputs/SmartLaunch/Distance/VelocityCompensatedDistanceM"
ROBOT_SPEED = "/RealOutputs/Drive/RobotSpeedMps"
BALL_COUNT = "/RealOutputs/Launcher/BallImpact/Count"

SAMPLE_STEP_US = 50_000
PASS_MODES = {"PASS", "LONG_PASS"}

LOW_CONTR_THRESHOLD = 0.3
OSCILLATING_CV = 0.3
HIGH_DIV_THRESHOLD = 15.0
CONVERGED_DIV = 10.0
HIGH_TOF = 1.0


@register_analyzer(id="velcomp_diagnostic", title="Velcomp diagnostic (PASS)")
def analyze(ctx: LogContext) -> AnalyzerResult:
    coord = ctx.signal(COORDINATOR_STATE)
    aim = ctx.signal(AIM_MODE)
    diverg = ctx.signal(DIVERGENCE)
    contr = ctx.signal(CONTRACTION)
    pass_contr = ctx.signal(PASS_CONTRACTION)
    tof = ctx.signal(TOF)
    dist = ctx.signal(DIST_VC)
    speed = ctx.signal(ROBOT_SPEED)
    balls = ctx.signal(BALL_COUNT)

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

    events = []
    counts: dict[str, int] = {}

    for s, e in firing:
        mode = aim.value_at(s, "") or aim.value_at(e - 1, "") or ""
        if mode not in PASS_MODES:
            continue

        dur = (e - s) / 1e6
        n = max(4, int((e - s) / SAMPLE_STEP_US))
        divs = []
        contrs = []
        pass_contrs = []
        tofs = []
        dists = []
        speeds = []
        for i in range(n):
            frac = i / (n - 1) if n > 1 else 0.5
            t = s + int((e - s) * frac)
            d = diverg.value_at(t, None)
            if d is not None:
                divs.append(float(d))
            c = pass_contr.value_at(t, None)
            if c is not None:
                pass_contrs.append(float(c))
            cg = contr.value_at(t, None)
            if cg is not None:
                contrs.append(float(cg))
            tf = tof.value_at(t, None)
            if tf is not None:
                tofs.append(float(tf))
            dv = dist.value_at(t, None)
            if dv is not None:
                dists.append(float(dv))
            sp = speed.value_at(t, None)
            if sp is not None:
                speeds.append(float(sp))

        if not divs:
            continue

        mean_div = statistics.mean(divs)
        stdev_div = statistics.stdev(divs) if len(divs) > 1 else 0
        cv = (stdev_div / mean_div) if mean_div > 0.1 else 0
        p10_div = _pct(divs, 10)
        p50_div = _pct(divs, 50)
        p90_div = _pct(divs, 90)
        max_div = max(divs)
        # linear slope (per second) — are we trending down?
        if len(divs) >= 4:
            xs = list(range(len(divs)))
            n_ = len(divs)
            mean_x = (n_ - 1) / 2
            sxy = sum((x - mean_x) * (y - mean_div) for x, y in zip(xs, divs))
            sxx = sum((x - mean_x) ** 2 for x in xs)
            slope_per_sample = sxy / sxx if sxx > 0 else 0
            slope_per_sec = slope_per_sample * (n_ / dur) if dur > 0 else 0
        else:
            slope_per_sec = 0.0

        mean_pass_contr = statistics.mean(pass_contrs) if pass_contrs else 0.0
        mean_contr = statistics.mean(contrs) if contrs else 0.0
        mean_tof = statistics.mean(tofs) if tofs else 0.0
        mean_dist = statistics.mean(dists) if dists else 0.0
        mean_speed = statistics.mean(speeds) if speeds else 0.0

        balls_before = int(balls.value_at(s - 1, 0) or 0)
        balls_after = int(balls.value_at(e, balls_before) or balls_before)
        balls_fired = max(0, balls_after - balls_before)

        # Classify
        if mean_tof > HIGH_TOF:
            verdict = "high_tof"
        elif mean_pass_contr < LOW_CONTR_THRESHOLD and mean_div > 5:
            verdict = "not_converging"
        elif mean_div > HIGH_DIV_THRESHOLD and cv < 0.15:
            verdict = "stuck_residual"
        elif mean_div > HIGH_DIV_THRESHOLD and cv > OSCILLATING_CV:
            verdict = "oscillating"
        elif mean_div < CONVERGED_DIV or slope_per_sec < -5:
            verdict = "converging"
        else:
            verdict = "mixed"
        counts[verdict] = counts.get(verdict, 0) + 1

        events.append({
            "start_s": ctx.rel_s(s),
            "end_s": ctx.rel_s(e),
            "duration_s": round(dur, 3),
            "aim_mode": mode,
            "balls_fired": balls_fired,
            "mean_divergence_deg": round(mean_div, 2),
            "p10_divergence_deg": round(p10_div, 2),
            "p50_divergence_deg": round(p50_div, 2),
            "p90_divergence_deg": round(p90_div, 2),
            "max_divergence_deg": round(max_div, 2),
            "divergence_cv": round(cv, 3),
            "divergence_slope_deg_per_s": round(slope_per_sec, 2),
            "mean_pass_contraction": round(mean_pass_contr, 3),
            "mean_contraction": round(mean_contr, 3),
            "mean_tof_s": round(mean_tof, 3),
            "mean_distance_m": round(mean_dist, 2),
            "mean_speed_mps": round(mean_speed, 2),
            "verdict": verdict,
        })

    summary = {
        "pass_intervals_examined": len(events),
    }
    for v in ("high_tof", "not_converging", "stuck_residual", "oscillating",
              "converging", "mixed"):
        summary[f"count_{v}"] = counts.get(v, 0)
    if events:
        summary["mean_divergence_deg_overall"] = round(
            statistics.mean(e["mean_divergence_deg"] for e in events), 2
        )
        summary["mean_pass_contraction_overall"] = round(
            statistics.mean(e["mean_pass_contraction"] for e in events), 3
        )
        summary["fired_intervals"] = sum(1 for e in events if e["balls_fired"] > 0)
        summary["zero_ball_intervals"] = sum(1 for e in events if e["balls_fired"] == 0)
    return AnalyzerResult(id="velcomp_diagnostic", title="Velcomp diagnostic (PASS)",
                          events=events, summary=summary)


def _pct(xs: list[float], p: float) -> float:
    if not xs:
        return 0.0
    s = sorted(xs)
    k = (len(s) - 1) * (p / 100.0)
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    f = k - lo
    return s[lo] * (1 - f) + s[hi] * f
