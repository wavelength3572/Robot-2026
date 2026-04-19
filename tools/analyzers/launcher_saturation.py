"""Launcher saturation / voltage health during firing.

Answers: during each firing interval, was the launcher controller ever
voltage-limited (applied output pegged at 1.0, or bus voltage too low to
command more)? Or was it happily tracking with headroom to spare?

Signals examined:
    /Launcher/LeaderAppliedVolts   — commanded voltage to leader
    /Launcher/LeaderAppliedOutput  — % of bus voltage (-1.0..1.0). This is
                                     the true saturation flag: if it pegs
                                     at 1.0, the controller wanted more but
                                     couldn't ask for more.
    /Launcher/LeaderBusVoltage     — instantaneous battery voltage at the
                                     motor. Sags under current draw.
    /Launcher/LeaderCurrentAmps    — stator current.
    /Launcher/WheelVelocityRPM vs /Launcher/TargetVelocityRPM — wheel-side
                                     tracking error (apples-to-apples).

Per-interval event:
    {start_s, end_s, duration_s, aim_mode,
     mean_applied_volts, max_applied_volts,
     mean_applied_output, max_applied_output,
     min_bus_voltage, mean_bus_voltage,
     max_current_a, mean_current_a,
     mean_rpm_error_pct, max_rpm_error_pct,  (wheel-side, signed)
     saturated_fraction,                     (% of samples where |output| > 0.95)
     low_bus_fraction,                       (% of samples where bus < 10V)
     verdict}                                (one of:
                                              "tracking"           — |err| < 3%, not saturated
                                              "controller_slack"   — |err| > 5% but output < 0.8 (not trying hard)
                                              "voltage_saturated"  — output pegged near 1.0
                                              "low_bus_brownout"   — bus < 9V for meaningful duration
                                              "mixed")

Summary (per match):
    firing_intervals_examined
    fleet_mean_rpm_error_pct
    intervals_tracking, intervals_slack, intervals_saturated, intervals_brownout
    fleet_max_current_a
    fleet_min_bus_voltage
"""

from __future__ import annotations

from log_context import AnalyzerResult, LogContext, register_analyzer

COORDINATOR_STATE = "/RealOutputs/SmartLaunch/CoordinatorState"
AIM_MODE = "/RealOutputs/SmartLaunch/Status/AimMode"
APPLIED_VOLTS = "/Launcher/LeaderAppliedVolts"
APPLIED_OUTPUT = "/Launcher/LeaderAppliedOutput"
BUS_VOLTAGE = "/Launcher/LeaderBusVoltage"
CURRENT_AMPS = "/Launcher/LeaderCurrentAmps"
WHEEL_RPM = "/Launcher/WheelVelocityRPM"
WHEEL_TARGET_RPM = "/Launcher/TargetVelocityRPM"

SATURATION_THRESHOLD = 0.95   # |appliedOutput| > this = pegged
LOW_BUS_THRESHOLD = 9.0
SIGNIFICANT_ERR_PCT = 5.0
SLACK_OUTPUT_THRESHOLD = 0.8  # if output < 0.8 AND err > threshold → controller has headroom
LONG_BROWNOUT_FRACTION = 0.25


@register_analyzer(id="launcher_saturation", title="Launcher saturation")
def analyze(ctx: LogContext) -> AnalyzerResult:
    coord = ctx.signal(COORDINATOR_STATE)
    aim = ctx.signal(AIM_MODE)
    volts = ctx.signal(APPLIED_VOLTS)
    output = ctx.signal(APPLIED_OUTPUT)
    bus = ctx.signal(BUS_VOLTAGE)
    current = ctx.signal(CURRENT_AMPS)
    wheel = ctx.signal(WHEEL_RPM)
    wheel_target = ctx.signal(WHEEL_TARGET_RPM)

    # Firing intervals
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

    events = []
    counts = {"tracking": 0, "controller_slack": 0, "voltage_saturated": 0,
              "low_bus_brownout": 0, "mixed": 0}
    all_errs = []
    fleet_max_current = 0.0
    fleet_min_bus = 1e6

    for s, e in firing:
        dur = (e - s) / 1e6
        if dur < 0.05:
            continue

        # Sample uniformly every 50 ms inside the interval.
        samples = max(2, int(dur / 0.05))
        volts_vals, out_vals, bus_vals, curr_vals, err_pct_vals = [], [], [], [], []
        for i in range(samples):
            frac = i / (samples - 1) if samples > 1 else 0.5
            t_us = s + int((e - s) * frac)
            volts_vals.append(float(volts.value_at(t_us, 0.0) or 0.0))
            out_vals.append(float(output.value_at(t_us, 0.0) or 0.0))
            bus_vals.append(float(bus.value_at(t_us, 0.0) or 0.0))
            curr_vals.append(float(current.value_at(t_us, 0.0) or 0.0))
            w = float(wheel.value_at(t_us, 0.0) or 0.0)
            wt = float(wheel_target.value_at(t_us, 0.0) or 0.0)
            if wt > 100:
                err_pct_vals.append(100 * (w - wt) / wt)

        if not err_pct_vals:
            continue

        mean_out = sum(out_vals) / len(out_vals)
        max_out = max(abs(v) for v in out_vals)
        saturated_frac = sum(1 for v in out_vals if abs(v) > SATURATION_THRESHOLD) / len(out_vals)
        min_bus = min(bus_vals)
        low_bus_frac = sum(1 for v in bus_vals if v < LOW_BUS_THRESHOLD) / len(bus_vals)
        max_curr = max(curr_vals)
        mean_err = sum(err_pct_vals) / len(err_pct_vals)
        max_err = max(err_pct_vals, key=abs)

        fleet_max_current = max(fleet_max_current, max_curr)
        fleet_min_bus = min(fleet_min_bus, min_bus)
        all_errs.extend(err_pct_vals)

        # Classify.
        if abs(mean_err) < 3.0 and saturated_frac < 0.05:
            verdict = "tracking"
        elif saturated_frac >= 0.2:
            verdict = "voltage_saturated"
        elif low_bus_frac >= LONG_BROWNOUT_FRACTION:
            verdict = "low_bus_brownout"
        elif abs(mean_err) > SIGNIFICANT_ERR_PCT and max_out < SLACK_OUTPUT_THRESHOLD:
            verdict = "controller_slack"
        else:
            verdict = "mixed"
        counts[verdict] += 1

        events.append({
            "start_s": ctx.rel_s(s),
            "end_s": ctx.rel_s(e),
            "duration_s": round(dur, 3),
            "aim_mode": aim.value_at(s, "") or aim.value_at(e - 1, "") or "NONE",
            "mean_applied_volts": round(sum(volts_vals) / len(volts_vals), 2),
            "max_applied_volts": round(max(volts_vals), 2),
            "mean_applied_output": round(mean_out, 3),
            "max_applied_output": round(max_out, 3),
            "min_bus_voltage": round(min_bus, 2),
            "mean_bus_voltage": round(sum(bus_vals) / len(bus_vals), 2),
            "max_current_a": round(max_curr, 1),
            "mean_current_a": round(sum(curr_vals) / len(curr_vals), 1),
            "mean_rpm_error_pct": round(mean_err, 1),
            "max_rpm_error_pct": round(max_err, 1),
            "saturated_fraction": round(saturated_frac, 3),
            "low_bus_fraction": round(low_bus_frac, 3),
            "verdict": verdict,
        })

    summary = {
        "firing_intervals_examined": len(events),
        "intervals_tracking": counts["tracking"],
        "intervals_controller_slack": counts["controller_slack"],
        "intervals_voltage_saturated": counts["voltage_saturated"],
        "intervals_low_bus_brownout": counts["low_bus_brownout"],
        "intervals_mixed": counts["mixed"],
        "fleet_mean_rpm_error_pct": round(sum(all_errs) / len(all_errs), 2) if all_errs else 0,
        "fleet_max_current_a": round(fleet_max_current, 1),
        "fleet_min_bus_voltage": round(fleet_min_bus, 2) if fleet_min_bus < 1e6 else 0,
    }
    return AnalyzerResult(id="launcher_saturation", title="Launcher saturation", events=events, summary=summary)
