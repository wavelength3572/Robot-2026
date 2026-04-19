"""Slug-based firing analysis.

A "slug" is a cluster of ball impacts with short gaps between them — a burst
of actual shooting. We cluster consecutive `Launcher/BallImpact/Count`
increments with gap <= `SLUG_GAP_MS`; each cluster is one slug.

This is the honest way to measure what the mechanism does when it's *actually
firing*: a 12 BPS target implies 83 ms between impacts. Anything longer is
either mechanism-limited or something is blocking the feed.

Summary (per match):
    slug_count                — number of bursts detected
    total_balls               — balls in any slug
    mean_slug_bps             — average of per-slug BPS
    best_slug_bps             — fastest sustained 2+ ball slug
    p10_gap_ms, p50_gap_ms    — inter-ball spacing percentiles (within slugs)
    gap_to_12bps_ms           — p10 minus 83ms (how far the fastest we observed
                                is from the 12 BPS target; negative = faster!)

Per-slug events:
    {start_s, end_s, duration_s, balls, bps, mean_gap_ms, min_gap_ms,
     during_firing, firing_state_at_start}
"""

from __future__ import annotations

import statistics

from log_context import AnalyzerResult, LogContext, register_analyzer

BALL_IMPACT_COUNT = "/RealOutputs/Launcher/BallImpact/Count"
BALL_TIME_SINCE_LAST_MS = "/RealOutputs/Launcher/BallImpact/TimeSinceLastMs"
COORDINATOR_STATE = "/RealOutputs/SmartLaunch/CoordinatorState"

SLUG_GAP_MS = 300  # gap between impacts > this starts a new slug
TARGET_BPS = 12.0
TARGET_GAP_MS = 1000.0 / TARGET_BPS  # 83.33 ms


@register_analyzer(id="slugs", title="Slugs (ball bursts)")
def analyze(ctx: LogContext) -> AnalyzerResult:
    count = ctx.signal(BALL_IMPACT_COUNT)
    time_since = ctx.signal(BALL_TIME_SINCE_LAST_MS)
    coord = ctx.signal(COORDINATOR_STATE)

    # Extract the timestamp of each ball impact by watching the monotonic
    # counter increment.
    impact_ts_us: list[int] = []
    prev = 0
    for ts, v in zip(count.timestamps_us, count.values):
        c = int(v or 0)
        if c > prev:
            # The counter can jump by >1 if we missed a sample; treat any
            # increment as "one impact at this timestamp" — the robot's own
            # BallImpact logic guarantees monotonic, so jumps are just timing.
            for _ in range(c - prev):
                impact_ts_us.append(ts)
            prev = c

    # Cluster into slugs.
    slugs: list[list[int]] = []
    current: list[int] = []
    for t in impact_ts_us:
        if current and (t - current[-1]) / 1000 > SLUG_GAP_MS:
            slugs.append(current)
            current = []
        current.append(t)
    if current:
        slugs.append(current)

    events: list[dict] = []
    all_gaps_ms: list[float] = []

    for slug in slugs:
        if len(slug) < 1:
            continue
        start = slug[0]
        end = slug[-1]
        dur = max((end - start) / 1e6, 0.001)
        balls = len(slug)
        # BPS of a slug is (balls - 1) gaps over duration, but for a 1-ball
        # "slug" we fall back to 1/dur which is unbounded — skip those for
        # summary stats.
        if balls >= 2:
            gaps_ms = [(slug[i + 1] - slug[i]) / 1000 for i in range(len(slug) - 1)]
            all_gaps_ms.extend(gaps_ms)
            bps = (balls - 1) / dur
            mean_gap = statistics.mean(gaps_ms)
            min_gap = min(gaps_ms)
        else:
            bps = 0.0
            mean_gap = 0.0
            min_gap = 0.0

        events.append(
            {
                "start_s": ctx.rel_s(start),
                "end_s": ctx.rel_s(end),
                "duration_s": round(dur, 3),
                "balls": balls,
                "bps": round(bps, 3),
                "mean_gap_ms": round(mean_gap, 1),
                "min_gap_ms": round(min_gap, 1),
                "firing_state_at_start": coord.value_at(start, ""),
            }
        )

    multi = [e for e in events if e["balls"] >= 2]

    summary = {
        "slug_count": len(events),
        "multi_ball_slug_count": len(multi),
        "total_balls": sum(e["balls"] for e in events),
        "singleton_slugs": sum(1 for e in events if e["balls"] == 1),
    }
    if multi:
        summary["mean_slug_bps"] = round(statistics.mean(e["bps"] for e in multi), 3)
        summary["median_slug_bps"] = round(statistics.median(e["bps"] for e in multi), 3)
        summary["best_slug_bps"] = round(max(e["bps"] for e in multi), 3)
        summary["largest_slug_balls"] = max(e["balls"] for e in multi)
    if all_gaps_ms:
        summary["p10_gap_ms"] = round(_pct(all_gaps_ms, 10), 1)
        summary["p50_gap_ms"] = round(_pct(all_gaps_ms, 50), 1)
        summary["min_gap_ms"] = round(min(all_gaps_ms), 1)
        # "Gap to 12 BPS" — positive = slower than target, negative = faster.
        summary["gap_to_12bps_ms_p10"] = round(_pct(all_gaps_ms, 10) - TARGET_GAP_MS, 1)
        summary["bps_at_p10_gap"] = round(1000.0 / _pct(all_gaps_ms, 10), 2)

    return AnalyzerResult(id="slugs", title="Slugs (ball bursts)", events=events, summary=summary)


def _pct(xs: list[float], p: float) -> float:
    if not xs:
        return 0.0
    s = sorted(xs)
    k = (len(s) - 1) * (p / 100.0)
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    frac = k - lo
    return s[lo] * (1 - frac) + s[hi] * frac
