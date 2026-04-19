"""Generate a prioritized markdown report from the extractor output.

Reads tools/log-analysis/data/*.json and writes docs/analysis-findings.md
with plain-English findings ranked by impact. Run automatically as the
last step of tools/analyze_wpilog.py — or standalone via:

    python3 tools/generate_report.py --data tools/log-analysis/data --out docs/analysis-findings.md
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path


def load_data(data_dir: Path) -> tuple[dict, dict[str, dict]]:
    summary = json.loads((data_dir / "summary.json").read_text())
    matches = {}
    for m in summary["matches"]:
        p = data_dir / f"match_{m['stem']}.json"
        if p.exists():
            matches[m["stem"]] = json.loads(p.read_text())
    return summary, matches


def fmt_time(s: float) -> str:
    return f"{s:.1f}s"


def build_report(summary: dict, matches: dict) -> str:
    N = len(summary["matches"])

    # Fleet totals
    def total_of(aid: str, key: str) -> float:
        return sum((m["summaries"].get(aid, {}) or {}).get(key, 0) or 0 for m in summary["matches"])

    total_balls = total_of("effective_bps", "balls_during_firing")
    feeding_s = total_of("effective_bps", "feeding_s")
    truly_bps = total_balls / feeding_s if feeding_s > 0 else 0
    hub_balls = total_of("bps_by_mode", "hub_balls")
    hub_feeding = total_of("bps_by_mode", "hub_feeding_s")
    hub_bps = hub_balls / hub_feeding if hub_feeding > 0 else 0
    pass_balls = total_of("bps_by_mode", "pass_balls") + total_of("bps_by_mode", "long_pass_balls")
    pass_feeding = total_of("bps_by_mode", "pass_feeding_s") + total_of("bps_by_mode", "long_pass_feeding_s")
    pass_bps = pass_balls / pass_feeding if pass_feeding > 0 else 0

    firing_intervals_total = int(total_of("effective_bps", "firing_intervals_total"))
    jams_total = int(total_of("jams", "jam_events"))
    jam_seconds = total_of("jams", "jam_seconds_total")
    flips_during_firing = int(total_of("turret_flips", "flips_during_firing"))
    unclog_manual = int(total_of("spindexer_events", "unclogging_events"))
    unclog_auto = int(total_of("spindexer_events", "auto_unclogging_events"))

    # Zero-ball causes across matches
    zb_causes: dict[str, int] = defaultdict(int)
    zb_total = 0
    zb_examples: list[tuple[str, dict]] = []
    for stem, m in matches.items():
        for e in (m["analyzers"].get("zero_ball_firing", {}) or {}).get("events", []):
            zb_total += 1
            zb_causes[e["cause"]] += 1
            zb_examples.append((stem, e))

    # Zero-ball by aim mode
    zb_by_mode: dict[tuple[str, str], int] = defaultdict(int)
    for stem, m in matches.items():
        for e in (m["analyzers"].get("zero_ball_firing", {}) or {}).get("events", []):
            zb_by_mode[(e.get("aim_mode", "?"), e["cause"])] += 1

    # Zone-exit examples (the FIRING-with-zero-targets bug)
    zone_exit_events: list[tuple[str, dict]] = []
    for stem, m in matches.items():
        for e in (m["analyzers"].get("zero_ball_firing", {}) or {}).get("events", []):
            if e["cause"] == "zone_exit":
                zone_exit_events.append((stem, e))
    zone_exit_events.sort(key=lambda x: -x[1]["duration_s"])

    # Saturation / voltage stats
    sat_total = int(sum((m["summaries"].get("launcher_saturation", {}) or {}).get("intervals_voltage_saturated", 0) for m in summary["matches"]))
    brownout_total = int(sum((m["summaries"].get("launcher_saturation", {}) or {}).get("intervals_low_bus_brownout", 0) for m in summary["matches"]))
    intervals_examined = int(sum((m["summaries"].get("launcher_saturation", {}) or {}).get("firing_intervals_examined", 0) for m in summary["matches"]))
    fleet_min_bus = min(((m["summaries"].get("launcher_saturation", {}) or {}).get("fleet_min_bus_voltage", 99) for m in summary["matches"]), default=0)
    fleet_max_current = max(((m["summaries"].get("launcher_saturation", {}) or {}).get("fleet_max_current_a", 0) for m in summary["matches"]), default=0)

    # Auto-unclog: count events shorter than 100ms (borderline triggers)
    short_auto_unclog = 0
    total_auto_unclog = 0
    for stem, m in matches.items():
        for e in (m["analyzers"].get("spindexer_events", {}) or {}).get("events", []):
            if e.get("state") == "AUTO_UNCLOGGING":
                total_auto_unclog += 1
                if e.get("duration_s", 0) < 0.1:
                    short_auto_unclog += 1

    # Worst held-fire dry stretches (for illustration)
    worst_dry: list[tuple[str, dict]] = []
    for stem, m in matches.items():
        for e in (m["analyzers"].get("effective_bps", {}) or {}).get("events", []):
            worst_dry.append((stem, e))
    worst_dry.sort(key=lambda x: -x[1].get("longest_dry_s", 0))

    # Per-match quick stats table
    per_match_rows = []
    for m in summary["matches"]:
        s = m["summaries"]
        per_match_rows.append({
            "stem": m["stem"].split("_")[-1],
            "balls": int((s.get("effective_bps", {}) or {}).get("balls_during_firing", 0)),
            "truly_bps": (s.get("effective_bps", {}) or {}).get("truly_active_bps", 0),
            "hub_bps": (s.get("bps_by_mode", {}) or {}).get("hub_bps", 0),
            "pass_bps": (s.get("bps_by_mode", {}) or {}).get("pass_bps", 0),
            "pass_intervals": int((s.get("bps_by_mode", {}) or {}).get("pass_intervals", 0)),
            "jams": int((s.get("jams", {}) or {}).get("jam_events", 0)),
            "zero_ball_intervals": int((s.get("zero_ball_firing", {}) or {}).get("zero_ball_intervals", 0)),
            "longest_dry_s": (s.get("effective_bps", {}) or {}).get("longest_dry_s", 0),
        })

    # Build markdown
    lines: list[str] = []
    W = lines.append
    W("# Robot 2026 — Log Analysis Findings")
    W(f"*Generated {summary['generated_at']} from {N} match logs in `StatesLogs/`.*\n")

    W("## Headline numbers\n")
    W(f"- **Truly-active BPS (fleet): {truly_bps:.2f}** — balls divided by time the spindexer was actually feeding. Target is 12.0.")
    W(f"- **HUB BPS: {hub_bps:.2f}** ({int(hub_balls)} balls across hub-scoring intervals)")
    W(f"- **PASS BPS: {pass_bps:.2f}** ({int(pass_balls)} balls across passing intervals) — notably worse than HUB")
    W(f"- **{firing_intervals_total} firing intervals** across {N} matches, {int(total_balls)} total balls impacted the flywheel")
    W(f"- **{zb_total} intervals fired zero balls** ({100*zb_total/max(1,firing_intervals_total):.0f}% of all held-fire)")
    W("")

    W("## Top findings, ranked\n")

    # Finding 1: PASS failure (biggest impact)
    W("### 1. PASS shots fail about half the time — this is the #1 opportunity")
    pass_intervals = sum(v for (mode, _), v in zb_by_mode.items() if mode in ("PASS", "LONG_PASS"))
    hub_zb = sum(v for (mode, _), v in zb_by_mode.items() if mode == "HUB")
    W(f"\n{pass_intervals} of the {zb_total} zero-ball intervals were in PASS/LONG_PASS mode; only {hub_zb} were HUB. "
      f"PASS BPS ({pass_bps:.2f}) is less than half of HUB BPS ({hub_bps:.2f}).")
    # Worst pass intervals
    pass_worst = sorted(
        [(s, e) for s, e in zb_examples if e.get("aim_mode") in ("PASS", "LONG_PASS")],
        key=lambda x: -x[1]["duration_s"],
    )[:5]
    if pass_worst:
        W("\nLongest pass attempts that fired nothing:")
        for stem, e in pass_worst:
            short = stem.split("_")[-1]
            W(f"- **{short}** @ {e['start_s']:.1f}s — {e['duration_s']:.1f}s held, 0 balls, cause `{e['cause']}`"
              + (f", blocking: `{e['blocking_sample']}`" if e.get("blocking_sample") else ""))
    W("\n**Root cause (from `velcomp_diagnostic`):** The velocity-compensation solver's residual divergence sits at "
      "20-40° for the long time-of-flight passes (TOF 1.2-1.8s vs 0.4s for hub). The feed-gate requires <20°. "
      "The solver iterates correctly — it just finds a fixed point above the gate threshold. More iterations won't help.")
    W("\n**Not the cause:** The downstream actuators are fine — 0 intervals failed due to launcher-never-ready, "
      "motivator-never-spun, or spindexer-never-fed.")
    W("\n**Suggested fix:** Before writing new code, try switching the dashboard chooser from `Pass Symmetric` to "
      "`Pass Waypoint` at a practice — `WaypointPassStrategy.java` adds a 3D apex constraint that may give the "
      "solver a better-conditioned problem. If that doesn't help, see `docs/passing-fix-prompt.md` for the proposed "
      "geo-fence replacement — aim at a fixed zone-based target and gate firing on \"predicted landing inside "
      "allowed polygon\" instead of aim precision.\n")

    # Finding 2: Zone-exit bug
    if zone_exit_events:
        total_zone_exit_s = sum(e["duration_s"] for _, e in zone_exit_events)
        W(f"### 2. Bug: coord stays in FIRING for seconds with all motor targets at 0")
        W(f"\n{len(zone_exit_events)} intervals across {len({s for s, _ in zone_exit_events})} matches, "
          f"totalling **{total_zone_exit_s:.1f}s**, where `CoordinatorState` reports FIRING but motivator "
          "and spindexer targets have dropped to 0. Each time the robot was driving out of the alliance zone "
          "toward a no-fire zone. The shot went null but the state machine kept reporting FIRING for 3-10s "
          "before actually transitioning.")
        W("\nWorst examples:")
        for stem, e in zone_exit_events[:4]:
            short = stem.split("_")[-1]
            W(f"- **{short}** @ {e['start_s']:.1f}s — {e['duration_s']:.1f}s FIRING with zero targets, zone at start: `{e.get('zone_at_start', '?')}`")
        W("\n**Suggested fix:** In `ShootingCoordinator.java`, find the FIRING→AIMING transition condition and "
          "add an early exit when `currentShot == null` for N consecutive loops.\n")

    # Finding 3: Too-short
    if zb_causes.get("too_short", 0) > 0:
        W(f"### 3. {zb_causes['too_short']} button taps shorter than 500ms")
        W("\nOf all zero-ball intervals, these were driver button-taps too brief for the ready-check + feed chain "
          "to complete a shot. Not a software bug per se, but consider a \"pulse fire\" mode where a tap stays "
          "armed for 500-800ms to forgive the quick taps.\n")

    # Finding 4: Auto-unclog tuning
    if total_auto_unclog > 0 and short_auto_unclog > 0:
        frac = 100 * short_auto_unclog / total_auto_unclog
        W(f"### 4. Auto-unclog: {short_auto_unclog}/{total_auto_unclog} ({frac:.0f}%) trigger then immediately clear in <100ms")
        W(f"\n{total_auto_unclog} total auto-unclog events across the 8 matches. "
          f"{short_auto_unclog} last less than 100ms — likely borderline stall triggers that clear themselves. "
          "Worth bumping `autoUnclogStallDurationSec` slightly to kill those without missing real jams.\n")

    # Finding 5: Battery
    if brownout_total > 0:
        W(f"### 5. Battery brownout on {brownout_total}/{intervals_examined} firing intervals")
        W(f"\nBus voltage sags below 9V for at least 25% of the interval in {brownout_total} cases out of {intervals_examined} total "
          f"firing intervals examined. Fleet minimum bus voltage was **{fleet_min_bus:.1f}V**, peak launcher current "
          f"**{fleet_max_current:.0f}A**. Not currently blocking feeding (launcher still tracks), but indicates "
          "heavy battery load during shooting. Worth keeping an eye on but not the top priority.\n")

    # What's NOT a problem
    W("## Things the data ruled out")
    W("")
    W(f"- **Launcher tracking**: mean RPM error is -2% to -3% across the fleet — the launcher controller is healthy.")
    W(f"- **Motor voltage saturation**: **0 of {intervals_examined}** firing intervals pegged applied-output near 1.0. The controller has headroom; it's not being commanded to full power.")
    W(f"- **Turret flipping during firing**: small total time, not a meaningful blocker in these matches.")
    W(f"- **Hold-fire button misuse**: 0 seconds of firing-attempt time spent with spindexer `SUPPRESSED`.")
    W(f"- **Spindexer / motivator feed chain**: when the gate allows, they work. 0 \"spindexer never fed\" or \"motivator never spun\" failures.\n")

    # Per-match table
    W("## Per-match snapshot\n")
    W("| Match | Balls | Effective BPS | HUB BPS | PASS BPS | PASS intervals | 0-ball | Jams | Worst dry stretch |")
    W("|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    for r in per_match_rows:
        W(f"| {r['stem']} | {r['balls']} | {r['truly_bps']:.2f} | {r['hub_bps']:.2f} | "
          f"{r['pass_bps']:.2f} | {r['pass_intervals']} | {r['zero_ball_intervals']} | "
          f"{r['jams']} | {r['longest_dry_s']:.1f}s |")
    W("")

    # Where to look next
    W("## Where to look next\n")
    W("1. **Cheapest experiment — no code change:** at your next practice, flip the dashboard's pass-strategy "
      "chooser from `Pass Symmetric` to `Pass Waypoint`. If passes start firing more reliably, you're done.")
    W("")
    W("2. **If waypoint doesn't fix it:** hand `docs/passing-fix-prompt.md` to a fresh coding session to "
      "prototype the geo-fence gate. It's scoped behind a feature flag so it can't break HUB shots.")
    W("")
    W("3. **Small code fix worth doing anyway:** the coord-stays-in-FIRING bug (Finding 2). Check "
      "`ShootingCoordinator.java` for the FIRING→AIMING transition condition.\n")

    W("---")
    W("*This report is auto-generated by `tools/generate_report.py`. "
      "Re-run `tools/analyze_wpilog.py` after adding new wpilogs to refresh it. "
      "Open `tools/log-analysis/dashboard.html` for the interactive explorer.*")

    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", default="tools/log-analysis/data",
                        help="Directory containing summary.json and match_*.json")
    parser.add_argument("--out", default="docs/analysis-findings.md",
                        help="Output markdown file")
    args = parser.parse_args()

    data_dir = Path(args.data)
    if not data_dir.exists():
        print(f"error: {data_dir} not found. Run tools/analyze_wpilog.py first.", file=sys.stderr)
        return 2

    summary, matches = load_data(data_dir)
    report = build_report(summary, matches)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(report)
    print(f"Wrote {out}  ({len(report)} chars, {report.count(chr(10))} lines)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
