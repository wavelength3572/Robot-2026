#!/usr/bin/env python3
"""WPILog analyzer CLI.

Parses one or more AdvantageKit .wpilog files, runs every registered analyzer,
and writes:

    <out>/summary.json        — one row per match (fleet view)
    <out>/match_<stem>.json   — per-match events, summaries, and sampled series

Usage:
    python3 tools/analyze_wpilog.py StatesLogs/*.wpilog --out tools/log-analysis/data

Adding a new analyzer:
    Drop a file in tools/analyzers/ with a function decorated
    @register_analyzer(id="...", title="..."). It will be picked up next run
    and appear as a column in the fleet view and a lane in the match view.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from log_context import LogContext, registered_analyzers

# Importing this registers all analyzers via the pkgutil walk in __init__.py.
import analyzers  # noqa: F401


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", help=".wpilog files to analyze")
    parser.add_argument("--out", required=True, help="output directory for JSON")
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    fleet_rows: list[dict] = []
    analyzer_ids: list[str] = []

    for i, log_path in enumerate(args.logs, 1):
        log_path = Path(log_path)
        print(f"[{i}/{len(args.logs)}] {log_path.name}", flush=True)
        t_start = time.time()
        ctx = LogContext(log_path)
        t_parse = time.time() - t_start

        results_by_id = {}
        for aid, title, fn in registered_analyzers():
            if aid not in analyzer_ids:
                analyzer_ids.append(aid)
            t0 = time.time()
            try:
                r = fn(ctx)
            except Exception as exc:
                print(f"  ! analyzer {aid} failed: {exc}", file=sys.stderr)
                continue
            results_by_id[aid] = {
                "id": r.id,
                "title": r.title,
                "events": r.events,
                "summary": r.summary,
                "series": r.series,
                "notes": r.notes,
            }
            print(f"  - {aid:<22} {time.time() - t0:5.2f}s  {len(r.events):>4} events")

        match_out = {
            "file": log_path.name,
            "stem": log_path.stem,
            "duration_s": round(ctx.duration_s(), 2),
            "t0_us": ctx.t0_us,
            "analyzers": results_by_id,
        }
        (out_dir / f"match_{log_path.stem}.json").write_text(json.dumps(match_out, default=_json_default))

        fleet_summary = {aid: r["summary"] for aid, r in results_by_id.items()}
        fleet_rows.append(
            {
                "file": log_path.name,
                "stem": log_path.stem,
                "duration_s": round(ctx.duration_s(), 2),
                "parse_seconds": round(t_parse, 2),
                "summaries": fleet_summary,
            }
        )

    fleet_out = {
        "matches": fleet_rows,
        "analyzer_order": analyzer_ids,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    (out_dir / "summary.json").write_text(json.dumps(fleet_out, default=_json_default, indent=2))
    print(f"\nWrote {len(fleet_rows)} match files + summary.json to {out_dir}")
    return 0


def _json_default(o):
    if hasattr(o, "__dict__"):
        return o.__dict__
    if isinstance(o, set):
        return list(o)
    try:
        return float(o)
    except (TypeError, ValueError):
        return str(o)


if __name__ == "__main__":
    sys.exit(main())
