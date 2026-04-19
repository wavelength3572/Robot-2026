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
import glob as _glob
import json
import sys
import time
from pathlib import Path

from log_context import LogContext, registered_analyzers

# Importing this registers all analyzers via the pkgutil walk in __init__.py.
import analyzers  # noqa: F401


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", help=".wpilog files to analyze (globs ok)")
    parser.add_argument("--out", required=True, help="output directory for JSON")
    args = parser.parse_args()

    # Expand globs ourselves so PowerShell users (which don't auto-expand
    # wildcards) get the same behavior as bash/zsh users.
    log_paths: list[Path] = []
    for pattern in args.logs:
        matches = _glob.glob(pattern)
        if matches:
            log_paths.extend(Path(m) for m in matches)
        elif Path(pattern).exists():
            log_paths.append(Path(pattern))
        else:
            print(f"warning: no files matched {pattern!r}", file=sys.stderr)
    if not log_paths:
        print("error: no .wpilog files found", file=sys.stderr)
        return 2

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    fleet_rows: list[dict] = []
    analyzer_ids: list[str] = []

    for i, log_path in enumerate(log_paths, 1):
        print(f"[{i}/{len(log_paths)}] {log_path.name}", flush=True)
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

    # Build a single self-contained HTML the user can double-click to open,
    # bypassing browsers' file:// CORS block on fetch().
    try:
        _write_standalone_html(out_dir, fleet_out)
    except Exception as exc:
        print(f"(standalone dashboard skipped: {exc})", file=sys.stderr)

    # Also regenerate the plain-English findings report.
    try:
        import generate_report
        summary, matches = generate_report.load_data(out_dir)
        report = generate_report.build_report(summary, matches)
        report_path = Path("docs/analysis-findings.md")
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(report)
        print(f"Wrote findings report: {report_path}")
    except Exception as exc:
        print(f"(findings report skipped: {exc})", file=sys.stderr)

    return 0


def _write_standalone_html(out_dir: Path, fleet_out: dict) -> None:
    """Emit tools/log-analysis/dashboard.html with CSS/JS/data all inlined."""
    web_dir = out_dir.parent  # tools/log-analysis/
    index_html = (web_dir / "index.html").read_text()
    styles_css = (web_dir / "styles.css").read_text()
    app_js = (web_dir / "app.js").read_text()
    chartjs = (web_dir / "vendor" / "chart.umd.min.js").read_text()

    # Gather every per-match JSON we just wrote
    matches = {}
    for match in fleet_out["matches"]:
        match_path = out_dir / f"match_{match['stem']}.json"
        if match_path.exists():
            matches[match["stem"]] = json.loads(match_path.read_text())
    bundle = {"summary": fleet_out, "matches": matches}
    bundle_json = json.dumps(bundle, default=_json_default)

    # Replace the three external resource references in index.html with inlined blocks.
    html = index_html
    # 1. The Chart.js <script src="vendor/...">  →  inline full library
    html = html.replace(
        '<script src="vendor/chart.umd.min.js"></script>',
        f"<script>\n{chartjs}\n</script>",
    )
    # 2. The CSS <link ...>  →  inline <style>
    html = html.replace(
        '<link rel="stylesheet" href="styles.css" />',
        f"<style>\n{styles_css}\n</style>",
    )
    # 3. The app <script src="app.js">  →  prepend embedded data, then inline the app
    data_block = f'<script>window.WPILOG_DATA = {bundle_json};</script>'
    app_block = f"<script>\n{app_js}\n</script>"
    html = html.replace('<script src="app.js"></script>', data_block + "\n" + app_block)

    dashboard_path = web_dir / "dashboard.html"
    dashboard_path.write_text(html)
    size_mb = dashboard_path.stat().st_size / (1024 * 1024)
    print(f"Wrote standalone dashboard: {dashboard_path}  ({size_mb:.1f} MB, double-click to open)")


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
