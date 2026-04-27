"""End-to-end: download + trim + track for one match. Writes a candidate-tracks JSON.

This is the single orchestrator the browser review tool consumes one
file at a time. Output schema:

    {
      "match_key": "2026micmp_qm45",
      "event_key": "2026micmp",
      "youtube_key": "abc123",
      "red_teams":  [3572, 1234, 5678],   # 3 team numbers
      "blue_teams": [9999, 1111, 2222],
      "auto_window": {"start_s": 6.0, "length_s": 17.0},
      "homography_source": "strategy-board/data/homography-2026arc.json",
      "tracks": [
        {
          "track_id": 0,
          "alliance": "red",
          "samples": [{"t": 0.03, "x_m": 4.4, "y_m": 7.5, ...}, ...]
        },
        ...
      ],
      "team_assignments": null    # filled in by the browser tool
    }

Usage:
    python tools/cv/extract_match.py 2026micmp_qm45 \\
            --tba-key $TBA_AUTH_KEY \\
            --homography strategy-board/data/homography-2026arc.json \\
            --out strategy-board/data/cv-traces/

Each invocation is idempotent: if the per-match output already exists and
--force is not given, it skips. Useful when re-running batches.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

# Allow running this file directly: `python tools/cv/extract_match.py ...`
# In that case `tools.cv.*` import paths don't resolve, so prepend the
# parent dir to sys.path and import siblings as bare modules.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import download_match  # type: ignore
    import track_bumpers  # type: ignore
    import homography as homography_mod  # type: ignore
else:
    from . import download_match
    from . import track_bumpers
    from . import homography as homography_mod


def extract_one(match_key: str,
                tba_key: str,
                homography_path: Path,
                out_dir: Path,
                cache_dir: Path,
                auto_start_s: float = 6.0,
                auto_length_s: float = 17.0,
                force: bool = False) -> Path:
    """Run the full pipeline for one match. Returns the output JSON path.

    Steps:
      1. TBA fetch (metadata + youtube key + alliances)
      2. yt-dlp download (cached)
      3. ffmpeg trim to auto period
      4. CV tracking
      5. Write merged JSON to {out_dir}/{event_key}-{match_key_short}.json
    """
    info = download_match.fetch_match(match_key, tba_key)
    if not info.youtube_key:
        raise RuntimeError(f"no YouTube video posted for {match_key}")

    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{info.event_key}_{info.match_key.split('_', 1)[-1]}.json"
    if out_path.exists() and not force:
        print(f"  exists, skipping (--force to override): {out_path}")
        return out_path

    full_video = download_match.download_youtube(info.youtube_key, cache_dir)

    with tempfile.TemporaryDirectory() as td:
        trimmed = Path(td) / "auto.mp4"
        download_match.trim_auto_period(full_video, trimmed, auto_start_s, auto_length_s)
        H = homography_mod.load_homography(homography_path)
        tracks = track_bumpers.track_bumpers(trimmed, H, visualize=False)

    payload = {
        "match_key": info.match_key,
        "event_key": info.event_key,
        "youtube_key": info.youtube_key,
        "red_teams": info.red_teams,
        "blue_teams": info.blue_teams,
        "auto_window": {"start_s": auto_start_s, "length_s": auto_length_s},
        "homography_source": str(homography_path),
        "tracks": track_bumpers.serialize_tracks(tracks),
        "team_assignments": None,  # filled in by browser review tool
    }
    out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"  → {out_path}")
    return out_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("match_keys", nargs="+",
                        help="One or more TBA match keys (e.g. 2026micmp_qm45)")
    parser.add_argument("--tba-key", default=os.environ.get("TBA_AUTH_KEY"),
                        help="TBA auth key (or env TBA_AUTH_KEY)")
    parser.add_argument("--homography", type=Path, required=True,
                        help="Path to homography JSON (from homography.py)")
    parser.add_argument("--out", type=Path, required=True,
                        help="Output directory for per-match JSONs")
    parser.add_argument("--cache", type=Path, default=Path(".cv-cache"),
                        help="Where to cache full downloaded videos")
    parser.add_argument("--auto-start", type=float, default=6.0)
    parser.add_argument("--auto-length", type=float, default=17.0)
    parser.add_argument("--force", action="store_true",
                        help="Re-process even if output already exists")
    args = parser.parse_args()

    if not args.tba_key:
        print("error: --tba-key (or env TBA_AUTH_KEY) is required", file=sys.stderr)
        return 2
    if not args.homography.exists():
        print(f"error: homography file not found: {args.homography}", file=sys.stderr)
        print("  run tools/cv/homography.py first to calibrate the camera", file=sys.stderr)
        return 2

    fail = 0
    for i, mk in enumerate(args.match_keys, 1):
        print(f"[{i}/{len(args.match_keys)}] {mk}")
        try:
            extract_one(
                mk, args.tba_key, args.homography, args.out, args.cache,
                args.auto_start, args.auto_length, args.force,
            )
        except Exception as exc:
            print(f"  FAILED: {exc}", file=sys.stderr)
            fail += 1
    print(f"\ndone. {len(args.match_keys) - fail} succeeded, {fail} failed.")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
