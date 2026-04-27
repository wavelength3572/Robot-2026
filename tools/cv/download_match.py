"""Fetch a match video from YouTube via TBA, trim to the autonomous period.

Concept (READ THIS):
    The Blue Alliance (TBA) catalogs every official FRC match. Their
    `match` endpoint includes a `videos` array — typically with a YouTube
    key for the official broadcast. We use TBA to look up the YouTube key,
    then yt-dlp to actually download the video file, then OpenCV to slice
    out just the 15 seconds of autonomous play.

    Why download instead of streaming via embed?
    - We need pixel-level access (for HSV thresholding, contour detection).
      Browser-embedded YouTube iframes don't expose pixels due to CORS.
    - Once downloaded, we can re-process the video as many times as we want
      without re-fetching. Useful for tuning HSV thresholds.

    Why local caching?
    - yt-dlp may fail mid-batch if YouTube rate-limits or the video is
      private. We cache to disk so a partial batch can resume.
    - Disk space: a 3-min match at 720p is ~30 MB. 60 matches = ~2 GB.
      That's fine for a laptop, but we trim to the auto period (15 sec ~
      3 MB) for the final cache.

    Auto-period boundaries:
    - FRC autonomous starts at the green-field-light go signal. We don't
      have a clock-synced signal in the video, so we use a hard offset:
      most FRC broadcasts open the auto period within 5-15 seconds of
      video start. We approximate `auto_start = video_start + 6s` and
      `auto_end = auto_start + 15s`. Scout corrects in the review tool
      if the camera-cut timing was unusual for a given match.

Usage:
    python tools/cv/download_match.py 2026micmp_qm45 \\
            --tba-key $TBA_AUTH_KEY \\
            --cache /tmp/cv-cache \\
            --out auto-period.mp4
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import requests


TBA_BASE = "https://www.thebluealliance.com/api/v3"
DEFAULT_AUTO_START_S = 6.0   # seconds into the video where auto roughly starts
DEFAULT_AUTO_LENGTH_S = 17.0  # auto period is 15s; we grab a 17s window for safety


@dataclass
class MatchInfo:
    match_key: str
    event_key: str
    youtube_key: str | None
    red_teams: list[int]   # 3 team numbers
    blue_teams: list[int]
    raw: dict              # original TBA payload, kept for debugging


def fetch_match(match_key: str, tba_key: str) -> MatchInfo:
    """Pull a match's metadata from TBA. Includes the YouTube key + alliances.

    TBA's match endpoint shape is documented at:
        https://www.thebluealliance.com/apidocs/v3#/Match
    Important fields: alliances.{red,blue}.team_keys (list of 'frc1234')
    and videos[i].{type,key} where type='youtube' is what we want.
    """
    url = f"{TBA_BASE}/match/{match_key}"
    resp = requests.get(url, headers={"X-TBA-Auth-Key": tba_key}, timeout=10)
    resp.raise_for_status()
    payload = resp.json()

    youtube_key = None
    for v in payload.get("videos") or []:
        if v.get("type") == "youtube" and v.get("key"):
            youtube_key = v["key"]
            break

    def _team_nums(side: str) -> list[int]:
        keys = payload.get("alliances", {}).get(side, {}).get("team_keys", [])
        return [int(k.replace("frc", "")) for k in keys]

    return MatchInfo(
        match_key=payload["key"],
        event_key=payload.get("event_key", ""),
        youtube_key=youtube_key,
        red_teams=_team_nums("red"),
        blue_teams=_team_nums("blue"),
        raw=payload,
    )


def download_youtube(youtube_key: str, cache_dir: Path) -> Path:
    """Use yt-dlp to fetch the full video. Returns the local path.

    yt-dlp's CLI is more reliable than its Python module across YouTube's
    constant API changes — we shell out. The format string '-f bv*[height<=720]+ba/b'
    asks for "best video up to 720p plus best audio, merged" — keeps file
    size down. 720p is plenty for tracking 6 robots.
    """
    cache_dir.mkdir(parents=True, exist_ok=True)
    # yt-dlp picks the extension; we use a stable filename root.
    output_template = str(cache_dir / f"{youtube_key}.%(ext)s")
    # If we already have it, skip.
    existing = list(cache_dir.glob(f"{youtube_key}.*"))
    existing = [p for p in existing if p.suffix in (".mp4", ".webm", ".mkv")]
    if existing:
        return existing[0]

    # Caller is responsible for ensuring yt-dlp is installed.
    cmd = [
        shutil.which("yt-dlp") or "yt-dlp",
        "-f", "bv*[height<=720]+ba/b[height<=720]/b",
        "-o", output_template,
        "--no-warnings",
        "--quiet",
        f"https://www.youtube.com/watch?v={youtube_key}",
    ]
    subprocess.run(cmd, check=True)
    found = list(cache_dir.glob(f"{youtube_key}.*"))
    found = [p for p in found if p.suffix in (".mp4", ".webm", ".mkv")]
    if not found:
        raise RuntimeError(f"yt-dlp produced no file for {youtube_key}")
    return found[0]


def trim_auto_period(video_path: Path,
                     out_path: Path,
                     start_s: float = DEFAULT_AUTO_START_S,
                     length_s: float = DEFAULT_AUTO_LENGTH_S) -> Path:
    """Cut just the auto period using ffmpeg, copy codec for speed.

    `-c copy` skips re-encoding — the trim is at keyframe boundaries which
    is fine for our purposes. If the broadcast's auto-start frame is
    inside a GOP we may grab a fraction of a second early; the tracker
    handles the slack.
    """
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        shutil.which("ffmpeg") or "ffmpeg",
        "-y", "-loglevel", "error",
        "-ss", f"{start_s:.3f}",
        "-i", str(video_path),
        "-t", f"{length_s:.3f}",
        "-c", "copy",
        str(out_path),
    ]
    subprocess.run(cmd, check=True)
    return out_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("match_key", help="TBA match key, e.g. 2026micmp_qm45")
    parser.add_argument("--tba-key", default=os.environ.get("TBA_AUTH_KEY"),
                        help="TBA Auth Key (or env TBA_AUTH_KEY)")
    parser.add_argument("--cache", type=Path, default=Path(".cv-cache"),
                        help="Directory for full downloaded videos")
    parser.add_argument("--out", type=Path, required=True,
                        help="Output trimmed auto-period file (e.g. auto.mp4)")
    parser.add_argument("--auto-start", type=float, default=DEFAULT_AUTO_START_S,
                        help=f"Seconds into video where auto begins (default {DEFAULT_AUTO_START_S})")
    parser.add_argument("--auto-length", type=float, default=DEFAULT_AUTO_LENGTH_S,
                        help=f"Auto window length in seconds (default {DEFAULT_AUTO_LENGTH_S})")
    parser.add_argument("--metadata-out", type=Path,
                        help="Optional: write match metadata JSON here for downstream use")
    args = parser.parse_args()

    if not args.tba_key:
        print("error: --tba-key (or env TBA_AUTH_KEY) is required", file=sys.stderr)
        return 2

    print(f"[1/3] fetching match metadata: {args.match_key}")
    info = fetch_match(args.match_key, args.tba_key)
    if not info.youtube_key:
        print(f"error: no YouTube video posted for {args.match_key} yet", file=sys.stderr)
        return 3
    print(f"      red={info.red_teams}  blue={info.blue_teams}  yt={info.youtube_key}")

    print(f"[2/3] downloading {info.youtube_key} (cached at {args.cache})")
    video_path = download_youtube(info.youtube_key, args.cache)
    print(f"      → {video_path}")

    print(f"[3/3] trimming auto period (start={args.auto_start}s, length={args.auto_length}s)")
    trim_auto_period(video_path, args.out, args.auto_start, args.auto_length)
    print(f"      → {args.out}")

    if args.metadata_out:
        args.metadata_out.parent.mkdir(parents=True, exist_ok=True)
        args.metadata_out.write_text(json.dumps({
            "match_key": info.match_key,
            "event_key": info.event_key,
            "youtube_key": info.youtube_key,
            "red_teams": info.red_teams,
            "blue_teams": info.blue_teams,
            "auto_start_s": args.auto_start,
            "auto_length_s": args.auto_length,
        }, indent=2), encoding="utf-8")
        print(f"      metadata → {args.metadata_out}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
