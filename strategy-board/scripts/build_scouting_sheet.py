#!/usr/bin/env python3
"""
Build a scouting CSV for a given FRC event, combining data from:
  - The Blue Alliance (roster, OPR/DPR/CCWM, rankings, awards)
  - Statbotics (EPA breakdowns + ranks)

Usage:
  TBA_KEY=xxxx python3 build_scouting_sheet.py 2026micmp2
  # or: echo "xxxx" > ~/.tba_key && python3 build_scouting_sheet.py 2026micmp2

Output: strategy-board/scouting-<eventkey>.csv
Columns are intentionally wide; delete what you don't use in the sheet.
The robot_style column is left blank for manual TURRET / DUMPER tagging.
"""

from __future__ import annotations

import csv
import os
import pathlib
import sys
import time
from typing import Any

import requests

TBA_BASE = "https://www.thebluealliance.com/api/v3"
SB_BASE = "https://api.statbotics.io/v3"
REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]

# Default read-only TBA key. Override via TBA_KEY env var or ~/.tba_key file.
# If this repo is public, rotate this key at https://www.thebluealliance.com/account
DEFAULT_TBA_KEY = "ojG6dDdRalOD0nCL71UN36yeEBTdCBpxWgOFaldjDuU5R3WqOZjLUpO39bisnRmm"


def load_tba_key() -> str:
    key = os.environ.get("TBA_KEY", "").strip()
    if key:
        return key
    key_file = pathlib.Path.home() / ".tba_key"
    if key_file.exists():
        return key_file.read_text().strip()
    return DEFAULT_TBA_KEY


def tba_get(path: str, key: str) -> Any:
    r = requests.get(f"{TBA_BASE}{path}", headers={"X-TBA-Auth-Key": key}, timeout=20)
    if r.status_code == 404:
        return None
    r.raise_for_status()
    return r.json()


def sb_get(path: str) -> Any:
    r = requests.get(f"{SB_BASE}{path}", timeout=20)
    if r.status_code == 404:
        return None
    r.raise_for_status()
    return r.json()


def fmt(v: Any, digits: int = 2) -> str:
    if v is None:
        return ""
    if isinstance(v, float):
        return f"{v:.{digits}f}"
    return str(v)


def build(event_key: str) -> pathlib.Path:
    key = load_tba_key()
    year = int(event_key[:4])

    print(f"[tba] fetching teams for {event_key}")
    teams = tba_get(f"/event/{event_key}/teams/simple", key) or []
    teams.sort(key=lambda t: t["team_number"])

    print(f"[tba] fetching oprs for {event_key}")
    oprs = tba_get(f"/event/{event_key}/oprs", key) or {}
    opr_map = oprs.get("oprs", {}) or {}
    dpr_map = oprs.get("dprs", {}) or {}
    ccwm_map = oprs.get("ccwms", {}) or {}

    print(f"[tba] fetching rankings for {event_key}")
    rankings = tba_get(f"/event/{event_key}/rankings", key) or {}
    rank_map: dict[int, dict] = {}
    for r in (rankings or {}).get("rankings", []) or []:
        num = int(r["team_key"].removeprefix("frc"))
        rank_map[num] = r

    rows: list[dict[str, str]] = []
    for i, t in enumerate(teams, 1):
        num = t["team_number"]
        tkey = f"frc{num}"
        print(f"[{i}/{len(teams)}] team {num}")

        awards = tba_get(f"/team/{tkey}/event/{event_key}/awards", key) or []
        sb_year = sb_get(f"/team_year/{num}/{year}") or {}
        sb_event = sb_get(f"/team_event/{num}/{event_key}") or {}

        epa_year = (sb_year.get("epa") or {}).get("breakdown", {}) or {}
        epa_event = (sb_event.get("epa") or {}).get("breakdown", {}) or {}
        ranks = (sb_year.get("epa") or {}).get("ranks", {}) or {}
        district_rank = (ranks.get("district") or {})
        country_rank = (ranks.get("country") or {})

        rank_row = rank_map.get(num, {})
        record = rank_row.get("record") or {}

        rows.append({
            "team": str(num),
            "nickname": t.get("nickname", ""),
            "city": ", ".join(filter(None, [t.get("city", ""), t.get("state_prov", "")])),
            "rookie_year": "",  # /teams/simple omits; fill from Statbotics
            "event_rank": fmt(rank_row.get("rank")),
            "record_w_l_t": "-".join(fmt(record.get(k)) for k in ("wins", "losses", "ties")) if record else "",
            "opr": fmt(opr_map.get(tkey)),
            "dpr": fmt(dpr_map.get(tkey)),
            "ccwm": fmt(ccwm_map.get(tkey)),
            "epa_total":   fmt((sb_year.get("epa") or {}).get("total_points", {}).get("mean")),
            "epa_auto":    fmt(epa_year.get("auto_points", {}).get("mean") if isinstance(epa_year.get("auto_points"), dict) else epa_year.get("auto_points")),
            "epa_teleop":  fmt(epa_year.get("teleop_points", {}).get("mean") if isinstance(epa_year.get("teleop_points"), dict) else epa_year.get("teleop_points")),
            "epa_endgame": fmt(epa_year.get("endgame_points", {}).get("mean") if isinstance(epa_year.get("endgame_points"), dict) else epa_year.get("endgame_points")),
            "epa_event":   fmt((sb_event.get("epa") or {}).get("total_points", {}).get("mean")),
            "winrate": fmt((sb_year.get("record") or {}).get("winrate")),
            "district_rank": fmt(district_rank.get("rank")),
            "country_rank": fmt(country_rank.get("rank")),
            "awards_this_event": "; ".join(a.get("name", "") for a in awards),
            "robot_style": "",  # manual: TURRET / DUMPER / HYBRID / UNKNOWN
            "notes": "",
            "tba_team": f"https://www.thebluealliance.com/team/{num}/{year}",
            "tba_media": f"https://www.thebluealliance.com/team/{num}/{year}#media",
            "chief_delphi_search": f"https://www.chiefdelphi.com/search?q=frc%20{num}%20{year}%20robot",
            "statbotics": f"https://www.statbotics.io/team/{num}/{year}",
        })

        time.sleep(0.1)  # be polite

    out_path = REPO_ROOT / "strategy-board" / f"scouting-{event_key}.csv"
    with out_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {out_path} ({len(rows)} teams)")
    return out_path


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: build_scouting_sheet.py <event_key>  e.g. 2026micmp2")
    build(sys.argv[1])
