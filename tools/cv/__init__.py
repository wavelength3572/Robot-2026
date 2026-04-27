"""Computer-vision pipeline for scouting opponent autonomous routines.

This package exists to teach as much as to ship. Each module is meant to be
read top-to-bottom, with comments explaining *why* the CV technique is the
right one for the FRC match-video problem. Pipeline overview:

    1. download_match.py   — fetch a YouTube match video via yt-dlp,
                             trim to the autonomous period
    2. homography.py       — calibrate a 3x3 transform that maps camera
                             pixels to field meters (so a robot's pixel
                             position becomes a real (x, y) on the field)
    3. track_bumpers.py    — find red and blue robots in each frame using
                             HSV color thresholding + contour detection,
                             link blobs across frames into per-robot tracks,
                             apply the homography to get field coordinates
    4. extract_match.py    — orchestrate the above for a single match;
                             write a candidate-tracks JSON for the browser
                             review tool
    5. batch_process.py    — run extract_match across many matches

The browser tool at strategy-board/auto-scout.html consumes the JSON output
and lets a scout assign team identities + correct any tracking mistakes.
"""
