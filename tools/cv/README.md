# `tools/cv/` — auto-scouting CV pipeline

A computer-vision pipeline for extracting opponent autonomous robot paths
from publicly available FRC match videos on YouTube. Built for Wavelength
3572 to scout the Archimedes 2026 division before alliance selection.

This is also a **teaching codebase** for our programmers. Every module is
heavily commented. Read in order: `download_match.py` → `homography.py` →
`track_bumpers.py` → `extract_match.py` → `batch_process.py`.

## Why CV at all?

Manual scouting (watch a video, sketch a path with your finger) takes ~5
minutes per auto. With 75 teams in the division and ~2 matches per team
worth scouting, that's ~12 hours of human time. CV-assisted scouting (let
the computer guess, scout reviews and corrects) drops it to ~30 seconds
per auto: a few hours instead of two days.

We are deliberately *not* building a fully-automated system. Identity
assignment (which colored blob = which team number) is hard, occlusion
breaks naive trackers, and venue lighting is unpredictable. The pipeline
produces "candidate tracks" — most are right, some are wrong. A human in
the browser tool corrects the wrong ones in seconds.

## The pipeline conceptually

```
match video frames         pixel coordinates           field coordinates
   (3 RGB channels)              (x, y)                    (m, m)
        │                           │                          │
        ▼                           ▼                          ▼
   ┌─────────┐              ┌──────────────┐           ┌──────────────┐
   │  HSV    │              │  perspective │           │  per-robot   │
   │ thresh- │  ──────────► │   homography │  ───────► │   tracks     │
   │ olding  │              │  3x3 matrix  │           │   over time  │
   └─────────┘              └──────────────┘           └──────────────┘
   color-blob                "if I see this              "robot 1 was
   detection per             pixel, where is             at (4.3, 7.6)
   frame: red & blue         that on the                 at t=2.4s"
                             field?"
```

### Step 1: HSV color thresholding (in `track_bumpers.py`)

Why not RGB? Because RGB couples *color* and *brightness* together. Red
under bright lights and red under shadows have very different RGB values
but the same HSV "Hue." HSV separates color (Hue) from saturation and
brightness, so a single threshold range works across the whole video.

We threshold for two ranges: red bumpers (Hue ≈ 0 OR ≈ 180 — red wraps)
and blue bumpers (Hue ≈ 110-130). The output is a binary mask:
255 where bumper-color pixels are, 0 elsewhere.

### Step 2: contour detection

OpenCV's `findContours` walks the binary mask and returns the outline
of each connected region. We compute each region's centroid (center of
mass) and area. Filter by area to drop noise (a single bright pixel
isn't a robot). The result is a list of `(x_pixel, y_pixel, area)` per
frame — one entry per detected blob.

### Step 3: perspective homography (in `homography.py`)

The match camera is mounted high in the corner of the venue, looking
down on the field at an angle. Pixels at the top of the frame represent
field positions farther away than pixels at the bottom. A direct
"pixel x → field x" mapping doesn't work because the perspective stretches.

Solution: a **perspective transform**, encoded as a 3×3 matrix `H`. If we
know 4 pixel points and their corresponding field points, OpenCV solves
for `H` such that `field_xy = H @ pixel_xy` (in homogeneous coordinates).

You only have to calibrate once per camera angle — different events have
different camera positions, so we need to re-do this on Day 1 of Worlds.

### Step 4: track linking

Per-frame blobs aren't enough. To say "this robot moved from A to B," we
need to associate a blob in frame N with the same robot's blob in
frame N+1. We use the simplest possible technique: nearest-neighbor in
field coordinates. If a red blob in frame N is < 0.6 m away from a red
blob in frame N+1, they're the same robot.

This breaks under occlusion (blob disappears for a few frames) and
cross-overs (two robots pass each other). The browser review tool fixes
these: scout sees a broken track, drags points, splits/joins.

## Usage

```bash
pip install -r tools/cv/requirements.txt

# One-time per event: calibrate the homography
python tools/cv/homography.py --video path/to/sample-match.mp4 \
       --out strategy-board/data/homography-2026arc.json

# Per match: download + extract candidate tracks
python tools/cv/extract_match.py 2026micmp_qm45 \
       --tba-key $TBA_AUTH_KEY \
       --homography strategy-board/data/homography-2026arc.json \
       --out strategy-board/data/cv-traces/

# Batch: process many matches overnight
python tools/cv/batch_process.py --teams 27,1114,9470,3476 \
       --matches-per-team 2 \
       --tba-key $TBA_AUTH_KEY \
       --out strategy-board/data/cv-traces/
```

Open `strategy-board/auto-scout.html` in your browser to review the
output and confirm/correct identities.

## A few honest caveats

- **Color thresholds need tuning per camera.** Check the visualizer mode
  on `track_bumpers.py` if accuracy is poor. The HSV ranges in `BUMPER_COLORS`
  are starting points, not ground truth.
- **Occlusion happens.** Two robots passing close together will swap or
  merge in the tracker. Plan on the scout fixing ~20% of tracks manually.
- **The auto-period clock is hard.** We don't have a synchronized robot
  clock from the video. We approximate "auto starts at the cued autoplay
  position" — usually within 0.5 s of true T+0. Good enough for path
  shape; not good enough for ball-firing event detection.

## File map

| File | Purpose |
|---|---|
| `homography.py` | Interactive calibrator: click 4 known field points → save 3x3 matrix |
| `download_match.py` | TBA → YouTube key → yt-dlp → trimmed auto-period clip |
| `track_bumpers.py` | HSV threshold + contour + tracker → per-robot field-coord traces |
| `extract_match.py` | Per-match orchestrator: download + track + write JSON |
| `batch_process.py` | Run extract_match across many teams / matches |

## Further reading for our programmers

- OpenCV color-space tutorial: https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html
- Perspective transforms: https://docs.opencv.org/4.x/da/d6e/tutorial_py_geometric_transformations.html
- Contour features: https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html
- yt-dlp documentation: https://github.com/yt-dlp/yt-dlp
- TBA API v3: https://www.thebluealliance.com/apidocs/v3
