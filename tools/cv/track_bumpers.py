"""Track red and blue robot bumpers across frames; output field-coordinate paths.

Concept (READ THIS):
    A FRC robot in the broadcast is a roughly-rectangular blob with a
    saturated colored stripe (the bumper) around its base. The bumper
    color tells us alliance side. We find these bumpers per frame, then
    link them across frames into per-robot tracks.

    The pipeline per frame:
      1. Convert BGR (OpenCV's default) to HSV color space.
         - Why HSV? Because Hue isolates *color* from brightness. A red
           bumper under bright lights and the same bumper in shadow
           share the same Hue but very different RGB values. HSV thresholds
           generalize across lighting; RGB thresholds don't.
      2. Threshold for "red bumper" and "blue bumper" separately.
         - Red Hue wraps around 0/180 (red is at both ends of the wheel),
           so we threshold two ranges and OR them together.
      3. Morphological opening (erode then dilate) to kill speckle noise.
      4. cv2.findContours on each binary mask. Each contour is a connected
         region; its centroid is our blob center.
      5. Filter by area (drop tiny contours that are confetti or LED rim).
      6. Apply the homography matrix to each blob centroid → (x_field, y_field).

    Then linking blobs across frames into tracks:
      - For each new frame's blob, look at all open tracks of the same color.
      - If any track ends within MAX_LINK_DIST_M and within MAX_LINK_GAP_S
        of this blob, extend that track. Otherwise start a new one.
      - When the auto period ends, close all open tracks. Drop tracks
        shorter than MIN_TRACK_FRAMES (probably noise).

    Why this simple scheme works often enough:
      - FRC robots move at most ~5 m/s. At 30 FPS that's < 0.17 m between
        frames. Setting MAX_LINK_DIST_M = 0.6 m gives 3-4× headroom.
      - Identity confusion happens when two same-color robots cross paths.
        We let the scout fix this in the browser tool.

    Why this scheme breaks (and how the review tool helps):
      - Occlusion (one robot in front of another) → the tracker loses one
        of them. Scout sees a broken track and fixes it.
      - Color similarity (red bumper vs red field-element pixels) → false
        positives. We filter by area, but venue lighting can defeat that.
        Scout sees a stray track and rejects it.

Usage (standalone, for tuning):
    python tools/cv/track_bumpers.py --video auto.mp4 \\
            --homography homography.json \\
            --visualize  # opens a window showing detections

Usage (as library):
    from tools.cv.track_bumpers import track_bumpers, Track
    tracks = track_bumpers(video_path, homography)
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

# --- Tuning constants (you'll likely adjust these per camera setup) -------

# HSV ranges for FRC bumpers. OpenCV's HSV: H in [0, 180), S/V in [0, 255].
# Red wraps the H boundary, so we have two ranges OR'd together.
BUMPER_COLORS: dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]] = {
    "red": [
        ((0, 130, 90), (10, 255, 255)),     # red side near H=0
        ((170, 130, 90), (180, 255, 255)),  # red side near H=180
    ],
    "blue": [
        ((100, 130, 90), (130, 255, 255)),  # blue
    ],
}

# Per-frame contour area (pixels^2) to count as a candidate robot. Tune
# per-resolution: at 720p with the typical broadcast camera, a robot's
# bumper occupies a few hundred to ~3000 px^2.
MIN_CONTOUR_AREA_PX = 200
MAX_CONTOUR_AREA_PX = 8000

# Track-linking thresholds.
MAX_LINK_DIST_M = 0.6     # max meters between consecutive samples to extend a track
MAX_LINK_GAP_S = 0.25     # max seconds gap between samples
MIN_TRACK_FRAMES = 5       # drop tracks with fewer than this many samples
MIN_TRACK_DURATION_S = 0.5  # drop tracks shorter than 0.5 seconds in time


@dataclass
class Sample:
    t_s: float
    x_m: float
    y_m: float
    pixel_x: float
    pixel_y: float
    area_px: float


@dataclass
class Track:
    track_id: int
    alliance: str  # "red" or "blue"
    samples: list[Sample] = field(default_factory=list)

    def last_t(self) -> float:
        return self.samples[-1].t_s if self.samples else -1.0

    def last_pos(self) -> tuple[float, float]:
        s = self.samples[-1]
        return s.x_m, s.y_m

    def duration_s(self) -> float:
        if len(self.samples) < 2:
            return 0.0
        return self.samples[-1].t_s - self.samples[0].t_s


def detect_blobs(frame_bgr: np.ndarray, alliance: str) -> list[tuple[float, float, float]]:
    """Find all bumper-colored blobs in one frame for one alliance.

    Returns list of (x_pixel, y_pixel, area_px). Pixel coords are float.
    """
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in BUMPER_COLORS[alliance]:
        sub = cv2.inRange(hsv, np.array(lo, dtype=np.uint8), np.array(hi, dtype=np.uint8))
        mask = cv2.bitwise_or(mask, sub)

    # Morphological opening to kill speckle. 3x3 kernel, 1 iteration.
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    # Dilate slightly to merge fragmented bumper segments into one blob.
    mask = cv2.dilate(mask, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out: list[tuple[float, float, float]] = []
    for c in contours:
        area = float(cv2.contourArea(c))
        if area < MIN_CONTOUR_AREA_PX or area > MAX_CONTOUR_AREA_PX:
            continue
        # Centroid via image moments. m00 == area; m10/m00 = mean x.
        m = cv2.moments(c)
        if m["m00"] <= 0:
            continue
        cx = m["m10"] / m["m00"]
        cy = m["m01"] / m["m00"]
        out.append((cx, cy, area))
    return out


def pixels_to_field(homography: np.ndarray,
                    points: list[tuple[float, float, float]]) -> list[tuple[float, float, float, float, float]]:
    """Apply the homography to a list of (px, py, area). Returns (xm, ym, px, py, area)."""
    if not points:
        return []
    pts = np.array([[(px, py)] for (px, py, _) in points], dtype=np.float32)
    out = cv2.perspectiveTransform(pts, homography)
    return [
        (float(out[i, 0, 0]), float(out[i, 0, 1]), points[i][0], points[i][1], points[i][2])
        for i in range(len(points))
    ]


def link_to_tracks(detections_per_frame: list[list[Sample]],
                   alliance: str,
                   start_track_id: int = 0) -> list[Track]:
    """Greedy nearest-neighbor track linker.

    For each frame in order, walk through its detections. For each
    detection, find the open track whose last sample is closest (under
    distance + time-gap thresholds). If found, extend that track. Else
    start a new track.

    "Open" = last sample was within MAX_LINK_GAP_S seconds. Once a track
    goes stale beyond that gap, it's closed.
    """
    open_tracks: list[Track] = []
    closed_tracks: list[Track] = []
    next_id = start_track_id

    for frame_dets in detections_per_frame:
        # Close any track that's gone stale relative to the latest frame.
        if frame_dets:
            t_now = frame_dets[0].t_s
            still_open = []
            for tr in open_tracks:
                if t_now - tr.last_t() <= MAX_LINK_GAP_S:
                    still_open.append(tr)
                else:
                    closed_tracks.append(tr)
            open_tracks = still_open

        # Greedy match each detection to the nearest open track.
        used_track_idx: set[int] = set()
        for det in frame_dets:
            best_idx = -1
            best_dist = MAX_LINK_DIST_M
            for i, tr in enumerate(open_tracks):
                if i in used_track_idx:
                    continue
                lx, ly = tr.last_pos()
                d = float(np.hypot(det.x_m - lx, det.y_m - ly))
                if d < best_dist:
                    best_dist = d
                    best_idx = i
            if best_idx >= 0:
                open_tracks[best_idx].samples.append(det)
                used_track_idx.add(best_idx)
            else:
                tr = Track(track_id=next_id, alliance=alliance, samples=[det])
                open_tracks.append(tr)
                next_id += 1

    closed_tracks.extend(open_tracks)

    # Filter short tracks; they're almost always noise.
    return [
        tr for tr in closed_tracks
        if len(tr.samples) >= MIN_TRACK_FRAMES and tr.duration_s() >= MIN_TRACK_DURATION_S
    ]


def track_bumpers(video_path: Path,
                  homography: np.ndarray,
                  visualize: bool = False) -> list[Track]:
    """Full per-video pipeline: read frames, detect, link, return tracks.

    Iterates the video in order, detecting both red and blue blobs per
    frame. Two independent track sets are linked (red-only and blue-only)
    and concatenated. Track IDs are unique across both sets.
    """
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"could not open {video_path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    red_per_frame: list[list[Sample]] = []
    blue_per_frame: list[list[Sample]] = []

    frame_idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t_s = frame_idx / fps

        for alliance, store in (("red", red_per_frame), ("blue", blue_per_frame)):
            blobs_px = detect_blobs(frame, alliance)
            field_pts = pixels_to_field(homography, blobs_px)
            samples = [
                Sample(t_s=t_s, x_m=xm, y_m=ym, pixel_x=px, pixel_y=py, area_px=area)
                for (xm, ym, px, py, area) in field_pts
            ]
            store.append(samples)

        if visualize:
            _draw_overlay(frame, red_per_frame[-1], blue_per_frame[-1])
            cv2.imshow("track_bumpers", frame)
            if (cv2.waitKey(1) & 0xFF) == 27:
                break
        frame_idx += 1

    cap.release()
    if visualize:
        cv2.destroyAllWindows()

    red_tracks = link_to_tracks(red_per_frame, "red", start_track_id=0)
    blue_tracks = link_to_tracks(blue_per_frame, "blue", start_track_id=len(red_tracks))
    print(f"  detected: {len(red_tracks)} red tracks, {len(blue_tracks)} blue tracks "
          f"over {n_frames} frames @ {fps:.1f} fps")
    return red_tracks + blue_tracks


def _draw_overlay(frame: np.ndarray,
                  red_samples: list[Sample],
                  blue_samples: list[Sample]) -> None:
    """Visualizer for tuning: draw detected blobs onto the frame in place."""
    for s in red_samples:
        cv2.circle(frame, (int(s.pixel_x), int(s.pixel_y)), 8, (0, 0, 255), 2)
        cv2.putText(frame, f"({s.x_m:.1f},{s.y_m:.1f})",
                    (int(s.pixel_x) + 10, int(s.pixel_y)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
    for s in blue_samples:
        cv2.circle(frame, (int(s.pixel_x), int(s.pixel_y)), 8, (255, 100, 0), 2)
        cv2.putText(frame, f"({s.x_m:.1f},{s.y_m:.1f})",
                    (int(s.pixel_x) + 10, int(s.pixel_y)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 100, 0), 1)


def serialize_tracks(tracks: list[Track]) -> list[dict]:
    """Convert a list of Tracks to plain JSON-friendly dicts."""
    return [
        {
            "track_id": tr.track_id,
            "alliance": tr.alliance,
            "samples": [
                {
                    "t": round(s.t_s, 3),
                    "x_m": round(s.x_m, 3),
                    "y_m": round(s.y_m, 3),
                    "pixel_x": round(s.pixel_x, 1),
                    "pixel_y": round(s.pixel_y, 1),
                    "area_px": round(s.area_px, 0),
                }
                for s in tr.samples
            ],
        }
        for tr in tracks
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", type=Path, required=True,
                        help="Auto-period video file (e.g. auto.mp4)")
    parser.add_argument("--homography", type=Path, required=True,
                        help="Homography JSON from tools/cv/homography.py")
    parser.add_argument("--out", type=Path,
                        help="Optional: write tracks JSON here")
    parser.add_argument("--visualize", action="store_true",
                        help="Show detection overlay during processing (interactive)")
    args = parser.parse_args()

    # Late import so the main module remains importable without homography.py
    # being structurally broken.
    from . import homography as homography_mod  # noqa: F401

    H = _load_homography(args.homography)
    print(f"loaded homography from {args.homography}")
    tracks = track_bumpers(args.video, H, visualize=args.visualize)

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(
            json.dumps({"tracks": serialize_tracks(tracks)}, indent=2),
            encoding="utf-8",
        )
        print(f"wrote {args.out}")
    return 0


def _load_homography(path: Path) -> np.ndarray:
    """Local copy to avoid relative-import drama when run as a script.

    Mirrors homography.load_homography(). Kept here for direct script use.
    """
    payload = json.loads(path.read_text(encoding="utf-8"))
    flat = payload["homography_3x3"]
    return np.array(flat, dtype=np.float64).reshape(3, 3)


if __name__ == "__main__":
    sys.exit(main())
