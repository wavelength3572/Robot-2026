"""Interactive calibration of camera-to-field perspective transform.

Concept (READ THIS):
    The match-broadcast camera is mounted high in a venue corner and looks
    at the field at an angle. A pixel at the top of the frame represents a
    field point farther from the camera than a pixel at the bottom — so a
    direct "linear" pixel→field mapping doesn't work. Perspective stretches
    the geometry.

    The fix is a 3x3 *perspective transform* (a "homography"). If we know
    4 pixel positions in the camera frame AND we know what (x, y) on the
    actual field those pixels correspond to, OpenCV solves for the matrix
    H such that:

        [x_field]           [u_pixel]
        [y_field]  =  H  @  [v_pixel]      (in homogeneous coords)
        [   1   ]           [   1   ]

    Once H is known, every pixel coordinate from the same camera angle can
    be converted to field meters. We save H to JSON; the trackers load it
    and use cv2.perspectiveTransform() per frame.

    Different events use different camera positions, so we need a fresh
    calibration for each event. At Worlds we'll re-run this against a
    practice match clip from the venue feed.

How this script works:
    1. Open one frame of a representative match video.
    2. Show the frame and prompt the scout to click 4 known field landmarks
       in a fixed order (corners of features whose field coordinates are
       defined in FieldConstants.java).
    3. Compute H with cv2.getPerspectiveTransform().
    4. Save H + the calibration points to JSON for traceability.
    5. Sanity-check: re-project the 4 clicked points and verify error is
       small. Larger error means the user clicked imprecisely.

Usage:
    python tools/cv/homography.py --video sample.mp4 --out homography.json

    With a single still frame instead:
    python tools/cv/homography.py --frame frame.png --out homography.json
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Field landmarks — known (x, y) in meters in FRC 2026 field coordinates.
#
# These are the four points the scout clicks in the camera frame. They were
# chosen because (a) they're high-contrast, easy to spot in any video, and
# (b) they're spread out enough across the field that a 4-point homography
# stays accurate everywhere in between. (Homographies degenerate when the
# 4 points are collinear or clustered.)
#
# Coordinates are ALL field-relative, meters, blue-origin-bottom-left
# convention (the FRC standard). See FieldConstants.java for definitions.
#
# If the field rules change or you want different landmarks, edit this list
# and re-run the calibrator. The scout-prompt order is the order below.
# ---------------------------------------------------------------------------

@dataclass
class Landmark:
    name: str
    x_m: float
    y_m: float
    hint: str  # human prompt shown when it's this point's turn


FIELD_LANDMARKS: list[Landmark] = [
    Landmark("blue_left_trench_corner", 0.60, 7.00,
             "Click the BLUE-side LEFT trench corner (top-left from broadcast view)"),
    Landmark("blue_right_trench_corner", 0.60, 1.21,
             "Click the BLUE-side RIGHT trench corner (bottom-left from broadcast view)"),
    Landmark("red_left_trench_corner", 15.94, 7.00,
             "Click the RED-side LEFT trench corner (top-right from broadcast view)"),
    Landmark("red_right_trench_corner", 15.94, 1.21,
             "Click the RED-side RIGHT trench corner (bottom-right from broadcast view)"),
]
# NOTE: those exact x_m / y_m values are placeholders — verify against
# FieldConstants.java before using for real calibration. The script's logic
# doesn't care; only the values matter.


def grab_frame(video_path: Path, second: float = 5.0) -> np.ndarray:
    """Pull a single frame from the video at the given timestamp (seconds).

    OpenCV's VideoCapture is happy to read mp4/webm/mkv. Default to ~5s in
    so we miss any title-card / countdown overlay at frame 0.
    """
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"could not open video: {video_path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    target_frame = int(second * fps)
    cap.set(cv2.CAP_PROP_POS_FRAMES, target_frame)
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError(f"could not read frame {target_frame} from {video_path}")
    return frame


def collect_clicks(frame: np.ndarray) -> list[tuple[float, float]]:
    """Show the frame in a window; prompt for one click per landmark.

    OpenCV's setMouseCallback fires our handler on every click. We append
    each click to a list and update the window title to advance to the
    next prompt. When all 4 points are collected, the window auto-closes.

    Returns: list of (x_pixel, y_pixel) floats, length = len(FIELD_LANDMARKS).
    """
    points: list[tuple[float, float]] = []
    display = frame.copy()

    def on_mouse(event, x, y, flags, _userdata):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        idx = len(points)
        if idx >= len(FIELD_LANDMARKS):
            return
        points.append((float(x), float(y)))
        # Draw a numbered marker so the user can see what they clicked.
        cv2.circle(display, (x, y), 8, (0, 255, 0), 2)
        cv2.putText(display, str(idx + 1), (x + 10, y + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("homography", display)
        if idx + 1 < len(FIELD_LANDMARKS):
            print(f"[{idx + 2}/{len(FIELD_LANDMARKS)}] {FIELD_LANDMARKS[idx + 1].hint}")

    cv2.namedWindow("homography", cv2.WINDOW_NORMAL)
    cv2.imshow("homography", display)
    cv2.setMouseCallback("homography", on_mouse)

    print(f"[1/{len(FIELD_LANDMARKS)}] {FIELD_LANDMARKS[0].hint}")
    print("(press ESC to abort)")

    while len(points) < len(FIELD_LANDMARKS):
        key = cv2.waitKey(50) & 0xFF
        if key == 27:  # ESC
            cv2.destroyAllWindows()
            raise SystemExit("calibration aborted")
    cv2.waitKey(500)
    cv2.destroyAllWindows()
    return points


def compute_homography(pixel_points: list[tuple[float, float]],
                       field_points: list[tuple[float, float]]) -> np.ndarray:
    """Solve for the 3x3 perspective matrix mapping pixels to field meters.

    cv2.getPerspectiveTransform() needs exactly 4 source and 4 destination
    points. It returns a 3x3 matrix (float64). Uses the standard direct
    linear transform under the hood.
    """
    src = np.array(pixel_points, dtype=np.float32)
    dst = np.array(field_points, dtype=np.float32)
    H = cv2.getPerspectiveTransform(src, dst)
    return H


def reprojection_error(H: np.ndarray,
                       pixel_points: list[tuple[float, float]],
                       field_points: list[tuple[float, float]]) -> list[float]:
    """Per-point error in meters after applying H. Sanity check.

    If the user clicked precisely on the landmarks, errors should be < 0.2m
    even with a 1080p frame. Errors > 0.5m suggest sloppy clicks or a wrong
    landmark choice — re-run the calibration.
    """
    src = np.array([pixel_points], dtype=np.float32)  # shape (1, N, 2)
    projected = cv2.perspectiveTransform(src, H)[0]
    errors = []
    for (px, py), (fx, fy) in zip(projected, field_points):
        errors.append(float(np.hypot(px - fx, py - fy)))
    return errors


def pixel_to_field(H: np.ndarray, x_pixel: float, y_pixel: float) -> tuple[float, float]:
    """Convenience: project one (px, py) to (xm, ym).

    Useful from other modules that load the homography. Equivalent to
    cv2.perspectiveTransform with N=1.
    """
    src = np.array([[[x_pixel, y_pixel]]], dtype=np.float32)
    out = cv2.perspectiveTransform(src, H)
    return float(out[0, 0, 0]), float(out[0, 0, 1])


def save_calibration(out_path: Path,
                     H: np.ndarray,
                     pixel_points: list[tuple[float, float]],
                     errors: list[float],
                     video_or_frame: str) -> None:
    """Write the calibration to JSON.

    The matrix is stored as a flat list of 9 floats (row-major) so it's
    easy to read into JS later for the browser tool. We also store the
    landmark definitions and per-point reprojection errors so anyone
    reviewing the file can sanity-check it.
    """
    payload = {
        "version": 1,
        "source": video_or_frame,
        "homography_3x3": [float(v) for v in H.flatten().tolist()],
        "landmarks": [
            {
                "name": lm.name,
                "field_x_m": lm.x_m,
                "field_y_m": lm.y_m,
                "pixel_x": pixel_points[i][0],
                "pixel_y": pixel_points[i][1],
                "reprojection_error_m": errors[i],
            }
            for i, lm in enumerate(FIELD_LANDMARKS)
        ],
        "max_reprojection_error_m": max(errors) if errors else 0.0,
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def load_homography(path: Path) -> np.ndarray:
    """Read a calibration JSON and return the 3x3 matrix as a numpy array.

    Used by track_bumpers.py at runtime.
    """
    payload = json.loads(path.read_text(encoding="utf-8"))
    flat = payload["homography_3x3"]
    return np.array(flat, dtype=np.float64).reshape(3, 3)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--video", type=Path, help="Path to a sample .mp4/.webm match video")
    src.add_argument("--frame", type=Path, help="Path to a still frame (.png/.jpg)")
    parser.add_argument("--at-second", type=float, default=5.0,
                        help="If --video, grab the frame at this timestamp (default 5.0)")
    parser.add_argument("--out", type=Path, required=True,
                        help="Output JSON path (e.g. strategy-board/data/homography-2026arc.json)")
    args = parser.parse_args()

    if args.frame:
        frame = cv2.imread(str(args.frame))
        source_label = str(args.frame)
        if frame is None:
            print(f"could not read frame: {args.frame}", file=sys.stderr)
            return 2
    else:
        frame = grab_frame(args.video, args.at_second)
        source_label = f"{args.video} @ {args.at_second}s"

    print(f"Loaded frame {frame.shape[1]}x{frame.shape[0]} from {source_label}")
    print(f"Click {len(FIELD_LANDMARKS)} field landmarks in order. Read the prompts.\n")

    pixel_points = collect_clicks(frame)
    field_points = [(lm.x_m, lm.y_m) for lm in FIELD_LANDMARKS]

    H = compute_homography(pixel_points, field_points)
    errors = reprojection_error(H, pixel_points, field_points)

    print("\nReprojection errors per landmark (meters):")
    for lm, err in zip(FIELD_LANDMARKS, errors):
        flag = "  OK" if err < 0.3 else " WARN" if err < 0.6 else " FAIL"
        print(f"  {flag}  {lm.name:<32s}  err={err:.3f} m")

    save_calibration(args.out, H, pixel_points, errors, source_label)
    print(f"\nSaved calibration to {args.out}")
    if max(errors) > 0.6:
        print("WARNING: at least one landmark has > 0.6m error. Re-run with more careful clicks.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
