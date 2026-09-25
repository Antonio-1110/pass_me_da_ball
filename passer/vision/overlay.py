"""Debug drawing on the lores frame (only used when a display is attached)."""

from __future__ import annotations

from typing import Iterable, List

import cv2
import numpy as np

from .pipeline import TrackingState

# Pose skeleton edges used for drawing (subset: arms + torso).
POSE_EDGES = [(11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
              (11, 23), (12, 24), (23, 24)]
HAND_EDGES = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
              (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15),
              (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17)]


def _pt(p) -> tuple:
    return int(p[0]), int(p[1])


def draw(st: TrackingState, lines: Iterable[str] = (), mirror: bool = False) -> np.ndarray:
    img = st.frame.lores.copy()
    for b in st.all_boxes:
        cv2.rectangle(img, _pt((b.x1, b.y1)), _pt((b.x2, b.y2)), (90, 90, 90), 1)
    if st.box is not None:
        b = st.box
        cv2.rectangle(img, _pt((b.x1, b.y1)), _pt((b.x2, b.y2)), (0, 255, 0), 2)
    if st.roi is not None:
        x1, y1, x2, y2 = st.roi
        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 160, 0), 1)
    for pts in st.hand_points:
        for a, c in HAND_EDGES:
            cv2.line(img, _pt(pts[a]), _pt(pts[c]), (0, 200, 255), 1)
    if st.pose_points:
        pp = st.pose_points
        for a, c in POSE_EDGES:
            if pp[a][2] > 0.5 and pp[c][2] > 0.5:
                cv2.line(img, _pt(pp[a]), _pt(pp[c]), (255, 0, 255), 2)

    h, w = img.shape[:2]
    cv2.line(img, (w // 2, 0), (w // 2, h), (60, 60, 200), 1)
    if mirror:
        img = cv2.flip(img, 1)

    text: List[str] = [
        f"infer {st.infer_fps:4.1f} fps  det {st.infer_ms:4.0f} ms  gest {st.gesture_ms:4.0f} ms",
        "dist " + (f"{st.distance_m:4.1f} m" if st.distance_m else "  -- ")
        + ("" if st.distance_reliable else " (held)")
        + ("   pan err " + (f"{st.pan_error_deg:+5.1f} deg" if st.pan_error_deg is not None else "--")),
        f"gesture {st.gesture.value}  (hand {st.hand_gesture.value}, pose {st.pose_gesture.value}) "
        f"x{st.gesture_progress[1]}",
        *lines,
    ]
    for i, t in enumerate(text):
        y = 18 + i * 18
        cv2.putText(img, t, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(img, t, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return img
