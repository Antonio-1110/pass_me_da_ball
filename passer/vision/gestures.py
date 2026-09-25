"""
gestures.py

Step 2 of the crop-and-infer pipeline: read the player's request from the
HIGH-RES region of interest around their body.

Two classifiers, both working on landmark lists so they can be unit-tested
without cameras or models:

  * Hands (close range): both palms open, fingers up, facing the camera
      -> CHEST pass  (the gesture prototyped in "webcam tracker test/").
  * Pose (fallback, 6-8 m where fingers are just a few blurry pixels):
      left arm out sideways ~90 deg, right arm down -> PASS_LEFT
      right arm out sideways ~90 deg, left arm down -> PASS_RIGHT
      both arms raised above the head               -> LOB

LEFT / RIGHT always mean the PLAYER's own left / right. MediaPipe Pose
labels landmarks from the person's point of view when it is given an
unmirrored image, which is why inference never runs on a flipped frame.

GestureDebouncer turns noisy per-frame guesses into a single confirmed
command.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Sequence

from ..config import GestureConfig


class Gesture(str, Enum):
    NONE = "none"
    CHEST = "chest"            # both open palms
    PASS_LEFT = "pass_left"    # lead pass to the player's left
    PASS_RIGHT = "pass_right"  # lead pass to the player's right
    LOB = "lob"                # high-arc lob

    @property
    def profile(self) -> str:
        """Which LauncherConfig.profiles entry this gesture fires."""
        return "lob" if self is Gesture.LOB else "chest"

    @property
    def lead_sign(self) -> int:
        """
        +1 = aim to the machine's right (player's left), -1 = the other way.
        """
        return {Gesture.PASS_LEFT: 1, Gesture.PASS_RIGHT: -1}.get(self, 0)


# ---------------------------------------------------------------------------
# Landmark indices (MediaPipe numbering, fixed by the models)
# ---------------------------------------------------------------------------
# Hands
H_WRIST, H_THUMB_IP, H_THUMB_TIP = 0, 3, 4
H_INDEX_MCP, H_PINKY_MCP = 5, 17
H_TIPS = (8, 12, 16, 20)
H_PIPS = (6, 10, 14, 18)

# Pose
P_NOSE = 0
P_L_SHOULDER, P_R_SHOULDER = 11, 12
P_L_ELBOW, P_R_ELBOW = 13, 14
P_L_WRIST, P_R_WRIST = 15, 16
P_L_HIP, P_R_HIP = 23, 24


# ---------------------------------------------------------------------------
# Hand classifier
# ---------------------------------------------------------------------------
PALM_RAISED_Y_THRESHOLD = 0.65      # wrist in upper part of the crop
PALM_TIP_ABOVE_WRIST_MARGIN = 0.02
PALM_FACING_CROSS_EPS = 1e-4


def _d2(a, b) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def is_open_palm(lm: Sequence) -> bool:
    """
    All four fingers extended (tip farther from wrist than PIP), thumb
    extended, and the hand raised with fingers pointing up. `lm` is the
    21-landmark list (objects with .x .y .z in crop-normalised coordinates).
    """
    wrist = lm[H_WRIST]
    extended = sum(_d2(lm[t], wrist) > _d2(lm[p], wrist) for t, p in zip(H_TIPS, H_PIPS))
    tips_above_pips = sum(lm[t].y < lm[p].y for t, p in zip(H_TIPS, H_PIPS))
    tips_above_wrist = sum(lm[t].y < wrist.y for t in H_TIPS)
    mean_tip_y = sum(lm[t].y for t in H_TIPS) / len(H_TIPS)
    thumb_extended = _d2(lm[H_THUMB_TIP], wrist) > _d2(lm[H_THUMB_IP], wrist)

    return (extended == 4 and thumb_extended
            and wrist.y < PALM_RAISED_Y_THRESHOLD
            and tips_above_pips >= 3 and tips_above_wrist >= 3
            and mean_tip_y < wrist.y - PALM_TIP_ABOVE_WRIST_MARGIN)


def palm_facing_camera(lm: Sequence, handedness_label: str,
                       angle_threshold_deg: float = 50.0,
                       invert: bool = False) -> tuple:
    """
    Palm-vs-back test from the normal of the (wrist, index MCP, pinky MCP)
    plane. Returns (facing, angle_deg or None).

    Mirroring the image flips both the cross-product sign and MediaPipe's
    handedness label, so the result is the same for mirrored and unmirrored
    input.
    """
    w, i, p = lm[H_WRIST], lm[H_INDEX_MCP], lm[H_PINKY_MCP]
    v1 = (i.x - w.x, i.y - w.y, i.z - w.z)
    v2 = (p.x - w.x, p.y - w.y, p.z - w.z)
    nx = v1[1] * v2[2] - v1[2] * v2[1]
    ny = v1[2] * v2[0] - v1[0] * v2[2]
    nz = v1[0] * v2[1] - v1[1] * v2[0]

    cross_z = -nz if handedness_label == "Left" else nz
    if abs(cross_z) < PALM_FACING_CROSS_EPS:
        return False, None
    norm = math.sqrt(nx * nx + ny * ny + nz * nz)
    if norm < 1e-8:
        return False, None
    angle = math.degrees(math.acos(min(1.0, abs(nz) / norm)))
    if angle > angle_threshold_deg:
        return False, angle
    facing = cross_z > 0
    return (not facing if invert else facing), angle


def classify_hands(hands: Sequence[tuple], cfg: GestureConfig) -> Gesture:
    """
    hands: [(landmarks, handedness_label), ...] from MediaPipe Hands.
    CHEST only if TWO hands are each an open palm facing the camera.
    """
    good = 0
    for lm, label in hands:
        if is_open_palm(lm):
            facing, _ = palm_facing_camera(lm, label, cfg.palm_angle_threshold_deg,
                                           cfg.invert_palm_orientation)
            good += facing
    return Gesture.CHEST if good >= 2 else Gesture.NONE


# ---------------------------------------------------------------------------
# Pose classifier (distance fallback)
# ---------------------------------------------------------------------------
def _angle_between(ax, ay, bx, by) -> float:
    na, nb = math.hypot(ax, ay), math.hypot(bx, by)
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    c = (ax * bx + ay * by) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def arm_elevation_deg(lm: Sequence, shoulder: int, wrist: int, hip: int) -> float:
    """
    Angle between the torso (shoulder -> hip) and the arm (shoulder -> wrist):
    0 = hanging down, 90 = straight out sideways, 180 = straight up.
    Works in image coordinates, so it is independent of distance.
    """
    s, w, h = lm[shoulder], lm[wrist], lm[hip]
    return _angle_between(h.x - s.x, h.y - s.y, w.x - s.x, w.y - s.y)


def classify_pose(lm: Sequence, cfg: GestureConfig) -> Gesture:
    """lm: 33 pose landmarks (.x .y .visibility)."""
    need = (P_L_SHOULDER, P_R_SHOULDER, P_L_WRIST, P_R_WRIST, P_L_HIP, P_R_HIP)
    if any(getattr(lm[i], "visibility", 1.0) < cfg.min_landmark_visibility for i in need):
        return Gesture.NONE

    left = arm_elevation_deg(lm, P_L_SHOULDER, P_L_WRIST, P_L_HIP)
    right = arm_elevation_deg(lm, P_R_SHOULDER, P_R_WRIST, P_R_HIP)

    head_y = lm[P_NOSE].y
    both_up = (left >= cfg.arm_overhead_min_deg and right >= cfg.arm_overhead_min_deg
               and lm[P_L_WRIST].y < head_y and lm[P_R_WRIST].y < head_y)
    if both_up:
        return Gesture.LOB

    def side(a):
        return cfg.arm_side_min_deg <= a <= cfg.arm_side_max_deg

    def down(a):
        return a <= cfg.arm_down_max_deg

    if side(left) and down(right):
        return Gesture.PASS_LEFT
    if side(right) and down(left):
        return Gesture.PASS_RIGHT
    return Gesture.NONE


def combine(hand_gesture: Gesture, pose_gesture: Gesture) -> Gesture:
    """
    Pose gestures are big deliberate arm movements, so they win; otherwise
    fall back to the hand gesture.
    """
    return pose_gesture if pose_gesture is not Gesture.NONE else hand_gesture


# ---------------------------------------------------------------------------
# Debounce
# ---------------------------------------------------------------------------
class GestureDebouncer:
    """
    Emits a gesture once it has been seen `confirm_count` consecutive times,
    then stays silent for `cooldown_sec` (and until the gesture changes), so
    one held pose = one pass.
    """

    def __init__(self, cfg: GestureConfig):
        self.cfg = cfg
        self._candidate = Gesture.NONE
        self._count = 0
        self._blocked_until = 0.0
        self._last_fired: Optional[Gesture] = None

    def update(self, g: Gesture, now: Optional[float] = None) -> Optional[Gesture]:
        now = time.monotonic() if now is None else now
        if g is not self._candidate:
            self._candidate, self._count = g, 0
        self._count += 1

        if g is Gesture.NONE:
            self._last_fired = None     # released -> next gesture may fire
            return None
        if now < self._blocked_until or g is self._last_fired:
            return None
        if self._count >= self.cfg.confirm_count:
            self._last_fired = g
            self._blocked_until = now + self.cfg.cooldown_sec
            return g
        return None

    @property
    def progress(self) -> tuple:
        return self._candidate, self._count
