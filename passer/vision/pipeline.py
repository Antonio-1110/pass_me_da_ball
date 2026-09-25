"""
pipeline.py

The multi-threaded crop-and-infer vision pipeline.

    [camera thread]  grabs frames, keeps only the newest FramePair
          |            (main = full res, lores = ~640 px)
          v
    [inference thread]
       1. person detector on lores                 (every frame)
       2. pick/keep the player (TargetTracker)
       3. distance + pan error from the lores box  (every frame)
       4. every N frames, or sooner when the box is stable:
            crop the padded box out of MAIN (hi-res) -> MediaPipe Hands
                                                     -> MediaPipe Pose (fallback)
          debounce -> confirmed command
          |
          v
    TrackingState (latest, polled by the app) + command queue

The app thread only reads TrackingState and never blocks on inference.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

from ..config import Config
from ..turret import pixel_to_pan_error_deg
from .camera import CameraThread, FramePair
from .detection import Box, PersonDetector, TargetTracker
from .distance import DistanceEstimator
from .gestures import (Gesture, GestureDebouncer, classify_hands, classify_pose,
                       combine)

log = logging.getLogger(__name__)


@dataclass
class TrackingState:
    frame: Optional[FramePair] = None
    box: Optional[Box] = None                 # lores coords
    all_boxes: List[Box] = field(default_factory=list)
    roi: Optional[Tuple[int, int, int, int]] = None   # lores coords of the gesture crop
    distance_m: Optional[float] = None
    distance_reliable: bool = False
    pan_error_deg: Optional[float] = None
    hand_gesture: Gesture = Gesture.NONE
    pose_gesture: Gesture = Gesture.NONE
    gesture: Gesture = Gesture.NONE           # combined raw guess
    gesture_progress: Tuple[Gesture, int] = (Gesture.NONE, 0)
    hand_points: List[List[Tuple[float, float]]] = field(default_factory=list)  # lores px
    pose_points: List[Tuple[float, float, float]] = field(default_factory=list)  # lores px + vis
    infer_ms: float = 0.0
    gesture_ms: float = 0.0
    infer_fps: float = 0.0


def roi_from_box(box: Box, scale: float, main_w: int, main_h: int,
                 pad_x: float, pad_top: float, pad_bottom: float) -> Tuple[int, int, int, int]:
    """Padded, clipped crop rectangle in MAIN-frame pixels."""
    b = box.scaled(scale)
    x1 = int(max(0, b.x1 - pad_x * b.w))
    x2 = int(min(main_w, b.x2 + pad_x * b.w))
    y1 = int(max(0, b.y1 - pad_top * b.h))
    y2 = int(min(main_h, b.y2 + pad_bottom * b.h))
    return x1, y1, x2, y2


class GestureModels:
    """MediaPipe Hands + Pose, run on the high-res ROI."""

    def __init__(self, cfg: Config):
        from .mp_compat import mp_solutions

        sol = mp_solutions()
        g = cfg.gesture
        # static_image_mode=True: crops arrive every few frames from a moving
        # window, so frame-to-frame landmark tracking would be stale anyway.
        self.hands = sol.hands.Hands(
            static_image_mode=True, max_num_hands=2,
            min_detection_confidence=g.hand_min_detection_conf,
            min_tracking_confidence=g.hand_min_tracking_conf)
        self.pose = sol.pose.Pose(
            static_image_mode=True, model_complexity=g.pose_model_complexity,
            min_detection_confidence=0.5)

    def run_hands(self, rgb: np.ndarray):
        res = self.hands.process(rgb)
        if not res.multi_hand_landmarks:
            return []
        labels = res.multi_handedness or []
        out = []
        for i, h in enumerate(res.multi_hand_landmarks):
            label = labels[i].classification[0].label if i < len(labels) else "Right"
            out.append((h.landmark, label))
        return out

    def run_pose(self, rgb: np.ndarray):
        res = self.pose.process(rgb)
        return res.pose_landmarks.landmark if res.pose_landmarks else None

    def close(self) -> None:
        self.hands.close()
        self.pose.close()


class VisionPipeline:
    def __init__(self, cfg: Config, camera: CameraThread, detector: PersonDetector,
                 gesture_models: Optional[GestureModels]):
        self.cfg = cfg
        self.camera = camera
        self.detector = detector
        self.gestures = gesture_models
        self.tracker = TargetTracker(cfg.target)
        self.distance = DistanceEstimator(cfg.distance, cfg.camera.hfov_deg)
        self.debouncer = GestureDebouncer(cfg.gesture)
        self.commands: "queue.Queue[Tuple[Gesture, TrackingState]]" = queue.Queue()

        self._state = TrackingState()
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    # -- public ------------------------------------------------------------
    def start(self) -> "VisionPipeline":
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="inference", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        self.detector.close()
        if self.gestures:
            self.gestures.close()

    def latest(self) -> TrackingState:
        with self._lock:
            return self._state

    def get_command(self) -> Optional[Tuple[Gesture, TrackingState]]:
        try:
            return self.commands.get_nowait()
        except queue.Empty:
            return None

    # -- worker ------------------------------------------------------------
    def _loop(self) -> None:
        last_id = 0
        prev_box: Optional[Box] = None
        since_gesture = 10 ** 6
        fps = 0.0
        t_prev = time.monotonic()
        g = self.cfg.gesture
        state = TrackingState()

        while self._running:
            fp = self.camera.get(after_id=last_id, timeout=0.5)
            if fp is None:
                if not self.camera.running:
                    break
                continue
            last_id = fp.frame_id
            t0 = time.monotonic()

            lh, lw = fp.lores.shape[:2]
            boxes = self.detector.detect(fp.lores)
            box = self.tracker.update(boxes)

            new = TrackingState(frame=fp, box=box, all_boxes=boxes)
            new.distance_m = self.distance.update(box, lw, lh)
            new.distance_reliable = self.distance.reliable
            new.pan_error_deg = (pixel_to_pan_error_deg(box.cx, lw, self.cfg.camera.hfov_deg)
                                 if box else None)

            # Carry forward the last gesture read so the overlay doesn't flicker.
            new.hand_gesture, new.pose_gesture = state.hand_gesture, state.pose_gesture
            new.gesture, new.hand_points, new.pose_points = (
                state.gesture, state.hand_points, state.pose_points)
            new.roi, new.gesture_ms = state.roi, state.gesture_ms

            since_gesture += 1
            stable = box is not None and prev_box is not None and \
                box.motion(prev_box) < g.stable_bbox_px
            due = since_gesture >= g.every_n_frames or \
                (stable and since_gesture >= max(1, g.every_n_frames // 2))

            if box is None:
                new.gesture = Gesture.NONE
                new.hand_points, new.pose_points, new.roi = [], [], None
                self.debouncer.update(Gesture.NONE)
            elif due and self.gestures is not None:
                since_gesture = 0
                tg = time.monotonic()
                self._run_gestures(fp, box, new)
                new.gesture_ms = (time.monotonic() - tg) * 1000.0
                cmd = self.debouncer.update(new.gesture)
                if cmd is not None:
                    log.info("GESTURE CONFIRMED: %s (distance=%s)", cmd.value,
                             f"{new.distance_m:.2f} m" if new.distance_m else "?")
                    self.commands.put((cmd, new))
            new.gesture_progress = self.debouncer.progress
            prev_box = box

            now = time.monotonic()
            new.infer_ms = (now - t0) * 1000.0
            dt = now - t_prev
            t_prev = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 / dt
            new.infer_fps = fps
            state = new
            with self._lock:
                self._state = new

    def _run_gestures(self, fp: FramePair, box: Box, st: TrackingState) -> None:
        g = self.cfg.gesture
        mh, mw = fp.main.shape[:2]
        s = fp.scale
        x1, y1, x2, y2 = roi_from_box(box, s, mw, mh, g.roi_pad_x, g.roi_pad_top, g.roi_pad_bottom)
        st.roi = (int(x1 / s), int(y1 / s), int(x2 / s), int(y2 / s))
        crop = fp.main[y1:y2, x1:x2]
        if crop.size == 0:
            return
        ch, cw = crop.shape[:2]
        k = min(1.0, g.roi_max_side / max(ch, cw))
        if k < 1.0:
            crop = cv2.resize(crop, (int(cw * k), int(ch * k)), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)

        def to_lores(nx: float, ny: float) -> Tuple[float, float]:
            return (x1 + nx * cw) / s, (y1 + ny * ch) / s

        hands = self.gestures.run_hands(rgb)
        st.hand_gesture = classify_hands(hands, g)
        st.hand_points = [[to_lores(p.x, p.y) for p in lm] for lm, _ in hands]

        st.pose_gesture = Gesture.NONE
        st.pose_points = []
        # Distance fallback: hands not found (too far / too blurry), or always.
        if g.always_run_pose or not hands:
            pose = self.gestures.run_pose(rgb)
            if pose is not None:
                st.pose_gesture = classify_pose(pose, g)
                st.pose_points = [(*to_lores(p.x, p.y), p.visibility) for p in pose]

        st.gesture = combine(st.hand_gesture, st.pose_gesture)
