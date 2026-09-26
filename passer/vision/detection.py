"""
detection.py

Step 1 of the crop-and-infer pipeline: find people in the LOW-RES stream.

Backends (DetectorConfig.backend):
  * "yolo"            - Ultralytics YOLOv8n person class. Handles several
                        people and partial bodies. On a Pi 5 expect ~8-12 fps
                        at imgsz=320 with PyTorch, 20+ fps after exporting to
                        NCNN (see config.py).
  * "mediapipe_pose"  - single-person, box derived from pose landmarks.
                        Lighter than YOLO but cannot handle a crowd.

Also contains TargetTracker, which picks ONE player to follow when several
people are visible (the "multi human detection problem" from note.txt).
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from ..config import DetectorConfig, TargetConfig

log = logging.getLogger(__name__)

YOLO_PERSON_CLASS_ID = 0  # COCO class 0 = "person"


@dataclass
class Box:
    x1: float
    y1: float
    x2: float
    y2: float
    conf: float = 1.0

    @property
    def w(self) -> float:
        return self.x2 - self.x1

    @property
    def h(self) -> float:
        return self.y2 - self.y1

    @property
    def cx(self) -> float:
        return (self.x1 + self.x2) / 2.0

    @property
    def cy(self) -> float:
        return (self.y1 + self.y2) / 2.0

    @property
    def area(self) -> float:
        return max(0.0, self.w) * max(0.0, self.h)

    def scaled(self, s: float) -> "Box":
        return Box(self.x1 * s, self.y1 * s, self.x2 * s, self.y2 * s, self.conf)

    def iou(self, other: "Box") -> float:
        ix1, iy1 = max(self.x1, other.x1), max(self.y1, other.y1)
        ix2, iy2 = min(self.x2, other.x2), min(self.y2, other.y2)
        inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
        union = self.area + other.area - inter
        return inter / union if union > 0 else 0.0

    def motion(self, other: "Box") -> float:
        """How much the box changed (centre + size), in pixels."""
        return max(abs(self.cx - other.cx), abs(self.cy - other.cy),
                   abs(self.w - other.w), abs(self.h - other.h))


class PersonDetector:
    def detect(self, bgr: np.ndarray) -> List[Box]:
        raise NotImplementedError

    def close(self) -> None:
        pass


class YoloPersonDetector(PersonDetector):
    def __init__(self, cfg: DetectorConfig):
        from ultralytics import YOLO

        self.cfg = cfg
        self.model = YOLO(cfg.yolo_model, task="detect")
        device = cfg.device
        if device is None:
            try:
                import torch
                device = "mps" if torch.backends.mps.is_available() else "cpu"
            except Exception:
                device = "cpu"
        self.device = device
        log.info("YOLO %s on %s (imgsz=%d)", cfg.yolo_model, device, cfg.yolo_imgsz)

    def detect(self, bgr: np.ndarray) -> List[Box]:
        results = self.model.predict(
            bgr,
            classes=[YOLO_PERSON_CLASS_ID],
            conf=self.cfg.yolo_conf,
            imgsz=self.cfg.yolo_imgsz,
            device=self.device,
            verbose=False,
        )
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return []
        xyxy = boxes.xyxy.cpu().numpy()
        conf = boxes.conf.cpu().numpy()
        return [Box(*map(float, b), conf=float(c)) for b, c in zip(xyxy, conf)]


class MediaPipePoseDetector(PersonDetector):
    def __init__(self, cfg: DetectorConfig):
        from .mp_compat import mp_solutions

        self._pose = mp_solutions().pose.Pose(
            static_image_mode=False, model_complexity=0,
            min_detection_confidence=0.5, min_tracking_confidence=0.5)

    def detect(self, bgr: np.ndarray) -> List[Box]:
        import cv2

        h, w = bgr.shape[:2]
        res = self._pose.process(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        if not res.pose_landmarks:
            return []
        pts = [(lm.x, lm.y) for lm in res.pose_landmarks.landmark if lm.visibility > 0.3]
        if len(pts) < 4:
            return []
        xs, ys = zip(*pts)
        # Landmarks sit inside the body outline; pad a little to approximate
        # a full-body box (head top, feet).
        x1, x2 = min(xs) * w, max(xs) * w
        y1, y2 = min(ys) * h, max(ys) * h
        px, py = 0.1 * (x2 - x1), 0.1 * (y2 - y1)
        return [Box(max(0, x1 - px), max(0, y1 - py), min(w, x2 + px), min(h, y2 + py))]

    def close(self) -> None:
        self._pose.close()


def make_person_detector(cfg: DetectorConfig) -> PersonDetector:
    if cfg.backend == "yolo":
        return YoloPersonDetector(cfg)
    if cfg.backend == "mediapipe_pose":
        return MediaPipePoseDetector(cfg)
    raise ValueError(f"Unknown detector backend: {cfg.backend}")


class TargetTracker:
    """
    Chooses which person is "the player".

    * No lock yet: pick the largest box (closest person).
    * Locked: keep the box with the best IoU against the last position, as
      long as it overlaps enough. Bystanders walking through are ignored.
    * Lost for longer than lost_after_sec: release the lock.
    """

    def __init__(self, cfg: TargetConfig):
        self.cfg = cfg
        self.box: Optional[Box] = None
        self._last_seen = 0.0

    def reset(self) -> None:
        self.box = None

    def update(self, boxes: List[Box], now: Optional[float] = None) -> Optional[Box]:
        now = time.monotonic() if now is None else now
        if self.box is not None and now - self._last_seen > self.cfg.lost_after_sec:
            self.box = None

        if not boxes:
            return None

        if self.box is None:
            chosen = max(boxes, key=lambda b: b.area)
        else:
            best = max(boxes, key=lambda b: b.iou(self.box))
            if best.iou(self.box) < self.cfg.min_iou_to_keep:
                return None          # our player isn't in this frame
            chosen = best

        self.box = chosen
        self._last_seen = now
        return chosen
