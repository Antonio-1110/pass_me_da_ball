"""
distance.py

Monocular range estimate from the person's box height (pinhole model):

    distance = focal_px * person_height_m / box_height_px

Good to roughly +/-10% when the whole body is in frame, which is plenty for
picking launch speed at 2-10 m. If the box touches the top or bottom edge
the person is cut off, the box is too short and the distance would be
over-estimated, so such frames are marked unreliable and the last good
estimate is held.
"""

from __future__ import annotations

from typing import Optional

from ..config import DistanceConfig
from ..turret import focal_length_px
from .detection import Box

EDGE_MARGIN_PX = 3


class DistanceEstimator:
    def __init__(self, cfg: DistanceConfig, hfov_deg: float):
        self.cfg = cfg
        self.hfov_deg = hfov_deg
        self.value: Optional[float] = None
        self.reliable = False

    def raw(self, box: Box, image_w: int, image_h: int) -> float:
        # Assumes square pixels, so the horizontal focal length applies vertically.
        f = focal_length_px(image_w, self.hfov_deg)
        return f * self.cfg.person_height_m / max(box.h, 1.0)

    def update(self, box: Optional[Box], image_w: int, image_h: int) -> Optional[float]:
        if box is None:
            self.reliable = False
            return self.value
        truncated = box.y1 <= EDGE_MARGIN_PX or box.y2 >= image_h - EDGE_MARGIN_PX
        if truncated:
            self.reliable = False
            return self.value
        d = min(self.cfg.max_m, max(self.cfg.min_m, self.raw(box, image_w, image_h)))
        a = self.cfg.smoothing
        self.value = d if self.value is None else a * d + (1 - a) * self.value
        self.reliable = True
        return self.value
