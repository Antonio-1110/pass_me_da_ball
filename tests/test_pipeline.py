"""End-to-end run of the threaded pipeline with a fake camera and fake models."""

import time

import numpy as np

from passer.config import Config
from passer.vision.camera import CameraSource, CameraThread
from passer.vision.detection import Box, PersonDetector
from passer.vision.gestures import Gesture
from passer.vision.pipeline import VisionPipeline
from test_gestures import body


class FakeSource(CameraSource):
    def open(self):
        pass

    def read(self):
        time.sleep(0.005)
        return np.zeros((1080, 1920, 3), np.uint8), np.zeros((360, 640, 3), np.uint8)

    def close(self):
        pass


class FakeDetector(PersonDetector):
    def detect(self, bgr):
        return [Box(280, 60, 360, 340, 0.9)]


class FakeModels:
    def __init__(self):
        self.crop_shapes = []

    def run_hands(self, rgb):
        self.crop_shapes.append(rgb.shape)
        return []                 # "too far for hands" -> pose fallback

    def run_pose(self, rgb):
        return body(90, 0)        # left arm out

    def close(self):
        pass


def test_pipeline_emits_confirmed_pose_command():
    cfg = Config()
    cfg.gesture.always_run_pose = False
    cam = CameraThread(FakeSource(cfg.camera)).start()
    models = FakeModels()
    pipe = VisionPipeline(cfg, cam, FakeDetector(), models).start()
    try:
        cmd = None
        deadline = time.monotonic() + 5
        while cmd is None and time.monotonic() < deadline:
            cmd = pipe.get_command()
            time.sleep(0.01)
        assert cmd is not None
        gesture, st = cmd
        assert gesture is Gesture.PASS_LEFT
        assert st.distance_m is not None and st.pan_error_deg == 0.0
        # ROI came from the hi-res frame, then limited to roi_max_side
        h, w, _ = models.crop_shapes[0]
        assert max(h, w) == cfg.gesture.roi_max_side
    finally:
        pipe.stop()
        cam.stop()
