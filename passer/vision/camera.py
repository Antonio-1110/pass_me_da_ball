"""
camera.py

Camera sources that all deliver the same thing: a FramePair holding

  * main  - full-resolution BGR frame (for high-detail gesture crops)
  * lores - small BGR frame, ~640 px wide (for fast body detection)

so nothing downstream cares whether it is running on a Pi Camera Module
(picamera2 hands us both streams from the ISP for free), a USB webcam, a
MacBook webcam, or a recorded video file.

CameraThread runs the source in its own thread and always keeps only the
newest frame, so slow inference never builds up latency.
"""

from __future__ import annotations

import logging
import platform
import threading
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from ..config import CameraConfig

log = logging.getLogger(__name__)


@dataclass
class FramePair:
    main: np.ndarray
    lores: np.ndarray
    timestamp: float
    frame_id: int

    @property
    def scale(self) -> float:
        """Multiply lores pixel coordinates by this to get main coordinates."""
        return self.main.shape[1] / self.lores.shape[1]


def lores_size_for(main_size: tuple, lores_width: int) -> tuple:
    """Same aspect ratio as main, even dimensions (YUV420 requirement)."""
    w, h = main_size
    lw = int(lores_width) & ~1
    lh = int(round(h * lw / w)) & ~1
    return lw, lh


class CameraSource:
    """Base class: open(), read() -> (main_bgr, lores_bgr) or None, close()."""

    def __init__(self, cfg: CameraConfig):
        self.cfg = cfg

    def open(self) -> None: ...
    def read(self) -> Optional[tuple]: ...
    def close(self) -> None: ...

    def _downscale(self, main: np.ndarray) -> np.ndarray:
        h, w = main.shape[:2]
        lw, lh = lores_size_for((w, h), self.cfg.lores_width)
        if lw >= w:
            return main
        return cv2.resize(main, (lw, lh), interpolation=cv2.INTER_AREA)


class Picamera2Source(CameraSource):
    """
    Raspberry Pi camera via libcamera/picamera2 (install with apt:
    `sudo apt install python3-picamera2`, and create the venv with
    --system-site-packages so it is visible).

    The ISP produces the main and lores streams simultaneously from the same
    sensor readout, so the lores stream is free (no CPU resize) and the two
    are pixel-aligned.
    """

    def open(self) -> None:
        from picamera2 import Picamera2
        from libcamera import Transform

        self.cam = Picamera2()
        lores = lores_size_for(self.cfg.main_size, self.cfg.lores_width)
        transform = Transform(hflip=1, vflip=1) if self.cfg.rotate_180 else Transform()
        config = self.cam.create_video_configuration(
            # "RGB888" in picamera2 is BGR byte order -> directly usable by OpenCV.
            main={"size": tuple(self.cfg.main_size), "format": "RGB888"},
            lores={"size": lores, "format": "YUV420"},
            transform=transform,
            controls={"FrameRate": float(self.cfg.fps)},
            buffer_count=4,
        )
        self.cam.configure(config)
        self.cam.start()
        try:  # Camera Module 3 has autofocus; others will just ignore this.
            from libcamera import controls
            self.cam.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        except Exception:
            pass
        log.info("picamera2 started: main=%s lores=%s", self.cfg.main_size, lores)

    def read(self) -> Optional[tuple]:
        request = self.cam.capture_request()
        try:
            main = request.make_array("main")
            yuv = request.make_array("lores")
        finally:
            request.release()
        lores = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
        # The array may be padded to the ISP stride; trim back to the real width.
        lw, _ = lores_size_for(self.cfg.main_size, self.cfg.lores_width)
        return main, lores[:, :lw]

    def close(self) -> None:
        try:
            self.cam.stop()
            self.cam.close()
        except Exception:
            pass


class OpenCVSource(CameraSource):
    """USB / built-in webcams via cv2.VideoCapture (macOS, Linux, Windows)."""

    def open(self) -> None:
        system = platform.system()
        if system == "Darwin":
            # Explicit AVFoundation avoids macOS handing us an iPhone
            # Continuity Camera instead of the built-in one.
            api = cv2.CAP_AVFOUNDATION
        elif system == "Linux":
            api = cv2.CAP_V4L2
        else:
            api = cv2.CAP_ANY
        self.cap = cv2.VideoCapture(self.cfg.index, api)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"Could not open camera index {self.cfg.index}. On macOS check "
                "System Settings -> Privacy & Security -> Camera; run "
                "`python -m passer.tools.list_cameras` to find the right index.")
        if self.cfg.opencv_set_size:
            w, h = self.cfg.main_size
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            self.cap.set(cv2.CAP_PROP_FPS, self.cfg.fps)
        # Keep the driver queue short so we always get a fresh frame.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError("Camera opened but returned no frame (permission "
                               "prompt pending, or wrong device).")
        log.info("OpenCV camera %d: %dx%d", self.cfg.index, frame.shape[1], frame.shape[0])

    def read(self) -> Optional[tuple]:
        ok, main = self.cap.read()
        if not ok or main is None:
            return None
        if self.cfg.rotate_180:
            main = cv2.rotate(main, cv2.ROTATE_180)
        return main, self._downscale(main)

    def close(self) -> None:
        self.cap.release()


class FileSource(CameraSource):
    """Replays a video file (looping) at roughly its native frame rate."""

    def open(self) -> None:
        if not self.cfg.file_path:
            raise ValueError("camera.file_path must be set for backend 'file'")
        self.cap = cv2.VideoCapture(self.cfg.file_path)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open video {self.cfg.file_path}")
        fps = self.cap.get(cv2.CAP_PROP_FPS) or self.cfg.fps
        self._period = 1.0 / fps
        self._next = time.monotonic()

    def read(self) -> Optional[tuple]:
        delay = self._next - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        self._next = max(self._next + self._period, time.monotonic())
        ok, main = self.cap.read()
        if not ok:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, main = self.cap.read()
            if not ok:
                return None
        return main, self._downscale(main)

    def close(self) -> None:
        self.cap.release()


def _picamera2_available() -> bool:
    try:
        import picamera2  # noqa: F401
        return True
    except Exception:
        return False


def make_camera_source(cfg: CameraConfig) -> CameraSource:
    backend = cfg.backend
    if backend == "auto":
        backend = "picamera2" if _picamera2_available() else "opencv"
    log.info("Camera backend: %s", backend)
    return {"picamera2": Picamera2Source,
            "opencv": OpenCVSource,
            "file": FileSource}[backend](cfg)


class CameraThread:
    """Acquisition thread holding only the most recent FramePair."""

    def __init__(self, source: CameraSource):
        self.source = source
        self._cond = threading.Condition()
        self._latest: Optional[FramePair] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.fps = 0.0

    def start(self) -> "CameraThread":
        self.source.open()
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="camera", daemon=True)
        self._thread.start()
        return self

    def _loop(self) -> None:
        frame_id = 0
        t_last = time.monotonic()
        failures = 0
        while self._running:
            try:
                out = self.source.read()
            except Exception:
                log.exception("camera read failed")
                out = None
            if out is None:
                failures += 1
                if failures > 50:
                    log.error("camera: too many consecutive failures, stopping")
                    self._running = False
                time.sleep(0.01)
                continue
            failures = 0
            main, lores = out
            now = time.monotonic()
            frame_id += 1
            dt = now - t_last
            t_last = now
            if dt > 0:
                self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)
            with self._cond:
                self._latest = FramePair(main, lores, now, frame_id)
                self._cond.notify_all()
        with self._cond:
            self._cond.notify_all()

    def get(self, after_id: int = 0, timeout: float = 1.0) -> Optional[FramePair]:
        """Newest frame with frame_id > after_id (waits up to `timeout`)."""
        with self._cond:
            ok = self._cond.wait_for(
                lambda: not self._running or (self._latest is not None
                                              and self._latest.frame_id > after_id),
                timeout=timeout)
            if not ok or self._latest is None or self._latest.frame_id <= after_id:
                return None
            return self._latest

    @property
    def running(self) -> bool:
        return self._running

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self.source.close()
