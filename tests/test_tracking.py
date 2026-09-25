import pytest

from passer.config import DistanceConfig, TargetConfig
from passer.turret import focal_length_px, pixel_to_pan_error_deg
from passer.vision.camera import lores_size_for
from passer.vision.detection import Box, TargetTracker
from passer.vision.distance import DistanceEstimator
from passer.vision.pipeline import roi_from_box


def test_pan_error_sign_and_edge():
    assert pixel_to_pan_error_deg(320, 640, 66) == 0
    assert pixel_to_pan_error_deg(640, 640, 66) == pytest.approx(33)
    assert pixel_to_pan_error_deg(0, 640, 66) == pytest.approx(-33)


def test_distance_pinhole():
    est = DistanceEstimator(DistanceConfig(person_height_m=1.75, smoothing=1.0), 66)
    f = focal_length_px(640, 66)
    h_px = f * 1.75 / 5.0          # a person 5 m away
    d = est.update(Box(300, 100, 340, 100 + h_px), 640, 480)
    assert d == pytest.approx(5.0)
    assert est.reliable


def test_distance_holds_when_truncated():
    est = DistanceEstimator(DistanceConfig(smoothing=1.0), 66)
    first = est.update(Box(300, 100, 340, 300), 640, 480)
    held = est.update(Box(300, 0, 340, 479), 640, 480)
    assert held == first and not est.reliable


def test_tracker_sticks_to_locked_player():
    tr = TargetTracker(TargetConfig())
    player = Box(100, 100, 200, 400)
    bigger_bystander = Box(400, 50, 600, 470)
    assert tr.update([player], now=0.0) == player
    moved = Box(110, 100, 210, 400)
    assert tr.update([bigger_bystander, moved], now=0.1) == moved


def test_tracker_releases_after_timeout():
    tr = TargetTracker(TargetConfig(lost_after_sec=0.5))
    tr.update([Box(100, 100, 200, 400)], now=0.0)
    other = Box(400, 50, 600, 470)
    assert tr.update([other], now=0.2) is None
    assert tr.update([other], now=1.0) == other


def test_lores_size_even_and_same_aspect():
    assert lores_size_for((1920, 1080), 640) == (640, 360)
    assert lores_size_for((2304, 1296), 640) == (640, 360)
    w, h = lores_size_for((1640, 1232), 640)
    assert w % 2 == 0 and h % 2 == 0


def test_roi_scaled_and_clipped():
    x1, y1, x2, y2 = roi_from_box(Box(0, 10, 100, 350), 3.0, 1920, 1080, 0.35, 0.25, 0.05)
    assert x1 == 0 and y1 == 0
    assert x2 == int(300 + 0.35 * 300)
    assert y2 == 1080
