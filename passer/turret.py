"""
turret.py

Two-axis pan control:

    launcher turret (stepper)  world angle L        slow, heavy, LEADS the player
      └─ camera servo          relative angle C     fast, light, keeps player in frame

    camera heading H = L + C
    player bearing   = H(at frame time) + pixel offset in the image

Each tick (~30 Hz):
  1. OBSERVE: a new vision result gives a world bearing. The camera heading
     is looked up at the FRAME's timestamp (AngleHistory), not "now", so the
     camera's own motion during inference latency doesn't corrupt it. The
     bearing (plus distance, when known) feeds the filters in prediction.py.
  2. LAUNCHER: target = aim override for a pending shot, or the predicted
     lead angle while tracking. Followed with speed/accel limits.
  3. CAMERA: target (relative) = predicted bearing now - launcher angle now.
     Subtracting the launcher angle is the feed-forward: when the turret
     swings, the servo counter-rotates, so the camera stays on the player.
     Speed/accel-limited and deadbanded to avoid motion blur and jitter.

An axis with no motor yet (physical=False) is treated as fixed at 0 for the
heading, so the whole stack still works with the camera and/or turret
unpowered; the commanded angles are still computed and shown.
"""

from __future__ import annotations

import bisect
import math
from collections import deque
from typing import Optional, Tuple

from .aiming import tracking_angle
from .config import CameraAxisConfig, LauncherAxisConfig, PredictionConfig
from .hardware.pan_axis import PanAxis
from .prediction import BearingTracker, PlayerEstimator


def focal_length_px(image_width: int, hfov_deg: float) -> float:
    return (image_width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


def pixel_to_pan_error_deg(cx: float, image_width: int, hfov_deg: float) -> float:
    """Positive = player is right of the image centre."""
    f = focal_length_px(image_width, hfov_deg)
    return math.degrees(math.atan((cx - image_width / 2.0) / f))


class MotionProfile:
    """
    Follows a (possibly moving) target position with velocity and
    acceleration limits, braking early so it doesn't overshoot.

    The target's own velocity is estimated and fed forward, so a smoothly
    moving target is followed with ~zero lag instead of trailing behind by
    the braking distance. Sudden jumps (faster than max_speed) are treated as
    new set-points, not motion.
    """

    def __init__(self, max_speed: float, max_accel: float, pos: float = 0.0):
        self.vmax, self.amax = max_speed, max_accel
        self.pos, self.vel = pos, 0.0
        self._prev_target: Optional[float] = None
        self.target_vel = 0.0

    def update(self, target: float, dt: float) -> float:
        if dt <= 0:
            return self.pos
        if self._prev_target is not None:
            tv = (target - self._prev_target) / dt
            if abs(tv) > self.vmax:
                tv = 0.0                           # a jump, not motion
            self.target_vel += 0.3 * (tv - self.target_vel)
        self._prev_target = target

        err = target - self.pos
        # Tracking error with the target's own motion this tick removed.
        e = err - self.target_vel * dt
        # Fastest closing speed (relative to the target) from which we can
        # still brake to a stop in discrete steps of dt without overshooting.
        ad = self.amax * dt
        v_brake = math.copysign(
            min(self.vmax, ad * (math.sqrt(0.25 + 2.0 * abs(e) / (ad * dt)) - 0.5)), e)
        if abs(err / dt - self.target_vel) <= abs(v_brake) + 1e-9:
            v_des = err / dt                       # close: land on the target this step
        else:
            v_des = self.target_vel + v_brake
        v_des = max(-self.vmax, min(self.vmax, v_des))
        self.vel += max(-self.amax * dt, min(self.amax * dt, v_des - self.vel))
        self.pos += self.vel * dt
        return self.pos


class AngleHistory:
    """Recent (t, launcher, camera) effective angles, for frame-time lookups."""

    def __init__(self, keep_s: float = 1.0):
        self.keep_s = keep_s
        self._t: deque = deque()
        self._v: deque = deque()

    def add(self, t: float, launcher: float, camera: float) -> None:
        self._t.append(t)
        self._v.append((launcher, camera))
        while self._t and t - self._t[0] > self.keep_s:
            self._t.popleft()
            self._v.popleft()

    def at(self, t: float) -> Optional[Tuple[float, float]]:
        if not self._t:
            return None
        ts = list(self._t)
        i = bisect.bisect_left(ts, t)
        if i <= 0:
            return self._v[0]
        if i >= len(ts):
            return self._v[-1]
        t0, t1 = ts[i - 1], ts[i]
        (l0, c0), (l1, c1) = self._v[i - 1], self._v[i]
        f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return l0 + f * (l1 - l0), c0 + f * (c1 - c0)


class TwoAxisTurret:
    def __init__(self, camera: PanAxis, launcher: PanAxis, cam_cfg: CameraAxisConfig,
                 lau_cfg: LauncherAxisConfig, pred_cfg: PredictionConfig):
        self.camera, self.launcher = camera, launcher
        self.cam_cfg, self.lau_cfg, self.pred_cfg = cam_cfg, lau_cfg, pred_cfg
        self.estimator = PlayerEstimator(pred_cfg)
        self.bearing = BearingTracker(pred_cfg)
        self.history = AngleHistory()
        self.cam_profile = MotionProfile(cam_cfg.max_speed_dps, cam_cfg.max_accel_dps2,
                                         camera.angle())
        self.lau_profile = MotionProfile(lau_cfg.max_speed_dps, lau_cfg.max_accel_dps2,
                                         launcher.angle())
        self.cam_target = camera.angle()
        self.launcher_target: Optional[float] = None
        self.aim_override: Optional[float] = None
        self._last_frame_id = -1
        self.last_bearing: Optional[float] = None

    # -- geometry ------------------------------------------------------------
    def _eff(self, axis: PanAxis) -> float:
        return axis.angle() if axis.physical else 0.0

    def camera_heading(self) -> float:
        """World heading of the camera optical axis right now."""
        return self._eff(self.launcher) + self._eff(self.camera)

    def heading_at(self, t: float) -> float:
        h = self.history.at(t)
        return self.camera_heading() if h is None else h[0] + h[1]

    # -- inputs --------------------------------------------------------------
    def observe(self, frame_id: int, t_frame: float, pan_error_deg: Optional[float],
                distance_m: Optional[float], reliable: bool) -> Optional[float]:
        """Feed one vision result (once per frame). Returns the world bearing."""
        if frame_id == self._last_frame_id or pan_error_deg is None:
            return None
        self._last_frame_id = frame_id
        b = self.heading_at(t_frame) + pan_error_deg
        self.last_bearing = b
        self.bearing.update(t_frame, b)
        if distance_m:
            self.estimator.update(t_frame, b, distance_m, reliable)
        return b

    def set_aim(self, angle_deg: Optional[float]) -> None:
        """Override the launcher target (a shot is being aimed). None = resume tracking."""
        self.aim_override = angle_deg

    # -- control ---------------------------------------------------------------
    def update(self, now: float, dt: float) -> None:
        # Limits are re-read every tick so they can be tuned live.
        self.cam_profile.vmax = self.cam_cfg.max_speed_dps
        self.cam_profile.amax = self.cam_cfg.max_accel_dps2
        self.lau_profile.vmax = self.lau_cfg.max_speed_dps
        self.lau_profile.amax = self.lau_cfg.max_accel_dps2
        if self.estimator.stale(now):
            self.estimator.reset()

        # Launcher: pending shot > predictive tracking > hold.
        target = self.aim_override
        if target is None and self.lau_cfg.track_between_shots and self.estimator.active:
            target = tracking_angle(self.estimator, now, self.lau_cfg, self.pred_cfg)
        if target is not None:
            self.launcher_target = self.launcher.clamp(target)
        if self.launcher_target is not None:
            self.launcher.move_to(self.lau_profile.update(self.launcher_target, dt))

        # Camera: keep the player centred, counter-rotating against the turret.
        b = self.bearing.at(now)
        if b is not None:
            want = self.camera.clamp(b - self._eff(self.launcher))
            if abs(want - self.cam_target) > self.cam_cfg.deadband_deg:
                self.cam_target = want
        self.camera.move_to(self.cam_profile.update(self.cam_target, dt))

        self.history.add(now, self._eff(self.launcher), self._eff(self.camera))

    # -- status --------------------------------------------------------------
    def aim_error(self, target_deg: float) -> float:
        return self.launcher.angle() - target_deg

    def on_target(self, target_deg: float) -> bool:
        return abs(self.aim_error(target_deg)) <= self.lau_cfg.on_target_deg

    def close(self) -> None:
        self.camera.close()
        self.launcher.close()
