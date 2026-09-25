"""
turret.py

Keeps the player centred. The camera rides on the pan axis, so the pixel
offset of the player's box from the image centre IS the pan error:

    error_deg = atan((cx - W/2) / focal_px)

A PID turns that error into a pan velocity, which is slew-limited and
integrated into an absolute angle setpoint for the PanAxis. The slew limit
and deadband keep the camera from jerking (motion blur ruins detection).

For a lead pass (PASS_LEFT / PASS_RIGHT) a lateral offset is added so the
launcher points to where the player should run to, not where they are.
"""

from __future__ import annotations

import math
from typing import Optional

from .config import TurretConfig
from .hardware.pan_axis import PanAxis


def focal_length_px(image_width: int, hfov_deg: float) -> float:
    return (image_width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


def pixel_to_pan_error_deg(cx: float, image_width: int, hfov_deg: float) -> float:
    """Positive = player is right of centre (turret should turn right)."""
    f = focal_length_px(image_width, hfov_deg)
    return math.degrees(math.atan((cx - image_width / 2.0) / f))


class PID:
    def __init__(self, kp: float, ki: float, kd: float, out_limit: float):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self._i = 0.0
        self._prev: Optional[float] = None

    def reset(self) -> None:
        self._i = 0.0
        self._prev = None

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0
        self._i += error * dt
        # anti-windup: keep the integral term alone within the output limit
        if self.ki > 0:
            lim = self.out_limit / self.ki
            self._i = max(-lim, min(lim, self._i))
        d = 0.0 if self._prev is None else (error - self._prev) / dt
        self._prev = error
        out = self.kp * error + self.ki * self._i + self.kd * d
        return max(-self.out_limit, min(self.out_limit, out))


class TurretController:
    def __init__(self, axis: PanAxis, cfg: TurretConfig):
        self.axis = axis
        self.cfg = cfg
        self.pid = PID(cfg.kp, cfg.ki, cfg.kd, cfg.max_speed_dps)
        self.setpoint = axis.angle()
        self.lead_deg = 0.0          # extra offset for lead passes
        self.last_error_deg: Optional[float] = None

    def set_lead(self, lateral_m: float, distance_m: Optional[float]) -> None:
        """
        Aim `lateral_m` to the side of the player. Positive = to the right as
        seen from the machine, which is the PLAYER'S LEFT (they face us).
        """
        if not lateral_m or not distance_m:
            self.lead_deg = 0.0
        else:
            self.lead_deg = math.degrees(math.atan2(lateral_m, distance_m))

    def update(self, error_deg: Optional[float], dt: float) -> float:
        """
        error_deg: pan error of the player (None if no player -> hold still).
        Returns the new absolute angle setpoint.
        """
        if error_deg is None:
            self.pid.reset()
            self.last_error_deg = None
            return self.setpoint

        # Error of the *launcher* relative to where we want it to point.
        err = error_deg - self.cfg.launcher_offset_deg + self.lead_deg
        self.last_error_deg = err
        if abs(err) < self.cfg.deadband_deg:
            err = 0.0
        speed = self.pid.update(err, dt)
        self.setpoint = self.axis.clamp(self.setpoint + speed * dt)
        self.axis.move_to(self.setpoint)
        return self.setpoint

    def on_target(self) -> bool:
        return (self.last_error_deg is not None
                and abs(self.last_error_deg) <= self.cfg.on_target_deg)
