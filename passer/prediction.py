"""
prediction.py

Where is the player, how fast are they moving, and where will they be?

World frame (top view), origin at the turret axis:

        +y  (straight ahead of the machine, bearing 0)
         ^
         |   player at bearing b, distance d:
         |      x = d sin b,  y = d cos b
         +------> +x  (right as seen from the machine, bearing +90)

Two filters, because the two axes need different things:

  * PlayerEstimator - constant-velocity Kalman filter on (x, y). Needs the
    distance. Drives the LAUNCHER, which must aim where the player will be
    when the ball arrives.
  * BearingTracker  - alpha-beta filter on bearing only. Works even when
    distance is missing. Drives the CAMERA servo, which only has to keep
    the player in frame.

Measurements carry the camera FRAME timestamp, so inference latency is
accounted for rather than treated as "now".
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

from .config import PredictionConfig


def polar_to_xy(bearing_deg: float, distance_m: float) -> Tuple[float, float]:
    b = math.radians(bearing_deg)
    return distance_m * math.sin(b), distance_m * math.cos(b)


def xy_to_polar(x: float, y: float) -> Tuple[float, float]:
    """(bearing_deg, distance_m)"""
    return math.degrees(math.atan2(x, y)), math.hypot(x, y)


class PlayerEstimator:
    def __init__(self, cfg: PredictionConfig):
        self.cfg = cfg
        self.reset()

    def reset(self) -> None:
        self.x: Optional[np.ndarray] = None       # [x, y, vx, vy]
        self.P: Optional[np.ndarray] = None
        self.t: Optional[float] = None
        self.n = 0

    @property
    def active(self) -> bool:
        return self.x is not None

    @property
    def ready(self) -> bool:
        """Enough measurements that the velocity estimate means something."""
        return self.active and self.n >= self.cfg.min_updates

    # -- Kalman internals --------------------------------------------------
    def _F_Q(self, dt: float):
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q = self.cfg.accel_noise_mps2 ** 2
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        Q = q * np.array([[dt4 / 4, 0, dt3 / 2, 0],
                          [0, dt4 / 4, 0, dt3 / 2],
                          [dt3 / 2, 0, dt2, 0],
                          [0, dt3 / 2, 0, dt2]])
        return F, Q

    def _measurement(self, bearing_deg: float, d: float, reliable: bool):
        b = math.radians(bearing_deg)
        z = np.array([d * math.sin(b), d * math.cos(b)])
        sd = self.cfg.range_noise_frac * d
        if not reliable:
            sd *= self.cfg.unreliable_range_factor
        sb = math.radians(self.cfg.bearing_noise_deg)
        # Jacobian of (x, y) w.r.t. (d, b): polar noise -> Cartesian covariance
        J = np.array([[math.sin(b), d * math.cos(b)],
                      [math.cos(b), -d * math.sin(b)]])
        R = J @ np.diag([sd * sd, sb * sb]) @ J.T
        return z, R

    # -- public --------------------------------------------------------------
    def update(self, t: float, bearing_deg: float, distance_m: float,
               reliable: bool = True) -> None:
        if self.t is not None and t - self.t > self.cfg.reset_after_s:
            self.reset()
        z, R = self._measurement(bearing_deg, distance_m, reliable)

        if self.x is None:
            self.x = np.array([z[0], z[1], 0.0, 0.0])
            vmax = self.cfg.max_speed_mps
            self.P = np.diag([R[0, 0], R[1, 1], vmax * vmax, vmax * vmax])
            self.t, self.n = t, 1
            return

        dt = max(1e-3, t - self.t)
        F, Q = self._F_Q(dt)
        x = F @ self.x
        P = F @ self.P @ F.T + Q

        H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        x = x + K @ (z - H @ x)
        P = (np.eye(4) - K @ H) @ P

        speed = math.hypot(x[2], x[3])
        if speed > self.cfg.max_speed_mps:
            x[2:] *= self.cfg.max_speed_mps / speed
        self.x, self.P, self.t = x, P, t
        self.n += 1

    def state_at(self, t: float) -> Optional[Tuple[float, float, float, float]]:
        """(x, y, vx, vy) extrapolated to time t. Velocity is zeroed until ready."""
        if self.x is None:
            return None
        x, y, vx, vy = self.x
        if not self.ready:
            vx = vy = 0.0
        dt = t - self.t
        return x + vx * dt, y + vy * dt, vx, vy

    def polar_at(self, t: float) -> Optional[Tuple[float, float]]:
        s = self.state_at(t)
        return None if s is None else xy_to_polar(s[0], s[1])

    def stale(self, now: float) -> bool:
        return self.t is None or now - self.t > self.cfg.reset_after_s


class BearingTracker:
    """alpha-beta filter on the player's world bearing (deg, deg/s)."""

    def __init__(self, cfg: PredictionConfig, alpha: float = 0.6, beta: float = 0.2):
        self.cfg = cfg
        self.alpha, self.beta = alpha, beta
        self.reset()

    def reset(self) -> None:
        self.b: Optional[float] = None
        self.rate = 0.0
        self.t: Optional[float] = None

    def update(self, t: float, bearing_deg: float) -> None:
        if self.b is None or self.t is None or t - self.t > self.cfg.reset_after_s:
            self.b, self.rate, self.t = bearing_deg, 0.0, t
            return
        dt = max(1e-3, t - self.t)
        pred = self.b + self.rate * dt
        r = bearing_deg - pred
        self.b = pred + self.alpha * r
        self.rate += self.beta * r / dt
        self.t = t

    def at(self, t: float) -> Optional[float]:
        if self.b is None or self.stale(t):
            return None
        return self.b + self.rate * (t - self.t)

    def stale(self, now: float) -> bool:
        return self.t is None or now - self.t > self.cfg.reset_after_s
