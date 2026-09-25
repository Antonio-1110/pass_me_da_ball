"""
physics.py

Ball flight after it leaves the arm: gravity plus quadratic air drag,
integrated with RK4. Everything here is in a frame whose origin is the
release point, with x horizontal towards the player and y up.

    a = (0, -g) - k * |v| * v,      k = rho * Cd * A / (2 m)

For a basketball k ~ 0.023 1/m, so at 10 m/s drag is ~2.3 m/s^2, about a
quarter of gravity. Ignoring it makes long passes land short.

Main entry points:
    solve_speed(dx, dy, angle, ball)       speed needed to pass through (dx, dy)
    solve_speed_and_angle(dx, dy, t, ball) both, from a measured flight time
    trajectory(v, angle, ball)             points for plotting / debugging
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .config import BallConfig

G = 9.81
V_MAX = 40.0          # search ceiling (m/s); far beyond anything the arm can do
DT = 0.002            # integration step (s)
T_MAX = 6.0           # give up after this long in the air


def drag_k(ball: BallConfig) -> float:
    if not ball.drag:
        return 0.0
    area = math.pi * (ball.diameter_m / 2.0) ** 2
    return 0.5 * ball.air_density * ball.drag_coeff * area / ball.mass_kg


def _accel(vx: float, vy: float, k: float) -> Tuple[float, float]:
    s = math.hypot(vx, vy)
    return -k * s * vx, -G - k * s * vy


def _step(x, y, vx, vy, k, dt):
    """One RK4 step of (x, y, vx, vy)."""
    a1x, a1y = _accel(vx, vy, k)
    v2x, v2y = vx + 0.5 * dt * a1x, vy + 0.5 * dt * a1y
    a2x, a2y = _accel(v2x, v2y, k)
    v3x, v3y = vx + 0.5 * dt * a2x, vy + 0.5 * dt * a2y
    a3x, a3y = _accel(v3x, v3y, k)
    v4x, v4y = vx + dt * a3x, vy + dt * a3y
    a4x, a4y = _accel(v4x, v4y, k)
    x += dt / 6.0 * (vx + 2 * v2x + 2 * v3x + v4x)
    y += dt / 6.0 * (vy + 2 * v2y + 2 * v3y + v4y)
    vx += dt / 6.0 * (a1x + 2 * a2x + 2 * a3x + a4x)
    vy += dt / 6.0 * (a1y + 2 * a2y + 2 * a3y + a4y)
    return x, y, vx, vy


@dataclass
class Crossing:
    y: float          # height when the ball reaches x = dx (relative to release)
    t: float          # time to get there
    vy: float         # vertical speed there (<0 means descending)


def cross_at(v: float, angle_deg: float, dx: float, ball: BallConfig,
             floor_y: float = -50.0) -> Optional[Crossing]:
    """
    Fly the ball until it reaches horizontal distance dx. Returns None if it
    drops below floor_y (relative to release) or times out first.
    """
    th = math.radians(angle_deg)
    vx, vy = v * math.cos(th), v * math.sin(th)
    k = drag_k(ball)
    if k == 0.0:  # closed form
        if vx <= 0:
            return None
        t = dx / vx
        return Crossing(y=vy * t - 0.5 * G * t * t, t=t, vy=vy - G * t)
    x = y = t = 0.0
    while t < T_MAX:
        nx, ny, nvx, nvy = _step(x, y, vx, vy, k, DT)
        if nx >= dx:
            f = (dx - x) / (nx - x) if nx > x else 0.0
            return Crossing(y=y + f * (ny - y), t=t + f * DT, vy=vy + f * (nvy - vy))
        x, y, vx, vy, t = nx, ny, nvx, nvy, t + DT
        if y < floor_y:
            return None
    return None


def vacuum_speed(dx: float, dy: float, angle_deg: float) -> float:
    """Drag-free launch speed to pass through (dx, dy); inf if impossible."""
    th = math.radians(angle_deg)
    denom = 2.0 * math.cos(th) ** 2 * (dx * math.tan(th) - dy)
    if dx <= 0 or denom <= 0:
        return math.inf
    return math.sqrt(G * dx * dx / denom)


def solve_speed(dx: float, dy: float, angle_deg: float,
                ball: BallConfig) -> Tuple[float, float]:
    """
    Launch speed (m/s) at angle_deg so the ball passes through (dx, dy),
    and the flight time. Returns (inf, 0) if unreachable.

    For a fixed angle the height at dx rises monotonically with speed, so a
    bisection starting from the drag-free answer (always a lower bound) works.
    """
    v0 = vacuum_speed(dx, dy, angle_deg)
    if not math.isfinite(v0):
        return math.inf, 0.0
    if drag_k(ball) == 0.0:
        return v0, dx / (v0 * math.cos(math.radians(angle_deg)))

    def height(v):
        c = cross_at(v, angle_deg, dx, ball)
        return -math.inf if c is None else c.y

    lo, hi = v0, V_MAX
    if height(hi) < dy:
        return math.inf, 0.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if height(mid) < dy:
            lo = mid
        else:
            hi = mid
        if hi - lo < 1e-4:
            break
    c = cross_at(hi, angle_deg, dx, ball)
    return hi, (c.t if c else 0.0)


def solve_speed_and_angle(dx: float, dy: float, flight_time_s: float,
                          ball: BallConfig) -> Tuple[float, float]:
    """
    Recover (speed, angle) of a real shot from where it ended up and how long
    it took (e.g. timed from video). Steeper launches take longer to cover
    the same distance, so bisect on angle.
    """
    lo = math.degrees(math.atan2(dy, dx)) + 0.5
    hi = 85.0

    def t_of(a):
        v, t = solve_speed(dx, dy, a, ball)
        return t if math.isfinite(v) else math.inf

    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if t_of(mid) < flight_time_s:
            lo = mid
        else:
            hi = mid
        if hi - lo < 1e-3:
            break
    angle = 0.5 * (lo + hi)
    return solve_speed(dx, dy, angle, ball)[0], angle


def trajectory(v: float, angle_deg: float, ball: BallConfig,
               x0: float = 0.0, y0: float = 0.0, stop_y: float = 0.0,
               dt: float = 0.01) -> List[Tuple[float, float, float]]:
    """(t, x, y) points from (x0, y0) until the ball falls to stop_y."""
    th = math.radians(angle_deg)
    x, y, vx, vy, t = x0, y0, v * math.cos(th), v * math.sin(th), 0.0
    k = drag_k(ball)
    pts = [(t, x, y)]
    while t < T_MAX and not (y < stop_y and vy < 0):
        x, y, vx, vy = _step(x, y, vx, vy, k, dt)
        t += dt
        pts.append((t, x, y))
    return pts
