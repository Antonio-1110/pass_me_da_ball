"""
aiming.py

Where the LAUNCHER should point.

A pass takes time to arrive: fire latency (Modbus writes + arm spin-up)
plus the ball's flight time. A moving player will have moved by then, so
the launcher aims at the predicted catch point:

    t_catch = fire_latency + flight_time(distance at t_catch)

flight_time depends on the distance, which depends on t_catch, so a few
fixed-point iterations are used (they converge in 2-3 steps because flight
time changes slowly with distance).

A PASS_LEFT / PASS_RIGHT gesture adds a sideways lead on top of that,
perpendicular to the line of sight.

Assumes the launcher turret's rotation axis passes (roughly) through the arm
pivot, so "distance from the turret" = "distance from the pivot".
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from .config import LauncherAxisConfig, LauncherConfig, PredictionConfig
from .kinematics import LaunchPlan, plan_launch
from .prediction import PlayerEstimator, xy_to_polar


@dataclass
class AimSolution:
    angle_deg: float          # launcher world angle
    distance_m: float         # to the catch point
    catch_in_s: float         # from now until the ball arrives
    catch_xy: tuple
    plan: Optional[LaunchPlan] = None


def apply_lateral_lead(x: float, y: float, lateral_m: float) -> tuple:
    """Shift (x, y) sideways, perpendicular to the line of sight (+ = machine's right)."""
    if not lateral_m:
        return x, y
    b = math.atan2(x, y)
    return x + lateral_m * math.cos(b), y - lateral_m * math.sin(b)


def solve_shot(est: PlayerEstimator, now: float, profile: str, lateral_m: float,
               launcher: LauncherConfig, axis: LauncherAxisConfig,
               iterations: int = 3) -> Optional[AimSolution]:
    """Full solution for a shot fired now: angle + launch plan at the catch point."""
    t = axis.fire_latency_s
    sol = None
    for _ in range(iterations):
        s = est.state_at(now + t)
        if s is None:
            return None
        x, y = apply_lateral_lead(s[0], s[1], lateral_m)
        bearing, d = xy_to_polar(x, y)
        plan = plan_launch(d, profile, launcher)
        sol = AimSolution(bearing, d, t, (x, y), plan)
        t_new = axis.fire_latency_s + plan.flight_time_s
        if abs(t_new - t) < 0.01 or plan.flight_time_s <= 0:
            break
        t = t_new
    return sol


def tracking_angle(est: PlayerEstimator, now: float, axis: LauncherAxisConfig,
                   pred: PredictionConfig) -> Optional[float]:
    """
    Cheap between-shots aim: lead by latency + a rough flight time
    (distance / nominal ball speed). Runs every tick, so no launch plan.
    """
    s = est.state_at(now)
    if s is None:
        return None
    _, d = xy_to_polar(s[0], s[1])
    t = axis.fire_latency_s + d / max(1.0, pred.nominal_ball_speed_mps)
    s = est.state_at(now + t)
    return xy_to_polar(s[0], s[1])[0]
