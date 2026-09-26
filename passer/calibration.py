"""
calibration.py

Closing the sim-to-real gap from test shots.

A calibration shot = "fired profile P at motor rpm R, ball landed X metres
from the pivot" (optionally: at height H, after T seconds of flight, timed
from video). From each shot we work backwards through the physics model to
the ball speed (and angle, if T is known) that the machine REALLY produced,
and compare with what the ideal arm model (v = omega * L) predicted.

    multiplier m = ideal speed / real speed

m > 1 means the machine throws weaker than ideal (slip, flex, drag
underestimated...), so commands must be boosted by m. The median over all
shots becomes `speed_scale`; if m drifts with distance, a per-profile
`speed_table` captures that. Shots with flight times also give
`angle_offset_deg`.
"""

from __future__ import annotations

import csv
import math
import statistics
import time
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Dict, List, Optional

from .config import LauncherConfig
from .kinematics import angle_offset, arm_rpm_to_ball_speed, release_angle_for, release_point
from .physics import solve_speed, solve_speed_and_angle, trajectory


@dataclass
class Shot:
    profile: str
    motor_rpm: float
    release_angle_deg: float      # commanded arm angle at release (psi)
    arm_length_m: float
    pivot_height_m: float
    gear_ratio: float
    landed_m: float               # horizontal distance from pivot where measured
    land_height_m: float = 0.0    # 0 = where it hit the floor
    flight_time_s: float = 0.0    # 0 = not measured
    timestamp: float = 0.0


@dataclass
class ShotAnalysis:
    shot: Shot
    ideal_speed: float            # omega * L
    real_speed: float             # back-solved from the landing point
    real_angle: float             # back-solved (or assumed) launch angle
    angle_error: Optional[float]  # real - nominal, only when flight time known

    @property
    def multiplier(self) -> float:
        return self.ideal_speed / self.real_speed if self.real_speed > 0 else math.nan


def make_shot(profile: str, motor_rpm: float, landed_m: float, cfg: LauncherConfig,
              land_height_m: float = 0.0, flight_time_s: float = 0.0) -> Shot:
    """Record a shot together with the geometry it was fired with."""
    return Shot(profile=profile, motor_rpm=motor_rpm,
                release_angle_deg=release_angle_for(profile, cfg),
                arm_length_m=cfg.arm_length_m, pivot_height_m=cfg.pivot_height_m,
                gear_ratio=cfg.gear_ratio, landed_m=landed_m,
                land_height_m=land_height_m, flight_time_s=flight_time_s,
                timestamp=time.time())


# ---------------------------------------------------------------------------
# Storage (plain CSV so it can be edited in a spreadsheet)
# ---------------------------------------------------------------------------
FIELDS = [f.name for f in fields(Shot)]


def save_shot(path: str | Path, shot: Shot) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    new = not path.exists()
    with path.open("a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            w.writeheader()
        w.writerow(asdict(shot))


def load_shots(path: str | Path) -> List[Shot]:
    with Path(path).open(newline="") as f:
        out = []
        for row in csv.DictReader(f):
            kw = {k: (row[k] if k == "profile" else float(row[k] or 0)) for k in FIELDS if k in row}
            out.append(Shot(**kw))
        return out


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------
def analyse(shot: Shot, cfg: LauncherConfig,
            assumed_angle_offset: Optional[float] = None) -> ShotAnalysis:
    psi = math.radians(shot.release_angle_deg)
    rx = -shot.arm_length_m * math.cos(psi)
    rh = shot.pivot_height_m + shot.arm_length_m * math.sin(psi)
    dx, dy = shot.landed_m - rx, shot.land_height_m - rh
    ideal = arm_rpm_to_ball_speed(shot.motor_rpm / shot.gear_ratio, shot.arm_length_m)
    nominal_angle = 90.0 - shot.release_angle_deg

    if shot.flight_time_s > 0:
        v, a = solve_speed_and_angle(dx, dy, shot.flight_time_s, cfg.ball)
        return ShotAnalysis(shot, ideal, v, a, a - nominal_angle)

    # No timing: assume the angle error we already know about.
    if assumed_angle_offset is None:
        prof = cfg.profiles.get(shot.profile)
        assumed_angle_offset = angle_offset(prof, cfg.calibration) if prof else 0.0
    a = nominal_angle + assumed_angle_offset
    v, _ = solve_speed(dx, dy, a, cfg.ball)
    return ShotAnalysis(shot, ideal, v if math.isfinite(v) else 0.0, a, None)


@dataclass
class Fit:
    speed_scale: float
    # Per profile (each releases at a different arm angle, so errors differ).
    # Only profiles that had timed shots appear here.
    angle_offsets: Dict[str, float]
    speed_table: Dict[str, List[List[float]]]
    analyses: List[ShotAnalysis]

    def yaml(self) -> str:
        lines = ["launcher:"]
        if self.angle_offsets:
            lines.append("  profiles:")
            for prof, off in self.angle_offsets.items():
                lines.append(f"    {prof}: {{speed_scale: 1.0, angle_offset_deg: {off:.1f}}}")
        lines += ["  calibration:",
                  f"    speed_scale: {self.speed_scale:.3f}",
                  "    angle_offset_deg: 0.0"]
        if self.speed_table:
            lines.append("    speed_table:")
            for prof, pts in self.speed_table.items():
                pts_s = ", ".join(f"[{d:.1f}, {m:.3f}]" for d, m in pts)
                lines.append(f"      {prof}: [{pts_s}]")
        return "\n".join(lines)


def fit(shots: List[Shot], cfg: LauncherConfig, table_bin_m: float = 0.5,
        table_threshold: float = 0.02) -> Fit:
    """
    1. Timed shots give each profile's launch-angle error (median).
    2. Untimed shots are analysed assuming that angle error (or the one
       already in the config if the profile had no timed shots).
    3. Global speed_scale = median speed multiplier. A per-profile
       speed_table is only produced when multipliers vary with distance by
       more than `table_threshold` (2%), so a few noisy shots don't overfit.
    """
    angle_offsets: Dict[str, float] = {}
    timed = [analyse(s, cfg) for s in shots if s.flight_time_s > 0]
    for prof in sorted({a.shot.profile for a in timed}):
        errs = [a.angle_error for a in timed if a.shot.profile == prof and a.real_speed > 0]
        if errs:
            angle_offsets[prof] = statistics.median(errs)

    timed_iter = iter(timed)
    analyses = [next(timed_iter) if s.flight_time_s > 0 else
                analyse(s, cfg, angle_offsets.get(s.profile)) for s in shots]
    good = [a for a in analyses if a.real_speed > 0 and math.isfinite(a.multiplier)]
    if not good:
        raise ValueError("no usable shots")
    scale = statistics.median(a.multiplier for a in good)

    table: Dict[str, List[List[float]]] = {}
    for prof in sorted({a.shot.profile for a in good}):
        bins: Dict[float, List[float]] = {}
        for a in good:
            if a.shot.profile == prof:
                key = round(a.shot.landed_m / table_bin_m) * table_bin_m
                bins.setdefault(key, []).append(a.multiplier / scale)
        pts = [[d, statistics.mean(ms)] for d, ms in sorted(bins.items())]
        if len(pts) >= 2 and max(abs(m - 1.0) for _, m in pts) > table_threshold:
            table[prof] = pts
    return Fit(scale, angle_offsets, table, analyses)


def predict_landing(profile: str, motor_rpm: float, cfg: LauncherConfig,
                    land_height_m: float = 0.0) -> float:
    """
    Where a fixed-rpm shot should land (distance from pivot) with the current
    global calibration applied (speed_table ignored: it depends on distance).
    Handy for choosing test rpms and for checking a fit.
    """
    prof = cfg.profiles[profile]
    psi = release_angle_for(profile, cfg)
    rx, rh = release_point(psi, cfg)
    ideal = arm_rpm_to_ball_speed(motor_rpm / cfg.gear_ratio, cfg.arm_length_m)
    v = ideal / (cfg.calibration.speed_scale * prof.speed_scale)
    angle = 90.0 - psi + angle_offset(prof, cfg.calibration)
    pts = trajectory(v, angle, cfg.ball, x0=rx, y0=rh, stop_y=land_height_m, dt=0.002)
    (_, xa, ya), (_, xb, yb) = pts[-2], pts[-1]
    f = (land_height_m - ya) / (yb - ya) if yb != ya else 0.0
    return xa + f * (xb - xa)
