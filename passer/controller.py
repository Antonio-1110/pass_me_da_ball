"""
controller.py

The machine's decision loop, independent of cameras and models so it can be
driven by the real vision pipeline (app.py) or by a simulation (tests).

Per tick:
  observe(vision result) -> turret filters
  turret.update()        -> camera servo tracks, launcher leads
  pending shot?          -> solve aim at the predicted catch point
                            -> launcher on target? -> fire the launch plan
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

from .aiming import AimSolution, solve_shot
from .config import Config
from .hardware.pan_axis import make_camera_axis, make_launcher_axis
from .hardware.ps100 import PS100
from .launcher import Launcher
from .turret import TwoAxisTurret
from .vision.gestures import Gesture

log = logging.getLogger("passer")


@dataclass
class PendingShot:
    gesture: Gesture
    created: float


class MachineController:
    def __init__(self, cfg: Config, turret: TwoAxisTurret, launcher: Launcher):
        self.cfg = cfg
        self.turret = turret
        self.launcher = launcher
        self.pending: Optional[PendingShot] = None
        self.last_solution: Optional[AimSolution] = None
        self.status = ""

    @classmethod
    def from_config(cls, cfg: Config) -> "MachineController":
        turret = TwoAxisTurret(make_camera_axis(cfg.camera_axis),
                               make_launcher_axis(cfg.launcher_axis),
                               cfg.camera_axis, cfg.launcher_axis, cfg.prediction)
        launcher = Launcher(PS100.from_config(cfg.ps100, cfg.app.dry_run),
                            cfg.launcher, cfg.app.reload_delay_s)
        return cls(cfg, turret, launcher)

    # ------------------------------------------------------------------
    def _set_status(self, msg: str, warn: bool = False) -> None:
        self.status = msg
        (log.warning if warn else log.info)(msg)

    def request_shot(self, gesture: Gesture, now: float) -> bool:
        if self.pending is not None:
            return False
        if not self.launcher.ready:
            self._set_status(f"{gesture.value}: launcher {self.launcher.state.value}, ignored", True)
            return False
        if not self.turret.estimator.active:
            self._set_status(f"{gesture.value}: no player position (distance) yet, ignored", True)
            return False
        self.pending = PendingShot(gesture, now)
        self._set_status(f"aiming {gesture.value}")
        return True

    def cancel(self, reason: str) -> None:
        if self.pending:
            self._set_status(f"{self.pending.gesture.value}: {reason}, cancelled", True)
        self.pending = None
        self.turret.set_aim(None)

    def tick(self, now: float, dt: float, frame_id: Optional[int] = None,
             t_frame: Optional[float] = None, pan_error_deg: Optional[float] = None,
             distance_m: Optional[float] = None, reliable: bool = False) -> None:
        if frame_id is not None and t_frame is not None:
            self.turret.observe(frame_id, t_frame, pan_error_deg, distance_m, reliable)
        if self.pending is not None:
            self._aim_and_maybe_fire(now)
        self.turret.update(now, dt)

    # ------------------------------------------------------------------
    def _aim_and_maybe_fire(self, now: float) -> None:
        p = self.pending
        lc = self.cfg.launcher_axis
        if now - p.created > lc.aim_timeout_s:
            self.cancel("could not aim in time")
            return
        if self.turret.estimator.stale(now):
            self.cancel("lost the player")
            return

        sol = solve_shot(self.turret.estimator, now, p.gesture.profile,
                         p.gesture.lead_sign * lc.lead_m, self.cfg.launcher, lc)
        if sol is None or sol.plan is None:
            return
        self.last_solution = sol
        self.turret.set_aim(sol.angle_deg)

        motor = self.turret.launcher.physical
        aimed = (not motor) or (not self.cfg.app.require_on_target) \
            or self.turret.on_target(sol.angle_deg)
        if not aimed:
            return
        if not sol.plan.ok:
            self.cancel("; ".join(sol.plan.warnings) or "no valid launch plan")
            return
        if self.launcher.fire(sol.plan):
            self._set_status(
                f"FIRED {p.gesture.value}: aim {sol.angle_deg:+.1f} deg, catch at "
                f"{sol.distance_m:.2f} m in {sol.catch_in_s:.2f} s, motor {sol.plan.motor_rpm} rpm")
        self.pending = None
        self.turret.set_aim(None)
