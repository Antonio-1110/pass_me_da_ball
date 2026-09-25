"""
launcher.py

Owns the throwing arm. Takes a LaunchPlan from kinematics.py and runs the
fire -> return-to-home -> reload cycle on its own thread so the tracking /
turret loop never stalls while the PS100 is moving.
"""

from __future__ import annotations

import logging
import threading
import time
from enum import Enum
from typing import Optional

from .config import LauncherConfig
from .hardware.ps100 import PS100
from .kinematics import LaunchPlan, estimate_move_time_s, return_move

log = logging.getLogger(__name__)


class LauncherState(str, Enum):
    READY = "ready"
    FIRING = "firing"
    RETURNING = "returning"
    RELOADING = "reloading"
    FAULT = "fault"


class Launcher:
    def __init__(self, drive: PS100, cfg: LauncherConfig, reload_delay_s: float):
        self.drive = drive
        self.cfg = cfg
        self.reload_delay_s = reload_delay_s
        self.state = LauncherState.READY
        self.last_plan: Optional[LaunchPlan] = None
        self._thread: Optional[threading.Thread] = None

    @property
    def ready(self) -> bool:
        return self.state is LauncherState.READY

    def fire(self, plan: LaunchPlan, block: bool = False) -> bool:
        """Start a throw. Returns False (and does nothing) if not allowed."""
        if not plan.ok:
            log.warning("Refusing to fire: %s", plan.summary())
            return False
        if not self.ready:
            log.warning("Launcher busy (%s), ignoring fire request", self.state.value)
            return False
        self.last_plan = plan
        self.state = LauncherState.FIRING
        log.info("FIRE %s", plan.summary())
        self._thread = threading.Thread(target=self._cycle, args=(plan,),
                                        name="launcher", daemon=True)
        self._thread.start()
        if block:
            self._thread.join()
        return True

    def _cycle(self, plan: LaunchPlan) -> None:
        try:
            m = plan.move
            if not self.drive.move(m.turns, m.pulses, m.rpm,
                                   est_time_s=estimate_move_time_s(m, self.cfg)):
                raise RuntimeError("throw move did not complete")

            self.state = LauncherState.RETURNING
            back = return_move(plan, self.cfg)
            if not self.drive.move(back.turns, back.pulses, back.rpm,
                                   est_time_s=estimate_move_time_s(back, self.cfg)):
                raise RuntimeError("return move did not complete")

            self.state = LauncherState.RELOADING
            time.sleep(self.reload_delay_s)
            self.state = LauncherState.READY
        except Exception:
            log.exception("Launcher fault -- sending stop; clear with reset()")
            self.state = LauncherState.FAULT
            try:
                self.drive.stop()
            except Exception:
                log.exception("stop command failed too")

    def reset(self) -> None:
        """Operator acknowledges a fault (arm must be back at home!)."""
        if self.state is LauncherState.FAULT:
            self.state = LauncherState.READY

    def stop(self) -> None:
        self.drive.stop()

    def wait(self, timeout: Optional[float] = None) -> None:
        if self._thread:
            self._thread.join(timeout)
