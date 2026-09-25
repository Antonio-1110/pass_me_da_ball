"""
pan_axis.py

Hardware abstraction for the (not yet chosen) motor that pans the camera /
launcher. Everything above this layer only speaks in degrees:

    axis.move_to(12.5)   # absolute pan angle, 0 = straight ahead, + = right
    axis.angle()         # best known current angle

To add real hardware, subclass PanAxis, implement those two methods, and
register it in make_pan_axis(). The TurretController already does the
smoothing (PID + slew-rate limit), so move_to() is called ~30x per second
with small steps and the backend can simply forward each one.

Candidate backends:
  * hobby/PWM servo      -> GpioServoPanAxis below (gpiozero)
  * stepper + driver     -> step/dir with microstepping, count steps
  * second PS100 / other RS-485 servo -> absolute position over Modbus
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod

from ..config import TurretConfig

log = logging.getLogger(__name__)


class PanAxis(ABC):
    def __init__(self, cfg: TurretConfig):
        self.cfg = cfg

    def clamp(self, deg: float) -> float:
        return max(self.cfg.min_deg, min(self.cfg.max_deg, deg))

    @abstractmethod
    def move_to(self, deg: float) -> None: ...

    @abstractmethod
    def angle(self) -> float: ...

    def close(self) -> None:
        pass


class VirtualPanAxis(PanAxis):
    """No motor: remembers the commanded angle. Lets the whole stack run."""

    def __init__(self, cfg: TurretConfig):
        super().__init__(cfg)
        self._deg = 0.0

    def move_to(self, deg: float) -> None:
        self._deg = self.clamp(deg)

    def angle(self) -> float:
        return self._deg


class GpioServoPanAxis(PanAxis):
    """
    Example backend: standard 50 Hz hobby servo on a Pi GPIO pin.
    Use a hardware-PWM pin (GPIO12/13/18/19) with the pigpio or lgpio pin
    factory to avoid jitter.
    """

    def __init__(self, cfg: TurretConfig, pin: int = 18):
        super().__init__(cfg)
        from gpiozero import AngularServo
        self._servo = AngularServo(pin, min_angle=cfg.min_deg, max_angle=cfg.max_deg)
        self._deg = 0.0
        self.move_to(0.0)

    def move_to(self, deg: float) -> None:
        self._deg = self.clamp(deg)
        self._servo.angle = self._deg

    def angle(self) -> float:
        return self._deg

    def close(self) -> None:
        self._servo.detach()


def make_pan_axis(cfg: TurretConfig) -> PanAxis:
    if cfg.backend in ("none", "virtual"):
        log.info("Pan axis: virtual (no motor attached)")
        return VirtualPanAxis(cfg)
    if cfg.backend == "gpio_servo":
        return GpioServoPanAxis(cfg)
    raise ValueError(f"Unknown turret backend: {cfg.backend}")
