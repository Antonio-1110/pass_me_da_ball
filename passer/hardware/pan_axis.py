"""
pan_axis.py

Hardware abstraction for the two pan axes. Everything above this layer only
speaks in degrees (+ = right as seen from the machine):

    axis.move_to(12.5)   # absolute angle in the axis' own frame
    axis.angle()         # best known current angle

  * camera axis   - hobby servo, angle RELATIVE to the launcher turret
  * launcher axis - stepper(s), angle in the WORLD frame

The controllers in turret.py already produce smooth, speed/accel-limited
setpoints ~30 times per second, so a backend can simply forward each one.

To add hardware: subclass PanAxis, implement move_to() and angle(), and
register it in make_camera_axis() / make_launcher_axis().

Launcher-stepper notes: generating step pulses from Python on the Pi is
jittery. Prefer a driver that takes position commands (a closed-loop
stepper over RS-485/CAN, or a small microcontroller running AccelStepper
fed over serial) and have move_to() send the absolute target. angle()
should return the driver's reported position if it has one.
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod

from ..config import CameraAxisConfig, LauncherAxisConfig

log = logging.getLogger(__name__)


class PanAxis(ABC):
    #: False for a placeholder axis with no motor behind it. The tracker then
    #: treats it as fixed at 0 deg, because the camera image won't move.
    physical: bool = True

    def __init__(self, min_deg: float, max_deg: float):
        self.min_deg, self.max_deg = min_deg, max_deg

    def clamp(self, deg: float) -> float:
        return max(self.min_deg, min(self.max_deg, deg))

    @abstractmethod
    def move_to(self, deg: float) -> None: ...

    @abstractmethod
    def angle(self) -> float: ...

    def close(self) -> None:
        pass


class VirtualPanAxis(PanAxis):
    """
    Remembers the commanded angle. With physical=False (the default) it stands
    in for a motor that doesn't exist yet; tests use physical=True to simulate
    a perfect motor.
    """

    def __init__(self, min_deg: float = -90.0, max_deg: float = 90.0, physical: bool = False):
        super().__init__(min_deg, max_deg)
        self.physical = physical
        self._deg = 0.0

    def move_to(self, deg: float) -> None:
        self._deg = self.clamp(deg)

    def angle(self) -> float:
        return self._deg


class GpioServoPanAxis(PanAxis):
    """
    Standard 50 Hz hobby servo on a Pi GPIO pin, for the camera. Use a
    hardware-PWM pin with the lgpio/pigpio pin factory to avoid jitter.
    angle() returns the commanded angle (hobby servos don't report position).
    """

    def __init__(self, cfg: CameraAxisConfig):
        super().__init__(cfg.min_deg, cfg.max_deg)
        from gpiozero import AngularServo

        self.cfg = cfg
        # Servo travel must cover the configured range after trim.
        span = max(abs(cfg.min_deg), abs(cfg.max_deg)) + abs(cfg.trim_deg)
        self._servo = AngularServo(cfg.gpio_pin, min_angle=-span, max_angle=span)
        self._deg = 0.0
        self.move_to(0.0)

    def move_to(self, deg: float) -> None:
        self._deg = self.clamp(deg)
        raw = -self._deg if self.cfg.invert else self._deg
        self._servo.angle = raw + self.cfg.trim_deg

    def angle(self) -> float:
        return self._deg

    def close(self) -> None:
        self._servo.detach()


def make_camera_axis(cfg: CameraAxisConfig) -> PanAxis:
    if cfg.backend in ("none", "virtual"):
        log.info("Camera axis: virtual (no servo attached)")
        return VirtualPanAxis(cfg.min_deg, cfg.max_deg)
    if cfg.backend == "gpio_servo":
        return GpioServoPanAxis(cfg)
    raise ValueError(f"Unknown camera_axis backend: {cfg.backend}")


def make_launcher_axis(cfg: LauncherAxisConfig) -> PanAxis:
    if cfg.backend in ("none", "virtual"):
        log.info("Launcher axis: virtual (no turret motor attached)")
        return VirtualPanAxis(cfg.min_deg, cfg.max_deg)
    raise ValueError(f"Unknown launcher_axis backend: {cfg.backend} "
                     "(add a driver in passer/hardware/pan_axis.py)")
