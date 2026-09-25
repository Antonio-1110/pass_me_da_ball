"""
kinematics.py

Turns "player is D metres away and wants pass type P" into the three numbers
the PS100 needs: whole motor turns (0x0202), leftover pulses (0x0203) and
motor speed in rpm (0x0204).

Arm geometry (side view, ball travels to the right, towards the player):

                 90 deg (arm straight up)
                    |
    0 deg  o--------+ pivot          player -->
  (home, arm        |
  pointing back)    |

The arm angle psi is measured from "pointing straight back" and increases as
the arm swings up and over. The ball's tip velocity is perpendicular to the
arm, so the ball's elevation when it leaves the cup is

    launch_angle = 90 - psi_release

i.e. releasing at psi = 70 deg gives a 20 deg flat chest pass; releasing at
psi = 35 deg gives a 55 deg lob. The commanded relative move is therefore

    sweep = psi_release - home_angle

and with a G:1 gearbox the motor has to turn G times as far and G times as
fast as the arm.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List

from .config import LauncherConfig, PassProfile

G = 9.81


@dataclass
class MotorMove:
    """A relative move in PS100 register units."""
    turns: int          # -> 0x0202 (signed)
    pulses: int         # -> 0x0203 (signed, same sign as turns)
    rpm: int            # -> 0x0204 (strictly positive)


@dataclass
class LaunchPlan:
    pass_type: str
    distance_m: float
    launch_angle_deg: float
    release_angle_deg: float      # arm angle psi at release
    sweep_deg: float              # arm rotation from home to release
    release_height_m: float
    exit_velocity_mps: float      # ball speed needed at release
    arm_rpm: float
    motor_rpm: int
    move: MotorMove
    flight_time_s: float
    ok: bool = True
    warnings: List[str] = field(default_factory=list)

    def summary(self) -> str:
        status = "OK" if self.ok else "REJECTED"
        s = (f"[{status}] {self.pass_type} @ {self.distance_m:.2f} m: "
             f"v={self.exit_velocity_mps:.2f} m/s, angle={self.launch_angle_deg:.0f} deg, "
             f"arm {self.arm_rpm:.1f} rpm -> motor {self.motor_rpm} rpm, "
             f"sweep {self.sweep_deg:.1f} deg -> turns={self.move.turns} pulses={self.move.pulses}, "
             f"flight {self.flight_time_s:.2f} s")
        for w in self.warnings:
            s += f"\n    ! {w}"
        return s


# ---------------------------------------------------------------------------
# Unit conversions
# ---------------------------------------------------------------------------
def arm_degrees_to_motor_pulses(arm_deg: float, gear_ratio: float, pulses_per_rev: int) -> int:
    """Arm rotation (deg) -> motor command pulses, through the gearbox."""
    return round(arm_deg / 360.0 * gear_ratio * pulses_per_rev)


def split_pulses(total_pulses: int, pulses_per_rev: int) -> tuple[int, int]:
    """
    Split a signed pulse count into (whole turns, remaining pulses), both
    carrying the same sign, as P4-2 / P4-3 expect.

    25000 -> (2, 5000);  -25000 -> (-2, -5000)
    """
    sign = -1 if total_pulses < 0 else 1
    turns, rem = divmod(abs(total_pulses), pulses_per_rev)
    return sign * turns, sign * rem


def arm_move(arm_deg: float, arm_rpm: float, cfg: LauncherConfig) -> MotorMove:
    """Build a register-ready MotorMove for an arm rotation at an arm speed."""
    total = arm_degrees_to_motor_pulses(arm_deg, cfg.gear_ratio, cfg.pulses_per_rev)
    turns, pulses = split_pulses(total, cfg.pulses_per_rev)
    rpm = max(1, int(math.ceil(abs(arm_rpm) * cfg.gear_ratio)))
    return MotorMove(turns=turns, pulses=pulses, rpm=rpm)


def motor_rpm_to_register(rpm: float) -> int:
    """0x0204 must be a strictly positive integer."""
    return max(1, int(round(abs(rpm))))


# ---------------------------------------------------------------------------
# Ballistics
# ---------------------------------------------------------------------------
def required_speed(dx: float, dy: float, angle_deg: float) -> float:
    """
    Launch speed for a drag-free projectile fired at angle_deg to land dx
    metres away horizontally and dy metres higher (dy may be negative).
    Returns math.inf if the target is not reachable at that angle.
    """
    th = math.radians(angle_deg)
    denom = 2.0 * math.cos(th) ** 2 * (dx * math.tan(th) - dy)
    if dx <= 0 or denom <= 0:
        return math.inf
    return math.sqrt(G * dx * dx / denom)


def accel_arm_degrees(motor_rpm: float, cfg: LauncherConfig) -> float:
    """
    Arm rotation used up just accelerating the motor from 0 to motor_rpm with
    the PS100's linear ramp (FA40 = ms per 1000 rpm).
    """
    if cfg.accel_ms_per_1000rpm <= 0:
        return 0.0
    accel_rpm_per_s = 1000.0 / (cfg.accel_ms_per_1000rpm / 1000.0)
    t = motor_rpm / accel_rpm_per_s
    motor_revs = (motor_rpm / 60.0) * t / 2.0
    return motor_revs / cfg.gear_ratio * 360.0


def plan_launch(distance_m: float, profile_name: str, cfg: LauncherConfig) -> LaunchPlan:
    """
    Compute everything needed for one pass. `distance_m` is the horizontal
    distance from the arm pivot to the player.
    """
    profile: PassProfile = cfg.profiles[profile_name]
    warnings: List[str] = []
    ok = True

    theta = profile.launch_angle_deg
    psi_rel = 90.0 - theta
    sweep = psi_rel - cfg.home_angle_deg
    if sweep <= 0:
        warnings.append(f"release angle {psi_rel:.1f} deg is not past home "
                        f"{cfg.home_angle_deg:.1f} deg; check home_angle_deg")
        ok = False

    L = cfg.arm_length_m
    psi = math.radians(psi_rel)
    release_x = -L * math.cos(psi)            # relative to pivot, +x towards player
    release_h = cfg.pivot_height_m + L * math.sin(psi)
    dx = distance_m - release_x
    dy = profile.target_height_m - release_h

    v = required_speed(dx, dy, theta)
    if not math.isfinite(v):
        warnings.append("target unreachable at this launch angle")
        ok = False
        v = 0.0

    omega = v / (L * cfg.efficiency) if v > 0 else 0.0       # rad/s at the arm
    arm_rpm = omega * 60.0 / (2.0 * math.pi)
    motor_rpm = motor_rpm_to_register(arm_rpm * cfg.gear_ratio)

    if motor_rpm > cfg.max_motor_rpm:
        warnings.append(f"needs {motor_rpm} motor rpm > max {cfg.max_motor_rpm}")
        ok = False
        motor_rpm = cfg.max_motor_rpm
    if ok and motor_rpm < cfg.min_motor_rpm:
        warnings.append(f"needs only {motor_rpm} motor rpm; raised to min {cfg.min_motor_rpm}")
        motor_rpm = cfg.min_motor_rpm

    ramp = accel_arm_degrees(motor_rpm, cfg)
    if sweep > 0 and ramp > sweep:
        warnings.append(f"accel ramp uses {ramp:.0f} deg of arm travel but sweep is only "
                        f"{sweep:.0f} deg -- ball will leave slow (lower FA40)")

    total = arm_degrees_to_motor_pulses(max(sweep, 0.0), cfg.gear_ratio, cfg.pulses_per_rev)
    turns, pulses = split_pulses(total, cfg.pulses_per_rev)
    move = MotorMove(turns=turns, pulses=pulses, rpm=motor_rpm)

    vx = v * math.cos(math.radians(theta))
    flight = dx / vx if vx > 0 else 0.0

    return LaunchPlan(
        pass_type=profile_name,
        distance_m=distance_m,
        launch_angle_deg=theta,
        release_angle_deg=psi_rel,
        sweep_deg=sweep,
        release_height_m=release_h,
        exit_velocity_mps=v,
        arm_rpm=arm_rpm,
        motor_rpm=motor_rpm,
        move=move,
        flight_time_s=flight,
        ok=ok,
        warnings=warnings,
    )


def return_move(plan: LaunchPlan, cfg: LauncherConfig) -> MotorMove:
    """The slow move that brings the arm back to home after a throw."""
    return MotorMove(turns=-plan.move.turns, pulses=-plan.move.pulses,
                     rpm=motor_rpm_to_register(cfg.return_rpm))


def estimate_move_time_s(move: MotorMove, cfg: LauncherConfig) -> float:
    """Trapezoidal-profile duration estimate (used for completion_mode='time')."""
    revs = abs(move.turns) + abs(move.pulses) / cfg.pulses_per_rev
    rpm = max(1, move.rpm)
    rps = rpm / 60.0
    if cfg.accel_ms_per_1000rpm <= 0:
        return revs / rps
    a = 1000.0 / 60.0 / (cfg.accel_ms_per_1000rpm / 1000.0)   # rev/s^2
    t_ramp = rps / a
    ramp_revs = rps * t_ramp            # accel + decel together
    if ramp_revs >= revs:               # triangular profile
        return 2.0 * math.sqrt(revs / a)
    return 2.0 * t_ramp + (revs - ramp_revs) / rps
