"""
kinematics.py

Turns "player is D metres away and wants pass type P" into the three numbers
the PS100 needs: whole motor turns (0x0202), leftover pulses (0x0203) and
motor speed in rpm (0x0204).

Arm geometry (side view, ball travels to the right, towards the player):

                 90 deg (arm straight up)
                    |
    0 deg  o--------+ pivot          player -->
  (arm pointing     |
   straight back)   |

The arm angle psi is measured from "pointing straight back" and increases as
the arm swings up and over. The ball's velocity is tangent to the arm, so:

    ball speed       v     = omega * arm_length
    ball elevation   theta = 90 - psi_release
    release point    x = -L cos(psi),  h = pivot_height + L sin(psi)

The plan is built in three layers:

  1. PHYSICS   - where the ball leaves (arm geometry) and the speed needed to
                 reach the catch point (physics.py, with air drag).
  2. CALIBRATE - sim-to-real corrections from CalibrationConfig and the
                 profile's own trims (speed scale/offset/table, angle offset,
                 distance correction).
  3. DRIVE     - omega -> arm rpm -> motor rpm (x gear ratio); sweep -> motor
                 pulses -> (turns, pulses). The sweep is extended by the
                 deceleration ramp so the arm is still at full speed when it
                 passes the release angle.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

from .config import CalibrationConfig, LauncherConfig, PassProfile
from .physics import G, solve_speed  # noqa: F401  (G re-exported for callers)


@dataclass
class MotorMove:
    """A relative move in PS100 register units."""
    turns: int          # -> 0x0202 (signed)
    pulses: int         # -> 0x0203 (signed, same sign as turns)
    rpm: int            # -> 0x0204 (strictly positive)


@dataclass
class LaunchPlan:
    pass_type: str
    distance_m: float             # as measured by vision
    target_distance_m: float      # after distance calibration
    launch_angle_deg: float       # desired ball elevation
    release_angle_deg: float      # arm angle psi at release (commanded)
    end_angle_deg: float          # arm angle where the move stops
    sweep_deg: float              # arm rotation home -> end
    release_x_m: float            # release point relative to pivot (+ = towards player)
    release_height_m: float
    physics_speed_mps: float      # speed the ideal model says is needed
    exit_velocity_mps: float      # speed actually commanded (after calibration)
    arm_rpm: float
    motor_rpm: int
    move: MotorMove
    flight_time_s: float
    ok: bool = True
    warnings: List[str] = field(default_factory=list)

    def summary(self) -> str:
        status = "OK" if self.ok else "REJECTED"
        s = (f"[{status}] {self.pass_type} @ {self.distance_m:.2f} m: "
             f"v={self.exit_velocity_mps:.2f} m/s (physics {self.physics_speed_mps:.2f}), "
             f"angle={self.launch_angle_deg:.0f} deg, "
             f"arm {self.arm_rpm:.1f} rpm -> motor {self.motor_rpm} rpm, "
             f"release at {self.release_angle_deg:.1f} deg, sweep {self.sweep_deg:.1f} deg "
             f"-> turns={self.move.turns} pulses={self.move.pulses}, "
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
    return MotorMove(turns=turns, pulses=pulses, rpm=motor_rpm_to_register(arm_rpm * cfg.gear_ratio))


def motor_rpm_to_register(rpm: float) -> int:
    """0x0204 must be a strictly positive integer."""
    return max(1, int(round(abs(rpm))))


def ball_speed_to_arm_rpm(v_mps: float, arm_length_m: float) -> float:
    """v = omega * L  ->  arm rpm."""
    return v_mps / arm_length_m * 60.0 / (2.0 * math.pi)


def arm_rpm_to_ball_speed(arm_rpm: float, arm_length_m: float) -> float:
    return arm_rpm * 2.0 * math.pi / 60.0 * arm_length_m


def _ramp_arm_degrees(motor_rpm: float, ms_per_1000rpm: float, gear_ratio: float) -> float:
    """Arm travel used by a linear ramp between 0 and motor_rpm."""
    if ms_per_1000rpm <= 0:
        return 0.0
    t = motor_rpm / 1000.0 * ms_per_1000rpm / 1000.0
    motor_revs = (motor_rpm / 60.0) * t / 2.0
    return motor_revs / gear_ratio * 360.0


def accel_arm_degrees(motor_rpm: float, cfg: LauncherConfig) -> float:
    """Arm travel used up accelerating to motor_rpm (PS100 FA40)."""
    return _ramp_arm_degrees(motor_rpm, cfg.accel_ms_per_1000rpm, cfg.gear_ratio)


def decel_arm_degrees(motor_rpm: float, cfg: LauncherConfig) -> float:
    """Arm travel used braking from motor_rpm to 0 (PS100 FA41)."""
    return _ramp_arm_degrees(motor_rpm, cfg.decel_ms_per_1000rpm, cfg.gear_ratio)


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
def release_point(psi_deg: float, cfg: LauncherConfig) -> tuple[float, float]:
    """(x relative to pivot, height above floor) of the ball at arm angle psi."""
    psi = math.radians(psi_deg)
    return -cfg.arm_length_m * math.cos(psi), cfg.pivot_height_m + cfg.arm_length_m * math.sin(psi)


# ---------------------------------------------------------------------------
# Calibration layer
# ---------------------------------------------------------------------------
def interp_table(points: Optional[list], x: float) -> float:
    """Piecewise-linear lookup, clamped at the ends. Empty table -> 1.0."""
    if not points:
        return 1.0
    pts = sorted((float(a), float(b)) for a, b in points)
    if x <= pts[0][0]:
        return pts[0][1]
    if x >= pts[-1][0]:
        return pts[-1][1]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if x0 <= x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return 1.0  # pragma: no cover


def corrected_distance(distance_m: float, cal: CalibrationConfig) -> float:
    return distance_m * cal.distance_scale + cal.distance_offset_m


def angle_offset(profile: PassProfile, cal: CalibrationConfig) -> float:
    """Total measured (real - planned) launch angle error to compensate."""
    return cal.angle_offset_deg + profile.angle_offset_deg


def speed_multiplier(profile_name: str, distance_m: float, cfg: LauncherConfig) -> float:
    cal = cfg.calibration
    return (cal.speed_scale * cfg.profiles[profile_name].speed_scale
            * interp_table(cal.speed_table.get(profile_name), distance_m))


def apply_speed_calibration(v_physics: float, profile_name: str, distance_m: float,
                            cfg: LauncherConfig) -> float:
    return (v_physics + cfg.calibration.speed_offset_mps) * \
        speed_multiplier(profile_name, distance_m, cfg)


# ---------------------------------------------------------------------------
# Planning
# ---------------------------------------------------------------------------
def release_angle_for(profile_name: str, cfg: LauncherConfig) -> float:
    """
    Arm angle at which to release for this profile, compensating a measured
    angle error: if the ball flies 3 deg steeper than planned, release 3 deg
    later (flatter).
    """
    prof = cfg.profiles[profile_name]
    return 90.0 - (prof.launch_angle_deg - angle_offset(prof, cfg.calibration))


def _build_move(psi_rel: float, motor_rpm: int,
                cfg: LauncherConfig, warnings: List[str]) -> tuple[MotorMove, float, float, bool]:
    """Sweep/registers for a release at psi_rel with the given motor speed."""
    ok = True
    if psi_rel <= cfg.home_angle_deg:
        warnings.append(f"release angle {psi_rel:.1f} deg is not past home "
                        f"{cfg.home_angle_deg:.1f} deg; check home_angle_deg")
        ok = False

    ramp_up = accel_arm_degrees(motor_rpm, cfg)
    if ramp_up > psi_rel - cfg.home_angle_deg > 0:
        warnings.append(f"accel ramp needs {ramp_up:.0f} deg of arm travel but only "
                        f"{psi_rel - cfg.home_angle_deg:.0f} deg before release -- ball "
                        f"leaves slow (lower FA40 or start the arm further back)")

    end = psi_rel + (decel_arm_degrees(motor_rpm, cfg) if cfg.release_at_decel_start else 0.0)
    if end > cfg.max_arm_angle_deg:
        warnings.append(f"move ends at {end:.0f} deg, past max_arm_angle_deg "
                        f"{cfg.max_arm_angle_deg:.0f} (lower FA41 or rpm)")
        ok = False

    sweep = end - cfg.home_angle_deg
    total = arm_degrees_to_motor_pulses(max(sweep, 0.0), cfg.gear_ratio, cfg.pulses_per_rev)
    turns, pulses = split_pulses(total, cfg.pulses_per_rev)
    return MotorMove(turns=turns, pulses=pulses, rpm=motor_rpm), end, sweep, ok


def plan_launch(distance_m: float, profile_name: str, cfg: LauncherConfig) -> LaunchPlan:
    """
    Compute everything for one pass. `distance_m` is the horizontal distance
    from the arm pivot to the player, as measured by vision.
    """
    prof = cfg.profiles[profile_name]
    cal = cfg.calibration
    warnings: List[str] = []
    ok = True

    # 1. physics -----------------------------------------------------------
    d = corrected_distance(distance_m, cal)
    theta = prof.launch_angle_deg
    psi_rel = release_angle_for(profile_name, cfg)
    rx, rh = release_point(psi_rel, cfg)
    dx, dy = d - rx, prof.target_height_m - rh

    v_phys, flight = solve_speed(dx, dy, theta, cfg.ball)
    if not math.isfinite(v_phys):
        warnings.append("target unreachable at this launch angle")
        ok = False
        v_phys, flight = 0.0, 0.0

    # 2. calibration -------------------------------------------------------
    v_cmd = apply_speed_calibration(v_phys, profile_name, d, cfg) if ok else 0.0

    # 3. drive -------------------------------------------------------------
    arm_rpm = ball_speed_to_arm_rpm(v_cmd, cfg.arm_length_m)
    motor_rpm = motor_rpm_to_register(arm_rpm * cfg.gear_ratio)
    if motor_rpm > cfg.max_motor_rpm:
        warnings.append(f"needs {motor_rpm} motor rpm > max {cfg.max_motor_rpm}")
        ok = False
        motor_rpm = cfg.max_motor_rpm
    if ok and motor_rpm < cfg.min_motor_rpm:
        warnings.append(f"needs only {motor_rpm} motor rpm; raised to min {cfg.min_motor_rpm}")
        motor_rpm = cfg.min_motor_rpm

    move, end, sweep, move_ok = _build_move(psi_rel, motor_rpm, cfg, warnings)

    return LaunchPlan(
        pass_type=profile_name,
        distance_m=distance_m,
        target_distance_m=d,
        launch_angle_deg=theta,
        release_angle_deg=psi_rel,
        end_angle_deg=end,
        sweep_deg=sweep,
        release_x_m=rx,
        release_height_m=rh,
        physics_speed_mps=v_phys,
        exit_velocity_mps=v_cmd,
        arm_rpm=arm_rpm,
        motor_rpm=motor_rpm,
        move=move,
        flight_time_s=flight,
        ok=ok and move_ok,
        warnings=warnings,
    )


def plan_fixed_rpm(motor_rpm: int, profile_name: str, cfg: LauncherConfig) -> LaunchPlan:
    """
    A throw at a chosen motor speed with the profile's release angle -- used
    for calibration shots, where the speed is the independent variable.
    """
    warnings: List[str] = []
    psi_rel = release_angle_for(profile_name, cfg)
    rx, rh = release_point(psi_rel, cfg)
    arm_rpm = motor_rpm / cfg.gear_ratio
    v = arm_rpm_to_ball_speed(arm_rpm, cfg.arm_length_m)
    ok = 0 < motor_rpm <= cfg.max_motor_rpm
    if not ok:
        warnings.append(f"motor rpm {motor_rpm} outside 1..{cfg.max_motor_rpm}")
    move, end, sweep, move_ok = _build_move(psi_rel, int(motor_rpm), cfg, warnings)
    return LaunchPlan(
        pass_type=profile_name, distance_m=0.0, target_distance_m=0.0,
        launch_angle_deg=cfg.profiles[profile_name].launch_angle_deg,
        release_angle_deg=psi_rel, end_angle_deg=end, sweep_deg=sweep,
        release_x_m=rx, release_height_m=rh, physics_speed_mps=v, exit_velocity_mps=v,
        arm_rpm=arm_rpm, motor_rpm=int(motor_rpm), move=move, flight_time_s=0.0,
        ok=ok and move_ok, warnings=warnings)


def return_move(plan: LaunchPlan, cfg: LauncherConfig) -> MotorMove:
    """The slow move that brings the arm back to home after a throw."""
    return MotorMove(turns=-plan.move.turns, pulses=-plan.move.pulses,
                     rpm=motor_rpm_to_register(cfg.return_rpm))


def estimate_move_time_s(move: MotorMove, cfg: LauncherConfig) -> float:
    """Trapezoidal-profile duration estimate (used for completion_mode='time')."""
    revs = abs(move.turns) + abs(move.pulses) / cfg.pulses_per_rev
    rps = max(1, move.rpm) / 60.0
    t_up = rps * 60.0 / 1000.0 * cfg.accel_ms_per_1000rpm / 1000.0
    t_dn = rps * 60.0 / 1000.0 * cfg.decel_ms_per_1000rpm / 1000.0
    ramp_revs = rps * (t_up + t_dn) / 2.0
    if ramp_revs >= revs and ramp_revs > 0:     # triangular profile: never reaches rpm
        k = math.sqrt(revs / ramp_revs)
        return (t_up + t_dn) * k
    return t_up + t_dn + (revs - ramp_revs) / rps
