import math

import pytest

from passer.config import LauncherConfig
from passer.config import BallConfig
from passer.kinematics import (G, accel_arm_degrees, arm_degrees_to_motor_pulses,
                               arm_move, estimate_move_time_s, plan_launch,
                               return_move, split_pulses)


def test_quarter_arm_turn_matches_spec_example():
    # 90 deg of arm = 1/4 arm rev = 2.5 motor revs = 2 turns + 5000 pulses
    total = arm_degrees_to_motor_pulses(90, 10, 10_000)
    assert total == 25_000
    assert split_pulses(total, 10_000) == (2, 5000)


def test_negative_sweep_keeps_sign_on_both_registers():
    assert split_pulses(-25_000, 10_000) == (-2, -5000)
    assert split_pulses(-3_000, 10_000) == (0, -3000)


def test_arm_speed_multiplied_by_gear_ratio():
    m = arm_move(90, arm_rpm=30, cfg=LauncherConfig())
    assert m.rpm == 300
    assert (m.turns, m.pulses) == (2, 5000)


def test_plan_hits_target_height_without_drag():
    cfg = LauncherConfig(ball=BallConfig(drag=False))
    for name in cfg.profiles:
        p = plan_launch(5.0, name, cfg)
        assert p.ok, p.warnings
        th = math.radians(p.launch_angle_deg)
        vx, vy = p.exit_velocity_mps * math.cos(th), p.exit_velocity_mps * math.sin(th)
        t = p.flight_time_s
        y = p.release_height_m + vy * t - 0.5 * G * t * t
        assert y == pytest.approx(cfg.profiles[name].target_height_m, abs=1e-6)
        assert p.motor_rpm > 0
        # release angle geometry
        assert p.release_angle_deg == pytest.approx(90 - p.launch_angle_deg)
        assert p.sweep_deg == pytest.approx(p.end_angle_deg - cfg.home_angle_deg)


def test_farther_needs_faster():
    cfg = LauncherConfig()
    rpms = [plan_launch(d, "chest", cfg).motor_rpm for d in (2, 4, 6, 8)]
    assert rpms == sorted(rpms) and len(set(rpms)) == 4


def test_over_max_rpm_is_rejected():
    cfg = LauncherConfig(max_motor_rpm=500)
    p = plan_launch(8.0, "chest", cfg)
    assert not p.ok
    assert p.motor_rpm == 500


def test_return_move_reverses_throw():
    cfg = LauncherConfig()
    p = plan_launch(4.0, "lob", cfg)
    back = return_move(p, cfg)
    assert (back.turns, back.pulses) == (-p.move.turns, -p.move.pulses)
    assert back.rpm == cfg.return_rpm


def test_accel_ramp_degrees():
    cfg = LauncherConfig(accel_ms_per_1000rpm=100, gear_ratio=10)
    # 0->1000 rpm in 0.1 s: avg 500 rpm * 0.1 s = 0.833 motor rev = 30 arm deg
    assert accel_arm_degrees(1000, cfg) == pytest.approx(30.0)


def test_move_time_estimate_positive():
    cfg = LauncherConfig()
    p = plan_launch(5.0, "chest", cfg)
    assert 0 < estimate_move_time_s(p.move, cfg) < 1.0
