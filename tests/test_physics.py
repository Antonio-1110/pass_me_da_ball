import math

import pytest

from passer.calibration import (analyse, fit, load_shots, make_shot, predict_landing,
                                save_shot)
from passer.config import BallConfig, LauncherConfig
from passer.kinematics import (arm_rpm_to_ball_speed, interp_table, plan_fixed_rpm,
                               plan_launch, release_angle_for, release_point)
from passer.physics import (G, cross_at, drag_k, solve_speed, solve_speed_and_angle,
                            trajectory, vacuum_speed)

NO_DRAG = BallConfig(drag=False)
BALL = BallConfig()


def test_vacuum_range_equation():
    assert vacuum_speed(10.0, 0.0, 45.0) == pytest.approx(math.sqrt(10.0 * G))
    assert vacuum_speed(3.0, 5.0, 10.0) == math.inf


def test_basketball_drag_constant():
    assert drag_k(BALL) == pytest.approx(0.0219, rel=0.05)
    assert drag_k(NO_DRAG) == 0.0


def test_rk4_matches_closed_form_without_drag():
    ball = BallConfig(drag=True, drag_coeff=0.0)   # forces the integrator path
    c = cross_at(10.0, 30.0, 6.0, ball)
    t = 6.0 / (10.0 * math.cos(math.radians(30)))
    assert c.t == pytest.approx(t, abs=1e-4)
    assert c.y == pytest.approx(10.0 * math.sin(math.radians(30)) * t - 0.5 * G * t * t, abs=1e-4)


@pytest.mark.parametrize("angle", [15.0, 35.0, 55.0])
def test_solve_speed_hits_target_with_drag(angle):
    v, t = solve_speed(7.0, 0.3, angle, BALL)
    c = cross_at(v, angle, 7.0, BALL)
    assert c.y == pytest.approx(0.3, abs=2e-3)
    assert c.t == pytest.approx(t, abs=1e-3)
    assert v > vacuum_speed(7.0, 0.3, angle)       # drag always costs speed


def test_solve_speed_and_angle_roundtrip():
    v0, a0 = 9.0, 40.0
    c = cross_at(v0, a0, 6.0, BALL)
    v, a = solve_speed_and_angle(6.0, c.y, c.t, BALL)
    assert a == pytest.approx(a0, abs=0.05)
    assert v == pytest.approx(v0, abs=0.02)


def test_speed_scales_with_arm_length():
    short, long_ = LauncherConfig(arm_length_m=0.5), LauncherConfig(arm_length_m=1.0)
    ps, pl = plan_launch(6.0, "chest", short), plan_launch(6.0, "chest", long_)
    # ~same ball speed, so a 2x longer arm needs ~half the rpm
    assert pl.motor_rpm == pytest.approx(ps.motor_rpm / 2, rel=0.1)


def test_calibration_knobs():
    base = plan_launch(6.0, "chest", LauncherConfig())
    cfg = LauncherConfig()
    cfg.calibration.speed_scale = 1.1
    assert plan_launch(6.0, "chest", cfg).exit_velocity_mps == pytest.approx(base.exit_velocity_mps * 1.1)

    cfg = LauncherConfig()
    cfg.profiles["chest"].speed_scale = 0.9
    assert plan_launch(6.0, "chest", cfg).exit_velocity_mps == pytest.approx(base.exit_velocity_mps * 0.9)

    cfg = LauncherConfig()
    cfg.calibration.speed_offset_mps = 0.5
    assert plan_launch(6.0, "chest", cfg).exit_velocity_mps == pytest.approx(base.exit_velocity_mps + 0.5)

    cfg = LauncherConfig()
    cfg.calibration.angle_offset_deg = 3.0       # machine throws 3 deg steep
    assert plan_launch(6.0, "chest", cfg).release_angle_deg == pytest.approx(base.release_angle_deg + 3)

    cfg = LauncherConfig()
    cfg.calibration.distance_scale = 0.5
    assert plan_launch(12.0, "chest", cfg).target_distance_m == pytest.approx(6.0)

    cfg = LauncherConfig()
    cfg.calibration.speed_table = {"chest": [[4.0, 1.0], [8.0, 1.2]]}
    assert plan_launch(6.0, "chest", cfg).exit_velocity_mps == pytest.approx(base.exit_velocity_mps * 1.1)


def test_interp_table():
    t = [[8.0, 1.2], [4.0, 1.0]]
    assert interp_table(t, 2.0) == 1.0 and interp_table(t, 10.0) == 1.2
    assert interp_table(t, 6.0) == pytest.approx(1.1)
    assert interp_table([], 5.0) == 1.0


def test_release_at_decel_start_extends_sweep():
    a, b = LauncherConfig(), LauncherConfig(release_at_decel_start=False)
    pa, pb = plan_launch(5.0, "chest", a), plan_launch(5.0, "chest", b)
    assert pa.end_angle_deg > pa.release_angle_deg
    assert pb.end_angle_deg == pytest.approx(pb.release_angle_deg)
    assert pa.sweep_deg > pb.sweep_deg


def _fake_real_shot(cfg, profile, rpm, speed_ratio, angle_err):
    """What a machine that throws `speed_ratio` x ideal, angle_err deg steep would do."""
    psi = release_angle_for(profile, cfg)
    rx, rh = release_point(psi, cfg)
    v = speed_ratio * arm_rpm_to_ball_speed(rpm / cfg.gear_ratio, cfg.arm_length_m)
    pts = trajectory(v, 90 - psi + angle_err, cfg.ball, x0=rx, y0=rh, stop_y=0.0, dt=0.0005)
    (ta, xa, ya), (tb, xb, yb) = pts[-2], pts[-1]
    f = ya / (ya - yb)
    return xa + f * (xb - xa), ta + f * (tb - ta)


def test_fit_recovers_speed_and_angle_error(tmp_path):
    cfg = LauncherConfig()
    path = tmp_path / "shots.csv"
    for prof in ("chest", "lob"):
        for rpm in (1000, 1300, 1600):
            landed, t = _fake_real_shot(cfg, prof, rpm, speed_ratio=0.9, angle_err=3.0)
            save_shot(path, make_shot(prof, rpm, landed, cfg, flight_time_s=t))
    shots = load_shots(path)
    assert len(shots) == 6
    result = fit(shots, cfg)
    assert result.speed_scale == pytest.approx(1 / 0.9, rel=0.01)
    assert result.angle_offsets["chest"] == pytest.approx(3.0, abs=0.1)
    assert result.angle_offsets["lob"] == pytest.approx(3.0, abs=0.1)
    assert result.speed_table == {}              # error is uniform -> no table
    assert "speed_scale: 1.11" in result.yaml()


def test_fit_builds_table_for_distance_dependent_error():
    cfg = LauncherConfig()
    shots = []
    for rpm, ratio in ((900, 0.95), (1300, 0.90), (1700, 0.85)):
        landed, _ = _fake_real_shot(cfg, "chest", rpm, ratio, 0.0)
        shots.append(make_shot("chest", rpm, landed, cfg))
    result = fit(shots, cfg)
    assert "chest" in result.speed_table
    mults = [m for _, m in result.speed_table["chest"]]
    assert mults == sorted(mults)                # weaker at long range -> bigger boost


def test_calibrated_plan_lands_on_target():
    """After fitting, a planned pass on the 'real' machine reaches the target."""
    cfg = LauncherConfig()
    shots = []
    for rpm in (1000, 1300, 1600):
        landed, t = _fake_real_shot(cfg, "chest", rpm, 0.88, 2.0)
        shots.append(make_shot("chest", rpm, landed, cfg, flight_time_s=t))
    r = fit(shots, cfg)
    cfg.calibration.speed_scale = r.speed_scale
    cfg.profiles["chest"].angle_offset_deg = r.angle_offsets["chest"]

    plan = plan_launch(6.0, "chest", cfg)
    real_v = 0.88 * arm_rpm_to_ball_speed(plan.arm_rpm, cfg.arm_length_m)
    real_angle = 90 - plan.release_angle_deg + 2.0
    c = cross_at(real_v, real_angle, 6.0 - plan.release_x_m, cfg.ball)
    assert plan.release_height_m + c.y == pytest.approx(1.3, abs=0.05)


def test_predict_landing_and_fixed_rpm_plan():
    cfg = LauncherConfig()
    d1, d2 = predict_landing("chest", 1000, cfg), predict_landing("chest", 1500, cfg)
    assert 0 < d1 < d2
    p = plan_fixed_rpm(1500, "chest", cfg)
    assert p.ok and p.motor_rpm == 1500
    an = analyse(make_shot("chest", 1500, d2, cfg), cfg)
    assert an.multiplier == pytest.approx(1.0, rel=0.01)


def test_untimed_shots_use_fitted_angle_of_their_profile():
    cfg = LauncherConfig()
    shots = []
    for rpm in (1000, 1300):
        landed, t = _fake_real_shot(cfg, "chest", rpm, 0.9, 4.0)
        shots.append(make_shot("chest", rpm, landed, cfg, flight_time_s=t))
    landed, _ = _fake_real_shot(cfg, "chest", 1600, 0.9, 4.0)
    shots.append(make_shot("chest", 1600, landed, cfg))           # no timing
    r = fit(shots, cfg)
    assert r.analyses[2].real_angle == pytest.approx(20 + 4.0, abs=0.1)
    assert r.analyses[2].multiplier == pytest.approx(1 / 0.9, rel=0.01)
