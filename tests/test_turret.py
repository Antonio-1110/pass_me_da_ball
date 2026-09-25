"""Two-axis turret, prediction and aiming, in a small simulated world."""

import math
import random

import pytest

from passer.aiming import apply_lateral_lead, solve_shot
from passer.config import (CameraAxisConfig, Config, LauncherAxisConfig, LauncherConfig,
                           PredictionConfig)
from passer.controller import MachineController
from passer.hardware.pan_axis import VirtualPanAxis
from passer.hardware.ps100 import PS100, SimTransport
from passer.hardware import ps100_registers as R
from passer.launcher import Launcher
from passer.prediction import PlayerEstimator, polar_to_xy, xy_to_polar
from passer.turret import AngleHistory, MotionProfile, TwoAxisTurret
from passer.vision.gestures import Gesture

DT = 1 / 30
LATENCY = 0.1          # capture -> vision result


def make_turret(camera_physical=True, launcher_physical=True, **lau):
    cam_cfg, lau_cfg, pred = CameraAxisConfig(), LauncherAxisConfig(**lau), PredictionConfig()
    cam = VirtualPanAxis(cam_cfg.min_deg, cam_cfg.max_deg, physical=camera_physical)
    lau_axis = VirtualPanAxis(lau_cfg.min_deg, lau_cfg.max_deg, physical=launcher_physical)
    return TwoAxisTurret(cam, lau_axis, cam_cfg, lau_cfg, pred)


class World:
    """
    Player moving on the floor; a camera whose image offset depends on the
    real camera heading at capture time; vision results arrive LATENCY later.
    """

    def __init__(self, turret, player, range_noise=0.0, seed=0):
        self.turret, self.player = turret, player      # player(t) -> (x, y)
        self.t = 0.0
        self.frame = 0
        self.queue = []                                 # (deliver_at, frame, t_cap, err, d)
        self.rng = random.Random(seed)
        self.range_noise = range_noise
        self.max_image_err = 0.0

    def true_polar(self, t=None):
        return xy_to_polar(*self.player(self.t if t is None else t))

    def step(self, controller=None):
        obs = None
        while self.queue and self.queue[0][0] <= self.t + 1e-9:
            obs = self.queue.pop(0)
        if controller is not None:
            kw = {} if obs is None else dict(frame_id=obs[1], t_frame=obs[2],
                                             pan_error_deg=obs[3], distance_m=obs[4],
                                             reliable=True)
            controller.tick(self.t, DT, **kw)
        else:
            if obs is not None:
                self.turret.observe(obs[1], obs[2], obs[3], obs[4], True)
            self.turret.update(self.t, DT)
        # camera exposes a frame with the axes where they were just commanded
        b, d = self.true_polar()
        err = b - self.turret.camera_heading()
        self.max_image_err = max(self.max_image_err, abs(err)) if self.t > 1.0 else 0.0
        self.frame += 1
        d_meas = d * (1 + self.rng.gauss(0, self.range_noise))
        self.queue.append((self.t + LATENCY, self.frame, self.t, err, d_meas))
        self.t += DT

    def run(self, seconds, controller=None):
        for _ in range(int(seconds / DT)):
            self.step(controller)


# ---------------------------------------------------------------------------
def test_motion_profile_limits_and_arrives():
    m = MotionProfile(max_speed=45, max_accel=90)
    vmax = amax = 0.0
    prev_v = 0.0
    for _ in range(200):
        p0 = m.pos
        m.update(40.0, DT)
        v = (m.pos - p0) / DT
        vmax, amax = max(vmax, abs(v)), max(amax, abs(v - prev_v) / DT)
        prev_v = v
    assert m.pos == pytest.approx(40.0, abs=1e-6)
    assert vmax <= 45 + 1e-6 and amax <= 90 + 1e-6


def test_motion_profile_follows_ramp_without_lag():
    m = MotionProfile(max_speed=45, max_accel=90)
    for i in range(300):
        m.update(20.0 * i * DT, DT)                # target moves at 20 deg/s
    assert m.pos == pytest.approx(20.0 * 299 * DT, abs=0.5)


def test_angle_history_interpolates():
    h = AngleHistory()
    h.add(0.0, 0.0, 10.0)
    h.add(1.0, 20.0, 0.0)
    assert h.at(0.5) == pytest.approx((10.0, 5.0))
    assert h.at(-1) == (0.0, 10.0) and h.at(5) == (20.0, 0.0)


def test_estimator_recovers_velocity():
    est = PlayerEstimator(PredictionConfig())
    rng = random.Random(1)
    for i in range(60):
        t = i * DT
        x, y = -3 + 2.0 * t, 6.0                  # crossing left -> right at 2 m/s
        b, d = xy_to_polar(x, y)
        est.update(t, b + rng.gauss(0, 0.5), d * (1 + rng.gauss(0, 0.05)))
    x, y, vx, vy = est.state_at(59 * DT)
    assert vx == pytest.approx(2.0, abs=0.4)
    assert vy == pytest.approx(0.0, abs=0.6)


def test_lateral_lead_is_perpendicular_to_line_of_sight():
    assert apply_lateral_lead(0.0, 5.0, 1.0) == pytest.approx((1.0, 5.0))
    x, y = apply_lateral_lead(*polar_to_xy(90.0, 5.0), 1.0)   # player hard right
    assert (x, y) == pytest.approx((5.0, -1.0), abs=1e-9)


def test_stationary_player_both_axes_converge():
    turret = make_turret()
    world = World(turret, lambda t: polar_to_xy(25.0, 6.0))
    world.run(4.0)
    assert turret.launcher.angle() == pytest.approx(25.0, abs=1.0)
    assert turret.camera_heading() == pytest.approx(25.0, abs=1.5)
    assert turret.camera.angle() == pytest.approx(0.0, abs=2.0)  # launcher did the work


def test_camera_counter_rotates_when_launcher_swings():
    turret = make_turret()
    world = World(turret, lambda t: polar_to_xy(0.0, 6.0))
    world.run(1.0)
    turret.set_aim(30.0)                             # e.g. a big lead pass
    world.run(2.0)
    assert turret.launcher.angle() == pytest.approx(30.0, abs=0.5)
    assert turret.camera.angle() == pytest.approx(-30.0, abs=1.5)
    assert world.max_image_err < 8.0                 # player never leaves the frame


def test_launcher_leads_moving_player_camera_follows():
    turret = make_turret()
    world = World(turret, lambda t: (-3 + 1.5 * t, 6.0), range_noise=0.03)
    world.run(3.0)
    b_now, _ = world.true_polar()
    assert turret.launcher.angle() > b_now + 2.0     # pointing ahead of the player
    assert turret.camera_heading() == pytest.approx(b_now, abs=4.0)
    assert world.max_image_err < 10.0


def test_virtual_camera_axis_counts_as_fixed():
    turret = make_turret(camera_physical=False, launcher_physical=False)
    turret.observe(1, 0.0, 12.0, 5.0, True)
    turret.update(0.0, DT)
    assert turret.last_bearing == pytest.approx(12.0)


def test_solve_shot_catch_point_is_consistent():
    est = PlayerEstimator(PredictionConfig())
    for i in range(30):
        t = i * DT
        est.update(t, *xy_to_polar(-2 + 2.0 * t, 6.0))
    now = 29 * DT
    sol = solve_shot(est, now, "chest", 0.0, LauncherConfig(), LauncherAxisConfig())
    x, y, _, _ = est.state_at(now + sol.catch_in_s)
    assert sol.catch_xy == pytest.approx((x, y), abs=0.1)
    assert sol.catch_in_s == pytest.approx(0.25 + sol.plan.flight_time_s, abs=0.02)
    assert sol.angle_deg > xy_to_polar(*est.state_at(now)[:2])[0]


def make_controller(**lau):
    cfg = Config()
    for k, v in lau.items():
        setattr(cfg.launcher_axis, k, v)
    turret = make_turret(**lau)
    transport = SimTransport(move_time_s=0.01)
    launcher = Launcher(PS100(transport, cfg.ps100), cfg.launcher, reload_delay_s=0)
    return MachineController(cfg, turret, launcher), transport


def test_controller_fires_lead_pass_at_moving_player():
    ctl, transport = make_controller()
    world = World(ctl.turret, lambda t: (-2 + 1.0 * t, 5.0))
    world.run(2.0, ctl)
    assert ctl.request_shot(Gesture.PASS_LEFT, world.t)
    for _ in range(90):
        world.step(ctl)
        if ctl.pending is None:
            break
    assert ctl.pending is None and ctl.status.startswith("FIRED"), ctl.status
    sol = ctl.last_solution
    b_now, _ = world.true_polar()
    # ahead of the player: motion (+x) and pass-left lead (+x) both push right
    assert sol.angle_deg > b_now + 5.0
    ctl.launcher.wait(2.0)
    assert (R.REG_POSITION_SPEED, sol.plan.motor_rpm) in transport.writes


def test_controller_refuses_without_distance_and_times_out():
    ctl, transport = make_controller(aim_timeout_s=0.5, max_speed_dps=1.0)
    assert not ctl.request_shot(Gesture.CHEST, 0.0)          # no player yet
    world = World(ctl.turret, lambda t: polar_to_xy(40.0, 6.0))
    ctl.turret.launcher_target = None
    ctl.cfg.launcher_axis.track_between_shots = False
    ctl.turret.lau_cfg.track_between_shots = False
    world.run(0.5, ctl)
    assert ctl.request_shot(Gesture.CHEST, world.t)
    world.run(1.0, ctl)                                       # turret far too slow
    assert ctl.pending is None and "could not aim" in ctl.status
    assert transport.writes == []
