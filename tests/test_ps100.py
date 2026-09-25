import pytest

from passer.config import LauncherConfig, PS100Config
from passer.hardware import ps100_registers as R
from passer.hardware.ps100 import PS100, SimTransport
from passer.kinematics import plan_launch
from passer.launcher import Launcher, LauncherState


def make(mode="change"):
    t = SimTransport(move_time_s=0.01)
    return PS100(t, PS100Config(completion_mode=mode, completion_timeout_s=1.0)), t


def test_move_writes_registers_in_order_with_rising_edge():
    drive, t = make()
    assert drive.move(2, 5000, 300)
    assert t.writes == [
        (R.REG_POSITION_TURNS, 2),
        (R.REG_POSITION_PULSES, 5000),
        (R.REG_POSITION_SPEED, 300),
        (R.REG_VIRTUAL_INPUT_CONTROL, 0),
        (R.REG_VIRTUAL_INPUT_CONTROL, 1),
    ]


@pytest.mark.parametrize("turns,pulses,rpm", [(1, 0, 0), (1, -5, 100), (0, 10_000, 100),
                                              (40_000, 0, 100)])
def test_invalid_moves_rejected(turns, pulses, rpm):
    drive, t = make()
    with pytest.raises(ValueError):
        drive.move(turns, pulses, rpm)
    assert t.writes == []


def test_completion_by_time():
    drive, _ = make("time")
    assert drive.move(0, 100, 100, est_time_s=0.01)


def test_timeout_when_drive_never_completes():
    t = SimTransport(move_time_s=10)
    drive = PS100(t, PS100Config(completion_timeout_s=0.05))
    assert drive.move(0, 100, 100) is False


def test_launcher_fires_then_returns():
    drive, t = make()
    cfg = LauncherConfig()
    plan = plan_launch(4.0, "chest", cfg)
    launcher = Launcher(drive, cfg, reload_delay_s=0)
    assert launcher.fire(plan, block=True)
    assert launcher.state is LauncherState.READY
    turns = [v for a, v in t.writes if a == R.REG_POSITION_TURNS]
    pulses = [v for a, v in t.writes if a == R.REG_POSITION_PULSES]
    assert turns == [plan.move.turns, -plan.move.turns]
    assert pulses == [plan.move.pulses, -plan.move.pulses]


def test_launcher_refuses_bad_plan():
    drive, t = make()
    cfg = LauncherConfig(max_motor_rpm=100)
    plan = plan_launch(8.0, "chest", cfg)
    assert not Launcher(drive, cfg, 0).fire(plan)
    assert t.writes == []


def test_launcher_fault_on_timeout():
    t = SimTransport(move_time_s=10)
    drive = PS100(t, PS100Config(completion_timeout_s=0.05))
    cfg = LauncherConfig()
    launcher = Launcher(drive, cfg, 0)
    launcher.fire(plan_launch(4.0, "chest", cfg), block=True)
    assert launcher.state is LauncherState.FAULT
    assert (R.REG_VIRTUAL_INPUT_CONTROL, R.CONTROL_POSITION_STOP) in t.writes
    launcher.reset()
    assert launcher.ready
