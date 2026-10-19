import pytest

from passer.config import LauncherConfig, PS100Config
from passer.hardware import ps100_registers as R
from passer.hardware.ps100 import PS100, SimTransport
from passer.kinematics import plan_launch
from passer.launcher import Launcher, LauncherState


def make(mode="level"):
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


def test_level_completion_waits_for_drop_then_rise():
    """The bit starts at 1 (in position): finishing on its first change would end at the START."""
    t = SimTransport(move_time_s=0.2)
    drive = PS100(t, PS100Config(completion_timeout_s=1.0))
    assert t.read_register(R.REG_OUTPUT_STATUS) & 1 == 1
    t0 = __import__("time").monotonic()
    assert drive.move(0, 100, 100)
    assert __import__("time").monotonic() - t0 >= 0.2


def test_level_completion_of_a_move_too_short_to_see():
    class Instant(SimTransport):
        def write_register(self, address, value):
            super().write_register(address, value)
            self.registers[R.REG_OUTPUT_STATUS] |= 1      # done before the first poll
    drive = PS100(Instant(), PS100Config(completion_timeout_s=1.0))
    assert drive.move(0, 100, 100, est_time_s=0.05)


def test_timeout_scales_with_long_moves():
    t = SimTransport(move_time_s=0.3)
    drive = PS100(t, PS100Config(completion_timeout_s=0.05))
    assert drive.move(0, 100, 30, est_time_s=0.3)        # 0.05 s alone would time out


def test_reverse_direction_negates_the_move_only():
    t = SimTransport(move_time_s=0.01)
    drive = PS100(t, PS100Config(reverse_direction=True))
    assert drive.move(2, 5000, 300)
    assert t.writes[:3] == [(R.REG_POSITION_TURNS, -2), (R.REG_POSITION_PULSES, -5000),
                            (R.REG_POSITION_SPEED, 300)]


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


@pytest.mark.parametrize("name,address", [
    ("PA21", 0x0015), ("PA24", 0x0018), ("PA50", 0x0032), ("PA65", 0x0041),
    ("P3-31", 0x011F), ("P3-34", 0x0122), ("P4-2", 0x0202), ("P4-4", 0x0204),
    ("fa14", 0x000E), ("P3-38", 0x0126), ("FC-0", 0x0100),
])
def test_param_address_matches_manual(name, address):
    assert R.param_address(name) == address


@pytest.mark.parametrize("bad", ["P5-1", "PA", "x14", "P3-x"])
def test_param_address_rejects_unknown(bad):
    with pytest.raises(ValueError):
        R.param_address(bad)


def test_runtime_registers_follow_the_address_rule():
    assert R.param_address("P3-31") == R.REG_VIRTUAL_INPUT_CONTROL
    assert R.param_address("P4-2") == R.REG_POSITION_TURNS
    assert R.param_address("P4-3") == R.REG_POSITION_PULSES
    assert R.param_address("P4-4") == R.REG_POSITION_SPEED


def test_check_settings_flags_mismatches():
    drive, t = make()
    for name, expected, _ in R.REQUIRED_SETTINGS:
        t.registers[R.param_address(name)] = expected
    t.registers[R.param_address("P3-38")] = 0
    rows = {name: (expected, actual) for name, expected, actual, _ in drive.check_settings()}
    assert rows["P3-38"] == (28, 0)
    assert rows["PA14"] == (3, 3)
    assert rows["PA40"][0] is None


def test_check_settings_survives_unreadable_parameter():
    class Flaky(SimTransport):
        def read_register(self, address):
            if address == R.param_address("PA74"):
                raise IOError("no answer")
            return super().read_register(address)

    drive = PS100(Flaky(), PS100Config())
    rows = {name: actual for name, _, actual, _ in drive.check_settings()}
    assert rows["PA74"] is None
    assert rows["PA4"] == 0


def test_read_param_is_signed():
    drive, t = make()
    t.registers[R.param_address("P4-3")] = 0xFFFF
    assert drive.read_param("P4-3") == -1


def test_read_position_combines_words_low_first():
    drive, t = make()
    for i, w in enumerate([0x5678, 0x1234, 0, 0]):
        t.registers[R.REG_ABS_POSITION + i] = w
    assert drive.read_position() == 0x12345678
    for i in range(4):
        t.registers[R.REG_ABS_POSITION + i] = 0xFFFF
    assert drive.read_position() == -1


def test_position_is_read_in_one_request():
    """Four separate reads can tear when the value wobbles around zero (-1 read as 2**32-1)."""
    drive, t = make()
    calls = []
    real = t.read_registers
    t.read_registers = lambda a, n: calls.append((a, n)) or real(a, n)
    drive.read_position()
    assert calls == [(R.REG_ABS_POSITION, 4)]


def test_monitor_decodes_status_registers(capsys):
    drive, t = make()
    t.registers[R.REG_ALARM] = 57
    t.registers[R.REG_POSITION_COMMAND] = 0xFFFE           # -2 across two words
    t.registers[R.REG_POSITION_COMMAND + 1] = 0xFFFF
    t.registers[R.REG_INPUT_STATUS] = 0b0001
    rows = {addr: value for addr, _, value in drive.monitor()}
    assert rows[R.REG_ALARM] == 57 and rows[R.REG_POSITION_COMMAND] == -2
    assert rows[R.REG_INPUT_STATUS] == 1 and len(rows) == len(R.MONITOR)


def test_cli_check_and_alarm(capsys):
    from passer.tools import ps100_cli
    ps100_cli.main(["--dry-run", "check"])
    out = capsys.readouterr().out
    assert "FIX PA14" in out and "setting(s) differ" in out
    ps100_cli.main(["alarm", "57"])
    assert "Modbus format error" in capsys.readouterr().out
    ps100_cli.main(["--dry-run", "read", "P3-38"])
    assert "0x0126" in capsys.readouterr().out
    ps100_cli.main(["--dry-run", "monitor"])
    assert "alarm (0 = none)" in capsys.readouterr().out
