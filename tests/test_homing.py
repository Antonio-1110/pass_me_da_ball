"""Homing from the motor's single-turn absolute encoder."""

import pytest
import yaml

from passer.config import Config, load_config
from passer.hardware.ps100 import PS100, SimTransport
from passer.homing import arm_offset_deg, at_home, home, save_home

N = 131072            # encoder counts per motor turn
PER_DEG = N / 36.0    # counts per arm degree at 10:1


def machine(home_counts=1000, reverse=True, rotor=1000):
    cfg = Config()
    cfg.ps100.home_rotor_counts, cfg.ps100.reverse_direction = home_counts, reverse
    t = SimTransport(move_time_s=0.01)
    t.set_rotor(rotor)
    return cfg, PS100(t, cfg.ps100), t


def test_offset_sign_and_wrap():
    cfg, _, _ = machine()
    assert arm_offset_deg(1000, cfg) == 0
    # reverse_direction: rotor counts rise as the arm moves AWAY from the throw
    assert arm_offset_deg(1000 + round(5 * PER_DEG), cfg) == pytest.approx(-5, abs=0.01)
    cfg.ps100.reverse_direction = False
    assert arm_offset_deg(1000 + round(5 * PER_DEG), cfg) == pytest.approx(5, abs=0.01)
    # across the 0 / 131071 seam: the short way round
    cfg.ps100.home_rotor_counts = 100
    assert arm_offset_deg(N - round(2 * PER_DEG) + 100, cfg) == pytest.approx(-2, abs=0.01)


@pytest.mark.parametrize("off_deg", [-12.0, -0.3, 3.0, 14.0])
def test_home_moves_the_arm_to_home(off_deg):
    cfg, drive, t = machine(reverse=True)
    t.set_rotor(1000 - round(off_deg * PER_DEG))   # reverse: toward the throw = fewer counts
    before, after = home(drive, cfg)
    assert before == pytest.approx(off_deg, abs=0.01) and abs(after) < 0.05


def test_home_refuses_far_offsets_and_uncalibrated():
    cfg, drive, t = machine()
    t.set_rotor(1000 + round(17 * PER_DEG))
    with pytest.raises(ValueError, match="within"):
        home(drive, cfg)
    assert t.writes == []
    cfg.ps100.home_rotor_counts = None
    with pytest.raises(ValueError, match="not calibrated"):
        home(drive, cfg)


def test_home_aborts_if_the_correction_goes_the_wrong_way():
    import dataclasses
    cfg, _, t = machine(reverse=False)      # homing believes the motor isn't reversed...
    drive = PS100(t, dataclasses.replace(cfg.ps100, reverse_direction=True))   # ...but it is
    t.set_rotor(1000 + round(6 * PER_DEG))
    with pytest.raises(RuntimeError, match="wrong way"):
        home(drive, cfg)
    assert len([w for w in t.writes if w[0] == 0x011F and w[1] == 1]) == 1   # only the 1 deg probe


def test_at_home():
    cfg, drive, t = machine()
    assert at_home(drive, cfg)[0]
    t.set_rotor(1000 + round(3 * PER_DEG))
    ok, why = at_home(drive, cfg)
    assert not ok and "ps100_cli home" in why
    cfg.ps100.home_rotor_counts = None
    assert "not calibrated" in at_home(drive, cfg)[1]


def test_save_home_keeps_the_rest_of_the_file(tmp_path):
    p = tmp_path / "rpi.yaml"
    p.write_text("# machine\nps100:\n  port: /dev/ttyUSB0   # adapter\n\napp:\n  dry_run: true\n")
    save_home(p, 4242)
    save_home(p, 4343)                       # updates, doesn't duplicate
    text = p.read_text()
    assert text.count("home_rotor_counts") == 1 and "# adapter" in text and "# machine" in text
    cfg = load_config(p)
    assert cfg.ps100.home_rotor_counts == 4343 and cfg.ps100.port == "/dev/ttyUSB0"
    assert yaml.safe_load(text)["app"] == {"dry_run": True}
