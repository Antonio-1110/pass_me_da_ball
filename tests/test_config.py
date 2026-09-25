from pathlib import Path

import pytest

from passer.config import load_config

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("name", ["rpi.yaml", "macbook.yaml"])
def test_shipped_configs_load(name):
    cfg = load_config(ROOT / "configs" / name)
    assert cfg.launcher.gear_ratio == 10
    assert {"chest", "lob"} <= set(cfg.launcher.profiles)


def test_overrides_and_unknown_keys():
    cfg = load_config(overrides={"launcher": {"profiles": {"lob": {"launch_angle_deg": 60}}},
                                 "camera": {"main_size": [1280, 720]}})
    assert cfg.launcher.profiles["lob"].launch_angle_deg == 60
    assert cfg.launcher.profiles["lob"].target_height_m == 2.0
    assert cfg.camera.main_size == (1280, 720)
    with pytest.raises(KeyError):
        load_config(overrides={"camera": {"nope": 1}})
