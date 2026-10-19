"""Player profiles: validation, storage, and applying/restoring them for a session."""

import pytest

from passer.config import Config, apply_overrides
from passer.players import PlayerStore, current_values, overrides, player_id, validate

PASSES = ["chest", "lob"]


def test_validate_keeps_only_whitelisted_fields_in_range():
    p = validate({"name": " Antonio ", "person_height_m": 1.78, "lead_m": None,
                  "passes": {"lob": {"launch_angle_deg": 60, "target_height_m": None}}}, PASSES)
    assert p == {"name": "Antonio", "person_height_m": 1.78, "passes": {"lob": {"launch_angle_deg": 60.0}}}
    for bad in ({"name": "x", "person_height_m": 3.5},                       # out of range
                {"name": "x", "max_motor_rpm": 9000},                         # not a player field
                {"name": "x", "passes": {"lob": {"speed_scale": 2}}},
                {"name": "x", "passes": {"rocket": {"launch_angle_deg": 30}}},
                {"name": "x", "person_height_m": "tall"},
                {"name": ""}):
        with pytest.raises((ValueError, TypeError)):
            validate(bad, PASSES)


def test_player_ids_are_file_safe():
    assert player_id("Antonio M.") == "antonio-m"
    with pytest.raises(ValueError):
        player_id("../..")


def test_profile_applies_and_restores_exactly():
    cfg = Config()
    before = (cfg.distance.person_height_m, cfg.launcher_axis.lead_m,
              cfg.launcher.profiles["lob"].launch_angle_deg, cfg.launcher.profiles["lob"].target_height_m)
    patch = overrides(validate({"name": "Tall", "person_height_m": 1.95, "lead_m": 2.0,
                                "passes": {"lob": {"launch_angle_deg": 60}}}, PASSES))
    restore = current_values(cfg, patch)
    apply_overrides(cfg, patch)
    assert cfg.distance.person_height_m == 1.95 and cfg.launcher_axis.lead_m == 2.0
    assert cfg.launcher.profiles["lob"].launch_angle_deg == 60.0
    apply_overrides(cfg, restore)
    assert (cfg.distance.person_height_m, cfg.launcher_axis.lead_m,
            cfg.launcher.profiles["lob"].launch_angle_deg,
            cfg.launcher.profiles["lob"].target_height_m) == before


def test_store_create_rename_select_delete(tmp_path):
    cfg, store = Config(), PlayerStore(tmp_path / "players")
    a = store.save({"name": "Ana", "person_height_m": 1.6}, cfg)
    assert a["id"] == "ana" and store.get("ana")["person_height_m"] == 1.6
    with pytest.raises(ValueError):
        store.save({"name": "ana"}, cfg)                      # duplicate
    store.select("ana")
    store.save({"id": "ana", "name": "Ana B", "person_height_m": 1.62}, cfg)   # rename
    assert store.get("ana") is None and store.selected == "ana-b"
    assert [p["id"] for p in store.list()] == ["ana-b"]
    store.delete("ana-b")
    assert store.list() == [] and store.selected is None
    with pytest.raises(ValueError):
        store.select("ghost")


def test_session_applies_selected_player_and_restores_after(tmp_path):
    from passer.app import PassingMachine
    from passer.controller import MachineController

    cfg = Config()
    m = PassingMachine.__new__(PassingMachine)          # skip camera/models
    m.cfg, m.ctl = cfg, MachineController.from_config(cfg)
    m.players, m._player_restore, m.session_player = PlayerStore(tmp_path), None, None
    m.players.save({"name": "Tall", "person_height_m": 1.95,
                    "passes": {"lob": {"target_height_m": 2.4}}}, cfg)
    m.players.select("tall")

    m._start_session(0.0, "test")
    assert m.ctl.session_active and m.session_player == "Tall"
    assert cfg.distance.person_height_m == 1.95 and cfg.launcher.profiles["lob"].target_height_m == 2.4
    m._end_session("test")
    assert not m.ctl.session_active and m.session_player is None
    assert cfg.distance.person_height_m == 1.75 and cfg.launcher.profiles["lob"].target_height_m == 2.0

    m.players.select(None)                                # machine settings only
    m._start_session(0.0, "test")
    assert m.session_player is None and cfg.distance.person_height_m == 1.75


def test_shutdown_ends_session_and_stops_drive(tmp_path):
    from unittest.mock import Mock

    from passer.app import PassingMachine
    from passer.controller import MachineController
    from passer.hardware import ps100_registers as R

    cfg = Config()
    cfg.app.headless = True
    m = PassingMachine.__new__(PassingMachine)
    m.cfg, m.ctl, m.web, m.pipeline, m.camera = cfg, MachineController.from_config(cfg), None, Mock(), Mock()
    m.players, m._player_restore, m.session_player = PlayerStore(tmp_path), None, None
    m.pipeline.stop.side_effect = RuntimeError("boom")    # one failing step must not skip the rest
    m._start_session(0.0, "test")
    m.shutdown()
    assert not m.ctl.session_active
    assert (R.REG_VIRTUAL_INPUT_CONTROL, R.CONTROL_POSITION_STOP) in m.ctl.launcher.drive.t.writes
    m.camera.stop.assert_called_once()


def test_live_session_refused_unless_the_arm_is_at_home(tmp_path):
    from passer.app import PassingMachine
    from passer.controller import MachineController

    cfg = Config()
    m = PassingMachine.__new__(PassingMachine)
    m.cfg, m.ctl = cfg, MachineController.from_config(cfg)       # simulated drive
    m.players, m._player_restore, m.session_player = PlayerStore(tmp_path), None, None
    cfg.app.dry_run = False                                       # as if LIVE
    m._start_session(0.0, "test")
    assert not m.ctl.session_active and "not calibrated" in m.ctl.status
    cfg.ps100.home_rotor_counts = m.ctl.launcher.drive.rotor_angle()
    m._start_session(0.0, "test")
    assert m.ctl.session_active
