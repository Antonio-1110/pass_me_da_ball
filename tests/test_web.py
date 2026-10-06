"""Dashboard server: HTTP API, safety rules, streaming."""

import json
import socket
import threading
import time
import urllib.error
import urllib.request

import numpy as np
import pytest
import yaml

from passer.config import Config, apply_overrides, field_docs, load_config
from passer.players import PlayerStore
from passer.web.server import WebServer


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def web(tmp_path):
    cfg = Config()
    cfg.web.host, cfg.web.port = "127.0.0.1", free_port()
    cfg.web.save_path = str(tmp_path / "live.yaml")
    actions, session = [], {"on": False}
    srv = WebServer(cfg, lambda: {"mode": "DRY RUN"},
                    lambda a: (actions.append(a) or True, f"{a} requested"),
                    in_session=lambda: session["on"],
                    players=PlayerStore(tmp_path / "players"))
    srv.actions, srv.session = actions, session
    srv.start()
    srv.url = f"http://127.0.0.1:{cfg.web.port}"
    yield srv
    srv.stop()


def call(srv, path, body=None, ctype="application/json", headers=None):
    data = None if body is None else (body if isinstance(body, bytes) else json.dumps(body).encode())
    req = urllib.request.Request(srv.url + path, data=data, headers=headers or {})
    if data is not None and ctype:
        req.add_header("Content-Type", ctype)
    try:
        with urllib.request.urlopen(req, timeout=3) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_index_and_config(web):
    with urllib.request.urlopen(web.url + "/", timeout=3) as r:
        assert b"Passer Control" in r.read()
    code, c = call(web, "/api/config")
    assert code == 200
    assert c["values"]["launcher"]["arm_length_m"] == 0.6
    assert "launcher.arm_length_m" in c["docs"]
    assert "app.dry_run" in c["read_only"] and "camera.fps" in c["restart_required"]


def test_live_update_changes_the_running_config(web):
    code, r = call(web, "/api/config", {"launcher": {"arm_length_m": 0.8,
                                                     "profiles": {"lob": {"launch_angle_deg": 60}}}})
    assert code == 200 and r["restart_needed"] == []
    assert web.cfg.launcher.arm_length_m == 0.8
    assert web.cfg.launcher.profiles["lob"].launch_angle_deg == 60.0
    code, r = call(web, "/api/config", {"camera": {"fps": 20}})
    assert r["restart_needed"] == ["camera.fps"]


def test_bad_updates_are_rejected_atomically(web):
    code, r = call(web, "/api/config", {"launcher": {"arm_length_m": 0.9, "gear_ratio": "ten"}})
    assert code == 400 and "gear_ratio" in r["error"]
    assert web.cfg.launcher.arm_length_m == 0.6          # nothing half-applied
    code, r = call(web, "/api/config", {"launcher": {"nope": 1}})
    assert code == 400 and "Unknown config key: launcher.nope" == r["error"]


@pytest.mark.parametrize("patch", [{"app": {"dry_run": False}}, {"web": {"allow_fire_live": True}},
                                   {"web": {"password": ""}}])
def test_safety_settings_are_read_only(web, patch):
    code, r = call(web, "/api/config", patch)
    assert code == 403
    assert web.cfg.app.dry_run is True and web.cfg.web.allow_fire_live is False


def test_post_requires_json_content_type(web):
    code, _ = call(web, "/api/action", b'{"action": "chest"}', ctype="text/plain")
    assert code == 415 and web.actions == []


def test_fire_actions_blocked_in_live_unless_allowed(web):
    assert call(web, "/api/action", {"action": "chest"})[0] == 200
    web.cfg.app.dry_run = False
    code, r = call(web, "/api/action", {"action": "lob"})
    assert code == 409 and "LIVE" in r["message"]
    assert call(web, "/api/action", {"action": "stop"})[0] == 200    # stop always works
    web.cfg.web.allow_fire_live = True
    assert call(web, "/api/action", {"action": "lob"})[0] == 200
    assert web.actions == ["chest", "stop", "lob"]
    assert call(web, "/api/action", {"action": "selfdestruct"})[0] == 409


def test_plan_preview_uses_live_settings(web):
    _, a = call(web, "/api/plan?distance=6&profile=chest")
    call(web, "/api/config", {"launcher": {"arm_length_m": 1.2}})
    _, b = call(web, "/api/plan?distance=6&profile=chest")
    assert b["motor_rpm"] < a["motor_rpm"]
    assert call(web, "/api/plan?distance=6&profile=nope")[0] == 400


def test_save_writes_only_changes_and_reloads(web):
    call(web, "/api/config", {"launcher": {"arm_length_m": 0.7}, "prediction": {"accel_noise_mps2": 4}})
    code, r = call(web, "/api/config/save", {})
    assert code == 200
    saved = yaml.safe_load(open(web.cfg.web.save_path))
    assert saved["launcher"] == {"arm_length_m": 0.7}
    assert saved["prediction"] == {"accel_noise_mps2": 4.0}
    assert load_config(web.cfg.web.save_path).launcher.arm_length_m == 0.7


def test_password_login(web):
    web.cfg.web.password = "s3cret"
    with urllib.request.urlopen(web.url + "/", timeout=3) as r:
        page = r.read()
    assert b"password" in page and b"Passer Control" in page and b"api/config" not in page
    assert call(web, "/api/status")[0] == 401
    assert call(web, "/api/config", {"launcher": {"arm_length_m": 1}})[0] == 401
    assert call(web, "/api/status", headers={"Cookie": "sid=guess"})[0] == 401
    assert call(web, "/api/login", {"password": "nope"})[0] == 401

    req = urllib.request.Request(web.url + "/api/login", json.dumps({"password": "s3cret"}).encode(),
                                 {"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=3) as r:
        cookie = r.headers["Set-Cookie"]
    assert "HttpOnly" in cookie
    auth = {"Cookie": cookie.split(";")[0]}
    code, st = call(web, "/api/status", headers=auth)
    assert code == 200 and st["login_required"] is True
    code, c = call(web, "/api/config", headers=auth)
    assert code == 200 and "password" not in c["values"]["web"]     # never sent to the browser
    assert call(web, "/api/logout", {}, headers=auth)[0] == 200
    assert call(web, "/api/status", headers=auth)[0] == 401


def test_settings_locked_during_a_session(web):
    web.session["on"] = True
    code, r = call(web, "/api/config", {"launcher": {"calibration": {"speed_scale": 1.1}}})
    assert code == 403 and "session" in r["error"]
    assert web.cfg.launcher.calibration.speed_scale == 1.0
    assert call(web, "/api/action", {"action": "session_end"})[0] == 200
    web.session["on"] = False
    assert call(web, "/api/config", {"launcher": {"calibration": {"speed_scale": 1.1}}})[0] == 200


def test_hard_limits_locked_in_live(web):
    web.cfg.app.dry_run = False
    code, r = call(web, "/api/config", {"launcher": {"max_motor_rpm": 9000}})
    assert code == 403 and web.cfg.launcher.max_motor_rpm == 3000
    assert call(web, "/api/config", {"launcher": {"calibration": {"speed_scale": 1.05}}})[0] == 200
    assert "launcher.max_motor_rpm" in call(web, "/api/config")[1]["read_only"]


def test_save_never_keeps_live_mode_or_cli_flags(web):
    web.cfg.app.dry_run = False                       # started with --live
    web.cfg.app.headless = True                       # and --headless
    web.save_exclude |= {"app.headless"}
    call(web, "/api/config", {"launcher": {"calibration": {"speed_scale": 1.05}}})
    assert call(web, "/api/config/save", {})[0] == 200
    saved = yaml.safe_load(open(web.cfg.web.save_path))
    assert "app" not in saved
    assert load_config(web.cfg.web.save_path).app.dry_run is True


def test_mjpeg_stream_serves_frames_and_counts_viewers(web):
    got = []

    def reader():
        with urllib.request.urlopen(web.url + "/stream.mjpg", timeout=3) as r:
            buf = b""
            while buf.count(b"\xff\xd8") < 2:
                buf += r.read(1024)
            got.append(buf)

    web.cfg.web.stream_fps = 30
    t = threading.Thread(target=reader, daemon=True)
    t.start()
    peak = 0
    for i in range(20):
        web.set_frame(np.full((90, 160, 3), i * 10, np.uint8), i + 1)
        for _ in range(10):
            peak = max(peak, web.viewers)
            time.sleep(0.005)
    t.join(3)
    assert got and got[0].count(b"Content-Type: image/jpeg") >= 2
    assert peak == 1
    time.sleep(0.2)
    assert web.viewers == 0                  # disconnect is noticed


def test_apply_overrides_is_all_or_nothing():
    cfg = Config()
    with pytest.raises(TypeError):
        apply_overrides(cfg, {"camera": {"hfov_deg": 70}, "app": {"dry_run": "no"}})
    assert cfg.camera.hfov_deg == 66.0
    apply_overrides(cfg, {"camera": {"hfov_deg": 70}})
    assert cfg.camera.hfov_deg == 70.0


def test_field_docs_cover_key_settings():
    d = field_docs()
    for k in ("launcher.arm_length_m", "launcher.calibration.speed_scale",
              "launcher_axis.fire_latency_s", "profile.launch_angle_deg"):
        assert d.get(k), k


def test_player_profiles_api(web):
    code, r = call(web, "/api/players")
    assert code == 200 and r["players"] == [] and r["selected"] is None
    assert r["base"]["passes"]["lob"]["launch_angle_deg"] == 55.0
    code, r = call(web, "/api/players", {"name": "Ana", "person_height_m": 1.6,
                                         "passes": {"chest": {"target_height_m": 1.2}}})
    assert code == 200 and r["player"]["id"] == "ana"
    assert call(web, "/api/players", {"name": "Bo", "person_height_m": 9})[0] == 400
    assert call(web, "/api/players", {"name": "Bo", "max_motor_rpm": 9000})[0] == 400
    assert call(web, "/api/players/select", {"id": "ana"})[0] == 200
    assert call(web, "/api/players")[1]["selected"] == "ana"
    assert call(web, "/api/players/select", {"id": "../etc"})[0] == 400
    assert web.cfg.distance.person_height_m == 1.75          # only applied when a session starts
    assert call(web, "/api/players/delete", {"id": "ana"})[0] == 200
    assert call(web, "/api/players")[1] == {**call(web, "/api/players")[1], "players": [], "selected": None}


def test_no_save_during_a_session(web):
    web.session["on"] = True
    code, r = call(web, "/api/config/save", {})
    assert code == 403 and "session" in r["error"]
