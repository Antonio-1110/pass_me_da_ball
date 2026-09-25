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
    actions = []
    srv = WebServer(cfg, lambda: {"mode": "DRY RUN"},
                    lambda a: (actions.append(a) or True, f"{a} requested"))
    srv.actions = actions
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
                                   {"web": {"token": ""}}])
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


def test_token_required_when_set(web):
    web.cfg.web.token = "s3cret"
    assert call(web, "/api/status")[0] == 401
    assert call(web, "/api/status?token=s3cret")[0] == 200
    assert call(web, "/api/status", headers={"Cookie": "token=s3cret"})[0] == 200
    assert call(web, "/api/config", {"launcher": {"arm_length_m": 1}})[0] == 401


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
