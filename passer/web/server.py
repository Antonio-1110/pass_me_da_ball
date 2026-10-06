"""
server.py -- the browser dashboard.

Runs inside the main app on a background thread (standard library only, so
nothing extra to install on the Pi) and serves:

    GET  /                    the dashboard (static/index.html), or the login page
    POST /api/login           {"password": "..."} -> login cookie
    POST /api/logout
    GET  /stream.mjpg         live camera view with the tracking overlay (MJPEG)
    GET  /snapshot.jpg        one frame
    GET  /api/status          machine state (polled by the page)
    GET  /api/config          all settings + defaults + help text + which need a restart
    POST /api/config          partial update, e.g. {"launcher": {"arm_length_m": 0.65}}
    POST /api/config/save     write everything that differs from defaults to web.save_path
    GET  /api/plan?distance=6&profile=chest   launch plan with the CURRENT settings
    GET  /api/players         player profiles, the editable fields, the selected player
    POST /api/players         create/update a profile {"id"?, "name", "person_height_m", ...}
    POST /api/players/delete  {"id": "..."}
    POST /api/players/select  {"id": "..." | null} -> used by the next session
    POST /api/action          {"action": "chest" | "lob" | "pass_left" | "pass_right" | "stop"
                               | "reset" | "session_start" | "session_end"}

Config edits are applied to the live Config object in place, so almost all
of them take effect on the next frame/tick. The few that are read only at
startup (camera resolution, model files, serial port, ...) are applied and
flagged "restart needed".

Set web.password to require a login: without it the server sends only the
login page and every API call is refused. With no password, anyone who can
reach the port has full control, so keep it on a private network.

Settings are locked while a session is running (the machine is in use) and,
in LIVE mode, the hard limits in LIVE_LOCKED are locked at all times.
"""

from __future__ import annotations

import dataclasses
import hmac
import json
import logging
import secrets
import threading
import time
from http import cookies
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, Optional, Tuple
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from ..config import Config, apply_overrides, field_docs, save_config, to_dict
from ..kinematics import plan_launch
from ..players import FIELDS, PASS_FIELDS, PlayerStore

log = logging.getLogger(__name__)

STATIC = Path(__file__).parent / "static"

# Settings only read at startup: applying them is allowed, but they only take
# effect after a restart. Entries are "section" or "section.field".
RESTART_REQUIRED = {
    "camera.backend", "camera.index", "camera.file_path", "camera.main_size",
    "camera.lores_width", "camera.fps", "camera.opencv_set_size", "camera.rotate_180",
    "detector.backend", "detector.yolo_model", "detector.device",
    "gesture.hand_min_detection_conf", "gesture.hand_min_tracking_conf",
    "gesture.pose_model_complexity",
    "ps100.port", "ps100.slave_id", "ps100.baudrate", "ps100.parity", "ps100.stopbits",
    "ps100.timeout_s", "ps100.address_offset",
    "camera_axis.backend", "camera_axis.gpio_pin", "camera_axis.min_deg", "camera_axis.max_deg",
    "launcher_axis.backend", "launcher_axis.min_deg", "launcher_axis.max_deg",
    "app.headless", "app.log_level",
}

# Never editable from the browser: switching to LIVE, or loosening the
# dashboard's own safety settings, must be done on the machine itself.
READ_ONLY = {
    "app.dry_run", "web.enabled", "web.host", "web.port", "web.password",
    "web.allow_fire_live", "web.save_path", "app.players_dir",
}

# The hard limits, geometry and trigger guards the launch-plan safety checks
# rest on. Tunable in dry run to set the machine up; locked while LIVE so a
# browser can't push a throw past them (calibration and profiles stay tunable,
# but every throw is still capped by these).
LIVE_LOCKED = {
    "launcher.arm_length_m", "launcher.pivot_height_m", "launcher.home_angle_deg",
    "launcher.max_arm_angle_deg", "launcher.gear_ratio", "launcher.pulses_per_rev",
    "launcher.max_motor_rpm", "launcher.min_motor_rpm", "launcher.return_rpm",
    "launcher.accel_ms_per_1000rpm", "launcher.decel_ms_per_1000rpm",
    "launcher.release_at_decel_start", "launcher_axis", "ps100",
    "app.require_on_target", "app.reload_delay_s",
    "gesture.confirm_count", "gesture.cooldown_sec", "distance.min_m", "distance.max_m",
}

FIRE_ACTIONS ={"chest", "lob", "pass_left", "pass_right"}
OTHER_ACTIONS = {"stop", "reset", "session_start", "session_end"}

# Never written by "Save": LIVE must always be asked for explicitly (--live).
NEVER_SAVED = {"app.dry_run"}


def _paths(d: dict, prefix: str = "") -> list:
    """Dotted paths of the leaves in a partial update (profiles/tables count as leaves)."""
    out = []
    for k, v in d.items():
        p = f"{prefix}{k}"
        if isinstance(v, dict) and k not in ("speed_table",) and not prefix.endswith("profiles."):
            out += _paths(v, p + ".")
        else:
            out.append(p)
    return out


def _matches(path: str, rules: set) -> bool:
    parts = path.split(".")
    return any(".".join(parts[:i]) in rules for i in range(1, len(parts) + 1))


class WebServer:
    def __init__(self, cfg: Config,
                 status_fn: Callable[[], Dict[str, Any]],
                 action_fn: Callable[[str], Tuple[bool, str]],
                 in_session: Callable[[], bool] = lambda: False,
                 save_exclude: Iterable[str] = (),
                 players: Optional[PlayerStore] = None):
        """
        in_session: True while the machine is in use (settings are locked).
        save_exclude: dotted config paths "Save" must not write, e.g. the ones
        set by command-line flags.
        """
        self.cfg = cfg
        self.status_fn = status_fn
        self.action_fn = action_fn
        self.in_session = in_session
        self.players = players
        self.save_exclude = NEVER_SAVED | set(save_exclude)
        self._logins: set = set()
        self._frame: Optional[np.ndarray] = None
        self._frame_id = 0
        self._jpeg: Optional[bytes] = None
        self._jpeg_id = -1
        self._lock = threading.Lock()
        self._clients = 0
        self._clients_lock = threading.Lock()
        self._httpd: Optional[ThreadingHTTPServer] = None
        self._docs = field_docs()
        self._defaults = to_dict(Config())

    # -- called by the app ------------------------------------------------
    @property
    def viewers(self) -> int:
        """Number of open video streams (the app only draws frames if > 0)."""
        return self._clients

    def set_frame(self, bgr: np.ndarray, frame_id: int) -> None:
        with self._lock:
            self._frame, self._frame_id = bgr, frame_id

    def start(self) -> "WebServer":
        server = self

        class Handler(_Handler):
            web = server

        w = self.cfg.web
        self._httpd = ThreadingHTTPServer((w.host, w.port), Handler)
        self._httpd.daemon_threads = True
        threading.Thread(target=self._httpd.serve_forever, kwargs={"poll_interval": 0.1},
                         name="web", daemon=True).start()
        shown = "localhost" if w.host in ("0.0.0.0", "") else w.host
        log.info("Dashboard at http://%s:%d/", shown, w.port)
        if not w.password:
            log.warning("Dashboard has no password (web.password): anyone who can reach "
                        "it can control the machine")
        elif w.password == "changeme":
            log.warning("Dashboard password is still the default 'changeme': set web.password")
        return self

    def stop(self) -> None:
        if self._httpd:
            self._httpd.shutdown()
            self._httpd.server_close()

    # -- helpers used by the handler ----------------------------------------
    def jpeg(self) -> Optional[bytes]:
        """Latest frame as JPEG, encoded once per frame however many viewers."""
        with self._lock:
            if self._frame is None:
                return None
            if self._jpeg_id != self._frame_id:
                q = int(self.cfg.web.jpeg_quality)
                ok, buf = cv2.imencode(".jpg", self._frame, [cv2.IMWRITE_JPEG_QUALITY, q])
                if ok:
                    self._jpeg, self._jpeg_id = buf.tobytes(), self._frame_id
            return self._jpeg

    # -- login ------------------------------------------------------------------
    def login(self, password: str) -> Optional[str]:
        """A new login cookie value if the password is right, else None."""
        if hmac.compare_digest(password.encode(), self.cfg.web.password.encode()):
            sid = secrets.token_urlsafe(32)
            self._logins.add(sid)
            log.info("Dashboard login")
            return sid
        log.warning("Dashboard login with a wrong password")
        time.sleep(1.0)          # slows down guessing
        return None

    def logged_in(self, sid: Optional[str]) -> bool:
        return not self.cfg.web.password or (sid is not None and sid in self._logins)

    def logout(self, sid: Optional[str]) -> None:
        self._logins.discard(sid)

    def config_payload(self) -> dict:
        values = to_dict(self.cfg)
        values["web"].pop("password", None)      # never sent to the browser
        defaults = to_dict(self._defaults)
        defaults["web"].pop("password", None)
        return {
            "values": values,
            "defaults": defaults,
            "docs": self._docs,
            "restart_required": sorted(RESTART_REQUIRED),
            "read_only": sorted(self.locked()),
            "save_path": self.cfg.web.save_path,
        }

    def update_config(self, patch: dict) -> dict:
        paths = _paths(patch)
        if self.in_session():
            raise PermissionError("settings are locked while a session is running -- end it first")
        locked = [p for p in paths if _matches(p, READ_ONLY)]
        if locked:
            raise PermissionError(f"read-only from the dashboard: {', '.join(locked)}")
        locked = [p for p in paths if _matches(p, self.locked())]
        if locked:
            raise PermissionError(f"locked while LIVE (change in dry run): {', '.join(locked)}")
        apply_overrides(self.cfg, patch)
        restart = [p for p in paths if _matches(p, RESTART_REQUIRED)]
        log.info("Dashboard changed %s%s", ", ".join(paths),
                 f" (restart needed: {', '.join(restart)})" if restart else "")
        return {"ok": True, "changed": paths, "restart_needed": restart,
                "values": to_dict(self.cfg)}

    def locked(self) -> set:
        """Settings the browser may not change right now."""
        return READ_ONLY if self.cfg.app.dry_run else READ_ONLY | LIVE_LOCKED

    def fire_allowed(self) -> bool:
        return self.cfg.app.dry_run or self.cfg.web.allow_fire_live

    def action(self, name: str) -> Tuple[bool, str]:
        if name in FIRE_ACTIONS:
            if not self.fire_allowed():
                return False, "firing from the dashboard is disabled in LIVE mode (web.allow_fire_live)"
        elif name not in OTHER_ACTIONS:
            return False, f"unknown action {name!r}"
        return self.action_fn(name)

    def save(self) -> dict:
        if self.in_session():   # would bake the session player's values into the file
            raise PermissionError("end the session before saving")
        return save_config(self.cfg, self.cfg.web.save_path, self.save_exclude)

    def players_payload(self) -> dict:
        if self.players is None:
            raise KeyError("player profiles are not enabled")
        return {
            "players": self.players.list(),
            "selected": self.players.selected,
            "fields": {k: dict(zip(("min", "max", "unit", "label"), v)) for k, v in FIELDS.items()},
            "pass_fields": {k: dict(zip(("min", "max", "unit", "label"), v))
                            for k, v in PASS_FIELDS.items()},
            # what the machine uses when a profile leaves a field out
            "base": {"person_height_m": self.cfg.distance.person_height_m,
                     "lead_m": self.cfg.launcher_axis.lead_m,
                     "passes": {n: {k: getattr(p, k) for k in PASS_FIELDS}
                                for n, p in self.cfg.launcher.profiles.items()}},
        }

    def players_post(self, path: str, body: dict) -> dict:
        if self.players is None:
            raise KeyError("player profiles are not enabled")
        if path == "/api/players":
            return {"ok": True, "player": self.players.save(body, self.cfg)}
        if path == "/api/players/delete":
            self.players.delete(str(body.get("id", "")))
        elif path == "/api/players/select":
            self.players.select(body.get("id"))
        else:
            raise KeyError("not found")
        return {"ok": True}

    def plan(self, distance: float, profile: str) -> dict:
        if profile not in self.cfg.launcher.profiles:
            raise KeyError(f"unknown profile {profile!r}")
        p = plan_launch(distance, profile, self.cfg.launcher)
        return dataclasses.asdict(p)


class _Handler(BaseHTTPRequestHandler):
    web: WebServer
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # keep the console quiet
        log.debug("web: " + fmt, *args)

    # -- plumbing -------------------------------------------------------------
    def _sid(self) -> Optional[str]:
        c = cookies.SimpleCookie(self.headers.get("Cookie", ""))
        return c["sid"].value if "sid" in c else None

    def _authorized(self) -> bool:
        return self.web.logged_in(self._sid())

    def _send(self, code: int, body: bytes, ctype: str, extra: Optional[dict] = None) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj: Any, code: int = 200) -> None:
        self._send(code, json.dumps(obj).encode(), "application/json")

    def _error(self, code: int, msg: str) -> None:
        self._json({"ok": False, "error": msg}, code)

    def _body(self) -> Any:
        n = int(self.headers.get("Content-Length") or 0)
        if n > 1_000_000:
            raise ValueError("request too large")
        return json.loads(self.rfile.read(n) or b"{}")

    # -- routes ---------------------------------------------------------------
    def do_GET(self):
        url = urlparse(self.path)
        q = parse_qs(url.query)
        if not self._authorized():
            if url.path in ("/", "/index.html"):
                return self._send(200, (STATIC / "login.html").read_bytes(),
                                  "text/html; charset=utf-8")
            return self._error(401, "log in first")
        try:
            if url.path in ("/", "/index.html"):
                return self._send(200, (STATIC / "index.html").read_bytes(),
                                  "text/html; charset=utf-8")
            if url.path == "/stream.mjpg":
                return self._stream()
            if url.path == "/snapshot.jpg":
                jpg = self.web.jpeg()
                return self._send(200, jpg, "image/jpeg") if jpg else self._error(503, "no frame yet")
            if url.path == "/api/status":
                st = dict(self.web.status_fn())
                st["fire_allowed"] = self.web.fire_allowed()
                st["login_required"] = bool(self.web.cfg.web.password)
                return self._json(st)
            if url.path == "/api/config":
                return self._json(self.web.config_payload())
            if url.path == "/api/players":
                return self._json(self.web.players_payload())
            if url.path == "/api/plan":
                d = float(q.get("distance", ["6"])[0])
                prof = q.get("profile", ["chest"])[0]
                return self._json(self.web.plan(d, prof))
            return self._error(404, "not found")
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as e:  # keep the server alive whatever happens
            log.exception("web GET %s failed", url.path)
            self._error(400, str(e))

    def do_POST(self):
        url = urlparse(self.path)
        # Browsers only send JSON cross-site after a CORS preflight, which this
        # server never approves -- so requiring it blocks other web pages from
        # silently POSTing (e.g. firing) to the machine.
        if not self.headers.get("Content-Type", "").startswith("application/json"):
            return self._error(415, "Content-Type must be application/json")
        if url.path == "/api/login":
            try:
                sid = self.web.login(str(self._body().get("password", "")))
            except ValueError as e:
                return self._error(400, str(e))
            if sid is None:
                return self._error(401, "wrong password")
            return self._send(200, b'{"ok": true}', "application/json",
                              {"Set-Cookie": f"sid={sid}; Path=/; HttpOnly; SameSite=Strict"})
        if not self._authorized():
            return self._error(401, "log in first")
        try:
            if url.path == "/api/logout":
                self.web.logout(self._sid())
                return self._send(200, b'{"ok": true}', "application/json",
                                  {"Set-Cookie": "sid=; Path=/; Max-Age=0; SameSite=Strict"})
            if url.path == "/api/config":
                return self._json(self.web.update_config(self._body()))
            if url.path == "/api/config/save":
                path = self.web.cfg.web.save_path
                changed = self.web.save()
                log.info("Dashboard saved settings to %s", path)
                return self._json({"ok": True, "path": path, "saved": changed})
            if url.path.startswith("/api/players"):
                return self._json(self.web.players_post(url.path, self._body()))
            if url.path == "/api/action":
                ok, msg = self.web.action(str(self._body().get("action", "")))
                return self._json({"ok": ok, "message": msg}, 200 if ok else 409)
            return self._error(404, "not found")
        except PermissionError as e:
            self._error(403, str(e))
        except (KeyError, TypeError, ValueError) as e:
            self._error(400, str(e.args[0]) if isinstance(e, KeyError) and e.args else str(e))
        except Exception as e:
            log.exception("web POST %s failed", url.path)
            self._error(500, str(e))

    def _stream(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        web = self.web
        with web._clients_lock:
            web._clients += 1
        last = -1
        try:
            while True:
                t0 = time.monotonic()
                jpg = web.jpeg()
                if jpg is not None and web._jpeg_id != last:
                    last = web._jpeg_id
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     + f"Content-Length: {len(jpg)}\r\n\r\n".encode()
                                     + jpg + b"\r\n")
                    self.wfile.flush()
                period = 1.0 / max(1.0, float(web.cfg.web.stream_fps))
                time.sleep(max(0.005, period - (time.monotonic() - t0)))
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            with web._clients_lock:
                web._clients -= 1
