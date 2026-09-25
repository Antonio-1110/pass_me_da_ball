"""
server.py -- the browser dashboard.

Runs inside the main app on a background thread (standard library only, so
nothing extra to install on the Pi) and serves:

    GET  /                    the dashboard page (static/index.html)
    GET  /stream.mjpg         live camera view with the tracking overlay (MJPEG)
    GET  /snapshot.jpg        one frame
    GET  /api/status          machine state (polled by the page)
    GET  /api/config          all settings + defaults + help text + which need a restart
    POST /api/config          partial update, e.g. {"launcher": {"arm_length_m": 0.65}}
    POST /api/config/save     write everything that differs from defaults to web.save_path
    GET  /api/plan?distance=6&profile=chest   launch plan with the CURRENT settings
    POST /api/action          {"action": "chest" | "lob" | "pass_left" | "pass_right" | "stop" | "reset"}

Config edits are applied to the live Config object in place, so almost all
of them take effect on the next frame/tick. The few that are read only at
startup (camera resolution, model files, serial port, ...) are applied and
flagged "restart needed".

There is no login. Anyone who can reach the port can change settings, so
keep it on a private network, or set web.token.
"""

from __future__ import annotations

import dataclasses
import json
import logging
import threading
import time
from http import cookies
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Dict, Optional, Tuple
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from ..config import Config, apply_overrides, field_docs, save_config, to_dict
from ..kinematics import plan_launch

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
    "app.dry_run", "web.enabled", "web.host", "web.port", "web.token",
    "web.allow_fire_live", "web.save_path",
}

FIRE_ACTIONS = {"chest", "lob", "pass_left", "pass_right"}
OTHER_ACTIONS = {"stop", "reset"}


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
                 action_fn: Callable[[str], Tuple[bool, str]]):
        self.cfg = cfg
        self.status_fn = status_fn
        self.action_fn = action_fn
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
        log.info("Dashboard at http://%s:%d/%s", shown, w.port,
                 f"?token={w.token}" if w.token else "")
        if w.host in ("0.0.0.0", "") and not w.token:
            log.warning("Dashboard is open to the whole network with no token (web.token)")
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

    def config_payload(self) -> dict:
        return {
            "values": to_dict(self.cfg),
            "defaults": self._defaults,
            "docs": self._docs,
            "restart_required": sorted(RESTART_REQUIRED),
            "read_only": sorted(READ_ONLY),
            "save_path": self.cfg.web.save_path,
        }

    def update_config(self, patch: dict) -> dict:
        paths = _paths(patch)
        locked = [p for p in paths if _matches(p, READ_ONLY)]
        if locked:
            raise PermissionError(f"read-only from the dashboard: {', '.join(locked)}")
        apply_overrides(self.cfg, patch)
        restart = [p for p in paths if _matches(p, RESTART_REQUIRED)]
        log.info("Dashboard changed %s%s", ", ".join(paths),
                 f" (restart needed: {', '.join(restart)})" if restart else "")
        return {"ok": True, "changed": paths, "restart_needed": restart,
                "values": to_dict(self.cfg)}

    def fire_allowed(self) -> bool:
        return self.cfg.app.dry_run or self.cfg.web.allow_fire_live

    def action(self, name: str) -> Tuple[bool, str]:
        if name in FIRE_ACTIONS:
            if not self.fire_allowed():
                return False, "firing from the dashboard is disabled in LIVE mode (web.allow_fire_live)"
        elif name not in OTHER_ACTIONS:
            return False, f"unknown action {name!r}"
        return self.action_fn(name)

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
    def _authorized(self, query: dict) -> bool:
        token = self.web.cfg.web.token
        if not token:
            return True
        if query.get("token", [None])[0] == token or self.headers.get("X-Token") == token:
            return True
        c = cookies.SimpleCookie(self.headers.get("Cookie", ""))
        return "token" in c and c["token"].value == token

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
        if not self._authorized(q):
            return self._error(401, "missing or wrong token (open /?token=...)")
        try:
            if url.path in ("/", "/index.html"):
                extra = {}
                if "token" in q:
                    extra["Set-Cookie"] = f"token={q['token'][0]}; Path=/; SameSite=Strict"
                return self._send(200, (STATIC / "index.html").read_bytes(),
                                  "text/html; charset=utf-8", extra)
            if url.path == "/stream.mjpg":
                return self._stream()
            if url.path == "/snapshot.jpg":
                jpg = self.web.jpeg()
                return self._send(200, jpg, "image/jpeg") if jpg else self._error(503, "no frame yet")
            if url.path == "/api/status":
                st = dict(self.web.status_fn())
                st["fire_allowed"] = self.web.fire_allowed()
                return self._json(st)
            if url.path == "/api/config":
                return self._json(self.web.config_payload())
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
        if not self._authorized(parse_qs(url.query)):
            return self._error(401, "missing or wrong token")
        # Browsers only send JSON cross-site after a CORS preflight, which this
        # server never approves -- so requiring it blocks other web pages from
        # silently POSTing (e.g. firing) to the machine.
        if not self.headers.get("Content-Type", "").startswith("application/json"):
            return self._error(415, "Content-Type must be application/json")
        try:
            if url.path == "/api/config":
                return self._json(self.web.update_config(self._body()))
            if url.path == "/api/config/save":
                path = self.web.cfg.web.save_path
                changed = save_config(self.web.cfg, path)
                log.info("Dashboard saved settings to %s", path)
                return self._json({"ok": True, "path": path, "saved": changed})
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
