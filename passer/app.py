"""
app.py -- main entry point.

    python -m passer.app --config configs/macbook.yaml            # laptop, dry run
    python -m passer.app --config configs/rpi.yaml --headless     # Pi over SSH, dry run
    python -m passer.app --config configs/rpi.yaml --live         # really fires!

A browser dashboard (live camera view, status, live settings) is served at
http://<pi-address>:8080/ unless --no-web; see passer/web/server.py.

Control loop (~30 Hz, main thread), see controller.py:
  1. read the newest TrackingState from the vision pipeline
  2. camera servo keeps the player centred; launcher turret leads the
     player's predicted position
  3. when a confirmed gesture arrives: aim at the predicted catch point,
     fire once the launcher is on target
  4. draw the debug view (unless headless)

Keys in the preview window:
  q quit | c manual chest pass | b manual lob | r reset launcher fault | s stop drive
"""

from __future__ import annotations

import argparse
import logging
import queue
import time

from .config import Config, load_config
from .controller import MachineController
from .vision.gestures import Gesture

log = logging.getLogger("passer")


class PassingMachine:
    def __init__(self, cfg: Config):
        from .vision.camera import CameraThread, make_camera_source
        from .vision.detection import make_person_detector
        from .vision.pipeline import GestureModels, VisionPipeline

        self.cfg = cfg
        self.camera = CameraThread(make_camera_source(cfg.camera))
        self.pipeline = VisionPipeline(cfg, self.camera,
                                       make_person_detector(cfg.detector),
                                       GestureModels(cfg))
        self.ctl = MachineController.from_config(cfg)
        self.actions: "queue.Queue[str]" = queue.Queue()
        self._status: dict = {}
        self.web = None
        if cfg.web.enabled:
            from .web.server import WebServer
            self.web = WebServer(cfg, lambda: self._status, self._web_action)

    # -- dashboard ---------------------------------------------------------
    def _web_action(self, name: str):
        """Called from the web thread. Stop is immediate; the rest run on the main loop."""
        if name == "stop":
            self.ctl.launcher.stop()
            return True, "stop sent to the drive"
        self.actions.put(name)
        return True, f"{name.replace('_', ' ')} requested"

    def _run_actions(self, now: float) -> None:
        shots = {"chest": Gesture.CHEST, "lob": Gesture.LOB,
                 "pass_left": Gesture.PASS_LEFT, "pass_right": Gesture.PASS_RIGHT}
        while not self.actions.empty():
            name = self.actions.get_nowait()
            if name in shots:
                self.ctl.request_shot(shots[name], now)
            elif name == "reset":
                self.ctl.launcher.reset()

    def _status_dict(self, st, now: float) -> dict:
        t, g = self.ctl.turret, self.cfg.gesture
        est = t.estimator.state_at(now)
        return {
            "mode": "DRY RUN" if self.cfg.app.dry_run else "LIVE",
            "dry_run": self.cfg.app.dry_run,
            "camera_fps": self.camera.fps,
            "infer_fps": st.infer_fps, "infer_ms": st.infer_ms, "gesture_ms": st.gesture_ms,
            "player": {
                "detected": st.box is not None,
                "distance_m": st.distance_m,
                "distance_reliable": st.distance_reliable,
                "pan_error_deg": st.pan_error_deg,
                "bearing_deg": t.last_bearing,
                "velocity": [est[2], est[3]] if est and t.estimator.ready else None,
            },
            "gesture": {
                "raw": st.gesture.value, "hand": st.hand_gesture.value, "pose": st.pose_gesture.value,
                "progress": st.gesture_progress[0].value, "progress_count": st.gesture_progress[1]
                if st.gesture_progress[0] is not Gesture.NONE else 0,
                "confirm_count": g.confirm_count,
            },
            "turret": {
                "launcher_deg": t.launcher.angle(), "launcher_target_deg": t.launcher_target,
                "camera_deg": t.camera.angle(), "camera_heading_deg": t.camera_heading(),
                "launcher_motor": t.launcher.physical, "camera_motor": t.camera.physical,
            },
            "launcher": {"state": self.ctl.launcher.state.value},
            "pending": self.ctl.pending.gesture.value if self.ctl.pending else None,
            "message": self.ctl.status,
        }

    def _status_lines(self):
        t = self.ctl.turret
        est = t.estimator.state_at(time.monotonic())
        vel = f"v=({est[2]:+.1f},{est[3]:+.1f}) m/s" if est and t.estimator.ready else "v=--"
        lt = t.launcher_target
        return [
            f"launcher {t.launcher.angle():+5.1f} -> {lt:+5.1f} deg" if lt is not None
            else f"launcher {t.launcher.angle():+5.1f} deg",
            f"camera {t.camera.angle():+5.1f} deg (rel)  bearing "
            + (f"{t.last_bearing:+5.1f}" if t.last_bearing is not None else "--") + f"  {vel}",
            f"arm {self.ctl.launcher.state.value}  {'DRY' if self.cfg.app.dry_run else 'LIVE'}",
            self.ctl.status,
        ]

    # ------------------------------------------------------------------
    def run(self) -> None:
        import cv2

        self.camera.start()
        self.pipeline.start()
        if self.web:
            self.web.start()
        headless = self.cfg.app.headless
        last_status = 0.0
        drawn_id = -1
        period = 1.0 / 30.0
        t_prev = time.monotonic()
        last_log = 0.0
        log.info("Running (%s). Ctrl+C to quit.",
                 "DRY RUN" if self.cfg.app.dry_run else "LIVE -- DRIVE WILL MOVE")
        try:
            while self.camera.running:
                now = time.monotonic()
                dt, t_prev = now - t_prev, now
                st = self.pipeline.latest()

                fr = st.frame
                self.ctl.tick(now, dt,
                              frame_id=fr.frame_id if fr else None,
                              t_frame=fr.timestamp if fr else None,
                              pan_error_deg=st.pan_error_deg,
                              distance_m=st.distance_m if st.box is not None else None,
                              reliable=st.distance_reliable)

                cmd = self.pipeline.get_command()
                if cmd is not None:
                    self.ctl.request_shot(cmd[0], now)
                self._run_actions(now)

                img = None
                if fr is not None and fr.frame_id != drawn_id and \
                        (not headless or (self.web and self.web.viewers > 0)):
                    from .vision.overlay import add_text, draw
                    drawn_id = fr.frame_id
                    scene = draw(st, mirror=self.cfg.camera.mirror_display, text=False)
                    if self.web:
                        self.web.set_frame(scene, fr.frame_id)
                    if not headless:
                        img = add_text(scene.copy(), st, self._status_lines())
                if self.web and now - last_status > 0.1:
                    last_status = now
                    self._status = self._status_dict(st, now)

                if headless:
                    if now - last_log > 1.0 and st.frame is not None:
                        last_log = now
                        log.info("fps=%.1f dist=%s err=%s gesture=%s | %s",
                                 st.infer_fps,
                                 f"{st.distance_m:.2f}" if st.distance_m else "-",
                                 f"{st.pan_error_deg:+.1f}" if st.pan_error_deg is not None else "-",
                                 st.gesture.value, " | ".join(self._status_lines()[:3]))
                    time.sleep(max(0.0, period - (time.monotonic() - now)))
                    continue

                if img is not None:
                    cv2.imshow("pass_me_da_ball (q to quit)", img)
                key = cv2.waitKey(max(1, int((period - (time.monotonic() - now)) * 1000))) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("c"):
                    self.ctl.request_shot(Gesture.CHEST, time.monotonic())
                elif key == ord("b"):
                    self.ctl.request_shot(Gesture.LOB, time.monotonic())
                elif key == ord("r"):
                    self.ctl.launcher.reset()
                elif key == ord("s"):
                    self.ctl.launcher.stop()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        if self.web:
            self.web.stop()
        self.pipeline.stop()
        self.camera.stop()
        self.ctl.turret.close()
        if not self.cfg.app.headless:
            import cv2
            cv2.destroyAllWindows()


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="Vision-guided passing machine")
    ap.add_argument("--config", help="YAML file with overrides (see configs/)")
    ap.add_argument("--live", action="store_true",
                    help="talk to the real PS100 (default is dry run)")
    ap.add_argument("--headless", action="store_true", help="no preview window")
    ap.add_argument("--video", help="replay a video file instead of a camera")
    ap.add_argument("--no-web", action="store_true", help="don't start the browser dashboard")
    ap.add_argument("--port", type=int, help="dashboard port (default 8080)")
    return ap.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    overrides: dict = {"app": {}}
    if args.live:
        overrides["app"]["dry_run"] = False
    if args.headless:
        overrides["app"]["headless"] = True
    if args.video:
        overrides["camera"] = {"backend": "file", "file_path": args.video}
    overrides["web"] = {}
    if args.no_web:
        overrides["web"]["enabled"] = False
    if args.port:
        overrides["web"]["port"] = args.port
    cfg = load_config(args.config, overrides)
    logging.basicConfig(level=cfg.app.log_level,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    PassingMachine(cfg).run()


if __name__ == "__main__":
    main()
