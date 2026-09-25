"""
app.py -- main entry point.

    python -m passer.app --config configs/macbook.yaml            # laptop, dry run
    python -m passer.app --config configs/rpi.yaml --headless     # Pi over SSH, dry run
    python -m passer.app --config configs/rpi.yaml --live         # really fires!

Control loop (~30 Hz, main thread):
  1. read the newest TrackingState from the vision pipeline
  2. drive the pan turret towards the player (+ lead offset if requested)
  3. when a confirmed gesture arrives: freeze distance, compute the launch
     plan, wait until the turret is on target, fire
  4. draw the debug view (unless headless)

Keys in the preview window:
  q quit | c manual chest pass | b manual lob | r reset launcher fault | s stop drive
"""

from __future__ import annotations

import argparse
import logging
import time
from dataclasses import dataclass
from typing import Optional

from .config import Config, load_config
from .hardware.pan_axis import make_pan_axis
from .hardware.ps100 import PS100
from .kinematics import plan_launch
from .launcher import Launcher
from .turret import TurretController
from .vision.gestures import Gesture

log = logging.getLogger("passer")

AIM_TIMEOUT_S = 2.0


@dataclass
class PendingShot:
    gesture: Gesture
    distance_m: float
    created: float


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
        self.axis = make_pan_axis(cfg.turret)
        self.turret = TurretController(self.axis, cfg.turret)
        self.launcher = Launcher(PS100.from_config(cfg.ps100, cfg.app.dry_run),
                                 cfg.launcher, cfg.app.reload_delay_s)
        self.pending: Optional[PendingShot] = None
        self.status = ""
        self._has_pan_motor = cfg.turret.backend not in ("none", "virtual")

    # ------------------------------------------------------------------
    def request_shot(self, gesture: Gesture, distance_m: Optional[float]) -> None:
        if distance_m is None:
            self.status = f"{gesture.value}: no distance estimate, ignored"
            log.warning(self.status)
            return
        if not self.launcher.ready:
            self.status = f"{gesture.value}: launcher {self.launcher.state.value}, ignored"
            log.warning(self.status)
            return
        self.pending = PendingShot(gesture, distance_m, time.monotonic())
        self.turret.set_lead(gesture.lead_sign * self.cfg.turret.lead_m, distance_m)
        self.status = f"aiming for {gesture.value} @ {distance_m:.1f} m"
        log.info(self.status)

    def _try_fire(self) -> None:
        p = self.pending
        if p is None:
            return
        aimed = self.turret.on_target() or not self._has_pan_motor \
            or not self.cfg.app.require_on_target
        if not aimed:
            if time.monotonic() - p.created > AIM_TIMEOUT_S:
                self.status = f"{p.gesture.value}: could not aim in time, cancelled"
                log.warning(self.status)
                self.pending = None
                self.turret.set_lead(0.0, None)
            return
        plan = plan_launch(p.distance_m, p.gesture.profile, self.cfg.launcher)
        self.launcher.fire(plan)
        self.status = plan.summary().splitlines()[0]
        self.pending = None
        self.turret.set_lead(0.0, None)

    # ------------------------------------------------------------------
    def run(self) -> None:
        import cv2

        self.camera.start()
        self.pipeline.start()
        headless = self.cfg.app.headless
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

                self.turret.update(st.pan_error_deg, dt)

                cmd = self.pipeline.get_command()
                if cmd is not None and self.pending is None:
                    self.request_shot(cmd[0], cmd[1].distance_m)
                self._try_fire()

                if headless:
                    if now - last_log > 1.0 and st.frame is not None:
                        last_log = now
                        log.info("fps=%.1f dist=%s err=%s pan=%.1f gesture=%s launcher=%s",
                                 st.infer_fps,
                                 f"{st.distance_m:.2f}" if st.distance_m else "-",
                                 f"{st.pan_error_deg:+.1f}" if st.pan_error_deg is not None else "-",
                                 self.turret.setpoint, st.gesture.value,
                                 self.launcher.state.value)
                    time.sleep(max(0.0, period - (time.monotonic() - now)))
                    continue

                if st.frame is not None:
                    from .vision.overlay import draw
                    img = draw(st, [
                        f"pan {self.turret.setpoint:+5.1f} deg  launcher {self.launcher.state.value}"
                        f"  {'DRY' if self.cfg.app.dry_run else 'LIVE'}",
                        self.status,
                    ], mirror=self.cfg.camera.mirror_display)
                    cv2.imshow("pass_me_da_ball (q to quit)", img)
                key = cv2.waitKey(max(1, int((period - (time.monotonic() - now)) * 1000))) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("c"):
                    self.request_shot(Gesture.CHEST, st.distance_m)
                elif key == ord("b"):
                    self.request_shot(Gesture.LOB, st.distance_m)
                elif key == ord("r"):
                    self.launcher.reset()
                elif key == ord("s"):
                    self.launcher.stop()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        self.pipeline.stop()
        self.camera.stop()
        self.axis.close()
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
    cfg = load_config(args.config, overrides)
    logging.basicConfig(level=cfg.app.log_level,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    PassingMachine(cfg).run()


if __name__ == "__main__":
    main()
