"""
config.py

Single source of truth for every tunable in the system. Defaults live in the
dataclasses below; a YAML file (see configs/) only needs to list the values
that differ for a particular machine, e.g. configs/rpi.yaml vs.
configs/macbook.yaml.

    cfg = load_config("configs/rpi.yaml")
    cfg.camera.backend      # "picamera2"
    cfg.launcher.gear_ratio # 10.0
"""

from __future__ import annotations

import copy
import dataclasses
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
@dataclass
class CameraConfig:
    # "auto" picks picamera2 if it is importable (i.e. on a Pi), else opencv.
    backend: str = "auto"            # auto | picamera2 | opencv | file
    index: int = 0                   # opencv device index
    file_path: Optional[str] = None  # for backend="file" (replay a video)

    # High-res stream used for gesture ROI crops. For the Pi Camera Module 3
    # 2304x1296 is a full-FOV binned mode that still runs at ~50 fps; for
    # Module 2 use 1640x1232 / 1920x1080.
    main_size: tuple = (1920, 1080)
    # Low-res stream for body detection. Must match main_size's aspect ratio
    # (picamera2 derives lores from the same sensor crop).
    lores_width: int = 640
    fps: int = 30
    # OpenCV only: request main_size from the device. Off by default because
    # some cameras (e.g. iPhone Continuity Camera) satisfy it by cropping
    # the sensor ("zoom in") instead of scaling. Turn on for USB webcams on
    # the Pi, which otherwise default to 640x480.
    opencv_set_size: bool = False

    # Only affects what is SHOWN on screen. Inference always runs on the
    # unmirrored image so MediaPipe Pose's LEFT/RIGHT refer to the player's
    # own left/right.
    mirror_display: bool = False
    rotate_180: bool = False         # camera mounted upside down

    # Horizontal field of view in degrees; used for pan-angle and distance
    # estimation. Pi Camera Module 3 = 66, Module 3 Wide = 102,
    # Module 2 = 62.2, MacBook FaceTime HD ~ 70 (approximate).
    hfov_deg: float = 66.0


# ---------------------------------------------------------------------------
# Vision
# ---------------------------------------------------------------------------
@dataclass
class DetectorConfig:
    backend: str = "yolo"            # yolo | mediapipe_pose
    # yolov8n.pt works everywhere. On a Pi, export once to NCNN for a
    # 3-4x speed-up:  yolo export model=yolov8n.pt format=ncnn imgsz=320
    # and point this at the resulting "yolov8n_ncnn_model" directory.
    yolo_model: str = "yolov8n.pt"
    yolo_imgsz: int = 320
    yolo_conf: float = 0.4
    device: Optional[str] = None     # None = auto (mps on Apple Silicon, else cpu)


@dataclass
class TargetConfig:
    # Multi-person handling: once a player is locked, keep following the box
    # that overlaps them most, rather than jumping to whoever is biggest.
    min_iou_to_keep: float = 0.2
    lost_after_sec: float = 1.0      # drop the lock if unseen for this long


@dataclass
class GestureConfig:
    # Run the (heavier) gesture models every N inference ticks, or sooner if
    # the bounding box has been stable (little motion) for a while.
    every_n_frames: int = 4
    stable_bbox_px: float = 12.0     # centre+size change (lores px) counted as "stable"
    # A gesture must be seen this many consecutive gesture ticks to count.
    confirm_count: int = 3
    # Seconds to ignore gestures after a command was issued.
    cooldown_sec: float = 2.0
    # ROI padding around the body box (fraction of box size) before cropping
    # the high-res frame, so raised hands are not clipped.
    roi_pad_x: float = 0.35
    roi_pad_top: float = 0.25
    roi_pad_bottom: float = 0.05
    # Max long side of the ROI fed to MediaPipe. Larger = more hand detail but
    # slower; the point of the hi-res crop is that the hands are now big
    # enough even after this resize.
    roi_max_side: int = 640
    # Also run the pose fallback on every gesture tick even if hands were
    # found (hands give CHEST, pose gives LEFT/RIGHT/LOB).
    always_run_pose: bool = True
    min_landmark_visibility: float = 0.5
    pose_model_complexity: int = 1   # 0 = lite (faster on the Pi), 1 = full, 2 = heavy
    # Arm-angle windows, measured between the torso (shoulder->hip) and the
    # arm (shoulder->wrist): 0 = arm hanging, 90 = horizontal, 180 = straight up.
    arm_side_min_deg: float = 65.0
    arm_side_max_deg: float = 125.0
    arm_down_max_deg: float = 45.0
    arm_overhead_min_deg: float = 140.0
    # Hand model
    hand_min_detection_conf: float = 0.6
    hand_min_tracking_conf: float = 0.5
    palm_angle_threshold_deg: float = 50.0
    invert_palm_orientation: bool = False


@dataclass
class DistanceConfig:
    # Pinhole estimate: distance = focal_px * person_height / bbox_height_px.
    person_height_m: float = 1.75
    smoothing: float = 0.3           # EMA alpha (1 = no smoothing)
    min_m: float = 1.0
    max_m: float = 12.0


# ---------------------------------------------------------------------------
# Two-axis turret
#
#   launcher turret (stepper, heavy)  : angle in the WORLD frame, 0 = straight
#                                       ahead of the machine, + = right
#     camera servo (hobby servo)      : angle RELATIVE to the launcher turret
#
#   camera heading (world) = launcher angle + camera angle
#   player bearing (world) = camera heading + pixel offset in the image
# ---------------------------------------------------------------------------
@dataclass
class CameraAxisConfig:
    """Hobby servo that pans the camera on top of the launcher turret."""
    # "none" = no servo yet: camera is treated as fixed at 0 relative to the
    # launcher; commands are still computed and shown. "gpio_servo" = gpiozero.
    backend: str = "none"
    gpio_pin: int = 18               # use a hardware-PWM pin: 12, 13, 18 or 19
    min_deg: float = -60.0           # relative to the launcher turret
    max_deg: float = 60.0
    trim_deg: float = 0.0            # servo angle that points the camera along the launcher
    invert: bool = False             # servo mounted so + turns left
    max_speed_dps: float = 120.0     # slew limit -> keeps motion blur down
    max_accel_dps2: float = 600.0
    deadband_deg: float = 1.0        # don't chase tiny errors (reduces jitter)


@dataclass
class LauncherAxisConfig:
    """Stepper-driven turret carrying the launcher (and the camera servo)."""
    backend: str = "none"            # add real drivers in passer/hardware/pan_axis.py
    min_deg: float = -90.0           # world frame
    max_deg: float = 90.0
    max_speed_dps: float = 45.0      # heavy: keep it gentle
    max_accel_dps2: float = 90.0
    on_target_deg: float = 2.0       # |aim error| below this counts as aimed
    # Keep leading the player between shots, so a fire request only needs a
    # small correction.
    track_between_shots: bool = True
    # Lateral lead for PASS_LEFT / PASS_RIGHT gestures, at the player.
    lead_m: float = 1.5
    # Seconds from "fire" to the ball leaving the arm (Modbus writes + arm
    # acceleration). Added to the flight time for motion prediction.
    fire_latency_s: float = 0.25
    aim_timeout_s: float = 2.0       # give up on a shot if not aimed by then


@dataclass
class PredictionConfig:
    """Constant-velocity Kalman filter on the player's floor position."""
    accel_noise_mps2: float = 3.0    # how hard players change direction
    range_noise_frac: float = 0.08   # distance measurement 1-sigma, fraction of range
    bearing_noise_deg: float = 0.7
    unreliable_range_factor: float = 3.0   # inflate range noise when distance is "held"
    max_speed_mps: float = 8.0       # clamp absurd velocity estimates
    min_updates: int = 5             # measurements before velocity is trusted
    reset_after_s: float = 1.0       # restart the filter after losing the player
    # Rough ball speed used to guess flight time while tracking (the exact
    # value comes from the launch plan when a shot is actually taken).
    nominal_ball_speed_mps: float = 9.0


# ---------------------------------------------------------------------------
# Launcher mechanics + PS100 drive
# ---------------------------------------------------------------------------
@dataclass
class PassProfile:
    launch_angle_deg: float          # ball elevation at release
    target_height_m: float           # height the ball should arrive at (catch height)
    # Per-profile sim-to-real trims, applied on top of CalibrationConfig.
    speed_scale: float = 1.0
    angle_offset_deg: float = 0.0


@dataclass
class BallConfig:
    """Size 7 basketball. Drag matters: ~10-20% extra speed needed at 6-10 m."""
    mass_kg: float = 0.62
    diameter_m: float = 0.24
    drag_coeff: float = 0.50
    air_density: float = 1.20
    drag: bool = True                # False = ideal vacuum parabola


@dataclass
class CalibrationConfig:
    """
    Sim-to-real corrections. The physics model gives the "ideal" command;
    these knobs bend it to match what the real machine does. Fit them with
    `python -m passer.tools.calibrate fit`, or tweak by hand:

      ball lands SHORT everywhere        -> raise speed_scale (e.g. 1.00 -> 1.08)
      short only at long range           -> add a speed_table point at that range
      ball flies STEEPER than planned    -> positive angle_offset_deg
      vision distance reads 10% too long -> distance_scale = 0.9
    """
    speed_scale: float = 1.0         # multiplies commanded ball speed
    speed_offset_mps: float = 0.0    # added to required ball speed before scaling
    angle_offset_deg: float = 0.0    # real launch angle minus planned
    distance_scale: float = 1.0      # applied to the vision distance
    distance_offset_m: float = 0.0
    # Per-profile distance-dependent speed multiplier, linearly interpolated
    # (clamped at the ends): {"chest": [[3.0, 1.00], [8.0, 1.06]], ...}
    speed_table: dict = field(default_factory=dict)


@dataclass
class LauncherConfig:
    # -- The main physical input ------------------------------------------
    arm_length_m: float = 0.60       # pivot to ball centre. Ball speed = omega * this.
    # -- Geometry -----------------------------------------------------------
    pivot_height_m: float = 0.90     # pivot height above floor (sets release height)
    # Arm angle convention (see kinematics.py): 0 deg = arm pointing straight
    # back (horizontal), 90 deg = straight up. Ball elevation at release is
    # 90 - release_angle. Starting below horizontal (negative) gives the arm
    # more travel to get up to speed before release.
    home_angle_deg: float = -45.0
    max_arm_angle_deg: float = 135.0 # mechanical limit / hard stop for the end of the sweep
    # -- Drive --------------------------------------------------------------
    gear_ratio: float = 10.0         # motor revs per arm rev
    pulses_per_rev: int = 10_000     # PS100 PA11
    max_motor_rpm: int = 3000
    min_motor_rpm: int = 50
    return_rpm: int = 100            # slow move back to home after a throw
    # PS100 FA40 / FA41: ms to go 0 <-> 1000 rpm (linear ramps).
    accel_ms_per_1000rpm: float = 50.0
    decel_ms_per_1000rpm: float = 50.0
    # The ball sits in an open cup, so it leaves as soon as the arm starts to
    # decelerate. True = extend the move so deceleration BEGINS at the release
    # angle (ball leaves at full speed). False = release at the end position.
    release_at_decel_start: bool = True
    ball: BallConfig = field(default_factory=BallConfig)
    calibration: CalibrationConfig = field(default_factory=CalibrationConfig)
    profiles: dict = field(default_factory=lambda: {
        "chest": PassProfile(launch_angle_deg=20.0, target_height_m=1.3),
        "lob": PassProfile(launch_angle_deg=55.0, target_height_m=2.0),
    })


@dataclass
class PS100Config:
    port: str = "/dev/ttyUSB0"
    slave_id: int = 1
    baudrate: int = 9600
    parity: str = "N"
    stopbits: int = 1
    timeout_s: float = 0.2
    # Some Modbus stacks/drives are 1-based; set to -1 if every register
    # read/write is off by one on the bench.
    address_offset: int = 0
    # "change": completion when status bit 0 toggles vs. its pre-move value.
    # "set":    completion when bit 0 reads 1.
    # "time":   don't poll, just wait the estimated move time.
    completion_mode: str = "change"
    completion_timeout_s: float = 3.0


# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------
@dataclass
class AppConfig:
    dry_run: bool = True             # never talk to the PS100 unless False
    headless: bool = False           # no preview window (Pi over SSH)
    require_on_target: bool = True   # only fire when the launcher turret is aimed
    reload_delay_s: float = 2.0      # time to reload a ball after the return move
    log_level: str = "INFO"


@dataclass
class WebConfig:
    """Browser dashboard: live camera view, status, and live config editing."""
    enabled: bool = True
    host: str = "0.0.0.0"            # reachable from other devices on the LAN
    port: int = 8080
    stream_fps: float = 15.0         # MJPEG preview rate (costs CPU on the Pi)
    jpeg_quality: int = 70
    # Fire buttons in the dashboard. Always available in dry run; in LIVE mode
    # only if this is true (anyone on the network could otherwise fire it).
    allow_fire_live: bool = False
    # Optional shared secret: open http://pi:8080/?token=... once; empty = no check.
    token: str = ""
    save_path: str = "configs/live.yaml"  # where "Save" writes the tuned settings


@dataclass
class Config:
    camera: CameraConfig = field(default_factory=CameraConfig)
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    target: TargetConfig = field(default_factory=TargetConfig)
    gesture: GestureConfig = field(default_factory=GestureConfig)
    distance: DistanceConfig = field(default_factory=DistanceConfig)
    camera_axis: CameraAxisConfig = field(default_factory=CameraAxisConfig)
    launcher_axis: LauncherAxisConfig = field(default_factory=LauncherAxisConfig)
    prediction: PredictionConfig = field(default_factory=PredictionConfig)
    launcher: LauncherConfig = field(default_factory=LauncherConfig)
    ps100: PS100Config = field(default_factory=PS100Config)
    app: AppConfig = field(default_factory=AppConfig)
    web: WebConfig = field(default_factory=WebConfig)


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------
def _coerce(current: Any, value: Any, where: str) -> Any:
    """
    Check/convert `value` to the type of the existing `current` value, so a
    typo in YAML or a bad value from the web UI fails loudly instead of
    silently breaking the running machine.
    """
    def bad(expected):
        raise TypeError(f"{where}: expected {expected}, got {value!r}")

    if isinstance(current, bool):
        if not isinstance(value, bool):
            bad("true/false")
        return value
    if isinstance(current, (int, float)):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            bad("a number")
        if not math.isfinite(value):
            bad("a finite number")
        if isinstance(current, int):
            if float(value) != int(value):
                bad("a whole number")
            return int(value)
        return float(value)
    if isinstance(current, str):
        if not isinstance(value, str):
            bad("text")
        return value
    if isinstance(current, tuple):
        if not isinstance(value, (list, tuple)) or len(value) != len(current):
            bad(f"a list of {len(current)} values")
        return tuple(_coerce(c, v, f"{where}[{i}]") for i, (c, v) in enumerate(zip(current, value)))
    if isinstance(current, dict) and not isinstance(value, dict):
        bad("a mapping")
    return value  # None-default (optional) fields accept anything


def _check_speed_table(value: Any, where: str) -> dict:
    if not isinstance(value, dict):
        raise TypeError(f"{where}: expected {{profile: [[distance, multiplier], ...]}}")
    out = {}
    for prof, pts in value.items():
        if not isinstance(pts, (list, tuple)):
            raise TypeError(f"{where}.{prof}: expected a list of [distance, multiplier]")
        out[prof] = [[_coerce(0.0, p[0], f"{where}.{prof}"), _coerce(0.0, p[1], f"{where}.{prof}")]
                     if isinstance(p, (list, tuple)) and len(p) == 2
                     else _coerce((0.0, 0.0), p, f"{where}.{prof}") for p in pts]
    return out


def _merge(obj: Any, data: dict, path: str = "") -> Any:
    """Recursively apply a dict of overrides onto a dataclass instance (type-checked)."""
    if not isinstance(data, dict):
        raise TypeError(f"{path or 'config'}: expected a mapping, got {data!r}")
    for key, value in data.items():
        where = f"{path}.{key}" if path else key
        if not hasattr(obj, key):
            raise KeyError(f"Unknown config key: {where}")
        current = getattr(obj, key)
        if dataclasses.is_dataclass(current):
            _merge(current, value, where)
        elif key == "profiles":
            if not isinstance(value, dict):
                raise TypeError(f"{where}: expected a mapping of profiles")
            profiles = dict(current)
            for name, prof in value.items():
                base = profiles.get(name) or PassProfile(launch_angle_deg=0.0, target_height_m=0.0)
                profiles[name] = _merge(dataclasses.replace(base), prof, f"{where}.{name}")
            setattr(obj, key, profiles)
        elif key == "speed_table":
            setattr(obj, key, _check_speed_table(value, where))
        else:
            setattr(obj, key, _coerce(current, value, where))
    return obj


def apply_overrides(cfg: "Config", data: dict) -> None:
    """
    Apply a partial update to a LIVE config: validated on a copy first, so
    either every value is applied or none is. Nested sections are mutated in
    place, so modules holding a reference to e.g. cfg.launcher see the change.
    """
    _merge(copy.deepcopy(cfg), data)
    _merge(cfg, data)


def to_dict(obj: Any) -> Any:
    """Config (or any part of it) as plain dicts/lists, e.g. for JSON/YAML."""
    if dataclasses.is_dataclass(obj):
        return {f.name: to_dict(getattr(obj, f.name)) for f in dataclasses.fields(obj)}
    if isinstance(obj, dict):
        return {k: to_dict(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [to_dict(v) for v in obj]
    return obj


def diff(current: dict, base: dict) -> dict:
    """Only the entries of `current` that differ from `base` (recursively)."""
    out = {}
    for k, v in current.items():
        b = base.get(k) if isinstance(base, dict) else None
        if isinstance(v, dict) and isinstance(b, dict) and k != "speed_table":
            d = diff(v, b)
            if d:
                out[k] = d
        elif v != b:
            out[k] = v
    return out


def save_config(cfg: "Config", path: str | Path) -> dict:
    """Write everything that differs from the defaults to a YAML file."""
    import yaml

    changed = diff(to_dict(cfg), to_dict(Config()))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write("# Saved from the web dashboard: only values that differ from the defaults.\n")
        yaml.safe_dump(changed, f, sort_keys=False)
    return changed


def field_docs() -> dict:
    """
    {"section.field": "help text"} taken from the comments in this file
    (trailing `# ...` on the field line, or the comment block right above
    it), so the web UI shows the same help text as the source.
    """
    import inspect
    import re

    field_pat = re.compile(r"^\s+(\w+)\s*:\s*[^#=]+(?:=[^#]*?)?(?:#\s*(.*))?$")
    comment_pat = re.compile(r"^\s+#\s?(.*)$")
    docs: dict = {}

    def walk(obj: Any, prefix: str) -> None:
        try:
            src = inspect.getsource(type(obj))
        except (OSError, TypeError):  # pragma: no cover
            return
        pending: list = []
        for line in src.splitlines():
            c = comment_pat.match(line)
            if c:
                pending.append(c.group(1).strip())
                continue
            m = field_pat.match(line)
            if m:
                text = m.group(2) or " ".join(pending)
                if text:
                    docs[f"{prefix}{m.group(1)}"] = text.strip()
            pending = []
        for f in dataclasses.fields(obj):
            v = getattr(obj, f.name)
            if dataclasses.is_dataclass(v):
                walk(v, f"{prefix}{f.name}.")

    root = Config()
    for f in dataclasses.fields(root):
        walk(getattr(root, f.name), f"{f.name}.")
    walk(PassProfile(launch_angle_deg=0.0, target_height_m=0.0), "profile.")
    return docs


def load_config(path: Optional[str | Path] = None, overrides: Optional[dict] = None) -> Config:
    cfg = Config()
    if path:
        import yaml  # only needed when a file is given
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        _merge(cfg, data)
    if overrides:
        _merge(cfg, overrides)
    return cfg
