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

import dataclasses
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
# Turret (camera / launcher pan axis)
# ---------------------------------------------------------------------------
@dataclass
class TurretConfig:
    # "none" = no pan motor yet; the controller still computes and logs the
    # target. Add new backends in passer/hardware/pan_axis.py.
    backend: str = "none"
    min_deg: float = -90.0
    max_deg: float = 90.0
    max_speed_dps: float = 60.0      # slew limit, keeps camera motion blur down
    kp: float = 2.0
    ki: float = 0.0
    kd: float = 0.15
    deadband_deg: float = 1.5        # don't chase tiny errors (reduces jitter)
    on_target_deg: float = 3.0       # |error| below this counts as "aimed"
    # Sideways lead for PASS_LEFT / PASS_RIGHT, as a lateral distance at the
    # player; converted to an angle using the measured distance.
    lead_m: float = 1.5
    # Launcher heading relative to camera optical axis (mounting offset).
    launcher_offset_deg: float = 0.0


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
    require_on_target: bool = True   # only fire when turret is aimed
    reload_delay_s: float = 2.0      # time to reload a ball after the return move
    log_level: str = "INFO"


@dataclass
class Config:
    camera: CameraConfig = field(default_factory=CameraConfig)
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    target: TargetConfig = field(default_factory=TargetConfig)
    gesture: GestureConfig = field(default_factory=GestureConfig)
    distance: DistanceConfig = field(default_factory=DistanceConfig)
    turret: TurretConfig = field(default_factory=TurretConfig)
    launcher: LauncherConfig = field(default_factory=LauncherConfig)
    ps100: PS100Config = field(default_factory=PS100Config)
    app: AppConfig = field(default_factory=AppConfig)


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------
def _merge(obj: Any, data: dict, path: str = "") -> Any:
    """Recursively apply a dict of overrides onto a dataclass instance."""
    for key, value in data.items():
        where = f"{path}.{key}" if path else key
        if not hasattr(obj, key):
            raise KeyError(f"Unknown config key: {where}")
        current = getattr(obj, key)
        if dataclasses.is_dataclass(current) and isinstance(value, dict):
            _merge(current, value, where)
        elif key == "profiles" and isinstance(value, dict):
            profiles = dict(current)
            for name, prof in value.items():
                base = profiles.get(name)
                if base is None:
                    profiles[name] = PassProfile(**prof)
                else:
                    profiles[name] = dataclasses.replace(base, **prof)
            setattr(obj, key, profiles)
        elif isinstance(current, tuple) and isinstance(value, (list, tuple)):
            setattr(obj, key, tuple(value))
        else:
            setattr(obj, key, value)
    return obj


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
