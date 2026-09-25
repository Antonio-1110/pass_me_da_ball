# pass_me_da_ball

A vision-guided basketball passing machine. A camera finds the player, a pan
turret keeps them centred, the player asks for a pass with a gesture, and a
PS100 servo drive swings a geared launch arm with the speed and sweep worked
out from the player's distance.

```text
 camera thread            inference thread                        main loop (30 Hz)
 ─────────────            ────────────────                        ─────────────────
 Pi Camera / webcam  ──►  YOLO on lores (640 px) ─► pick player ─► turret PID ─► PanAxis (TBD motor)
 main + lores frames      distance + pan error                    confirmed gesture
                          every N frames:                            │
                           crop hi-res ROI ─► MediaPipe Hands        ▼
                                          └► MediaPipe Pose     kinematics ─► PS100 over RS-485
                           debounce ─► command queue            (fire, return, reload)
```

## Layout

| Path | What it does |
|---|---|
| `passer/app.py` | Main entry point and state machine: track, aim, fire |
| `passer/config.py` | Every tunable, with defaults. `configs/*.yaml` override them per machine |
| `passer/vision/camera.py` | `picamera2` (Pi CSI camera), `opencv` (Mac/USB webcam) and `file` (replay) sources, all giving a hi-res + lores frame pair. Acquisition runs on its own thread |
| `passer/vision/detection.py` | Person detector (YOLOv8 or MediaPipe Pose) and `TargetTracker`, which stays locked on one player when several people are in view |
| `passer/vision/pipeline.py` | Inference thread, crop-and-infer, gesture scheduling |
| `passer/vision/gestures.py` | Hand and pose gesture classifiers, plus the debouncer |
| `passer/vision/distance.py` | Distance from the height of the person's box (pinhole model) |
| `passer/turret.py` | Pan error from pixels, PID with slew limit, lead offset |
| `passer/hardware/pan_axis.py` | **Plug-in point for the camera/turret motor** (not chosen yet) |
| `passer/kinematics.py` | Distance and pass type to exit velocity, then arm rpm, then PS100 registers |
| `passer/hardware/ps100.py` | Modbus RTU driver, plus a simulated drive for dry runs |
| `passer/launcher.py` | Fire, return and reload cycle, on its own thread |
| `passer/tools/` | `plan_pass`, `ps100_cli`, `list_cameras` |
| `webcam tracker test/` | Original MacBook prototype (kept for reference) |
| `PS100_RS485_CONFIGURATION.md` | Drive parameters, wiring and register notes |

## Quick start

**MacBook (development):**
```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements-mac.txt
python -m passer.app --config configs/macbook.yaml     # dry run: PS100 is simulated
```

**Raspberry Pi 5 + Camera Module 3:**
```bash
sudo apt install -y python3-picamera2 python3-opencv
python3 -m venv --system-site-packages .venv && . .venv/bin/activate
pip install -r requirements-pi.txt
yolo export model=yolov8n.pt format=ncnn imgsz=320     # optional, 3-4x faster; then set detector.yolo_model
python -m passer.app --config configs/rpi.yaml --headless          # dry run
python -m passer.app --config configs/rpi.yaml --headless --live   # REALLY moves the arm
```

Other flags: `--video clip.mp4` replays a recording through the whole stack.
Preview keys: `q` quit, `c` / `b` manual chest / lob, `r` clear a launcher fault, `s` stop the drive.

**Tests** (no camera or models needed): `pip install -r requirements-dev.txt && pytest`

## Gestures

| Gesture | Range | Command |
|---|---|---|
| Both palms open, fingers up, facing camera | close (hands readable) | **Chest pass** at the player |
| Left arm out sideways (~90°), right arm down | any (pose fallback) | **Pass left**: lead pass to the player's left |
| Right arm out sideways, left arm down | any | **Pass right** |
| Both arms straight up above the head | any | **High-arc lob** |

Left and right are the **player's** own sides. Inference always runs on the
unmirrored image; `camera.mirror_display` only flips the preview. A gesture has
to hold for `gesture.confirm_count` gesture ticks. After that, one held gesture
gives one pass.

At 6–8 m the hands are only a few pixels wide in the 640 px detection frame. So
the hand model gets a crop taken from the full-resolution frame around the
player. When no hands are found, the pose model on the same crop still reads
the arm postures.

## Launch math (see `passer/kinematics.py`)

- The arm angle ψ is 0° when the arm points straight back and 90° when it points straight up. The ball's elevation at release is `90° − ψ_release`, and the commanded move is `sweep = ψ_release − home_angle`.
- The exit speed comes from projectile motion from the release point (pivot height + arm geometry) to the target height at the player's distance. `ω = v / (arm_length × efficiency)`.
- Gearbox: `motor_rpm = arm_rpm × 10` is written to `0x0204`. `motor_pulses = sweep/360 × 10 × 10000` is split into whole turns (`0x0202`) and remaining pulses (`0x0203`). Then `0x011F` gets 0 → 1.
- After the throw the arm goes back to home with the reverse move at `return_rpm`.
- `python -m passer.tools.plan_pass` prints the register values for 2–10 m. It warns when the acceleration ramp (FA40) takes up more of the sweep than is available.

## Calibration checklist

1. `camera.hfov_deg`: set it for your lens. It drives both pan error and distance.
2. Distance: stand at 3 m and 6 m and compare with the preview. Adjust `distance.person_height_m` (it is effectively a scale factor).
3. `launcher.arm_length_m`, `pivot_height_m`, `home_angle_deg`: measure these on the machine.
4. `launcher.efficiency`: fire at a known distance, measure where the ball lands, and scale.
5. `ps100.completion_mode`: check how `0x1010` bit 0 behaves with `ps100_cli status` (see the PS100 doc, §7). Use `time` if it is unreliable.
6. Test with the arm unloaded, at low rpm and small sweeps first (`ps100_cli arm 10 --rpm 30`).

## Adding the camera/turret motor

The pan motor hasn't been chosen yet. Everything above it works in degrees:
`PanAxis.move_to(deg)` and `PanAxis.angle()`. To add a motor:

1. Subclass `PanAxis` in `passer/hardware/pan_axis.py` (there is a hobby-servo example in `GpioServoPanAxis`).
2. Register it in `make_pan_axis()` and set `turret.backend` in your YAML.
3. Tune `turret.kp/kd/max_speed_dps`. The PID and slew limit are already in `turret.py`, so the backend only has to forward small position steps.

Until a motor exists, `turret.backend: none` uses a virtual axis. The machine
then fires without waiting to be aimed, and lead passes are logged but have no
effect.

## Open items

- Voice activation as an alternative trigger (see `webcam tracker test/note.txt`).
- Predict the player's motion (lead from velocity rather than a fixed `lead_m`).
- Check on the hardware how the PS100 handles register writes during a move (PS100 doc §10).
