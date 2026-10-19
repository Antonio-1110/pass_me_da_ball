# pass_me_da_ball

A vision-guided basketball passing machine. A camera on a small servo keeps
the player in view. The launcher sits on a stepper turret, predicts where
the player is going and aims ahead of them. The player asks for a pass with a
gesture, and a PS100 servo drive swings a geared launch arm with the speed
and sweep worked out from the distance to the catch point.

```text
 camera thread            inference thread                        main loop (30 Hz)
 ─────────────            ────────────────                        ─────────────────
 Pi Camera / webcam  ──►  YOLO on lores (640 px) ─► pick player ─► bearing + distance ─► player filter
 main + lores frames      distance + pan error                      camera servo: keep player centred
                          every N frames:                           launcher turret: lead the player
                           crop hi-res ROI ─► MediaPipe Hands       confirmed gesture ─► aim at catch point
                                          └► MediaPipe Pose         kinematics ─► PS100 over RS-485
                           debounce ─► command queue                (fire, return, reload)
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
| `passer/controller.py` | Decision loop: observe, track, aim a pending shot, fire (no camera needed, so it's testable) |
| `passer/turret.py` | Two-axis control: camera servo on top of the launcher turret, frame-time heading lookup, speed/accel-limited motion |
| `passer/prediction.py` | Kalman filter on the player's floor position and velocity, plus a bearing filter for the camera |
| `passer/aiming.py` | Where the launcher points: predicted catch point (fire latency + flight time) plus gesture lead |
| `passer/hardware/pan_axis.py` | **Plug-in point for the camera servo and the launcher stepper** |
| `passer/web/` | Browser dashboard: live camera view, status, live settings, launch preview |
| `passer/physics.py` | Ball flight with gravity and air drag; solves the launch speed to reach a catch point |
| `passer/kinematics.py` | Arm model (arm length → release point and ball speed), calibration layer, PS100 registers |
| `passer/calibration.py` | Fits the sim-to-real corrections from test shots |
| `passer/hardware/ps100.py` | Modbus RTU driver, plus a simulated drive for dry runs |
| `passer/launcher.py` | Fire, return and reload cycle, on its own thread |
| `passer/tools/` | `plan_pass`, `ps100_cli`, `list_cameras` |
| `webcam tracker test/` | Original MacBook prototype (kept for reference) |
| `docs/ps100.md` | PS100 drive: wiring, settings, registers, alarm codes, bench status |
| `docs/manuals/` | PS100 manuals (PDF, Chinese) |

## Quick start

**MacBook (development):**
```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements-mac.txt
python -m passer.app --config configs/macbook.yaml     # dry run: PS100 is simulated
```

**Raspberry Pi 5 + Camera Module 3:**
Raspberry Pi OS:
```bash
sudo apt install -y python3-picamera2 python3-opencv
python3 -m venv --system-site-packages .venv && . .venv/bin/activate
pip install -r requirements-pi.txt
```

Ubuntu (no `python3-picamera2` package; Ubuntu's libcamera also lacks the Pi 5 pipeline, so build Raspberry Pi's fork and install picamera2 from pip):
```bash
sudo apt install -y git python3-pip python3-venv python3-dev build-essential pkg-config \
    meson ninja-build cmake python3-yaml python3-ply python3-jinja2 libyaml-dev \
    libssl-dev libdw-dev libunwind-dev libudev-dev libevent-dev libdrm-dev libjpeg-dev \
    libpng-dev libcap-dev libgnutls28-dev openssl pybind11-dev libboost-dev python3-opencv
sudo usermod -aG video $USER          # camera device access; log out/in afterwards
# build outside this repo, with no venv active (meson must use the system python3 that has ply/yaml/jinja2)
git clone https://github.com/raspberrypi/libcamera.git ~/src/libcamera
meson setup ~/src/libcamera/build ~/src/libcamera --buildtype=release \
    -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp \
    -Dv4l2=true -Dgstreamer=disabled -Dtest=false -Dlc-compliance=disabled \
    -Dcam=enabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
ninja -C ~/src/libcamera/build
sudo ninja -C ~/src/libcamera/build install
# picamera2 also needs pykms (kms++), which Ubuntu doesn't package either
git clone https://github.com/tomba/kmsxx.git ~/src/kmsxx
meson setup ~/src/kmsxx/build ~/src/kmsxx --buildtype=release \
    -Dpykms=enabled -Domap=disabled -Dkmscube=false -Dutils=false
ninja -C ~/src/kmsxx/build
sudo ninja -C ~/src/kmsxx/build install
sudo ldconfig                         # separate line: run it even if an install step complains
python3 -m venv --system-site-packages .venv && . .venv/bin/activate
# if `python -c "import libcamera, pykms"` fails, point the venv at the installed bindings:
#   printf '%s\n' /usr/local/lib/python3/dist-packages \
#       /usr/local/lib/aarch64-linux-gnu/python3.12/site-packages \
#       > .venv/lib/python3.12/site-packages/libcamera.pth
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu   # else pip pulls GBs of NVIDIA CUDA wheels
pip install -r requirements-pi.txt picamera2
python -c "from picamera2 import Picamera2; print(Picamera2.global_camera_info())"   # [] until a camera is connected
cam -l                                  # libcamera's own camera list
```
If you'd rather skip the build, use a USB webcam instead: set `camera.backend: opencv` in `configs/rpi.yaml` and run `python -m passer.tools.list_cameras` to find the index.

Then, on either OS:
```bash
yolo export model=yolov8n.pt format=ncnn imgsz=320     # optional, 3-4x faster; then set detector.yolo_model
./run.sh            # dry run; dashboard on :8080 (= python -m passer.app --config configs/rpi.yaml --headless)
./run.sh --live     # REALLY moves the arm; asks you to confirm the area is clear
./stop.sh           # stop it from another terminal (clean shutdown, force only if it hangs)
```
While LIVE, the dashboard locks the hard limits and geometry (max rpm, arm angles, gear ratio, turret limits, trigger guards); set those in dry run.

Other flags: `--video clip.mp4` replays a recording through the whole stack.
Preview keys: `q` quit, `c` / `b` manual chest / lob, `r` clear a launcher fault, `s` stop the drive.

**Tests** (no camera or models needed): `pip install -r requirements-dev.txt && pytest`

## Dashboard

While the app runs it serves a dashboard at `http://<pi-address>:8080/`. Open
it from a phone or laptop on the same network. `--port` changes the port and
`--no-web` turns it off.

- **Camera:** live view with the player box, gesture crop and skeleton. Frames are only drawn and encoded while someone is watching.
- **Status:** distance, bearing and speed, gesture progress, both turret axes, arm state, frame rates.
- **Launch preview:** pick a distance and pass type to see ball speed, motor rpm, register values and warnings, computed with the current settings.
- **Login:** set `web.password` (the configs ship with `changeme`; change it). Until you log in, the server sends only the login page and refuses every API call.
- **Sessions:** the machine only throws during a session. Start or end one from the dashboard, or by holding your forearms crossed in an X in front of your chest (see Gestures). Outside a session it tracks but ignores throw gestures. With `--no-web` a session starts automatically.
- **Players:** per-player profiles (height, side-pass lead, and arc and catch height for each pass type), stored as `configs/players/<name>.yaml`. Pick the player when starting a session; their profile applies for that session and the machine's own values come back when it ends. The crossed-arms gesture uses the last picked player. Profiles can only set those fields, within safe ranges, never the hard limits.
- **Settings:** every config value, with help text taken from `config.py`. Each change asks for confirmation, then applies to the running machine straight away and is type-checked (a bad value is rejected and nothing is half-applied). Changed values are highlighted, with a reset to default. **Save** writes everything that differs from the defaults to `configs/live.yaml` (never `app.dry_run` or other command-line flags); start with `PASSER_CONFIG=configs/live.yaml ./run.sh` to reuse it.
  - All settings are **locked while a session is running**.
  - In LIVE mode, the hard limits and geometry (max rpm, arm angles, gear ratio, turret limits, trigger guards) are locked even outside a session; set them in dry run.
  - Settings read only at startup (camera resolution, model files, serial port, axis backends) show a **restart** tag.
  - `app.dry_run` and the dashboard's own safety settings are **locked**: switching to LIVE is done on the machine with `--live`.
- **Actions:** start/end session, chest, lob, pass left/right, stop drive, reset fault. The fire buttons need a running session, and only work in dry run unless `web.allow_fire_live: true` is set in the config file. They also ask for confirmation in LIVE. Stop always works.

**Security:** the login is a plain password over HTTP, so keep the Pi on a private network too. With an empty `web.password` there is no login at all. The server only accepts JSON POSTs, which stops other websites from triggering actions through your browser.

## Gestures

| Gesture | Range | Command |
|---|---|---|
| Both palms open, fingers up, facing camera | close (hands readable) | **Chest pass** at the player |
| Left arm out sideways (~90°), right arm down | any (pose fallback) | **Pass left**: lead pass to the player's left |
| Right arm out sideways, left arm down | any | **Pass right** |
| Both arms straight up above the head | any | **High-arc lob** |
| Forearms crossed in an X in front of the chest, held ~1 s | any | **Start / end a session** (not a throw) |

Left and right are the **player's** own sides. Inference always runs on the
unmirrored image; `camera.mirror_display` only flips the preview. A gesture has
to hold for `gesture.confirm_count` gesture ticks (`session_confirm_count` for
the session gesture, so it takes a longer hold). After that, one held gesture
gives one pass. Throw gestures only work during a session.

At 6–8 m the hands are only a few pixels wide in the 640 px detection frame. So
the hand model gets a crop taken from the full-resolution frame around the
player. When no hands are found, the pose model on the same crop still reads
the arm postures.

## Physics model (`physics.py`, `kinematics.py`)

**The main input is `launcher.arm_length_m`.** Once you know it, you choose
two things for every pass: the arm speed ω and the arm angle where the ball
leaves (ψ). Everything else follows from them:

- **Ball speed** `v = ω × arm_length`
- **Launch angle** `θ = 90° − ψ`, because the ball leaves tangent to the arm. The arm angle ψ is 0° when the arm points straight back and 90° when it points straight up.
- **Release point** `x = −L cos ψ`, `h = pivot_height + L sin ψ`

So besides the arm length, the model needs `pivot_height_m` (it sets the release height) and each profile's catch height.

For each pass, the planner:

1. Takes the profile's launch angle and catch height, and the player's distance from vision.
2. Simulates the ball flight **with air drag** (RK4) and solves for the speed that reaches the catch point. Drag adds about 6–8% to the speed needed at 8–10 m compared with a vacuum parabola. `ball.drag: false` switches it off.
3. Applies the **calibration layer** (below).
4. Converts ω into arm rpm, then motor rpm (× gear ratio), which goes to `0x0204`. The sweep is split into turns and pulses for `0x0202` / `0x0203`.
5. Deals with the drive's speed ramps. Because the ball sits in an open cup, it leaves as soon as the arm starts to **decelerate**. So the move is extended by the FA41 braking travel: braking starts exactly at the release angle, while the arm is still at full speed. The planner warns when the FA40 acceleration ramp doesn't fit before release, and refuses when the move would go past `max_arm_angle_deg`.

Explore designs without hardware:
```bash
python -m passer.tools.plan_pass                       # table for 2–10 m
python -m passer.tools.plan_pass --arm-length 0.8      # what would a longer arm need?
python -m passer.tools.plan_pass -d 6 --plot traj.png  # trajectory plot (matplotlib)
```

## Sim-to-real calibration

The model won't match the real machine exactly (ball slip in the cup, arm
flex, the exact moment of release, how accurate the vision distance is). The
corrections are kept separate from the physics, so you can adjust them
without touching it:

| Knob (`launcher.calibration.*`) | Use it when |
|---|---|
| `speed_scale` | The ball lands short (raise it) or long (lower it) at all ranges |
| `speed_offset_mps` | Fixed speed loss, added before scaling |
| `speed_table: {chest: [[3, 1.0], [8, 1.06]]}` | The error changes with distance; linear interpolation per profile |
| `angle_offset_deg` | The ball flies steeper (+) or flatter (−) than planned; the release angle shifts to compensate |
| `distance_scale`, `distance_offset_m` | The vision distance is off |
| `profiles.<name>.speed_scale` / `.angle_offset_deg` | Trims for a single pass type |

**Fitting them from test shots:**
```bash
python -m passer.tools.calibrate predict --profile chest --rpm 1000 1300 1600  # where the model expects them to land
python -m passer.tools.calibrate fire --profile chest --rpm 1300 --live        # fire, then type the landing distance
python -m passer.tools.calibrate add --profile lob --rpm 1300 --landed 6.9 --flight-time 1.45
python -m passer.tools.calibrate fit                                           # prints YAML to paste into your config
```
Measure landing distance from the pivot to where the ball first hits the
floor. If you also time the flight from slow-motion video (release → floor),
the fit can separate a speed error from an angle error for that pass type.
Without timing it can only correct speed. Use at least 3 rpms per pass type,
covering the distances you care about. Shots are logged to
`calibration/shots.csv`.

**Other things to set:**
1. `camera.hfov_deg`: set it for your lens. It drives both pan error and distance.
2. Distance: stand at 3 m and 6 m and compare with the preview. Adjust `distance.person_height_m` or `calibration.distance_scale`.
3. `launcher.arm_length_m`, `pivot_height_m`, `home_angle_deg`, `max_arm_angle_deg`: measure these on the machine.
4. `accel_ms_per_1000rpm` / `decel_ms_per_1000rpm`: copy them from the drive's FA40 / FA41.
5. `ps100.completion_mode`: `level` (default) matches how the drive's `0x1010` bit 0 behaves (see `docs/ps100.md`, "A move"). Use `time` if it is unreliable.
6. Test with the arm unloaded, at low rpm and small sweeps first (`ps100_cli arm 10 --rpm 30`).

## Two-axis turret

```text
 launcher turret (stepper, world angle L)      slow and heavy: points where the player WILL be
   └─ camera servo (hobby servo, angle C relative to the launcher)   fast and light: keeps the player in frame

 camera heading = L + C          player bearing = camera heading (at frame time) + offset in the image
```

Each tick (~30 Hz):

1. **Observe.** The vision result becomes a world bearing. The camera heading is looked up at the **frame's timestamp**, so the camera's own motion during inference latency (~0.1 s) doesn't skew it. The bearing, plus distance when known, goes into the player filters.
2. **Launcher.** Between shots it follows the predicted lead angle, so a shot only needs a small correction (`track_between_shots`). When a gesture is confirmed, it aims at the **predicted catch point**: where the player will be after `fire_latency_s` plus the ball's flight time. It fires once it is within `on_target_deg`. PASS_LEFT / PASS_RIGHT add `lead_m` sideways.
3. **Camera.** Its target is the predicted bearing now minus the launcher angle. That subtraction is a feed-forward: when the turret swings, the servo counter-rotates, so the player stays in frame (1° drift in simulation during a 30° swing).

Both axes use a speed- and acceleration-limited follower with velocity feed-forward: no lag on a moving target, no overshoot on a step.

**Adding the motors.** Everything above `passer/hardware/pan_axis.py` works in degrees (`move_to(deg)`, `angle()`).

- **Camera servo:** already supported. Set `camera_axis.backend: gpio_servo` with `gpio_pin`, `trim_deg` (servo angle that points along the launcher) and `invert` if it turns the wrong way.
- **Launcher stepper(s):** subclass `PanAxis` and register it in `make_launcher_axis()`. Prefer a driver that takes position commands (a closed-loop stepper over RS-485/CAN, or a microcontroller running AccelStepper over serial). Pulsing steps from Python on the Pi is jittery.

Until a motor exists, `backend: none` uses a virtual axis that counts as fixed
at 0°. The stack still runs and shows the commanded angles. With no launcher
motor, shots fire without waiting to be aimed.

**Tuning:** `launcher_axis.max_speed_dps / max_accel_dps2` (keep the heavy
turret gentle), `fire_latency_s` (time from the fire command to the ball
leaving the arm; measure it), and `prediction.*` (how jumpy the player filter
is).

## Open items

- Voice activation as an alternative trigger (see `webcam tracker test/note.txt`).
- Driver for the launcher turret stepper(s) once the hardware is chosen.
- Check on the hardware how the PS100 handles register writes during a move (`docs/ps100.md`).
