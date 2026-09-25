"""
human_tracker.py

Tracks a human in a webcam feed using a YOLOv8 person detector, crops the
bounding box, and runs MediaPipe Hands (legacy Solutions API) on the crop to
detect an "open palm" gesture. Prints:
  - LEFT / CENTER / RIGHT depending on where the person's bounding box is,
    relative to the frame (useful later for driving a servo).
  - "OPEN PALM DETECTED" whenever an open palm is seen inside the crop.

Tested target: MacBook Air M1, built-in webcam / iPhone Continuity Camera.

WHY YOLO INSTEAD OF OPENCV's HOG DETECTOR:
The HOG + Linear SVM detector built into OpenCV is a pre-deep-learning
technique trained narrowly on full-body pedestrian images. It performs
poorly on partial/close-up views (e.g. sitting at a desk near the camera).
YOLOv8n (the "nano" variant) is a real, modern deep learning object
detector trained on ~120k diverse COCO images, and handles partial bodies,
close range, and varied poses far better, while still being small and fast
enough to run in real time on an M1.

WHY THE LEGACY MEDIAPIPE API, NOT THE NEW TASKS API:
MediaPipe's newer Tasks API (HandLandmarker) currently has an unresolved bug
on macOS where its palm-detection sub-graph unconditionally tries to init a
Metal (GPU) helper, crashing with:
    F0000 ... graph_service.h:139] Check failed: service_ Service is unavailable.
    ... DrishtiMetalHelper initWithCalculatorContext ...
This happens even when delegate=BaseOptions.Delegate.CPU is set explicitly.
The older legacy `mp.solutions.hands` API doesn't hit this path and is known
to work reliably on M1/M2 Macs -- but ONLY on mediapipe==0.10.9. Newer
mediapipe releases removed/broke `mp.solutions` entirely on some platforms.

Install deps (inside a virtualenv is strongly recommended):
    pip install opencv-python
    pip install mediapipe==0.10.9
    pip install ultralytics

First run will auto-download the yolov8n.pt weights file (~6MB) into the
current directory -- this requires an internet connection once.

Run:
    python3 human_tracker.py

Press 'q' in the preview window to quit.
"""

import time
import math
import cv2
import mediapipe as mp
from ultralytics import YOLO

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
CAM_INDEX = 0                 # run list_cameras.py if unsure which index is right
PROCESS_WIDTH = 640            # frames are resized (not cropped) to this width in software, for consistent performance across different camera native resolutions
CENTER_DEADZONE_RATIO = 0.15  # +/- 15% of frame width counts as "CENTER"
PRINT_COOLDOWN_SEC = 0.5      # avoid spamming terminal every single frame
PALM_COOLDOWN_SEC = 0.5

YOLO_MODEL = "yolov8n.pt"     # nano variant: smallest/fastest, good enough for this use case
YOLO_CONF_THRESHOLD = 0.4     # minimum detection confidence to accept a person box
YOLO_PERSON_CLASS_ID = 0      # COCO class 0 = "person"


# ---------------------------------------------------------------------------
# Human detector wrapper (YOLOv8, deep learning object detector)
# ---------------------------------------------------------------------------
class HumanDetector:
    def __init__(self, device=None):
        self.model = YOLO(YOLO_MODEL)

        if device is None:
            # Prefer Apple Metal (MPS) acceleration on M1/M2 Macs if available,
            # otherwise fall back to CPU (still plenty fast for yolov8n).
            try:
                import torch
                device = "mps" if torch.backends.mps.is_available() else "cpu"
            except Exception:
                device = "cpu"
        self.device = device
        print(f"[INFO] YOLO running on device: {self.device}")

    def detect(self, frame):
        """Returns the highest-confidence person bounding box (x, y, w, h) or None."""
        results = self.model.predict(
            frame,
            classes=[YOLO_PERSON_CLASS_ID],
            conf=YOLO_CONF_THRESHOLD,
            device=self.device,
            verbose=False,
        )

        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return None

        confs = boxes.conf.cpu().numpy()
        best_idx = confs.argmax()
        x1, y1, x2, y2 = boxes.xyxy.cpu().numpy()[best_idx]
        x, y = int(x1), int(y1)
        w, h = int(x2 - x1), int(y2 - y1)
        return (x, y, w, h)


# ---------------------------------------------------------------------------
# Open palm gesture classifier using MediaPipe Hands landmarks
# ---------------------------------------------------------------------------
mp_hands = mp.solutions.hands

FINGER_TIPS = [
    mp_hands.HandLandmark.INDEX_FINGER_TIP,
    mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
    mp_hands.HandLandmark.RING_FINGER_TIP,
    mp_hands.HandLandmark.PINKY_TIP,
]
FINGER_PIPS = [
    mp_hands.HandLandmark.INDEX_FINGER_PIP,
    mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
    mp_hands.HandLandmark.RING_FINGER_PIP,
    mp_hands.HandLandmark.PINKY_PIP,
]
THUMB_TIP = mp_hands.HandLandmark.THUMB_TIP
THUMB_IP = mp_hands.HandLandmark.THUMB_IP
WRIST = mp_hands.HandLandmark.WRIST
INDEX_MCP = mp_hands.HandLandmark.INDEX_FINGER_MCP
PINKY_MCP = mp_hands.HandLandmark.PINKY_MCP

# If palm-facing detection comes out backwards on your setup (triggers when
# the BACK of the hand faces the camera instead of the palm), flip this.
INVERT_PALM_ORIENTATION = False

# Raised-palm / facing-camera thresholds (tweak if needed)
PALM_RAISED_Y_THRESHOLD = 0.65  # wrist.y must be above this (0=top, 1=bottom of crop)
PALM_TIP_ABOVE_WRIST_MARGIN = 0.02  # mean tip.y must be this much above wrist.y
PALM_FACING_CROSS_EPS = 1e-4  # ignore tiny cross-product values (noise)
PALM_ANGLE_THRESHOLD_DEG = 50.0  # max angle (deg) between palm-normal and camera axis
ENABLE_PALM_DEBUG_OVERLAY = True  # show angle/norm/cross on the crop for tuning

def is_open_palm(hand_landmarks) -> bool:
    """
    Simple heuristic: a hand is an "open palm" if all four fingers
    (index..pinky) are extended, i.e. the tip is farther from the wrist
    than the pip joint is. Thumb is checked more loosely since its
    geometry differs.
    """
    lm = hand_landmarks.landmark
    wrist = lm[WRIST]

    def dist(a, b):
        return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5

    # Count extended fingers (distance-based), and also check vertical
    # direction (tips above pips and above wrist) to ensure the hand
    # is raised with fingers pointing upward.
    extended_count = 0
    tips_above_pips = 0
    tips_above_wrist = 0
    tip_ys = []
    for tip_idx, pip_idx in zip(FINGER_TIPS, FINGER_PIPS):
        tip = lm[tip_idx]
        pip = lm[pip_idx]
        if dist(tip, wrist) > dist(pip, wrist):
            extended_count += 1
        if tip.y < pip.y:
            tips_above_pips += 1
        if tip.y < wrist.y:
            tips_above_wrist += 1
        tip_ys.append(tip.y)

    thumb_extended = dist(lm[THUMB_TIP], wrist) > dist(lm[THUMB_IP], wrist)

    # Basic raised/pointing-up heuristic:
    # - wrist is in the upper part of the crop
    # - most finger tips are above their PIP joints
    # - most finger tips are above the wrist
    # - mean tip Y is meaningfully above wrist.y (margin to avoid noise)
    mean_tip_y = sum(tip_ys) / len(tip_ys)
    wrist_raised = wrist.y < PALM_RAISED_Y_THRESHOLD
    fingers_pointing_up = (
        tips_above_pips >= 3 and tips_above_wrist >= 3
        and mean_tip_y < wrist.y - PALM_TIP_ABOVE_WRIST_MARGIN
    )

    return (extended_count == 4 and thumb_extended
            and wrist_raised and fingers_pointing_up)


def is_palm_facing_camera(hand_landmarks, handedness_label: str):
    """
    Determines whether the palm (vs. the back of the hand) is facing the
    camera, using the 2D cross product of the vectors from the wrist to the
    index-finger and pinky knuckles (MCP joints). This is more reliable
    than using MediaPipe's z-depth values, which are fairly noisy for this.

    As the hand rotates about the forearm axis, the index MCP and pinky MCP
    swap their apparent left/right order relative to the wrist -- that
    swap is what this cross product picks up on.

    `handedness_label` should be "Left" or "Right" as reported by MediaPipe
    (chirality flips the sign of the cross product, so we normalize for it).
    """
    lm = hand_landmarks.landmark
    wrist = lm[WRIST]
    index_mcp = lm[INDEX_MCP]
    pinky_mcp = lm[PINKY_MCP]

    # Build 3D vectors (use landmark z for depth) from wrist to index/pinky MCPs
    v1x, v1y, v1z = (
        index_mcp.x - wrist.x,
        index_mcp.y - wrist.y,
        index_mcp.z - wrist.z,
    )
    v2x, v2y, v2z = (
        pinky_mcp.x - wrist.x,
        pinky_mcp.y - wrist.y,
        pinky_mcp.z - wrist.z,
    )

    # 3D cross product = palm normal
    nx = v1y * v2z - v1z * v2y
    ny = v1z * v2x - v1x * v2z
    nz = v1x * v2y - v1y * v2x

    # Use the 2D cross sign logic (nz) for palm-vs-back orientation, but
    # also require the palm normal to be sufficiently aligned with the
    # camera Z axis (i.e. not roughly perpendicular).
    cross_z = nz
    if handedness_label == "Left":
        cross_z = -cross_z

    if abs(cross_z) < PALM_FACING_CROSS_EPS:
        return False, None, None, cross_z

    # Angle between palm normal and camera axis (Z). Use absolute nz so
    # we measure closeness to axis regardless of normal pointing sign.
    norm = (nx * nx + ny * ny + nz * nz) ** 0.5
    if norm < 1e-8:
        return False, None, None, cross_z

    cos_to_z = abs(nz) / norm
    cos_to_z = max(-1.0, min(1.0, cos_to_z))
    angle_deg = math.degrees(math.acos(cos_to_z))

    # If palm plane is too close to perpendicular (angle ~ 90deg), reject.
    if angle_deg > PALM_ANGLE_THRESHOLD_DEG:
        return False, angle_deg, norm, cross_z

    facing_camera = cross_z > 0
    if INVERT_PALM_ORIENTATION:
        facing_camera = not facing_camera

    return facing_camera, angle_deg, norm, cross_z


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main():
    # CAP_AVFOUNDATION is explicit about which macOS camera backend to use.
    # Without it, or if a paired iPhone is nearby with Continuity Camera on,
    # macOS may hand you the iPhone's camera instead of the built-in one.
    # Run list_cameras.py first if you're not sure which index is which.
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_AVFOUNDATION)
    # NOTE: We deliberately do NOT call cap.set(CAP_PROP_FRAME_WIDTH/HEIGHT).
    # Forcing a specific capture resolution can make some cameras (notably
    # iPhone Continuity Camera) satisfy the request by CROPPING a smaller
    # region from the sensor rather than downscaling the full field of view
    # -- which looks like an unwanted "zoom in". Instead we capture at
    # whatever native resolution the camera gives us, and resize in
    # software after reading each frame (see PROCESS_WIDTH below), which
    # always preserves the full original field of view.

    if not cap.isOpened():
        print("ERROR: Could not open webcam. Check camera permissions in "
              "System Settings -> Privacy & Security -> Camera for your "
              "terminal/IDE, and re-run.")
        return

    ret, _ = cap.read()
    if not ret:
        print("ERROR: Camera opened but returned no frame. This usually means "
              "the camera permission prompt hasn't been answered yet, or the "
              "wrong device (e.g. iPhone Continuity Camera) is being used. "
              "Grant permission in System Settings and/or run list_cameras.py "
              "to find the correct CAM_INDEX.")
        cap.release()
        return

    detector = HumanDetector()

    last_position_print = 0.0
    last_palm_print = 0.0
    last_position_label = None

    with mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=2,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.5,
    ) as hands:

        while True:
            ret, frame = cap.read()
            if not ret:
                print("WARNING: Failed to grab frame.")
                break

            frame = cv2.flip(frame, 1)  # mirror for natural webcam feel

            # Software resize (preserves full field of view, unlike setting
            # CAP_PROP_FRAME_WIDTH/HEIGHT on the capture device, which can
            # crop instead of scale on some cameras).
            h0, w0 = frame.shape[:2]
            if w0 > PROCESS_WIDTH:
                scale = PROCESS_WIDTH / w0
                frame = cv2.resize(frame, (PROCESS_WIDTH, int(h0 * scale)))

            frame_h, frame_w = frame.shape[:2]

            bbox = detector.detect(frame)

            if bbox is not None:
                x, y, w, h = bbox
                x, y = max(0, x), max(0, y)
                x2, y2 = min(frame_w, x + w), min(frame_h, y + h)

                cv2.rectangle(frame, (x, y), (x2, y2), (0, 255, 0), 2)

                # --- LEFT / CENTER / RIGHT positioning ---
                bbox_center_x = (x + x2) / 2.0
                frame_center_x = frame_w / 2.0
                deadzone = frame_w * CENTER_DEADZONE_RATIO

                if bbox_center_x < frame_center_x - deadzone:
                    position_label = "LEFT"
                elif bbox_center_x > frame_center_x + deadzone:
                    position_label = "RIGHT"
                else:
                    position_label = "CENTER"

                now = time.time()
                if (position_label != last_position_label) or (
                    now - last_position_print > PRINT_COOLDOWN_SEC
                ):
                    # print(f"[POSITION] Human is {position_label} "
                    #      f"(bbox_center_x={bbox_center_x:.0f}, frame_center_x={frame_center_x:.0f})")
                    last_position_print = now
                    last_position_label = position_label

                # --- Crop the human bbox and run MediaPipe Hands on it ---
                crop = frame[y:y2, x:x2]
                if crop.size > 0:
                    crop_rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
                    results = hands.process(crop_rgb)

                    if results.multi_hand_landmarks:
                        handedness_list = results.multi_handedness or []
                        valid_open_palms = 0
                        detected_hands = len(results.multi_hand_landmarks)

                        for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                            handedness_label = "Right"
                            if idx < len(handedness_list):
                                handedness_label = handedness_list[idx].classification[0].label

                            open_palm = is_open_palm(hand_landmarks)
                            facing_camera, angle_deg, norm, cross_z = is_palm_facing_camera(hand_landmarks, handedness_label)

                            if open_palm and facing_camera:
                                valid_open_palms += 1

                            mp.solutions.drawing_utils.draw_landmarks(
                                crop, hand_landmarks, mp_hands.HAND_CONNECTIONS
                            )

                            # Debug overlay for palm diagnostics
                            if ENABLE_PALM_DEBUG_OVERLAY:
                                y0 = 16
                                dy = 18
                                font = cv2.FONT_HERSHEY_SIMPLEX
                                color = (0, 255, 255)
                                if angle_deg is None:
                                    txt1 = f"cross_z={cross_z:.4f}"
                                    cv2.putText(crop, txt1, (6, y0), font, 0.5, color, 1)
                                else:
                                    txt1 = f"angle={angle_deg:.1f}deg"
                                    txt2 = f"norm={norm:.3f} cross_z={cross_z:.4f}"
                                    cv2.putText(crop, txt1, (6, y0), font, 0.5, color, 1)
                                    cv2.putText(crop, txt2, (6, y0+dy), font, 0.5, color, 1)

                        if detected_hands >= 2 and valid_open_palms >= 2:
                            now = time.time()
                            if now - last_palm_print > PALM_COOLDOWN_SEC:
                                print("[GESTURE] OPEN PALM DETECTED (both hands facing camera)")
                                last_palm_print = now
                    frame[y:y2, x:x2] = crop
            else:
                pass

            cv2.imshow("Human Tracker (press 'q' to quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()