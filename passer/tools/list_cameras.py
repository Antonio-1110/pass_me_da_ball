"""
Find which camera to use.

    python -m passer.tools.list_cameras

On a Pi, lists libcamera (CSI ribbon) cameras via picamera2 and their sensor
modes. Then, on any OS, opens OpenCV indices 0..4 one at a time with a
preview (any key = next, q = stop). Use --no-preview over SSH.
"""

import argparse
import platform

import cv2


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-index", type=int, default=5)
    ap.add_argument("--no-preview", action="store_true")
    args = ap.parse_args(argv)

    try:
        from picamera2 import Picamera2
        cams = Picamera2.global_camera_info()
        print(f"libcamera cameras: {len(cams)}")
        for i, c in enumerate(cams):
            print(f"  [{i}] {c.get('Model')} {c.get('Id')}")
            with Picamera2(i) as cam:
                for m in cam.sensor_modes:
                    print(f"      mode {m['size']} @ {m.get('fps', 0):.0f} fps "
                          f"crop={m.get('crop_limits')}")
    except ImportError:
        print("picamera2 not available (not a Pi, or venv lacks --system-site-packages)")

    api = {"Darwin": cv2.CAP_AVFOUNDATION, "Linux": cv2.CAP_V4L2}.get(platform.system(), cv2.CAP_ANY)
    for idx in range(args.max_index):
        cap = cv2.VideoCapture(idx, api)
        if not cap.isOpened():
            print(f"OpenCV index {idx}: not available")
            continue
        ok, frame = cap.read()
        if not ok or frame is None:
            print(f"OpenCV index {idx}: opened but no frame (permission / busy?)")
            cap.release()
            continue
        print(f"OpenCV index {idx}: OK {frame.shape[1]}x{frame.shape[0]}")
        if not args.no_preview:
            cv2.putText(frame, f"index {idx}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("camera check", frame)
            key = cv2.waitKey(0) & 0xFF
            cv2.destroyAllWindows()
            if key == ord("q"):
                cap.release()
                break
        cap.release()


if __name__ == "__main__":
    main()
