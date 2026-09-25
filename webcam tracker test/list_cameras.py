"""
list_cameras.py

Diagnostic tool: opens each camera index one at a time (using the
AVFoundation backend explicitly, which is the correct one on macOS) and
shows a preview window with the index number, so you can identify which
index is your Mac's built-in webcam vs. an iPhone Continuity Camera.

Run this BEFORE running human_tracker.py if the wrong camera keeps opening.

Usage:
    python3 list_cameras.py

For each index, a window opens. Press any key to move to the next index.
Press 'q' to stop checking indices early.
"""

import cv2

MAX_INDEX_TO_CHECK = 5

for idx in range(MAX_INDEX_TO_CHECK):
    print(f"\nTrying camera index {idx} ...")
    cap = cv2.VideoCapture(idx, cv2.CAP_AVFOUNDATION)

    if not cap.isOpened():
        print(f"  Index {idx}: could not open (no camera here).")
        cap.release()
        continue

    ret, frame = cap.read()
    if not ret or frame is None:
        print(f"  Index {idx}: opened but couldn't read a frame "
              f"(often means permission wasn't granted, or the device "
              f"needs a moment — try again after granting camera access).")
        cap.release()
        continue

    print(f"  Index {idx}: OK — showing preview. Press any key for next index, "
          f"'q' to quit.")
    cv2.putText(frame, f"Camera index: {idx}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Camera check", frame)
    key = cv2.waitKey(0) & 0xFF
    cap.release()
    cv2.destroyAllWindows()

    if key == ord('q'):
        break

print("\nDone. Use the index that showed your Mac's built-in camera "
      "as CAM_INDEX in human_tracker.py, and pass cv2.CAP_AVFOUNDATION "
      "explicitly for reliability on macOS.")
