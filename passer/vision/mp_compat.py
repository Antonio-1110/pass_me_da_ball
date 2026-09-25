"""
MediaPipe import shim.

We use the legacy `mp.solutions` API (Hands / Pose). On macOS the newer Tasks
API crashes trying to initialise Metal even with a CPU delegate, and the
legacy API is only reliable there on mediapipe==0.10.9. On the Pi (aarch64
Linux) the legacy API is available in the 0.10.x wheels too; newer releases
have been dropping it, so pin the version (see requirements-*.txt).
"""


def mp_solutions():
    try:
        import mediapipe as mp
    except ImportError as e:  # pragma: no cover
        raise ImportError("mediapipe is not installed (pip install mediapipe==0.10.9 "
                          "on macOS, mediapipe==0.10.18 on the Pi)") from e
    if not hasattr(mp, "solutions"):  # pragma: no cover
        raise ImportError(f"mediapipe {mp.__version__} has no legacy `solutions` API; "
                          "install a 0.10.x version that still ships it")
    return mp.solutions
