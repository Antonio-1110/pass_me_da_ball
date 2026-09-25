import math
from types import SimpleNamespace as P

from passer.config import GestureConfig
from passer.vision.gestures import (Gesture, GestureDebouncer, P_L_ELBOW, P_L_HIP,
                                    P_L_SHOULDER, P_L_WRIST, P_NOSE, P_R_ELBOW, P_R_HIP,
                                    P_R_SHOULDER, P_R_WRIST, arm_elevation_deg,
                                    classify_pose, combine)

CFG = GestureConfig()


def body(left_arm_deg: float, right_arm_deg: float, vis: float = 0.9):
    """
    Synthetic pose, person facing the camera (unmirrored), image coords
    (y down). The person's LEFT side is on the image RIGHT.
    Arm angle: 0 = hanging down, 90 = out sideways, 180 = straight up.
    """
    lm = [P(x=0.5, y=0.5, visibility=vis) for _ in range(33)]
    lm[P_NOSE] = P(x=0.5, y=0.2, visibility=vis)
    ls, rs = (0.6, 0.35), (0.4, 0.35)
    lm[P_L_SHOULDER] = P(x=ls[0], y=ls[1], visibility=vis)
    lm[P_R_SHOULDER] = P(x=rs[0], y=rs[1], visibility=vis)
    lm[P_L_HIP] = P(x=0.58, y=0.65, visibility=vis)
    lm[P_R_HIP] = P(x=0.42, y=0.65, visibility=vis)

    def arm(sh, deg, outward):
        a = math.radians(deg)
        dx, dy = outward * math.sin(a), math.cos(a)  # 0 deg -> straight down
        return (P(x=sh[0] + 0.1 * dx, y=sh[1] + 0.1 * dy, visibility=vis),
                P(x=sh[0] + 0.2 * dx, y=sh[1] + 0.2 * dy, visibility=vis))

    lm[P_L_ELBOW], lm[P_L_WRIST] = arm(ls, left_arm_deg, +1)
    lm[P_R_ELBOW], lm[P_R_WRIST] = arm(rs, right_arm_deg, -1)
    return lm


def test_arm_elevation_measure():
    lm = body(90, 0)
    assert abs(arm_elevation_deg(lm, P_L_SHOULDER, P_L_WRIST, P_L_HIP) - 90) < 5
    assert arm_elevation_deg(lm, P_R_SHOULDER, P_R_WRIST, P_R_HIP) < 10


def test_left_arm_out_is_pass_left():
    assert classify_pose(body(90, 0), CFG) is Gesture.PASS_LEFT


def test_right_arm_out_is_pass_right():
    assert classify_pose(body(0, 90), CFG) is Gesture.PASS_RIGHT


def test_both_up_is_lob():
    assert classify_pose(body(170, 170), CFG) is Gesture.LOB


def test_neutral_and_ambiguous_are_none():
    assert classify_pose(body(0, 0), CFG) is Gesture.NONE
    assert classify_pose(body(90, 90), CFG) is Gesture.NONE   # T-pose: which way?


def test_low_visibility_is_none():
    assert classify_pose(body(90, 0, vis=0.1), CFG) is Gesture.NONE


def test_combine_prefers_pose():
    assert combine(Gesture.CHEST, Gesture.LOB) is Gesture.LOB
    assert combine(Gesture.CHEST, Gesture.NONE) is Gesture.CHEST


def test_gesture_profile_and_lead():
    assert Gesture.LOB.profile == "lob"
    assert Gesture.PASS_LEFT.profile == "chest"
    assert Gesture.PASS_LEFT.lead_sign == 1 and Gesture.PASS_RIGHT.lead_sign == -1


def test_debouncer_needs_consecutive_hits_and_release():
    d = GestureDebouncer(GestureConfig(confirm_count=3, cooldown_sec=1.0))
    t = 0.0
    out = [d.update(Gesture.LOB, t := t + 0.1) for _ in range(3)]
    assert out == [None, None, Gesture.LOB]
    # still held long after cooldown: must not re-fire
    assert all(d.update(Gesture.LOB, t := t + 1.0) is None for _ in range(5))
    # release, then again
    d.update(Gesture.NONE, t := t + 0.1)
    out = [d.update(Gesture.LOB, t := t + 0.1) for _ in range(3)]
    assert out[-1] is Gesture.LOB


def test_debouncer_resets_on_flicker():
    d = GestureDebouncer(GestureConfig(confirm_count=3, cooldown_sec=0))
    seq = [Gesture.CHEST, Gesture.CHEST, Gesture.NONE, Gesture.CHEST, Gesture.CHEST]
    assert all(d.update(g, i) is None for i, g in enumerate(seq))
