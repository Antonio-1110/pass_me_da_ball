"""
homing.py -- find the arm's home after power-up with the motor's encoder.

The motor has a single-turn absolute encoder (PA62 = 4, 17-bit): it keeps the
rotor angle within one motor revolution through power-off (0x100D), but not
the turn count. Through the gearbox one motor turn is 360 / gear_ratio degrees
of arm (36 at 10:1), so the encoder pins the arm's angle exactly within that
window. If the arm is placed by eye within half a window of home (+-18 deg),
the rotor angle says precisely how far off it is.

    ps100_cli home --set   arm placed exactly at home: record the rotor angle
    ps100_cli home         arm placed roughly at home: move it there exactly

Assumes the arm can't slip on the gearbox shaft: the encoder measures the
motor, not the arm.
"""

from __future__ import annotations

import logging
import re
from pathlib import Path
from typing import Tuple

from .config import Config
from .hardware import ps100_registers as R
from .hardware.ps100 import PS100
from .kinematics import arm_degrees_to_motor_pulses, split_pulses

log = logging.getLogger(__name__)

HOME_RPM = 30                 # motor rpm for homing moves (3 arm rpm at 10:1)
AT_HOME_TOLERANCE_DEG = 0.5   # arm degrees


def window_deg(cfg: Config) -> float:
    """Arm degrees per motor revolution: the span the encoder can tell apart."""
    return 360.0 / cfg.launcher.gear_ratio


def arm_offset_deg(rotor: int, cfg: Config) -> float:
    """
    How far the arm is from home, + = toward the throw, from the rotor angle.
    Only meaningful within half a window of home.
    """
    home = cfg.ps100.home_rotor_counts
    if home is None:
        raise ValueError("home not calibrated: place the arm exactly at home and run "
                         "'ps100_cli home --set'")
    n = R.ENCODER_COUNTS_PER_REV
    diff = (rotor - home + n // 2) % n - n // 2          # shortest way, motor counts
    arm = diff / n * window_deg(cfg)
    # Encoder counts rise with a positive raw command; reverse_direction makes
    # a positive raw command turn the arm away from the throw.
    return -arm if cfg.ps100.reverse_direction else arm


def offset_now(drive: PS100, cfg: Config) -> float:
    return arm_offset_deg(drive.rotor_angle(), cfg)


def _move_arm(drive: PS100, cfg: Config, arm_deg: float) -> None:
    lc = cfg.launcher
    turns, pulses = split_pulses(arm_degrees_to_motor_pulses(arm_deg, lc.gear_ratio,
                                                             lc.pulses_per_rev), lc.pulses_per_rev)
    if turns == 0 and pulses == 0:
        return
    est = abs(turns * lc.pulses_per_rev + pulses) / lc.pulses_per_rev / HOME_RPM * 60 + 0.3
    if not drive.move(turns, pulses, HOME_RPM, est_time_s=est):
        raise RuntimeError("homing move did not complete")


def home(drive: PS100, cfg: Config, max_offset_deg: float = 15.0) -> Tuple[float, float]:
    """
    Move the arm to home. Returns (offset before, offset after) in arm degrees.
    Refuses offsets beyond max_offset_deg (the arm may be in the wrong window),
    and checks with a 1 degree first step that the correction goes the right way.
    """
    before = offset_now(drive, cfg)
    if abs(before) > max_offset_deg:
        raise ValueError(f"arm reads {before:+.1f} deg from home, more than {max_offset_deg:.0f}: "
                         f"place it within about {max_offset_deg:.0f} deg of home by eye and retry")
    first = max(-1.0, min(1.0, -before))
    if abs(before) > 1.0:
        _move_arm(drive, cfg, first)
        mid = offset_now(drive, cfg)
        if abs(mid) > abs(before) - 0.5:
            raise RuntimeError(f"homing went the wrong way ({before:+.2f} -> {mid:+.2f} deg): "
                               "check reverse_direction")
        _move_arm(drive, cfg, -mid)
    else:
        _move_arm(drive, cfg, -before)
    after = offset_now(drive, cfg)
    if abs(after) > AT_HOME_TOLERANCE_DEG:
        raise RuntimeError(f"still {after:+.2f} deg from home after homing")
    log.info("Homed: %+.2f -> %+.2f deg", before, after)
    return before, after


def at_home(drive: PS100, cfg: Config) -> Tuple[bool, str]:
    """Is the arm at home now? (ok, reason) -- for refusing to start a session."""
    try:
        off = offset_now(drive, cfg)
    except ValueError as e:
        return False, str(e)
    except Exception as e:  # no answer from the drive
        return False, f"can't read the encoder: {e}"
    if abs(off) > AT_HOME_TOLERANCE_DEG:
        return False, f"arm is {off:+.1f} deg from home: run 'ps100_cli home'"
    return True, f"arm at home ({off:+.2f} deg)"


def save_home(path: str | Path, counts: int) -> None:
    """Write ps100.home_rotor_counts into a YAML config, keeping its comments."""
    p = Path(path)
    text = p.read_text(encoding="utf-8")
    line = f"  home_rotor_counts: {counts}     # rotor angle at home, set by 'ps100_cli home --set'"
    if re.search(r"^  home_rotor_counts:.*$", text, flags=re.M):
        text = re.sub(r"^  home_rotor_counts:.*$", line, text, count=1, flags=re.M)
    elif re.search(r"^ps100:\s*$", text, flags=re.M):
        text = re.sub(r"^ps100:\s*$", "ps100:\n" + line, text, count=1, flags=re.M)
    else:
        text = text.rstrip("\n") + "\n\nps100:\n" + line + "\n"
    p.write_text(text, encoding="utf-8")
