"""
Print launch plans / PS100 register values without any hardware.

    python -m passer.tools.plan_pass                    # table for 2..10 m
    python -m passer.tools.plan_pass -d 5 -t lob        # one plan
    python -m passer.tools.plan_pass --config configs/rpi.yaml
"""

import argparse

from ..config import load_config
from ..kinematics import plan_launch


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config")
    ap.add_argument("-d", "--distance", type=float, help="metres from arm pivot")
    ap.add_argument("-t", "--type", default=None, help="profile name (chest, lob, ...)")
    args = ap.parse_args(argv)
    cfg = load_config(args.config)
    lc = cfg.launcher

    types = [args.type] if args.type else list(lc.profiles)
    if args.distance is not None:
        for t in types:
            print(plan_launch(args.distance, t, lc).summary())
        return

    print(f"gear {lc.gear_ratio}:1, arm {lc.arm_length_m} m, pivot {lc.pivot_height_m} m, "
          f"efficiency {lc.efficiency}")
    print(f"{'type':6} {'dist':>5} {'v m/s':>6} {'arm rpm':>8} {'0x0204':>7} "
          f"{'0x0202':>7} {'0x0203':>7}  ok")
    for t in types:
        for d in range(2, 11):
            p = plan_launch(float(d), t, lc)
            print(f"{t:6} {d:5.1f} {p.exit_velocity_mps:6.2f} {p.arm_rpm:8.1f} "
                  f"{p.motor_rpm:7d} {p.move.turns:7d} {p.move.pulses:7d}  "
                  f"{'yes' if p.ok else 'NO'}{'  ' + '; '.join(p.warnings) if p.warnings else ''}")


if __name__ == "__main__":
    main()
