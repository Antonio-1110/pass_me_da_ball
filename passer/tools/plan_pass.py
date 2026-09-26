"""
Print launch plans / PS100 register values without any hardware.

    python -m passer.tools.plan_pass                          # table for 2..10 m
    python -m passer.tools.plan_pass -d 5 -t lob              # one plan
    python -m passer.tools.plan_pass --arm-length 0.8         # try a longer arm
    python -m passer.tools.plan_pass --no-drag                # ideal parabola
    python -m passer.tools.plan_pass -d 6 --plot traj.png     # needs matplotlib
"""

import argparse

from ..config import load_config
from ..kinematics import plan_launch
from ..physics import trajectory


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config")
    ap.add_argument("-d", "--distance", type=float, help="metres from arm pivot")
    ap.add_argument("-t", "--type", default=None, help="profile name (chest, lob, ...)")
    ap.add_argument("--arm-length", type=float, help="override launcher.arm_length_m")
    ap.add_argument("--pivot-height", type=float, help="override launcher.pivot_height_m")
    ap.add_argument("--no-drag", action="store_true")
    ap.add_argument("--plot", help="save a trajectory plot (with -d) to this PNG")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    lc = cfg.launcher
    if args.arm_length:
        lc.arm_length_m = args.arm_length
    if args.pivot_height:
        lc.pivot_height_m = args.pivot_height
    if args.no_drag:
        lc.ball.drag = False

    types = [args.type] if args.type else list(lc.profiles)
    if args.distance is not None:
        plans = [plan_launch(args.distance, t, lc) for t in types]
        for p in plans:
            print(p.summary())
        if args.plot:
            _plot(plans, lc, args.plot)
        return

    cal = lc.calibration
    print(f"arm {lc.arm_length_m} m, pivot {lc.pivot_height_m} m, gear {lc.gear_ratio}:1, "
          f"drag {'on' if lc.ball.drag else 'off'}, speed_scale {cal.speed_scale}, "
          f"angle_offset {cal.angle_offset_deg}")
    print(f"{'type':6} {'dist':>5} {'phys v':>7} {'cmd v':>6} {'arm rpm':>8} {'0x0204':>7} "
          f"{'0x0202':>7} {'0x0203':>7}  ok")
    for t in types:
        for d in range(2, 11):
            p = plan_launch(float(d), t, lc)
            print(f"{t:6} {d:5.1f} {p.physics_speed_mps:7.2f} {p.exit_velocity_mps:6.2f} "
                  f"{p.arm_rpm:8.1f} {p.motor_rpm:7d} {p.move.turns:7d} {p.move.pulses:7d}  "
                  f"{'yes' if p.ok else 'NO'}{'  ' + '; '.join(p.warnings) if p.warnings else ''}")


def _plot(plans, lc, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 4))
    for p in plans:
        if not p.ok:
            continue
        pts = trajectory(p.physics_speed_mps, p.launch_angle_deg, lc.ball,
                         x0=p.release_x_m, y0=p.release_height_m)
        ax.plot([x for _, x, _ in pts], [y for _, _, y in pts], label=p.pass_type)
        ax.plot([p.target_distance_m], [lc.profiles[p.pass_type].target_height_m], "o")
    ax.axhline(0, color="grey", lw=0.8)
    ax.set_xlabel("distance from pivot (m)")
    ax.set_ylabel("height (m)")
    ax.set_aspect("equal")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"saved {path}")


if __name__ == "__main__":
    main()
