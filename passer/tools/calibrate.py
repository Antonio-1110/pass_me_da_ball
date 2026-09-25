"""
Sim-to-real calibration workflow.

  1. See what the model expects (no hardware):
       python -m passer.tools.calibrate predict --profile chest --rpm 1200 1500 1800

  2. Fire test shots at fixed rpms and type in where the ball landed
     (horizontal distance from the arm pivot to where it first hit the floor):
       python -m passer.tools.calibrate fire --profile chest --rpm 1500 --live
     or log shots you fired some other way:
       python -m passer.tools.calibrate add --profile chest --rpm 1500 --landed 6.3 [--flight-time 0.82]

     Timing the flight from slow-motion video (release -> floor) also lets
     the fit recover the launch-angle error, not just the speed error.

  3. Fit and paste the printed YAML into your config file:
       python -m passer.tools.calibrate fit

Shots are appended to calibration/shots.csv (change with --shots).
Aim for 3+ rpms per profile spanning the distances you care about.
"""

import argparse
import logging

from ..calibration import fit, load_shots, make_shot, predict_landing, save_shot
from ..config import load_config
from ..hardware.ps100 import PS100
from ..kinematics import plan_fixed_rpm
from ..launcher import Launcher

DEFAULT_SHOTS = "calibration/shots.csv"


def _ask_float(prompt: str) -> float:
    s = input(prompt).strip()
    return float(s) if s else 0.0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config")
    ap.add_argument("--shots", default=DEFAULT_SHOTS)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("predict", help="predicted landing distance for given rpms")
    p.add_argument("--profile", default="chest")
    p.add_argument("--rpm", type=float, nargs="+", required=True)

    f = sub.add_parser("fire", help="fire a fixed-rpm test shot, then record where it landed")
    f.add_argument("--profile", default="chest")
    f.add_argument("--rpm", type=int, required=True)
    f.add_argument("--live", action="store_true", help="use the real PS100 (default: dry run)")

    a = sub.add_parser("add", help="record a shot manually")
    a.add_argument("--profile", default="chest")
    a.add_argument("--rpm", type=float, required=True)
    a.add_argument("--landed", type=float, required=True, help="m from pivot")
    a.add_argument("--land-height", type=float, default=0.0)
    a.add_argument("--flight-time", type=float, default=0.0)

    sub.add_parser("fit", help="fit calibration from recorded shots")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    cfg = load_config(args.config)
    lc = cfg.launcher

    if args.cmd == "predict":
        for rpm in args.rpm:
            d = predict_landing(args.profile, rpm, lc)
            print(f"{args.profile} @ {rpm:.0f} motor rpm -> lands {d:.2f} m from pivot")

    elif args.cmd == "fire":
        plan = plan_fixed_rpm(args.rpm, args.profile, lc)
        print(f"{args.profile} test shot: motor {plan.motor_rpm} rpm "
              f"(ideal ball speed {plan.exit_velocity_mps:.2f} m/s), release at "
              f"{plan.release_angle_deg:.1f} deg, sweep {plan.sweep_deg:.1f} deg -> "
              f"turns={plan.move.turns} pulses={plan.move.pulses}")
        for w in plan.warnings:
            print(f"  ! {w}")
        print(f"model predicts landing at {predict_landing(args.profile, args.rpm, lc):.2f} m")
        if not plan.ok:
            return
        drive = PS100.from_config(cfg.ps100, dry_run=not args.live)
        Launcher(drive, lc, reload_delay_s=0).fire(plan, block=True)
        landed = _ask_float("Landed at (m from pivot, blank = discard): ")
        if landed <= 0:
            print("discarded")
            return
        t = _ask_float("Flight time in s (blank = not measured): ")
        save_shot(args.shots, make_shot(args.profile, args.rpm, landed, lc, flight_time_s=t))
        print(f"saved to {args.shots}")

    elif args.cmd == "add":
        save_shot(args.shots, make_shot(args.profile, args.rpm, args.landed, lc,
                                        args.land_height, args.flight_time))
        print(f"saved to {args.shots}")

    elif args.cmd == "fit":
        result = fit(load_shots(args.shots), lc)
        print(f"{'profile':8} {'rpm':>6} {'landed':>7} {'ideal v':>8} {'real v':>7} "
              f"{'mult':>6} {'angle err':>9}")
        for an in result.analyses:
            s = an.shot
            ae = f"{an.angle_error:+.1f}" if an.angle_error is not None else "-"
            print(f"{s.profile:8} {s.motor_rpm:6.0f} {s.landed_m:7.2f} {an.ideal_speed:8.2f} "
                  f"{an.real_speed:7.2f} {an.multiplier:6.3f} {ae:>9}")
        print("\nPaste into your config (replaces the calibration block; reset any "
              "per-profile speed_scale to 1.0):\n")
        print(result.yaml())


if __name__ == "__main__":
    main()
