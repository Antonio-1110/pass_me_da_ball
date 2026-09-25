"""
Bench tool for the PS100 over RS-485. Start with the arm unloaded / safe!

    python -m passer.tools.ps100_cli status
    python -m passer.tools.ps100_cli read 0x0204
    python -m passer.tools.ps100_cli arm 30 --rpm 60        # arm +30 deg at 60 MOTOR rpm
    python -m passer.tools.ps100_cli arm -30 --rpm 60       # and back
    python -m passer.tools.ps100_cli throw 4 --type chest   # full plan + return
    python -m passer.tools.ps100_cli stop

Add --dry-run to print the register writes without a drive attached, and
--port /dev/ttyUSB1 etc. to override the config.
"""

import argparse
import logging

from ..config import load_config
from ..hardware import ps100_registers as R
from ..hardware.ps100 import PS100, SimTransport
from ..kinematics import arm_degrees_to_motor_pulses, plan_launch, split_pulses
from ..launcher import Launcher


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config")
    ap.add_argument("--port")
    ap.add_argument("--dry-run", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    r = sub.add_parser("read")
    r.add_argument("register", type=lambda s: int(s, 0))
    a = sub.add_parser("arm", help="relative ARM move in degrees (gearbox applied)")
    a.add_argument("degrees", type=float)
    a.add_argument("--rpm", type=int, default=60, help="MOTOR rpm")
    t = sub.add_parser("throw")
    t.add_argument("distance", type=float)
    t.add_argument("--type", default="chest")
    sub.add_parser("stop")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.DEBUG if args.dry_run else logging.INFO,
                        format="%(levelname)s %(message)s")
    cfg = load_config(args.config)
    if args.port:
        cfg.ps100.port = args.port
    drive = PS100.from_config(cfg.ps100, dry_run=args.dry_run)

    if args.cmd == "status":
        v = drive.t.read_register(R.REG_OUTPUT_STATUS)
        print(f"0x{R.REG_OUTPUT_STATUS:04X} = 0x{v:04X} (bit0={v & 1})")
    elif args.cmd == "read":
        v = drive.t.read_register(args.register)
        print(f"0x{args.register:04X} = {v} (0x{v:04X})")
    elif args.cmd == "arm":
        lc = cfg.launcher
        total = arm_degrees_to_motor_pulses(args.degrees, lc.gear_ratio, lc.pulses_per_rev)
        turns, pulses = split_pulses(total, lc.pulses_per_rev)
        ok = drive.move(turns, pulses, args.rpm)
        print("done" if ok else "no completion signal (check P3-20=16 / completion_mode)")
    elif args.cmd == "throw":
        plan = plan_launch(args.distance, args.type, cfg.launcher)
        print(plan.summary())
        if plan.ok:
            Launcher(drive, cfg.launcher, reload_delay_s=0).fire(plan, block=True)
    elif args.cmd == "stop":
        drive.stop()

    if isinstance(drive.t, SimTransport):
        print("register writes:", ", ".join(f"0x{a:04X}<-{v}" for a, v in drive.t.writes))


if __name__ == "__main__":
    main()
