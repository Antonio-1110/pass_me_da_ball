"""
Bench tool for the PS100 over RS-485. Start with the arm unloaded / safe!

    python -m passer.tools.ps100_cli check                  # read every setting the move needs
    python -m passer.tools.ps100_cli monitor                # mode, alarm, inputs, position command...
    python -m passer.tools.ps100_cli status
    python -m passer.tools.ps100_cli pos                    # encoder position (did it move?)
    python -m passer.tools.ps100_cli read 0x0204            # or a name: read P3-38
    python -m passer.tools.ps100_cli alarm 57               # what the code on the display means
    python -m passer.tools.ps100_cli home --set             # arm EXACTLY at home: record it
    python -m passer.tools.ps100_cli home                   # after power-up: arm near home -> exact
    python -m passer.tools.ps100_cli home --check           # how far from home is it?
    python -m passer.tools.ps100_cli arm 30 --rpm 60        # arm +30 deg at 60 MOTOR rpm
    python -m passer.tools.ps100_cli arm -30 --rpm 60       # and back
    python -m passer.tools.ps100_cli throw 4 --type chest   # full plan + return
    python -m passer.tools.ps100_cli stop

Add --dry-run to print the register writes without a drive attached, and
--port /dev/ttyUSB1 etc. to override the config.

Register meanings, alarm codes and a checklist for a move that fails are in
docs/ps100.md.
"""

import argparse
import logging
from pathlib import Path

from ..config import load_config
from ..hardware import ps100_registers as R
from ..hardware.ps100 import PS100, SimTransport
from ..kinematics import arm_degrees_to_motor_pulses, plan_launch, split_pulses
from ..homing import at_home, home, save_home
from ..launcher import Launcher


DEFAULT_CONFIG = "configs/rpi.yaml"


def main(argv=None):
    ap = argparse.ArgumentParser()
    # The bench tool talks to the real machine, so default to its config: the
    # built-in defaults lack e.g. reverse_direction and would swing the arm
    # the wrong way.
    ap.add_argument("--config", default=DEFAULT_CONFIG if Path(DEFAULT_CONFIG).is_file() else None,
                    help=f"YAML config (default: {DEFAULT_CONFIG} if present)")
    ap.add_argument("--port")
    ap.add_argument("--dry-run", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check", help="read every drive setting the RS-485 move depends on")
    sub.add_parser("monitor", help="every status register: mode, alarm, inputs, position command...")
    sub.add_parser("status")
    sub.add_parser("pos", help="encoder absolute position (raw counts)")
    r = sub.add_parser("read")
    r.add_argument("register", type=parse_register, help="0x0204, or a parameter name like P3-38")
    al = sub.add_parser("alarm", help="explain an alarm number shown on the drive display")
    al.add_argument("code", type=int)
    h = sub.add_parser("home", help="move the arm to home with the encoder (place it within ~15 deg first)")
    h.add_argument("--set", action="store_true", help="the arm is exactly at home: record the encoder angle")
    h.add_argument("--check", action="store_true", help="only report how far the arm is from home")
    a = sub.add_parser("arm", help="relative ARM move in degrees (gearbox applied)")
    a.add_argument("degrees", type=float)
    a.add_argument("--rpm", type=int, default=60, help="MOTOR rpm")
    t = sub.add_parser("throw")
    t.add_argument("distance", type=float)
    t.add_argument("--type", default="chest")
    sub.add_parser("stop")
    args = ap.parse_args(argv)

    if args.cmd == "alarm":
        print(f"alarm {args.code}: {R.ALARM_CODES.get(args.code, 'not in the manual table')}")
        return

    logging.basicConfig(level=logging.DEBUG if args.dry_run else logging.INFO,
                        format="%(levelname)s %(message)s")
    cfg = load_config(args.config)
    logging.info("config: %s", args.config or "built-in defaults")
    if args.port:
        cfg.ps100.port = args.port
    drive = PS100.from_config(cfg.ps100, dry_run=args.dry_run)

    if args.cmd == "check":
        bad = print_check(drive.check_settings())
        print(f"{bad} setting(s) differ from what the RS-485 move needs" if bad
              else "all required settings match")
    elif args.cmd == "monitor":
        for addr, label, value in drive.monitor():
            note = ""
            if addr == R.REG_ALARM and value:
                note = f"  <-- {R.ALARM_CODES.get(value, 'not in the manual table')}"
            elif addr in (R.REG_INPUT_STATUS, R.REG_OUTPUT_STATUS):
                note = f"  ({value & 0xF:04b})"
            print(f"0x{addr:04X}  {label:<40} {value}{note}")
    elif args.cmd == "status":
        v = drive.t.read_register(R.REG_OUTPUT_STATUS)
        print(f"0x{R.REG_OUTPUT_STATUS:04X} = 0x{v:04X} (bit0={v & 1})")
    elif args.cmd == "pos":
        print(f"position = {drive.read_position()}")
    elif args.cmd == "read":
        v = drive.t.read_register(args.register)
        print(f"0x{args.register:04X} = {v} (0x{v:04X})")
    elif args.cmd == "arm":
        lc = cfg.launcher
        total = arm_degrees_to_motor_pulses(args.degrees, lc.gear_ratio, lc.pulses_per_rev)
        turns, pulses = split_pulses(total, lc.pulses_per_rev)
        before = try_position(drive)
        ok = drive.move(turns, pulses, args.rpm)
        after = try_position(drive)
        moved = None if before is None or after is None else after != before
        if moved is not None:
            print(f"position {before} -> {after} (change {after - before})")
        if ok:
            print("done")
        elif moved:
            print("motor moved but no completion signal: check P3-20 = 16 and "
                  "completion_timeout_s (long slow moves), or use completion_mode 'time'")
        elif moved is False:
            print("motor did not move: run 'check' and look at the drive display for an alarm")
        else:
            print("no completion signal: run 'check', 'pos' and look at the drive display")
    elif args.cmd == "throw":
        plan = plan_launch(args.distance, args.type, cfg.launcher)
        print(plan.summary())
        if plan.ok:
            Launcher(drive, cfg.launcher, reload_delay_s=0).fire(plan, block=True)
    elif args.cmd == "home":
        if args.set:
            counts = drive.rotor_angle()
            if args.config and not args.dry_run:
                save_home(args.config, counts)
                print(f"home set: rotor angle {counts} saved to {args.config}")
            else:
                print(f"rotor angle {counts} (not saved: dry run or no config file)")
        elif args.check:
            print(at_home(drive, cfg)[1])
        else:
            before, after = home(drive, cfg)
            print(f"homed: arm was {before:+.2f} deg from home, now {after:+.2f}")
    elif args.cmd == "stop":
        drive.stop()

    if isinstance(drive.t, SimTransport):
        print("register writes:", ", ".join(f"0x{a:04X}<-{v}" for a, v in drive.t.writes))


def parse_register(text: str) -> int:
    try:
        return int(text, 0)
    except ValueError:
        return R.param_address(text)


def try_position(drive):
    try:
        return drive.read_position()
    except Exception as e:
        logging.warning("could not read the encoder position: %s", e)
        return None


def print_check(rows) -> int:
    """Print the settings table; return how many required settings differ."""
    bad = 0
    for name, expected, actual, what in rows:
        if actual is None:
            mark, want = "??? ", " (read failed)"
            bad += expected is not None
        elif expected is None:
            mark, want = "    ", ""
        elif actual in (expected if isinstance(expected, tuple) else (expected,)):
            mark, want = "ok  ", ""
        else:
            need = " or ".join(map(str, expected)) if isinstance(expected, tuple) else expected
            mark, want = "FIX ", f" (needs {need})"
            bad += 1
        print(f"{mark}{name:<6} 0x{R.param_address(name):04X} = {actual}{want}  {what}")
    return bad


if __name__ == "__main__":
    main()
