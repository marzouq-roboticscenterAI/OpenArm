#!/usr/bin/env python3
"""demo_move.py — prove the Ranger Air moves under CAN command control.

This performs a SMALL, SHORT, controlled forward nudge and then stops. It is
deliberately conservative so it is safe to run as a first motion test.

    python3 demo_move.py                 # 0.1 m/s forward for 1.5 s on can0
    python3 demo_move.py --speed 0.15 --duration 2.0
    python3 demo_move.py --iface can1

SAFETY CHECKLIST — read before running:
  * Put the robot on blocks/stands with wheels OFF the ground for the very
    first test, OR ensure at least ~2 m of clear space ahead.
  * On the FS remote, set SWB to the TOP position (command-control mode).
    The remote always has priority; if SWB is in remote mode, CAN motion
    commands are ignored.
  * Release the physical e-stop (twist clockwise) before running.
  * Keep a hand on the e-stop. Ctrl-C also stops and drops to standby.
"""

import argparse
import sys
import time

from rangerair import RangerAir, MotionMode


def main() -> int:
    ap = argparse.ArgumentParser(description="Ranger Air proof-of-movement nudge")
    ap.add_argument("--iface", default="can0", help="CAN interface (default can0)")
    ap.add_argument("--speed", type=float, default=0.1,
                    help="forward speed in m/s (default 0.1)")
    ap.add_argument("--duration", type=float, default=1.5,
                    help="seconds to drive (default 1.5)")
    ap.add_argument("--yes", action="store_true",
                    help="skip the interactive confirmation prompt")
    args = ap.parse_args()

    print(__doc__)
    if not args.yes:
        resp = input(f"Drive forward {args.speed} m/s for {args.duration}s on "
                     f"{args.iface}? [y/N] ").strip().lower()
        if resp != "y":
            print("aborted.")
            return 1

    with RangerAir(args.iface) as bot:
        print(f"\nConnecting on {args.iface}...")
        if not bot.wait_for_feedback(timeout=3.0):
            print("ERROR: no feedback from chassis. Is it powered and the bus up?")
            return 1

        s = bot.state.system
        print(f"Link OK. battery={s.voltage:.1f}V  ctrl_mode={s.mode.name}  "
              f"system={'OK' if s.normal else 'ERROR'}")
        faults = bot.state.faults
        if faults:
            print(f"Active faults: {', '.join(faults)} -> sending clear_errors()")

        # 1) enter CAN command mode, 2) clear any faults / release e-stop error
        bot.enable()
        bot.clear_errors()
        time.sleep(0.2)

        # 3) ensure Ackermann (straight-line) kinematics
        bot.set_mode(MotionMode.ACKERMANN)

        # confirm the chassis actually accepted command mode
        s = bot.state.system
        if s and s.mode.name != "CAN_COMMAND":
            print(f"WARNING: chassis reports ctrl_mode={s.mode.name}, not CAN_COMMAND.")
            print("         Check that remote SWB is in the TOP (command) position.")

        # 4) drive forward, holding via the 50 Hz heartbeat, then stop
        print(f"\n>>> driving forward at {args.speed} m/s for {args.duration}s ...")
        bot.drive(linear=args.speed)
        t_end = time.monotonic() + args.duration
        while time.monotonic() < t_end:
            m = bot.state.motion
            ws = bot.state.wheel_speeds
            fb_v = f"{m.linear_mps:+.3f}" if m else "  ?  "
            fb_w = (" ".join(f"{w:+.2f}" for w in ws)) if ws else "?"
            print(f"    reported v={fb_v} m/s  wheels=[{fb_w}]")
            time.sleep(0.2)

        print(">>> stopping.")
        bot.stop()
        time.sleep(0.3)

    print("\nDone. Chassis returned to standby.")
    print("If the wheels turned, CAN command control is proven end-to-end. ✅")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
