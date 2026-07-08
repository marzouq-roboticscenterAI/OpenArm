#!/usr/bin/env python3
"""
Example driver for ds2c.py — the Python port of ds2c_test_move.sh.

This reproduces the shell script's behaviours and adds live data readout.
Run with no args for the connectivity check + slow up/down move.

    python3 example.py                 # check + tiny relative up/down move
    python3 example.py --check-only    # just the connectivity/fault check
    python3 example.py --status        # print live status/position/velocity, then exit
    python3 example.py --estop         # fire quick-stop + disable and exit
    python3 example.py --jog-up        # continuous jog UP, auto-stops after 20s

SAFETY: software stop only. Keep a hand on real power / a physical E-stop.
"""

import sys
import time

from ds2c import DS2CServo, DriveFault, SDOAbort, pps_to_rpm


CHANNEL = "can0"
NODE = 1                 # or None to auto-detect from heartbeat
TARGET_PULSES = 100      # ~3.6 deg at 10000 pulses/rev (positive = physically DOWN)
MAX_VELOCITY_PPS = 20000 # software speed limit = 120 rpm. Lower this to feel safer.


def main(argv):
    check_only = "--check-only" in argv
    status_only = "--status" in argv
    estop_only = "--estop" in argv
    jog_up = "--jog-up" in argv

    servo = DS2CServo(channel=CHANNEL, node=NODE or 1,
                      max_velocity_pps=MAX_VELOCITY_PPS)

    if estop_only:
        with servo:
            servo.emergency_stop()
        return 0

    with servo:  # auto quick-stop + disable on Ctrl+C / any error
        if NODE is None:
            servo.detect_node()

        # Pre-flight: read + decode statusword, clear a fault if latched.
        state = servo.check()

        # Belt-and-suspenders: also program the ceiling into the drive itself,
        # so it clamps speed internally even if something bypasses this library.
        # Optional objects — skip gracefully if this drive doesn't support them.
        try:
            servo.apply_drive_speed_limit()
        except SDOAbort:
            print("  (drive doesn't support 0x607F/0x6080 — software limit only)")

        if status_only:
            print(f"  state    : {state}")
            print(f"  position : {servo.read_position()} pulses "
                  f"({servo.read_position_rev():.4f} rev)")
            vel = servo.read_velocity()
            print(f"  velocity : {vel} pulses/s ({pps_to_rpm(vel):.1f} rpm)")
            print(f"  mode     : {servo.read_mode()}")
            return 0

        if check_only:
            print("Communication confirmed. Nothing was enabled or moved.")
            return 0

        if jog_up:
            # Velocity mode must be configured BEFORE enable (see ds2c docs).
            # 5000 pps = 30 rpm; well under the 20000 pps limit.
            servo.jog_up(velocity=5000, max_seconds=20)
            return 0

        # Default: enable, tiny relative move down and back up, reading position.
        servo.enable()
        servo.configure_position_mode(velocity=200, accel=1000, decel=1000)

        print(f"position before move: {servo.read_position()} pulses")
        servo.move_relative(TARGET_PULSES, label="down (positive)")
        print(f"position after down : {servo.read_position()} pulses")

        time.sleep(0.5)
        servo.move_relative(-TARGET_PULSES, label="up (negative)")
        print(f"position after up   : {servo.read_position()} pulses")

        print("Up/down move complete — drive left enabled, holding original position.")
        print("Run 'python3 example.py --estop' any time to quick-stop and de-energize.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except DriveFault as e:
        print(f"ABORT: {e}", file=sys.stderr)
        sys.exit(1)
