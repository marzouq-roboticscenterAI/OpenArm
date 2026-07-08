#!/usr/bin/env python3
"""monitor.py — read-only listener for the Ranger Air CAN bus.

Sends nothing. Just decodes and prints chassis feedback so you can confirm the
link is alive and the robot is healthy *before* commanding any motion.

    python3 monitor.py            # listen on can0
    python3 monitor.py can1       # a different interface

Ctrl-C to quit.
"""

import sys
import time

from rangerair import RangerAir


def main() -> int:
    iface = sys.argv[1] if len(sys.argv) > 1 else "can0"
    print(f"Listening on {iface} (read-only, no commands sent). Ctrl-C to stop.\n")

    # auto_start spins up the RX thread; we never call enable(), so nothing is
    # ever transmitted to the chassis.
    with RangerAir(iface) as bot:
        if not bot.wait_for_feedback(timeout=3.0):
            print("No feedback received in 3 s.")
            print("  - Is the chassis powered on?")
            print("  - Is CAN wiring correct and the bus up at 500 kbit/s?")
            return 1

        try:
            while True:
                s = bot.state
                sysfb = s.system
                if sysfb:
                    faults = s.faults
                    fault_txt = ", ".join(faults) if faults else "none"
                    mode = s.motion_mode.name if s.motion_mode else "?"
                    print(
                        f"[{time.strftime('%H:%M:%S')}] "
                        f"sys={'OK' if sysfb.normal else 'ERROR'} "
                        f"ctrl={sysfb.mode.name:<13} "
                        f"mode={mode:<9} "
                        f"batt={sysfb.voltage:4.1f}V "
                        f"faults=[{fault_txt}]"
                    )
                    if s.motion:
                        m = s.motion
                        print(
                            f"           motion: v={m.linear_mps:+.3f} m/s  "
                            f"w={m.angular_rps:+.3f} rad/s  "
                            f"steer={m.steer_rad:+.3f} rad"
                        )
                else:
                    print("waiting for system frame (0x211)...")
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nstopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
