"""``python -m roverservo`` — connect to the rig and serve the control website.

Examples::

    python -m roverservo                         # auto-detect, serve :8080
                                                 #   (starts with whatever is
                                                 #    connected — rover-only or
                                                 #    servo-only is fine)
    python -m roverservo --port 9000
    python -m roverservo --rover-can can0 --servo-can can1   # force the mapping
    python -m roverservo --no-configure          # don't sudo-set bitrates
    python -m roverservo --require-both          # fail unless BOTH are found
    python -m roverservo --identify-only         # just print the CAN mapping, exit
"""

from __future__ import annotations

import argparse
import sys

from . import _deps
from .interfaces import InterfaceError, identify
from .robot import Rover
from .server import serve


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(prog="roverservo", description=__doc__)
    ap.add_argument("--host", default="0.0.0.0", help="bind address (default 0.0.0.0)")
    ap.add_argument("--port", type=int, default=8080, help="TCP port (default 8080)")
    ap.add_argument("--rover-can", default=None, help="force the rover's CAN interface")
    ap.add_argument("--servo-can", default=None, help="force the servo's CAN interface")
    ap.add_argument("--servo-node", type=int, default=1, help="DS2-C CANopen node id")
    ap.add_argument("--max-vel-pps", type=int, default=100_000,
                    help="servo software speed limit in pulses/s (100000 = 600 rpm)")
    ap.add_argument("--no-configure", action="store_true",
                    help="do not run sudo ip to set bitrates; use interfaces as-is")
    ap.add_argument("--require-both", action="store_true",
                    help="refuse to start unless BOTH devices are found "
                         "(default: start with whatever is connected)")
    ap.add_argument("--one-device", action="store_true",
                    help=argparse.SUPPRESS)   # deprecated: now the default; kept as a no-op
    ap.add_argument("--identify-only", action="store_true",
                    help="print which interface is which, then exit")
    args = ap.parse_args(argv)

    missing = _deps.missing_siblings()
    if missing:
        print(f"ERROR: expected sibling libraries not found: {', '.join(missing)}.\n"
              f"       roverservo must live next to them inside RoverServoLib/.",
              file=sys.stderr)
        return 2

    if args.identify_only:
        try:
            print(identify(configure=not args.no_configure))
        except InterfaceError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 1
        return 0

    rig = Rover(
        rover_iface=args.rover_can,
        servo_iface=args.servo_can,
        servo_node=args.servo_node,
        max_velocity_pps=args.max_vel_pps,
    )
    try:
        rig.connect(require_both=args.require_both, configure=not args.no_configure)
    except InterfaceError as exc:
        print(f"\nERROR: {exc}\n", file=sys.stderr)
        return 1

    # Summarise what actually came up so a missing device is obvious.
    rover_s = rig.rover_iface if rig.rover else "NOT CONNECTED"
    servo_s = rig.servo_iface if rig.servo else "NOT CONNECTED"
    print(f"\n  devices:  rover -> {rover_s}   servo -> {servo_s}")
    if rig.rover is None or rig.servo is None:
        print("  (running with a missing device — its panel will show as offline; "
              "pass --require-both to make this fatal instead.)")

    try:
        serve(rig, host=args.host, port=args.port)
    finally:
        rig.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
