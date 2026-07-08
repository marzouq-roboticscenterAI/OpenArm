#!/usr/bin/env bash
#
# calibrate.sh - dedicated calibration for the OpenArm-C arms.
#
#   * builds the app, brings up the CAN bus(es), then calibrates EVERY present
#     joint on each: hang-zero for J1/J2, gripper for J8, center for the rest.
#   * NEVER hangs: absent/dropped joints are probed and skipped; each joint sweep
#     is bounded by a wall-clock deadline inside the app.
#   * ONE Ctrl-C stops the whole run cleanly (the current joint's torque is cut,
#     the app aborts and disables the motor).
#   * writes/merges results into openarm-c/arm_calib.txt (calibrating one arm
#     does NOT wipe the other's calibration).
#
# Usage:
#   ./calibrate.sh                 # autodetect can*, calibrate all present joints
#   ./calibrate.sh can1            # only the LEFT arm  (can0 = right, can1 = left)
#   ./calibrate.sh can0 can1       # both
#
# NB: joints WILL move into their hardstops. Keep clear; Ctrl-C aborts + cuts torque.
#
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP="$HERE/openarm-c"
cd "$APP"

BITRATE="${CAN_BITRATE:-1000000}"

# Ctrl-C during build/bring-up just quits; once we exec the app, its own SIGINT
# handler aborts the sweep and disables the motor.
trap 'echo; echo "^C -- calibration aborted."; exit 130' INT TERM

echo "== build =="
make >/dev/null

# interfaces: explicit args, else every can*
IFACES=()
for a in "$@"; do case "$a" in can[0-9]*) IFACES+=("$a");; *) echo "ignoring arg: $a";; esac; done
if [ "${#IFACES[@]}" -eq 0 ]; then
  mapfile -t IFACES < <(ls /sys/class/net 2>/dev/null | grep -E '^can[0-9]+$' | sort)
fi
[ "${#IFACES[@]}" -ge 1 ] || { echo "No CAN interface present -- plug in the USB-CAN adapter."; exit 1; }
echo "== calibrating on: ${IFACES[*]} =="

# bring up any that are down
for c in "${IFACES[@]}"; do
  if ! ip link show "$c" 2>/dev/null | grep -q "state UP"; then
    echo "== bringing $c up @ ${BITRATE} (sudo) =="
    ./canup.sh "$c" "$BITRATE" || echo "!! could not bring $c up (continuing)"
  fi
done

echo "== calibration (joints WILL move; Ctrl-C aborts) =="
# exec so Ctrl-C is delivered straight to the app, which cuts torque + disables.
exec ./openarm --calibrate "${IFACES[@]}"
