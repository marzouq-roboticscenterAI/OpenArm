#!/usr/bin/env bash
#
# run.sh - one command: build, bring the CAN bus(es) up, and launch OpenArm-C
# (web dashboard + auto Scheme-1 gamepad control).
#
# Buses:
#   can0 / can1  arms      @ 1 Mbit/s  (can1=LEFT, can0=RIGHT)
#   can2         Ranger    @ 500 kbit/s (AgileX Ranger Air rover)
#   can3         DS2-C     @ 1 Mbit/s  (lift servo, CANopen)
# Override interface names with RANGER_CAN / DS2C_CAN and bitrates with
# CAN_BITRATE / RANGER_BITRATE / DS2C_BITRATE.
#
# Ctrl-C cleanly stops everything: we `exec` the C app, whose SIGINT handler
# disables every motor, stops the rover, quick-stops the lift, and shuts down.
#
#   ./run.sh                      # autodetect arms, rover on can2, lift on can3
#   ./run.sh --port 9000 can0     # explicit port / arm interface
#   PORT=9000 ./run.sh            # via env
#
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

BITRATE="${CAN_BITRATE:-1000000}"           # arms
RANGER_IF="${RANGER_CAN:-can2}"; RANGER_BR="${RANGER_BITRATE:-500000}"
DS2C_IF="${DS2C_CAN:-can3}";     DS2C_BR="${DS2C_BITRATE:-1000000}"
export RANGER_CAN="$RANGER_IF" DS2C_CAN="$DS2C_IF"

# Before we exec the app, a Ctrl-C during build/bring-up should just quit.
trap 'echo; echo "^C -- aborted."; exit 130' INT TERM

# --- split args: CAN interfaces vs pass-through options (--port etc) ---
OPTS=(); IFACES=()
while [ $# -gt 0 ]; do
  case "$1" in
    can[0-9]*)      IFACES+=("$1") ;;
    --port|--web)   OPTS+=("$1" "$2"); shift ;;
    *)              OPTS+=("$1") ;;
  esac
  shift
done
# env PORT convenience
[ -n "${PORT:-}" ] && OPTS+=(--port "$PORT")

echo "== build =="
make >/dev/null

exists() { ip link show "$1" >/dev/null 2>&1; }
is_up()  { ip link show "$1" 2>/dev/null | grep -q "state UP"; }
bring_up() {  # bring_up IFACE BITRATE
  local c="$1" br="$2"
  exists "$c" || { echo "!! $c not present -- skipping"; return 1; }
  if ! is_up "$c"; then
    echo "== bringing $c up @ ${br} (sudo) =="
    ./canup.sh "$c" "$br" || echo "!! could not bring $c up (continuing)"
  fi
}

# --- arm interfaces: explicit args, else every can* EXCEPT the rover/lift buses ---
if [ "${#IFACES[@]}" -eq 0 ]; then
  mapfile -t IFACES < <(ls /sys/class/net 2>/dev/null | grep -E '^can[0-9]+$' \
                        | grep -vx -e "$RANGER_IF" -e "$DS2C_IF" | sort)
fi
[ "${#IFACES[@]}" -ge 1 ] || { echo "No arm CAN interface present -- plug in the USB-CAN adapter."; exit 1; }
echo "== arm interfaces: ${IFACES[*]}  |  rover: ${RANGER_IF}@${RANGER_BR}  lift: ${DS2C_IF}@${DS2C_BR} =="

# --- bring buses up at their respective bitrates ---
for c in "${IFACES[@]}"; do bring_up "$c" "$BITRATE"; done
bring_up "$RANGER_IF" "$RANGER_BR" || true
bring_up "$DS2C_IF"   "$DS2C_BR"   || true

echo "== launching OpenArm-C -- Ctrl-C stops (disables all motors, halts rover, quick-stops lift) =="
# exec: replace this shell so Ctrl-C is delivered straight to the C app, which
# cleans up (disables motors, stops rover/lift, joins threads) and exits.
exec ./openarm "${OPTS[@]}" "${IFACES[@]}"
