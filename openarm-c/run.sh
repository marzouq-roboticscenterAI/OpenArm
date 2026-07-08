#!/usr/bin/env bash
#
# run.sh - one command: build, bring the CAN bus(es) up, and launch OpenArm-C
# (web dashboard + auto Scheme-1 gamepad control).
#
# Ctrl-C cleanly stops everything: we `exec` the C app, whose SIGINT handler
# disables every motor and shuts down the control thread + HTTP server before
# exiting. A trap covers the setup phase (before exec) too.
#
#   ./run.sh                      # autodetect can*, dashboard on :8080
#   ./run.sh --port 9000 can0     # explicit port / interface
#   PORT=9000 ./run.sh            # via env
#
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

BITRATE="${CAN_BITRATE:-1000000}"

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

# --- interfaces: explicit or autodetect every can* ---
if [ "${#IFACES[@]}" -eq 0 ]; then
  mapfile -t IFACES < <(ls /sys/class/net 2>/dev/null | grep -E '^can[0-9]+$' | sort)
fi
[ "${#IFACES[@]}" -ge 1 ] || { echo "No CAN interface present -- plug in the USB-CAN adapter."; exit 1; }
echo "== CAN interfaces: ${IFACES[*]} =="

# --- bring up any that are down (canup.sh sudo's internally) ---
for c in "${IFACES[@]}"; do
  if ! ip link show "$c" 2>/dev/null | grep -q "state UP"; then
    echo "== bringing $c up @ ${BITRATE} (sudo) =="
    ./canup.sh "$c" "$BITRATE" || echo "!! could not bring $c up (continuing)"
  fi
done

echo "== launching OpenArm-C -- Ctrl-C stops (disables all motors) =="
# exec: replace this shell so Ctrl-C is delivered straight to the C app, which
# cleans up (disables motors, joins threads) and exits.
exec ./openarm "${OPTS[@]}" "${IFACES[@]}"
