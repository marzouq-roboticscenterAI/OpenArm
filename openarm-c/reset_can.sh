#!/usr/bin/env bash
#
# reset_can.sh - recover a wedged / bus-off / stuck-DOWN SocketCAN adapter.
#
# Brings each interface fully down, lets the controller settle, then back up at
# its correct bitrate with bus-off auto-recovery (via canup.sh). Use this when a
# bus stops passing frames after an error burst, a hot-unplug, or ENOBUFS -- it's
# the software equivalent of power-cycling the adapter.
#
# Bitrate per interface matches run.sh:
#   can2 (Ranger)  -> 500 kbit/s      everything else -> 1 Mbit/s
# Override names with RANGER_CAN / DS2C_CAN and rates with
# CAN_BITRATE / RANGER_BITRATE / DS2C_BITRATE.
#
#   ./reset_can.sh                # reset every can* present
#   ./reset_can.sh can0 can2      # reset just these
#
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

BITRATE="${CAN_BITRATE:-1000000}"           # arms / default
RANGER_IF="${RANGER_CAN:-can2}"; RANGER_BR="${RANGER_BITRATE:-500000}"
DS2C_IF="${DS2C_CAN:-can3}";     DS2C_BR="${DS2C_BITRATE:-1000000}"

# pick the right bitrate for a given interface name
rate_for() {
  case "$1" in
    "$RANGER_IF") echo "$RANGER_BR" ;;
    "$DS2C_IF")   echo "$DS2C_BR" ;;
    *)            echo "$BITRATE" ;;
  esac
}

# interfaces: explicit args, else every can* the kernel exposes
IFACES=("$@")
if [ "${#IFACES[@]}" -eq 0 ]; then
  mapfile -t IFACES < <(ls /sys/class/net 2>/dev/null | grep -E '^can[0-9]+$' | sort)
fi
[ "${#IFACES[@]}" -ge 1 ] || { echo "No CAN interface present -- plug in the USB-CAN adapter."; exit 1; }

for c in "${IFACES[@]}"; do
  if ! ip link show "$c" >/dev/null 2>&1; then
    echo "!! $c not present -- skipping"
    continue
  fi
  br="$(rate_for "$c")"
  echo "== resetting $c @ ${br} (sudo) =="
  # Force it down first so a bus-off / stuck state is cleared, then settle briefly
  # before re-configuring (some USB-CAN bridges need a moment to release).
  sudo ip link set "$c" down 2>/dev/null || true
  sleep 0.3
  ./canup.sh "$c" "$br" || echo "!! could not bring $c back up"
done

echo "== done. Current state: =="
for c in "${IFACES[@]}"; do
  ip link show "$c" >/dev/null 2>&1 && { ip -details -brief link show "$c" 2>/dev/null || ip link show "$c"; }
done
