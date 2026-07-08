#!/usr/bin/env bash
#
# setup_can.sh — bring up BOTH CAN adapters for the rover + servo demo.
#
# Two USB-CAN adapters are plugged into this computer:
#   * the Ranger Air rover  -> 500 kbit/s (CAN 2.0B)
#   * the DS2-C servo lift  -> 1 Mbit/s   (CANopen)
#
# Linux does not guarantee which adapter becomes can0 vs can1, so this script
# does NOT assume a fixed mapping. It brings every CAN interface up so that
# `python -m roverservo` (or roverservo.interfaces.identify) can probe each one
# and work out which is which by bitrate + traffic.
#
# Strategy: we don't know an interface's role yet, so we bring each one up at a
# neutral bitrate. `roverservo` will reconfigure the correct bitrate per role
# during identification. If you already KNOW the mapping, pass roles explicitly:
#
#   ./setup_can.sh                      # bring up all can* interfaces (auto)
#   ./setup_can.sh rover=can0 servo=can1   # set 500k on can0, 1M on can1
#
# Requires sudo (re-invokes itself with sudo if needed).

set -uo pipefail

ROVER_BITRATE=500000
SERVO_BITRATE=1000000
DEFAULT_BITRATE=500000     # neutral guess for auto mode; roverservo fixes it later
TXQUEUELEN=1000

if [[ "${EUID}" -ne 0 ]]; then
  echo "[setup_can] elevating with sudo..."
  exec sudo -- "$0" "$@"
fi

modprobe gs_usb 2>/dev/null || true

bring_up() {
  local iface="$1" bitrate="$2"
  if ! ip link show "$iface" >/dev/null 2>&1; then
    echo "[setup_can] ERROR: interface '$iface' not found (adapter unplugged?)." >&2
    return 1
  fi
  echo "[setup_can] configuring $iface @ ${bitrate} bit/s..."
  ip link set "$iface" down 2>/dev/null || true
  ip link set "$iface" type can bitrate "$bitrate" || { echo "  failed to set bitrate on $iface" >&2; return 1; }
  ip link set "$iface" txqueuelen "$TXQUEUELEN" 2>/dev/null || true
  ip link set "$iface" up || { echo "  failed to bring up $iface" >&2; return 1; }
  echo "[setup_can]   $iface is UP."
}

# explicit role args?  rover=canX servo=canY
ROVER_IF=""; SERVO_IF=""
for arg in "$@"; do
  case "$arg" in
    rover=*) ROVER_IF="${arg#rover=}" ;;
    servo=*) SERVO_IF="${arg#servo=}" ;;
    *) echo "[setup_can] ignoring unknown arg: $arg" >&2 ;;
  esac
done

if [[ -n "$ROVER_IF" || -n "$SERVO_IF" ]]; then
  [[ -n "$ROVER_IF" ]] && bring_up "$ROVER_IF" "$ROVER_BITRATE"
  [[ -n "$SERVO_IF" ]] && bring_up "$SERVO_IF" "$SERVO_BITRATE"
else
  mapfile -t IFACES < <(ip -br link show type can 2>/dev/null | awk '{print $1}')
  if [[ ${#IFACES[@]} -eq 0 ]]; then
    echo "[setup_can] ERROR: no CAN interfaces found. Plug in the USB-CAN adapters." >&2
    echo "            Check: lsusb | grep -i can" >&2
    exit 1
  fi
  echo "[setup_can] found interfaces: ${IFACES[*]}"
  for iface in "${IFACES[@]}"; do
    bring_up "$iface" "$DEFAULT_BITRATE"
  done
  echo
  echo "[setup_can] All interfaces up at a neutral ${DEFAULT_BITRATE} bit/s."
  echo "[setup_can] Now run:  python3 -m roverservo"
  echo "[setup_can] (it probes each adapter and sets the right bitrate per device)."
fi

echo
echo "[setup_can] Current CAN interfaces:"
ip -br link show type can | sed 's/^/    /'
