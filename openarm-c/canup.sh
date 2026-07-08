#!/usr/bin/env bash
# canup.sh - bring a CAN interface up as classic CAN 2.0.
#   sudo ./canup.sh can0 1000000
set -euo pipefail
IFACE="${1:-can0}"; BITRATE="${2:-1000000}"
sudo ip link set "$IFACE" down 2>/dev/null || true
# restart-ms: auto-recover from a bus-off (transient error burst) instead of
# latching DOWN. Some adapters (candleLight/gs_usb) DON'T support it and reject
# the whole command ("Device doesn't support restart from Bus Off") -- so fall
# back to a plain bring-up in that case. No 'fd on' -> classic CAN 2.0 frames.
if ! sudo ip link set "$IFACE" type can bitrate "$BITRATE" restart-ms 100 2>/dev/null; then
  echo "canup: $IFACE has no bus-off restart support; configuring without restart-ms"
  sudo ip link set "$IFACE" type can bitrate "$BITRATE"
fi
sudo ip link set "$IFACE" txqueuelen 1000 2>/dev/null || true
sudo ip link set "$IFACE" up
ip -details -brief link show "$IFACE" || ip link show "$IFACE"
