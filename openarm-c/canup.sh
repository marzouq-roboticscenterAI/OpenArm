#!/usr/bin/env bash
# canup.sh - bring a CAN interface up as classic CAN 2.0 with bus-off auto-recovery.
#   sudo ./canup.sh can0 1000000
set -euo pipefail
IFACE="${1:-can0}"; BITRATE="${2:-1000000}"
sudo ip link set "$IFACE" down 2>/dev/null || true
# restart-ms: auto-recover from a bus-off (e.g. a transient error burst) instead
# of latching the interface DOWN. No 'fd on' -> classic CAN 2.0 frames only.
sudo ip link set "$IFACE" type can bitrate "$BITRATE" restart-ms 100
sudo ip link set "$IFACE" txqueuelen 1000 2>/dev/null || true
sudo ip link set "$IFACE" up
ip -details -brief link show "$IFACE" || ip link show "$IFACE"
