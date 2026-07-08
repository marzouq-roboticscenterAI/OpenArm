#!/usr/bin/env bash
#
# setup_can.sh — bring up the Ranger Air CAN link on a gs_usb / SocketCAN adapter.
#
# The Ranger Air chassis speaks CAN 2.0B at 500 kbit/s. This script loads the
# gs_usb driver, configures the interface bitrate, and brings it up. It is
# idempotent: re-running it re-applies the config cleanly.
#
# Usage:
#   ./setup_can.sh                 # configure can0 at 500k (default)
#   ./setup_can.sh can1            # a different interface
#   sudo ./setup_can.sh            # run directly as root
#
# The 'ip link' operations require root, so the script re-invokes itself with
# sudo if you are not already root.
#
set -euo pipefail

IFACE="${1:-can0}"
BITRATE=500000
TXQUEUELEN=1000

if [[ "${EUID}" -ne 0 ]]; then
    echo "[setup_can] elevating with sudo for interface configuration..."
    exec sudo -- "$0" "$IFACE"
fi

echo "[setup_can] loading gs_usb kernel module (if needed)..."
modprobe gs_usb || true

if ! ip link show "$IFACE" >/dev/null 2>&1; then
    echo "[setup_can] ERROR: interface '$IFACE' not found." >&2
    echo "            Is the USB-CAN adapter plugged in? Check: lsusb | grep -i can" >&2
    exit 1
fi

echo "[setup_can] bringing '$IFACE' down to reconfigure..."
ip link set "$IFACE" down 2>/dev/null || true

echo "[setup_can] setting '$IFACE' to ${BITRATE} bit/s..."
ip link set "$IFACE" type can bitrate "$BITRATE"

echo "[setup_can] setting tx queue length to ${TXQUEUELEN}..."
ip link set "$IFACE" txqueuelen "$TXQUEUELEN" || true

echo "[setup_can] bringing '$IFACE' up..."
ip link set "$IFACE" up

echo "[setup_can] done. Interface state:"
ip -details -statistics link show "$IFACE" | sed 's/^/    /'
echo
echo "[setup_can] '$IFACE' is up at ${BITRATE} bit/s. You can now run the driver."
