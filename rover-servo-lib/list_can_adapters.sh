#!/usr/bin/env bash
#
# list_can_adapters.sh — show each USB-CAN adapter's STABLE identity.
#
# Interface names (can0/can1/...) are handed out in USB enumeration order and
# are NOT stable across reboots or re-plugs. To pin a device to a fixed name you
# need something that doesn't change: the adapter's USB serial number, or the
# physical USB port it's in. This script prints both for every CAN interface so
# you can fill in 99-roverservo-can.rules (the udev template) and stop guessing.
#
# Usage:  ./list_can_adapters.sh
#
set -uo pipefail

ifaces=$(ip -br link show type can 2>/dev/null | awk '{print $1}')
if [[ -z "$ifaces" ]]; then
  echo "No CAN interfaces found."
  echo "  - Plug in the USB-CAN adapters."
  echo "  - If needed: sudo modprobe gs_usb    (check: lsusb | grep -i can)"
  exit 1
fi

echo "CAN adapters currently attached:"
echo
for iface in $ifaces; do
  syspath="/sys/class/net/$iface"
  echo "=== $iface ==="

  # Human-readable identity (vendor/model/serial/port).
  udevadm info -q property -p "$syspath" 2>/dev/null \
    | grep -E '^(ID_VENDOR|ID_VENDOR_ID|ID_MODEL|ID_MODEL_ID|ID_SERIAL_SHORT|ID_PATH)=' \
    | sed 's/^/    /'

  # The exact tokens you paste into a udev rule:
  serial=$(udevadm info -a -p "$syspath" 2>/dev/null | grep -m1 'ATTRS{serial}==')
  kernels=$(udevadm info -a -p "$syspath" 2>/dev/null | grep -m1 'KERNELS=="[0-9]')
  echo "    --- udev match candidates ---"
  echo "    by serial : ${serial:-  (no serial exposed — use the port match below)}" | sed 's/  */ /2'
  echo "    by port   : ${kernels:-  (unavailable)}" | sed 's/  */ /2'
  echo
done

cat <<'EOF'
Next steps:
  1. Pick a match line per adapter (prefer 'by serial'; use 'by port' if two
     adapters share a serial — some cheap gs_usb clones do).
  2. Edit 99-roverservo-can.rules with those matches and a NAME per device.
  3. sudo cp 99-roverservo-can.rules /etc/udev/rules.d/
     sudo udevadm control --reload && sudo udevadm trigger
  4. Re-plug (or reboot). `ip -br link show type can` should now show your names.
EOF
