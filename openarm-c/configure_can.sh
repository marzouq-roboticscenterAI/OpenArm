#!/usr/bin/env bash
#
# configure_can.sh — one-shot CAN setup for the OpenArm-C rig (needs root; it
# re-invokes itself with sudo). It:
#   1. installs stable udev names (99-openarm-can.rules) so EVERY future boot
#      pins the adapters to fixed roles, and
#   2. renames the LIVE adapters to those roles right now (no reboot needed), and
#   3. brings each bus up at its correct bitrate:
#        can0, can1 = arms        @ 1 Mbit/s
#        can2       = Ranger rover @ 500 kbit/s
#        can3       = DS-2C servo  @ 1 Mbit/s
#
# Physical adapters are matched by a STABLE identity (unique serial, else USB
# port) — canN enumeration order is not stable and is exactly what this fixes.
#
#   ./configure_can.sh
#
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "${EUID:-$(id -u)}" -ne 0 ]; then
  echo "[configure_can] elevating with sudo..."
  exec sudo -- "$0" "$@"
fi

RULES_SRC="$HERE/99-openarm-can.rules"
RULES_DST="/etc/udev/rules.d/99-openarm-can.rules"

# ---- identity -> role -------------------------------------------------------
# Ranger's candleLight adapter exposes a real, unique serial (best match).
RANGER_SERIAL="004100354759530820353131"
# The PEAK/XCAN adapters share a bogus serial, so match them by USB port.
declare -A PORT_NAME=(
  ["3-2.2.3:1.0"]="can3"      # DS-2C servo (XCAN-USB)
  ["3-2.2.4.3:1.0"]="can0"    # arm A   (swap can0/can1 here if left/right
  ["3-2.2.4.4:1.0"]="can1"    # arm B    is reversed)
)
declare -A NAME_BR=( [can0]=1000000 [can1]=1000000 [can2]=500000 [can3]=1000000 )

iface_serial(){ udevadm info -q property -p "/sys/class/net/$1" 2>/dev/null | sed -n 's/^ID_SERIAL_SHORT=//p'; }
iface_port(){ basename "$(readlink -f "/sys/class/net/$1/device" 2>/dev/null)" 2>/dev/null; }
target_name(){                                   # echo target canN, or "" if unknown
  local ser port
  ser="$(iface_serial "$1")"
  [ "$ser" = "$RANGER_SERIAL" ] && { echo can2; return; }
  port="$(iface_port "$1")"
  echo "${PORT_NAME[$port]:-}"
}

# ---- 1. install persistent udev rule ----------------------------------------
if [ -f "$RULES_SRC" ]; then
  cp "$RULES_SRC" "$RULES_DST"
  udevadm control --reload 2>/dev/null || true
  echo "[configure_can] installed $RULES_DST (fixes names on every future boot)"
else
  echo "[configure_can] WARN: $RULES_SRC not found — skipping persistent rule"
fi

# ---- 2. plan live renames ---------------------------------------------------
mapfile -t IFACES < <(ls /sys/class/net 2>/dev/null | grep -E '^can[0-9]+$' | sort)
[ "${#IFACES[@]}" -ge 1 ] || { echo "[configure_can] no CAN interfaces present — plug in the adapters."; exit 1; }

declare -A CUR_TO_TGT
for ifc in "${IFACES[@]}"; do
  tgt="$(target_name "$ifc")"
  if [ -n "$tgt" ]; then
    CUR_TO_TGT["$ifc"]="$tgt"
    echo "[configure_can] $ifc (serial=$(iface_serial "$ifc") port=$(iface_port "$ifc")) -> $tgt"
  else
    echo "[configure_can] $ifc: UNRECOGNIZED adapter — leaving as-is"
  fi
done

# ---- 3. rename live via temp names (avoids canN name collisions on swap) -----
i=0; declare -A TMP_TO_TGT
for cur in "${!CUR_TO_TGT[@]}"; do
  tmp="oacan$i"; i=$((i+1))
  ip link set "$cur" down 2>/dev/null || true
  if ip link set "$cur" name "$tmp" 2>/dev/null; then
    TMP_TO_TGT["$tmp"]="${CUR_TO_TGT[$cur]}"
  else
    echo "[configure_can] rename $cur -> $tmp failed"
  fi
done
for tmp in "${!TMP_TO_TGT[@]}"; do
  tgt="${TMP_TO_TGT[$tmp]}"
  ip link set "$tmp" name "$tgt" 2>/dev/null || echo "[configure_can] rename $tmp -> $tgt failed"
done

# ---- 4. bring each bus up at its role bitrate -------------------------------
# Retry the config+up: candleLight/gs_usb adapters often reject the first 'up'
# right after a rename (USB re-init race), so we try a few times before giving up.
for tgt in can0 can1 can2 can3; do
  ip link show "$tgt" >/dev/null 2>&1 || continue
  br="${NAME_BR[$tgt]}"
  up_ok=0
  for attempt in 1 2 3 4; do
    ip link set "$tgt" down 2>/dev/null || true
    # restart-ms auto-recovers from bus-off, but candleLight/gs_usb adapters
    # reject it ("Device doesn't support restart from Bus Off") -> fall back to
    # a plain config. No 'fd on' -> classic CAN 2.0 frames.
    if ! ip link set "$tgt" type can bitrate "$br" restart-ms 100 2>/dev/null; then
      ip link set "$tgt" type can bitrate "$br" 2>/dev/null || { sleep 0.4; continue; }
    fi
    ip link set "$tgt" txqueuelen 1000 2>/dev/null || true
    if ip link set "$tgt" up 2>/dev/null; then up_ok=1; break; fi
    sleep 0.4
  done
  if [ "$up_ok" = 1 ]; then echo "[configure_can] $tgt UP @ ${br}"
  else echo "[configure_can] !! $tgt would not come up @ ${br} after retries — try: ./reset_can.sh $tgt"; fi
done

echo "[configure_can] done. Current CAN interfaces:"
ip -br link show type can
echo "[configure_can] now run:  ./run.sh"
