#!/usr/bin/env bash
#
# DS2-C servo (CANopen/CiA402) — connectivity check, then a tiny, slow
# relative move up and back down, with an emergency-stop path wired to
# Ctrl+C/any error.
#
# IMPORTANT: this is a SOFTWARE stop only. If the CAN adapter is unplugged or
# the drive hangs, this script cannot stop the motor. Keep a hand on real
# mains/DC power (or a physical E-stop) the whole time you run this.
#
# Protocol values are taken from the vendor's DS2-C CAN message spec + EDS:
#   - CANopen node = 1 (confirmed via DIP switch SW1=ON,SW2=OFF,SW3=OFF)
#   - bus rate = 1,000,000 bps (confirmed by live bitrate scan against this unit —
#     the manual's documented factory default of 500 kbps does NOT match this drive)
#   - default gear ratio = 10000 pulses/rev
#   - controlword 6/7/15 = CiA402 enable sequence
#   - controlword 0x5F   = start relative positioning
#   - controlword 0x0B   = quick stop ("急停" in the vendor test sheet)
#
# NOTE ON DIRECTION: a positive TARGET_PULSES (the move labeled "up" below)
# was confirmed on hardware to drive the carriage DOWN, not up — polarity
# (607Eh) is inverted relative to the "up"/"down" labels. The labels are kept
# as-is for now; just remember they're physically backwards.
#
# Usage:
#   ./ds2c_test_move.sh              run the connectivity check + slow up/down move
#   ./ds2c_test_move.sh --check-only run just the connectivity/statusword check, then exit
#   ./ds2c_test_move.sh --dry-run    print every CAN frame instead of sending it
#   ./ds2c_test_move.sh --estop      just fire the stop/disable frames and exit
#   ./ds2c_test_move.sh --jog-up     continuous profile-velocity move up, runs until
#                                    Ctrl+C/error (quick-stop) or JOG_MAX_SECONDS elapses
#
# Tunables (env vars):
#   IFACE=can0  BITRATE=1000000  NODE=<auto-detected, or force a number>
#   TARGET_PULSES=100   VELOCITY_PPS=200   ACCEL=1000   DECEL=1000
#   MOVE_TIMEOUT=10
#   JOG_VELOCITY_PPS=50000   JOG_ACCEL=50000   JOG_DECEL=50000   JOG_MAX_SECONDS=20
#   (only used with --jog-up; JOG_ACCEL/DECEL are separate from ACCEL/DECEL above so
#   jog mode doesn't silently inherit the tiny default meant for the fixed-distance move)

set -uo pipefail

IFACE="${IFACE:-can0}"
BITRATE="${BITRATE:-1000000}"
NODE="${NODE:-}"
TARGET_PULSES="${TARGET_PULSES:-100}"     # ~3.6 degrees at 10000 pulses/rev
VELOCITY_PPS="${VELOCITY_PPS:-200}"       # ~1.2 rpm
ACCEL="${ACCEL:-1000}"
DECEL="${DECEL:-1000}"
MOVE_TIMEOUT="${MOVE_TIMEOUT:-10}"
JOG_VELOCITY_PPS="${JOG_VELOCITY_PPS:-50000}"   # ~25mm/s at 2000 pulses/mm
JOG_ACCEL="${JOG_ACCEL:-50000}"                 # ramp to full jog speed in ~1s
JOG_DECEL="${JOG_DECEL:-50000}"
JOG_MAX_SECONDS="${JOG_MAX_SECONDS:-20}"        # auto-stop cap even if Ctrl+C is missed
DRY_RUN=0
ESTOP_ONLY=0
CHECK_ONLY=0
JOG_UP=0

for arg in "$@"; do
  case "$arg" in
    --dry-run) DRY_RUN=1 ;;
    --estop) ESTOP_ONLY=1 ;;
    --check-only) CHECK_ONLY=1 ;;
    --jog-up) JOG_UP=1 ;;
    *) echo "Unknown argument: $arg" >&2; exit 2 ;;
  esac
done

log() { printf '[%(%H:%M:%S)T] %s\n' -1 "$*" >&2; }
die() { log "ABORT: $*"; exit 1; }

need() { command -v "$1" >/dev/null 2>&1 || die "missing required tool: $1"; }
need cansend
need candump
need ip

raw_send() {
  local frame="$1"
  if [[ $DRY_RUN -eq 1 ]]; then
    log "DRY-RUN would send: cansend $IFACE $frame"
  else
    cansend "$IFACE" "$frame"
  fi
}

# --- SDO frame builders (expedited transfer, little-endian) -----------------
sdo_write() {
  local index=$1 sub=$2 size=$3 value=$4
  local idx_lo idx_hi cmd data txid
  idx_lo=$(printf '%02X' $((index & 0xFF)))
  idx_hi=$(printf '%02X' $(((index >> 8) & 0xFF)))
  case $size in
    1) cmd=2F; data=$(printf '%02X000000' $((value & 0xFF))) ;;
    2) cmd=2B; data=$(printf '%02X%02X0000' $((value & 0xFF)) $(((value >> 8) & 0xFF))) ;;
    4) cmd=23; data=$(printf '%02X%02X%02X%02X' $((value & 0xFF)) $(((value >> 8) & 0xFF)) $(((value >> 16) & 0xFF)) $(((value >> 24) & 0xFF))) ;;
    *) die "sdo_write: bad size $size" ;;
  esac
  txid=$(printf '%03X' $((0x600 + NODE)))
  raw_send "${txid}#${cmd}${idx_lo}${idx_hi}$(printf '%02X' "$sub")${data}"
}

# Reads a 16-bit SDO value; echoes decimal value on stdout, returns 1 on timeout
sdo_read_u16() {
  local index=$1 sub=$2
  local idx_lo idx_hi rxid txid tmpfile dumper line b5 b6
  idx_lo=$(printf '%02X' $((index & 0xFF)))
  idx_hi=$(printf '%02X' $(((index >> 8) & 0xFF)))
  rxid=$(printf '%03X' $((0x580 + NODE)))
  txid=$(printf '%03X' $((0x600 + NODE)))
  tmpfile=$(mktemp)
  ( timeout 1 candump -n 1 "${IFACE},${rxid}:7FF" >"$tmpfile" 2>/dev/null ) &
  dumper=$!
  sleep 0.1
  raw_send "${txid}#40${idx_lo}${idx_hi}$(printf '%02X' "$sub")00000000"
  wait "$dumper" 2>/dev/null
  line=$(cat "$tmpfile"); rm -f "$tmpfile"
  if [[ $DRY_RUN -eq 1 ]]; then
    log "DRY-RUN would read index 0x$(printf '%04X' "$index") sub $sub — simulating statusword 0x0237 (operation enabled, no fault)"
    echo $((0x0237))
    return 0
  fi
  [[ -z "$line" ]] && return 1
  b5=$(echo "$line" | awk '{print $8}')   # data byte 5 (low)
  b6=$(echo "$line" | awk '{print $9}')   # data byte 6 (high)
  [[ -z "$b5" || -z "$b6" ]] && return 1
  echo $((16#${b6}${b5}))
}

nmt_enter_operational() {
  local node_hex
  node_hex=$(printf '%02X' "$NODE")
  raw_send "000#01${node_hex}"
}

# --- emergency stop -----------------------------------------------------
ESTOPPED=0
emergency_stop() {
  [[ $ESTOPPED -eq 1 ]] && return
  ESTOPPED=1
  log "!!! EMERGENCY STOP !!!"
  if [[ -n "${NODE:-}" ]]; then
    local txid
    txid=$(printf '%03X' $((0x600 + NODE)))
    raw_send "${txid}#2B4060000B000000"   # controlword=0x000B quick stop
    sleep 0.05
    raw_send "${txid}#2B4060000000000000" 2>/dev/null || raw_send "${txid}#2B4060000000000000"
  fi
  log "Quick-stop + disable-voltage frames sent. Verify the motor has actually stopped."
}
trap 'emergency_stop; exit 1' INT TERM ERR

if [[ $ESTOP_ONLY -eq 1 ]]; then
  [[ -z "$NODE" ]] && NODE=1
  emergency_stop
  exit 0
fi

# --- bring up the CAN interface -----------------------------------------
iface_up=0
current_rate=$(ip -details link show "$IFACE" 2>/dev/null | grep -oP 'bitrate \K[0-9]+' || true)
if [[ "$current_rate" == "$BITRATE" ]] && ip link show "$IFACE" 2>/dev/null | grep -q "state UP"; then
  iface_up=1
fi

if [[ $iface_up -eq 0 ]]; then
  if [[ $DRY_RUN -eq 1 ]]; then
    log "DRY-RUN would bring up $IFACE at ${BITRATE}bps (needs sudo) — skipping real setup."
  else
    log "$IFACE is down — bringing it up at ${BITRATE}bps (needs sudo)."
    sudo ip link set "$IFACE" down 2>/dev/null || true
    sudo ip link set "$IFACE" type can bitrate "$BITRATE" || die "failed to configure $IFACE"
    sudo ip link set "$IFACE" up || die "failed to bring up $IFACE"
    iface_up=1
  fi
fi
[[ $iface_up -eq 1 ]] && log "$IFACE is up at ${BITRATE}bps."

# --- detect node id from heartbeat/bootup if not forced -----------------
if [[ -z "$NODE" ]]; then
  if [[ $DRY_RUN -eq 1 && $iface_up -eq 0 ]]; then
    NODE=1
    log "DRY-RUN, interface not actually up — skipping heartbeat listen, assuming node $NODE."
  else
    log "Listening 3s for a heartbeat/bootup frame (700h-77Fh) to detect the node ID..."
    hb_line=$(timeout 3 candump "${IFACE}" 2>/dev/null | grep -E ' (70[0-9A-F]|7[0-6][0-9A-F]) ' | head -1 || true)
    if [[ -n "$hb_line" ]]; then
      hb_id=$(echo "$hb_line" | awk '{print $3}')
      NODE=$((16#${hb_id} - 16#700))
      log "Detected node ID $NODE (heartbeat/bootup on ${hb_id}h)."
    else
      NODE=1
      log "No heartbeat seen — defaulting to node 1 (factory default). Set NODE=<n> to override."
    fi
  fi
fi

log "Reading statusword (6041h) before touching anything..."
status=$(sdo_read_u16 0x6041 0x00) || die "no SDO response from node $NODE — check wiring/node id/bitrate before proceeding"
log "Statusword = 0x$(printf '%04X' "$status")"
if (( status & 0x0008 )); then
  log "Fault bit is set. Sending fault-reset (controlword bit7) once..."
  sdo_write 0x6040 0x00 2 0x0080
  sleep 0.2
  status=$(sdo_read_u16 0x6041 0x00) || die "no response after fault reset"
  (( status & 0x0008 )) && die "drive still faulted (statusword 0x$(printf '%04X' "$status")) — resolve the fault before moving"
fi

if [[ $CHECK_ONLY -eq 1 ]]; then
  log "Communication confirmed: node $NODE responding at ${BITRATE}bps, statusword 0x$(printf '%04X' "$status"), no fault."
  log "--check-only requested — stopping here. Nothing was enabled or moved."
  trap - INT TERM ERR
  exit 0
fi

log "Node $NODE responding, no fault. Entering NMT operational..."
nmt_enter_operational
sleep 0.1

# Per the vendor manual, profile-velocity mode (3.3.2) has no separate edge-triggered
# start step like profile-position mode (3.2.3) does — the motor starts moving as soon
# as the enable sequence (6->7->15) completes, using whatever mode/velocity was already
# loaded. So for --jog-up, mode/accel/decel/target-velocity must be written BEFORE
# enabling, not after.
if [[ $JOG_UP -eq 1 ]]; then
  jog_velocity=$((-JOG_VELOCITY_PPS))   # negative = physically up (see NOTE ON DIRECTION at top)
  log "Configuring velocity mode: vel=${JOG_VELOCITY_PPS} pps, accel/decel=${JOG_ACCEL}/${JOG_DECEL} pps^2"
  sdo_write 0x6060 0x00 1 3            # mode of operation = 3 (profile velocity)
  sdo_write 0x6083 0x00 4 "$JOG_ACCEL"
  sdo_write 0x6084 0x00 4 "$JOG_DECEL"
  sdo_write 0x60FF 0x00 4 "$jog_velocity"
  sleep 0.05
fi

log "Enabling drive (controlword 6 -> 7 -> 15), 100ms apart..."
sdo_write 0x6040 0x00 2 6;  sleep 0.1
sdo_write 0x6040 0x00 2 7;  sleep 0.1
sdo_write 0x6040 0x00 2 15; sleep 0.1

status=$(sdo_read_u16 0x6041 0x00) || die "no statusword response after enable"
log "Statusword after enable = 0x$(printf '%04X' "$status")"
if [[ $DRY_RUN -eq 0 ]] && (( (status & 0x006F) != 0x0027 )); then
  emergency_stop
  die "drive did not reach 'operation enabled' (statusword 0x$(printf '%04X' "$status")) — stopped for safety"
fi

if [[ $JOG_UP -eq 1 ]]; then
  log "auto-stop after ${JOG_MAX_SECONDS}s if not interrupted first"

  if [[ $DRY_RUN -eq 1 ]]; then
    log "DRY-RUN: skipping the run-until-stopped wait."
    trap - INT TERM ERR
    exit 0
  fi

  log "Moving — Ctrl+C any time to quick-stop."
  elapsed=0
  while (( elapsed < JOG_MAX_SECONDS * 10 )); do
    sleep 0.1
    elapsed=$((elapsed + 1))
  done
  log "JOG_MAX_SECONDS safety cap reached — stopping automatically."
  emergency_stop
  exit 0
fi

log "Configuring position mode: step=${TARGET_PULSES} pulses, vel=${VELOCITY_PPS} pps, accel/decel=${ACCEL}/${DECEL} pps^2"
sdo_write 0x6060 0x00 1 1            # mode of operation = 1 (position)
sdo_write 0x6081 0x00 4 "$VELOCITY_PPS"
sdo_write 0x6083 0x00 4 "$ACCEL"
sdo_write 0x6084 0x00 4 "$DECEL"

# One relative move: load target, rising edge on controlword bit4 to start,
# poll for target-reached, then drop bit4 so the next move gets its own edge.
do_relative_move() {
  local delta=$1 label=$2
  log "Moving '${label}': ${delta} pulses relative..."
  sdo_write 0x607A 0x00 4 "$delta"
  sdo_write 0x6040 0x00 2 0x005F   # bit4 0->1: start relative move

  if [[ $DRY_RUN -eq 1 ]]; then
    log "DRY-RUN: skipping wait-for-target-reached for '${label}'."
    sdo_write 0x6040 0x00 2 0x000F
    return 0
  fi

  log "Polling statusword for 'target reached' (bit10), timeout ${MOVE_TIMEOUT}s..."
  local elapsed=0 reached=0 status
  while (( elapsed < MOVE_TIMEOUT * 10 )); do
    status=$(sdo_read_u16 0x6041 0x00) || true
    if [[ -n "${status:-}" ]] && (( status & 0x0400 )); then
      reached=1
      break
    fi
    sleep 0.1
    elapsed=$((elapsed + 1))
  done

  if [[ $reached -eq 1 ]]; then
    log "'${label}' reached target."
    sdo_write 0x6040 0x00 2 0x000F   # drop bit4, ready for next move's rising edge
    return 0
  else
    log "Timed out waiting for '${label}' to reach target — stopping for safety."
    emergency_stop
    exit 1
  fi
}

do_relative_move "$TARGET_PULSES" "up"   # confirmed on hardware: this actually moves the carriage DOWN (see NOTE ON DIRECTION at top)

if [[ "${ONEWAY:-0}" -eq 1 ]]; then
  if [[ $DRY_RUN -eq 1 ]]; then
    log "Dry run complete (one-way) — no frames were actually sent."
    trap - INT TERM ERR
    exit 0
  fi
  log "One-way move complete — drive left enabled, holding the NEW position (did not return)."
  log "Run './ds2c_test_move.sh --estop' any time to quick-stop and de-energize."
  trap - INT TERM ERR
  exit 0
fi

[[ $DRY_RUN -eq 0 ]] && sleep 0.5
do_relative_move "$((-TARGET_PULSES))" "down"

if [[ $DRY_RUN -eq 1 ]]; then
  log "Dry run complete — no frames were actually sent."
  trap - INT TERM ERR
  exit 0
fi

log "Up/down move complete — drive left enabled, holding the original position."
log "Run './ds2c_test_move.sh --estop' any time to quick-stop and de-energize."
trap - INT TERM ERR
