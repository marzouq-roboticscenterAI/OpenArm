#!/usr/bin/env bash
#
# reset_can.sh - recover a wedged / bus-off / stuck-DOWN SocketCAN adapter.
# Thin wrapper around openarm-c/reset_can.sh (which knows the per-interface
# bitrates: can2=500k rover, everything else=1M).
#
#   ./reset_can.sh                # reset every can* present
#   ./reset_can.sh can0 can2      # reset just these
#
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$HERE/openarm-c/reset_can.sh" "$@"
