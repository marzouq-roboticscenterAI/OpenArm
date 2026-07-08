#!/usr/bin/env bash
#
# configure_can.sh - one-shot CAN setup: install stable udev names, rename the
# live adapters to their roles (can0/can1=arms, can2=Ranger, can3=DS-2C servo),
# and bring each bus up at its correct bitrate. Needs root (self-elevates).
# Thin wrapper around openarm-c/configure_can.sh.
#
#   ./configure_can.sh    # then: ./run.sh
#
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$HERE/openarm-c/configure_can.sh" "$@"
