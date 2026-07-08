#!/usr/bin/env bash
#
# run.sh - launch OpenArm-C: the C rewrite (CAN 2.0 motor control + web dashboard
# + automatic Control Scheme 1 when a gamepad is connected).
#
# This replaces the original Python WebUI launcher. To get the old Python app
# back:  git checkout upstream/main -- run.sh app.py backend.py  (then recreate
# venv + certs).
#
# One command; Ctrl-C stops everything cleanly (all motors disabled on exit).
#
#   ./run.sh                     # autodetect can*, dashboard on :8080
#   ./run.sh --port 9000 can0    # explicit port / interface
#
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# exec so Ctrl-C is delivered straight down to the C app (clean motor shutdown).
exec "$HERE/openarm-c/run.sh" "$@"
