#!/usr/bin/env bash
set -Eeuo pipefail

RUN_DIR="$(
    cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
    pwd
)"

WS="$(
    readlink -f "$RUN_DIR/.."
)"

SUPERVISOR="$WS/src/sorting_cell_control/scripts/safety_supervisor.py"

source "$RUN_DIR/_common.sh"

if [[ ! -f "$SUPERVISOR" ]]; then
    echo "ERROR: Safety supervisor missing:"
    echo "  $SUPERVISOR"
    exit 1
fi

echo "========================================"
echo "TERMINAL 11: SAFETY SUPERVISOR"
echo "========================================"
echo
echo "Safety states:"
echo "  RUNNING"
echo "  MANUAL_PAUSE"
echo "  PROTECTIVE_STOP"
echo "  E_STOP"
echo
echo "Operator controls:"
echo "  /safety/pause"
echo "  /safety/resume"
echo "  /safety/emergency_stop"
echo "  /safety/reset"
echo
echo "Gate input:"
echo "  /safety/gate_open"
echo
echo "Initial state:"
echo "  MANUAL_PAUSE"
echo
echo "A fresh startup requires RESUME."
echo

exec python3 -u \
    "$SUPERVISOR"
