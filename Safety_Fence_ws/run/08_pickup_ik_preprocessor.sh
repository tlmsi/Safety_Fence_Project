#!/usr/bin/env bash
set -Eeuo pipefail

LOCK_FILE="${XDG_RUNTIME_DIR:-/tmp}/safety_fence_08_pickup_ik_preprocessor.lock"
exec 9>"$LOCK_FILE"

if ! flock -n 9; then
    echo "ERROR: Pickup IK preprocessor is already running."
    exit 1
fi

RUN_DIR="$(
    cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
    pwd
)"

WS="$(
    readlink -f "$RUN_DIR/.."
)"

PREPROCESSOR="$WS/src/sorting_cell_control/scripts/pickup_ik_preprocessor.py"

source "$RUN_DIR/_common.sh"

if [[ ! -f "$PREPROCESSOR" ]]; then
    echo "ERROR: Pickup IK preprocessor missing:"
    echo "  $PREPROCESSOR"
    exit 1
fi

echo "========================================"
echo "TERMINAL 8: PARALLEL PICKUP IK"
echo "========================================"
echo
echo "Observes: RED / GREEN / BLUE"
echo "Robot motion: NONE"
echo "Purpose: prepare the next pickup while the robot is busy"
echo

exec python3 -u \
    "$PREPROCESSOR"
