#!/usr/bin/env bash
set -Eeuo pipefail

RUN_DIR="$(
    cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
    pwd
)"

WS="$(
    readlink -f "$RUN_DIR/.."
)"

COORDINATOR="$WS/src/sorting_cell_control/scripts/sorting_coordinator.py"

source "$RUN_DIR/_common.sh"

if [[ ! -f "$COORDINATOR" ]]; then
    echo "ERROR: Sorting coordinator missing:"
    echo "  $COORDINATOR"
    exit 1
fi

echo "========================================"
echo "TERMINAL 7: UNIFIED SORTING COORDINATOR"
echo "========================================"
echo
echo "Mode: cached MoveIt automation"
echo "Colors: RED / GREEN / BLUE"
echo

exec python3 -u \
    "$COORDINATOR" \
    "$@"
