#!/usr/bin/env bash
set -Eeuo pipefail

RUN_DIR="$(
    cd -- "$(
        dirname -- "${BASH_SOURCE[0]}"
    )"
    pwd
)"

source "$RUN_DIR/_common.sh"

AUTOMATION="$REAL_WS/src/sorting_cell_control/scripts/red_automation.py"

MODE="${1:-plan}"

case "$MODE" in
    plan|run)
        ;;
    *)
        echo "Usage:"
        echo "  $0 plan"
        echo "  $0 run"
        exit 2
        ;;
esac

echo "========================================"
echo "RED PICKUP / DROP AUTOMATION"
echo "========================================"
echo
echo "Mode:"
echo "  $MODE"
echo

exec python3 -u \
    "$AUTOMATION" \
    "$MODE"
