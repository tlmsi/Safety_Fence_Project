#!/usr/bin/env bash
set -Eeuo pipefail

RUN_DIR="$(
    cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
    pwd
)"

WS="$(
    readlink -f "$RUN_DIR/.."
)"

SCRIPTS="$WS/src/sorting_cell_control/scripts"

PLANNER="$SCRIPTS/red_automation.py"
RUNTIME="$SCRIPTS/red_automation_runtime.py"

source "$RUN_DIR/_common.sh"

MODE="${1:-run}"

case "$MODE" in
    plan)
        exec python3 -u \
            "$PLANNER" \
            plan
        ;;

    run)
        exec python3 -u \
            "$RUNTIME"
        ;;

    *)
        echo "Usage:"
        echo "  $0 plan"
        echo "  $0 run"
        exit 2
        ;;
esac
