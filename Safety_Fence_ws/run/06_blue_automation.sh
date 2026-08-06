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

PLANNER="$SCRIPTS/blue_automation.py"
V1_RUNTIME="$SCRIPTS/blue_automation_runtime.py"
MOVEIT_RUNTIME="$SCRIPTS/blue_automation_moveit.py"

source "$RUN_DIR/_common.sh"

MODE="${1:-run}"

case "$MODE" in
    plan)
        exec python3 -u \
            "$PLANNER" \
            plan
        ;;

    check)
        exec python3 -u \
            "$V1_RUNTIME" \
            --solve-only
        ;;

    run)
        echo "Running original V1 direct-trajectory automation."
        exec python3 -u \
            "$V1_RUNTIME"
        ;;

    moveit-check)
        exec python3 -u \
            "$MOVEIT_RUNTIME" \
            --solve-only
        ;;

    commission)
        echo "Commissioning blue MoveIt trajectory cache."
        exec python3 -u \
            "$MOVEIT_RUNTIME" \
            --commission-cache
        ;;

    cached)
        echo "Running blue automation with commissioned trajectories."
        exec python3 -u \
            "$MOVEIT_RUNTIME" \
            --use-cache
        ;;

    moveit)
        echo "Running V1.1 MoveIt automation."
        exec python3 -u \
            "$MOVEIT_RUNTIME"
        ;;

    *)
        echo "Usage:"
        echo "  $0 plan          Generate the cached path"
        echo "  $0 check         Validate V1 dynamic pickup IK"
        echo "  $0 run           Run original V1 automation"
        echo "  $0 moveit-check  Validate V1.1 pickup inputs"
        echo "  $0 commission    Plan and save fixed MoveIt trajectories"
        echo "  $0 cached        Use saved fixed MoveIt trajectories"
        echo "  $0 moveit        Run V1.1 with normal replanning"
        exit 2
        ;;
esac
