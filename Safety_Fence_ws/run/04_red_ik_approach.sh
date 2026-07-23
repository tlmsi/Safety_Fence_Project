#!/usr/bin/env bash
set -Eeuo pipefail

RUN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$RUN_DIR/_common.sh"

CONTROL="$REAL_WS/src/sorting_cell_control"

SOLVER="$CONTROL/scripts/red_pickup_ik_solver.py"
EXECUTOR="$CONTROL/scripts/move_to_pose.py"

IK_POSE="$CONTROL/config/red_pickup_approach_ik.json"
HOME_POSE="$CONTROL/config/home_pose.json"

MODE="${1:-dry}"

case "$MODE" in
    dry|move)
        ;;
    *)
        echo "Usage:"
        echo "  $0 dry"
        echo "  $0 move"
        exit 2
        ;;
esac

for file in \
    "$SOLVER" \
    "$EXECUTOR" \
    "$HOME_POSE"
do
    if [[ ! -f "$file" ]]; then
        echo "ERROR: Required file missing:"
        echo "  $file"
        exit 1
    fi
done

echo "========================================"
echo "RED PICKUP IK APPROACH TEST"
echo "========================================"
echo
echo "Mode:"
echo "  $MODE"
echo
echo "The target is calculated from:"
echo "  sorting_cell_world.sdf"
echo "  conveyor/model.sdf"
echo "  sorting_cell_ur.urdf.xacro"
echo

python3 -u "$SOLVER" \
    --clearance 0.15 \
    --output "$IK_POSE"

if [[ "$MODE" == "dry" ]]; then
    echo
    echo "========================================"
    echo "IK DRY RUN PASSED"
    echo "========================================"
    echo
    echo "No robot command was sent."
    echo
    echo "Execute with:"
    echo "  $0 move"
    exit 0
fi

echo
echo "Moving to red pickup approach..."

python3 -u "$EXECUTOR" \
    "$IK_POSE" \
    --duration 6.0

sleep 2

echo
echo "Returning to saved home pose..."

python3 -u "$EXECUTOR" \
    "$HOME_POSE" \
    --duration 6.0

echo
echo "========================================"
echo "RED PICKUP APPROACH TEST PASSED"
echo "========================================"
