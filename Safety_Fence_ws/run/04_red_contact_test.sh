#!/usr/bin/env bash
set -Eeuo pipefail

RUN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$RUN_DIR/_common.sh"

CONTROL="$REAL_WS/src/sorting_cell_control"

SOLVER="$CONTROL/scripts/red_pickup_ik_solver.py"
EXECUTOR="$CONTROL/scripts/move_to_pose.py"
BOX_WAITER="$CONTROL/scripts/wait_for_box_present.py"

APPROACH_POSE="$CONTROL/config/red_pickup_approach_ik.json"
CONTACT_POSE="$CONTROL/config/red_pickup_contact_ik.json"
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

echo "========================================"
echo "RED PICKUP NEAR-CONTACT TEST"
echo "========================================"
echo
echo "Mode:"
echo "  $MODE"
echo
echo "Near-contact offset:"
echo "  0.012 m above calculated box surface"
echo

echo "Calculating approach IK..."

python3 -u "$SOLVER" \
    --target approach \
    --clearance 0.15 \
    --contact-offset 0.012 \
    --output "$APPROACH_POSE"

echo
echo "Calculating near-contact IK..."

python3 -u "$SOLVER" \
    --target contact \
    --clearance 0.15 \
    --contact-offset 0.012 \
    --output "$CONTACT_POSE"

if [[ "$MODE" == "dry" ]]; then
    echo
    echo "========================================"
    echo "RED CONTACT IK DRY RUN PASSED"
    echo "========================================"
    echo
    echo "No movement command was sent."
    echo
    echo "Execute with:"
    echo "  $0 move"
    exit 0
fi

echo
echo "Verifying that a box is present..."

python3 -u "$BOX_WAITER" \
    --timeout 20

echo
echo "Moving to red pickup approach..."

python3 -u "$EXECUTOR" \
    "$APPROACH_POSE" \
    --duration 5.0

sleep 1

echo
echo "Descending to the near-contact point..."

python3 -u "$EXECUTOR" \
    "$CONTACT_POSE" \
    --duration 3.0

echo
echo "Holding near the red box for two seconds..."

sleep 2

echo
echo "Retreating vertically to approach..."

python3 -u "$EXECUTOR" \
    "$APPROACH_POSE" \
    --duration 3.0

sleep 1

echo
echo "Returning home..."

python3 -u "$EXECUTOR" \
    "$HOME_POSE" \
    --duration 5.0

echo
echo "========================================"
echo "RED NEAR-CONTACT TEST PASSED"
echo "========================================"
echo
echo "No suction command was sent."
