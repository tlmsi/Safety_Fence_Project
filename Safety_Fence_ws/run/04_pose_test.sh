#!/usr/bin/env bash
set -Eeuo pipefail

RUN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$RUN_DIR/_common.sh"

CONTROL="$REAL_WS/src/sorting_cell_control"
EXECUTOR="$CONTROL/scripts/move_to_pose.py"
TEST_POSE="$CONTROL/config/test_pose.json"
HOME_POSE="$CONTROL/config/home_pose.json"

echo "========================================"
echo "TERMINAL 4: SAVED POSE TEST"
echo "========================================"
echo
echo "Sequence:"
echo "  1. Move to phase_3_test_pose"
echo "  2. Pause for one second"
echo "  3. Return to home"
echo

python3 -u "$EXECUTOR" \
    "$TEST_POSE" \
    --duration 3.0

sleep 1

python3 -u "$EXECUTOR" \
    "$HOME_POSE" \
    --duration 3.0

echo
echo "========================================"
echo "PHASE 3 PASSED"
echo "========================================"
echo "Saved-pose execution is working."
