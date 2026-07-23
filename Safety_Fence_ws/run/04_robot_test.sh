#!/usr/bin/env bash
set -Eeuo pipefail

RUN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$RUN_DIR/_common.sh"

TEST_PROGRAM="$REAL_WS/src/sorting_cell_control/scripts/robot_joint_smoke_test.py"

if [[ ! -f "$TEST_PROGRAM" ]]; then
    echo "ERROR: Robot test program missing:"
    echo "  $TEST_PROGRAM"
    exit 1
fi

echo "========================================"
echo "TERMINAL 4: ROBOT MOVEMENT PHASE 1"
echo "========================================"
echo
echo "This test will:"
echo "  1. Read the current robot joint positions"
echo "  2. Move wrist_3_joint by 0.12 rad"
echo "  3. Return to the exact starting position"
echo
echo "No conveyor or suction commands will be sent."
echo

exec python3 -u "$TEST_PROGRAM" \
    --delta 0.12 \
    --duration 2.5
