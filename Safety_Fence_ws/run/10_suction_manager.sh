#!/usr/bin/env bash

RUN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$RUN_DIR/_common.sh"

echo "========================================"
echo "TERMINAL 10: SUCTION MANAGER"
echo "========================================"
echo
echo "Responsibilities:"
echo "  Exact physical-box suction only"
echo "  Dynamic Gazebo joint creation"
echo "  Attach / detach acknowledgement"
echo
echo "Camera:"
echo "  NONE"
echo
echo "Detector / conveyor:"
echo "  NONE"
echo
echo "Box spawning:"
echo "  NONE"
echo

exec python3 \
  "$REAL_WS/src/sorting_cell_control/scripts/suction_manager.py"
