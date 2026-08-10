#!/usr/bin/env bash

RUN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$RUN_DIR/_common.sh"

echo "========================================"
echo "TERMINAL 9: CONTINUOUS BOX FEEDER"
echo "========================================"
echo
echo "16 RED + 16 GREEN + 16 BLUE"
echo "48 boxes total"
echo
echo "Spawn trigger:"
echo "  confirmed physical pickup clear"
echo
echo "Replacement color:"
echo "  shuffled from remaining RGB inventory"
echo
echo "Spawn position:"
echo "  randomized laterally across conveyor"
echo
echo "Conveyor control:"
echo "  NONE"
echo

exec python3 \
  "$REAL_WS/src/sorting_cell_control/scripts/continuous_box_feeder.py"
