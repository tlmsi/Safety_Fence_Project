#!/usr/bin/env bash

RUN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$RUN_DIR/_common.sh"

echo "========================================"
echo "TERMINAL 3: DETECTOR + CONVEYOR"
echo "========================================"
echo
echo "Responsibilities:"
echo "  Camera-based box detection"
echo "  Pickup-zone detection"
echo "  Conveyor stop / restart"
echo
echo "Suction:"
echo "  NONE"
echo
echo "Box spawning:"
echo "  NONE"
echo

exec ros2 run \
  sorting_cell_perception \
  color_sort_detector
