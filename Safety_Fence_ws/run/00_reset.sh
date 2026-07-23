#!/usr/bin/env bash
set +e

echo "Stopping Safety Fence simulation processes..."

pkill -SIGINT -f '[s]ort_red_green_quick.sh' 2>/dev/null || true
pkill -SIGINT -f '[s]orting_step.sh' 2>/dev/null || true
pkill -SIGINT -f '[r]ed_arc_transfer.py' 2>/dev/null || true
pkill -SIGINT -f '[g]reen_arc_transfer.py' 2>/dev/null || true
pkill -SIGINT -f '[e]xecute_saved_path.py' 2>/dev/null || true
pkill -SIGINT -f '[c]artesian_ik_move.py' 2>/dev/null || true

pkill -SIGINT -f '[c]olor_sort_detector.py' 2>/dev/null || true
pkill -SIGINT -f '[c]olor_sort_detector' 2>/dev/null || true
pkill -SIGINT -f '[p]arameter_bridge' 2>/dev/null || true
pkill -SIGINT -f '[r]os_gz_bridge' 2>/dev/null || true

pkill -SIGINT -f '[r]os2 topic echo' 2>/dev/null || true
pkill -SIGINT -f '[r]os2 topic pub.*conveyor' 2>/dev/null || true
pkill -SIGINT -f '[g]z topic.*suction' 2>/dev/null || true

pkill -SIGINT -f '[r]os2 launch ur_simulation_gz' 2>/dev/null || true
pkill -SIGINT -f '[c]ontroller_manager' 2>/dev/null || true
pkill -SIGINT -f '[r]obot_state_publisher' 2>/dev/null || true
pkill -SIGINT -f '[s]pawner' 2>/dev/null || true
pkill -SIGINT -f '[g]z sim' 2>/dev/null || true
pkill -SIGINT -f '[g]zserver' 2>/dev/null || true
pkill -SIGINT -f '[g]zclient' 2>/dev/null || true
pkill -SIGINT -f '[r]viz2' 2>/dev/null || true

sleep 4

pkill -SIGKILL -f '[s]ort_red_green_quick.sh' 2>/dev/null || true
pkill -SIGKILL -f '[s]orting_step.sh' 2>/dev/null || true
pkill -SIGKILL -f '[c]olor_sort_detector.py' 2>/dev/null || true
pkill -SIGKILL -f '[c]olor_sort_detector' 2>/dev/null || true
pkill -SIGKILL -f '[p]arameter_bridge' 2>/dev/null || true
pkill -SIGKILL -f '[g]z sim' 2>/dev/null || true
pkill -SIGKILL -f '[g]zserver' 2>/dev/null || true
pkill -SIGKILL -f '[g]zclient' 2>/dev/null || true

ros2 daemon stop 2>/dev/null || true

rm -f /tmp/sorting_cell_detected_color.log

echo
echo "========================================"
echo "SAFETY FENCE RESET COMPLETE"
echo "========================================"
echo "Terminal windows were not closed."
