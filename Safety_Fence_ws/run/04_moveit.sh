#!/usr/bin/env bash

RUN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$RUN_DIR/_common.sh"

echo "========================================"
echo "TERMINAL: MOVEIT (move_group)"
echo "========================================"
echo
echo "Responsibilities:"
echo "  MoveGroup planning/execution services"
echo "  Consumed directly by the sorting"
echo "  coordinator and per-colour automation"
echo "  scripts via ROS interfaces."
echo
echo "Headless: RViz is NOT launched."
echo "  Coordinator/automation talk to"
echo "  MoveGroup over services/actions and"
echo "  never needed the RViz window."
echo
echo "use_sim_time: true"
echo "  Matches the controllers, which"
echo "  already run on simulation time."
echo

exec ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur5e \
    launch_rviz:=false \
    use_sim_time:=true
