#!/usr/bin/env bash

RUN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$RUN_DIR/_common.sh"

WORLD="$REAL_WS/src/sorting_cell_gazebo/worlds/sorting_cell_world.sdf"
DESCRIPTION="$REAL_WS/src/sorting_cell_description/urdf/sorting_cell_ur.urdf.xacro"

GAZEBO_PLUGIN_DIR="$REAL_WS/install/sorting_cell_gazebo/lib"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$GAZEBO_PLUGIN_DIR${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_GUI_PLUGIN_PATH="$GAZEBO_PLUGIN_DIR${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"


if [[ ! -f "$WORLD" ]]; then
    echo "ERROR: World file missing:"
    echo "  $WORLD"
    exit 1
fi

if [[ ! -f "$DESCRIPTION" ]]; then
    echo "ERROR: Robot description missing:"
    echo "  $DESCRIPTION"
    exit 1
fi

echo "========================================"
echo "TERMINAL 1: GAZEBO SIMULATION"
echo "========================================"
echo
echo "Workspace:"
echo "  $REAL_WS"
echo
echo "World:"
echo "  $WORLD"
echo
echo "Robot description:"
echo "  $DESCRIPTION"
echo

exec ros2 launch ur_simulation_gz ur_sim_control.launch.py \
    ur_type:=ur5e \
    world_file:="$WORLD" \
    description_file:="$DESCRIPTION" \
    initial_joint_controller:=joint_trajectory_controller \
    launch_rviz:=false \
    gazebo_gui:=false
