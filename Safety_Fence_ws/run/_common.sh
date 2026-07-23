#!/usr/bin/env bash
set -Eeuo pipefail

RUN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REAL_WS="$(readlink -f "$RUN_DIR/..")"

if [[ ! -d "$REAL_WS/src" ]]; then
    echo "ERROR: Safety Fence workspace unavailable:"
    echo "  $REAL_WS"
    exit 1
fi

if [[ ! -f "$REAL_WS/install/setup.bash" ]]; then
    echo "ERROR: Safety Fence workspace has not been built:"
    echo "  $REAL_WS"
    exit 1
fi

# Clear inherited ROS workspace overlays.
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset ROS_PACKAGE_PATH
unset PYTHONPATH
unset LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH

unset GZ_PARTITION
unset IGN_PARTITION
unset ROS_DOMAIN_ID
unset ROS_LOCALHOST_ONLY
unset AMENT_TRACE_SETUP_FILES

set +u

source /opt/ros/lyrical/setup.bash
source "$HOME/ur_gz_ws/install/setup.bash"
source "$REAL_WS/install/setup.bash"

set -u

export GZ_SIM_RESOURCE_PATH="$REAL_WS/src/sorting_cell_gazebo:${GZ_SIM_RESOURCE_PATH:-}"
