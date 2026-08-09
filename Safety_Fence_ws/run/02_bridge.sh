#!/usr/bin/env bash

# Prevent duplicate bridge instances.
LOCK_FILE="${XDG_RUNTIME_DIR:-/tmp}/safety_fence_02_bridge.lock"
exec 9>"$LOCK_FILE"

if ! flock -n 9; then
    echo "ERROR: 02_bridge.sh is already running in another terminal."
    exit 1
fi

RUN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$RUN_DIR/_common.sh"

echo "========================================"
echo "TERMINAL 2: ROS-GAZEBO BRIDGE"
echo "========================================"
echo
echo "Camera:   Gazebo -> ROS"
echo "Conveyor: ROS -> Gazebo"
echo "Clock:    provided by the simulation"
echo

exec ros2 run ros_gz_bridge parameter_bridge \
    '/sorting_camera/image@sensor_msgs/msg/Image[gz.msgs.Image' \
    '/conveyor/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double' \
    '/safety/gui/command@std_msgs/msg/String[gz.msgs.StringMsg' \
    '/safety/state@std_msgs/msg/String]gz.msgs.StringMsg' \
    '/safety/gate_visual_open@std_msgs/msg/Bool]gz.msgs.Boolean'
