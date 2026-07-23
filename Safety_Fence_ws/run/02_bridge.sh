#!/usr/bin/env bash

RUN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$RUN_DIR/_common.sh"

echo "========================================"
echo "TERMINAL 2: ROS-GAZEBO BRIDGE"
echo "========================================"
echo
echo "Camera:   Gazebo -> ROS"
echo "Conveyor: ROS -> Gazebo"
echo

exec ros2 run ros_gz_bridge parameter_bridge \
    '/sorting_camera/image@sensor_msgs/msg/Image[gz.msgs.Image' \
    '/conveyor/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double'
