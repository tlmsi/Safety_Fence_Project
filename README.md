# Safety Fence Project — V1

## Overview

The Safety Fence Project is a ROS 2 and Gazebo robotic sorting-cell simulation. It detects red, green, and blue boxes on a conveyor, estimates their positions in world coordinates, stops the conveyor at the pickup point, and commands a robot with a suction gripper to place each box in its matching bin.

V1 includes working red, green, and blue sorting paths.

## System Design

The simulated cell contains:

- A robot arm with a suction gripper
- A conveyor belt
- An RGB camera mounted on the side of the safety fence
- Red, green, and blue boxes
- Red, green, and blue sorting bins
- A safety fence and complete Gazebo environment

The ROS 2 workspace is divided into packages for perception, control, simulation, behavior, interfaces, bringup, robot description, and tools.

## Camera and Perception

The RGB camera is mounted on the side of the cell, aligned with the pickup position on the same X-axis, raised above the conveyor, and pointed downward toward the pickup area. Its field of view is configured for approximately 2× zoom.

The perception node:

1. Converts the camera image to HSV.
2. Detects red, green, and blue objects.
3. Finds the detected box position in image coordinates.
4. Projects a ray from the camera through the detected image point.
5. Intersects that ray with the known conveyor plane.
6. Calculates the box position in the Gazebo world frame.
7. Publishes the detected pose for the robot automation.

Important topics include:

```text
/perception/box_pose
/perception/detected_color
/perception/object_in_pickup_zone
/sorting_camera/image
/sorting_camera/debug
```

During validation, the detected pickup position closely matched the expected physical location.

## Conveyor Logic

The detector controls the conveyor automatically:

```text
Pickup position clear
        ↓
Conveyor runs

Box reaches pickup position
        ↓
Conveyor stops

Robot removes the box
        ↓
Pickup position becomes clear
        ↓
Conveyor starts again
```

Several consecutive clear frames are required before restarting the conveyor to reduce sensitivity to brief detection losses.

The conveyor side walls were lowered so their top surfaces are flush with the belt.

## Robot Automation

The robot receives the live detected pose from:

```text
/perception/box_pose
```

It calculates a dynamic approach and suction-contact pose for the current box. After attachment, it follows the cached transfer path for the detected color, releases the box in the matching bin, and returns above the pickup area.

V1 includes:

- Red box → red bin
- Green box → green bin
- Blue box → blue bin

The configured release location for all three colors is the down-left corner of the corresponding bin.

## Project Structure

```text
Safety_Fence_Project/
├── README.md
└── Safety_Fence_ws/
    ├── run/
    │   ├── 00_reset.sh
    │   ├── 01_simulation.sh
    │   ├── 02_bridge.sh
    │   ├── 03_detector.sh
    │   ├── 04_red_automation.sh
    │   ├── 05_green_automation.sh
    │   └── 06_blue_automation.sh
    └── src/
        ├── sorting_cell_behavior/
        ├── sorting_cell_bringup/
        ├── sorting_cell_control/
        ├── sorting_cell_description/
        ├── sorting_cell_gazebo/
        ├── sorting_cell_interfaces/
        ├── sorting_cell_perception/
        └── sorting_cell_tools/
```

## Build

```bash
cd ~/Safety_Fence_Project/Safety_Fence_ws

source /opt/ros/lyrical/setup.bash
source ~/ur_gz_ws/install/setup.bash

colcon build --symlink-install
```

## Run the Complete System

### Reset once

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/00_reset.sh
```

### Terminal 1 — Simulation

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/01_simulation.sh
```

Wait until Gazebo and the robot finish loading.

### Terminal 2 — ROS–Gazebo bridge

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/02_bridge.sh
```

### Terminal 3 — Detector and conveyor control

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/03_detector.sh
```

## Red Automation

Validate without moving the robot:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_red_automation.sh check
```

Run:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_red_automation.sh run
```

## Green Automation

Generate the cached path when required:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/05_green_automation.sh plan
```

Validate without moving:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/05_green_automation.sh check
```

Run:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/05_green_automation.sh run
```

## Blue Automation

Generate the cached path when required:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/06_blue_automation.sh plan
```

Validate without moving:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/06_blue_automation.sh check
```

Run:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/06_blue_automation.sh run
```

`plan` calculates and saves a transfer path without moving the robot.  
`check` validates the live pickup pose and IK solution without moving the robot.

## Monitoring

Source the environment first:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ur_gz_ws/install/setup.bash
source ~/Safety_Fence_Project/Safety_Fence_ws/install/setup.bash
```

Detected pose:

```bash
ros2 topic echo /perception/box_pose
```

Detected color:

```bash
ros2 topic echo /perception/detected_color
```

Pickup state:

```bash
ros2 topic echo /perception/object_in_pickup_zone
```

Conveyor command:

```bash
ros2 topic echo /conveyor/cmd_vel
```

Processed camera view:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```text
/sorting_camera/debug
```

## V1 Results

The following functions were successfully tested:

- Automatic conveyor start when the pickup area is clear
- Automatic conveyor stop when a box reaches the pickup position
- RGB detection of red, green, and blue boxes
- Conversion from image coordinates to world coordinates
- Dynamic robot pickup using the detected pose
- Suction attachment and detachment
- Red, green, and blue sorting
- Automatic conveyor restart after box removal
- Robot return above the pickup area after placement

This repository represents Version 1 of the Safety Fence Project.

## Scope

This project is a simulation and automation prototype. It is not a certified industrial safety system.
