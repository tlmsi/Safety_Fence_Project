# Safety Fence Project — V1.2

## Overview

The **Safety Fence Project** is a ROS 2, Gazebo and MoveIt robotic sorting-cell simulation built around a UR5e robot.

The system automatically detects red, green and blue boxes on a conveyor, determines their pickup positions from a camera, prepares robot inverse kinematics, picks each box using a simulated suction gripper, and places it into the corresponding sorting bin.

Version **V1.2** also includes a complete simulated machine-safety layer with:

- Manual Pause
- Protective Stop
- Emergency Stop
- controlled robot deceleration
- safety-fence operator GUI
- hinged safety door
- four PNP proximity sensors
- four metal door targets
- fail-safe door monitoring
- industrial stack-light indication

The project is designed as an integrated automation and machine-safety simulation rather than a collection of independent robot motions.

---

# System Overview

The simulated cell contains:

- UR5e industrial robot
- suction end effector
- conveyor belt
- RGB sorting camera
- red, green and blue boxes
- red, green and blue sorting bins
- continuous randomized box feeder
- safety fence
- hinged safety door
- four PNP proximity sensors
- four metal PNP targets
- Emergency Stop
- industrial stack light
- Gazebo safety-control GUI
- MoveIt motion planning

The current main architecture is:

```text
                     ┌─────────────────┐
                     │  RGB Camera     │
                     └────────┬────────┘
                              │
                              ▼
                     ┌─────────────────┐
                     │ Color Detector  │
                     │ + Conveyor Ctrl │
                     └────────┬────────┘
                              │
                   settled box / generation
                              │
              ┌───────────────┴───────────────┐
              ▼                               ▼
     ┌──────────────────┐          ┌─────────────────────┐
     │ Parallel Pickup  │          │ Unified Sorting     │
     │ IK Preprocessor  │─────────▶│ Coordinator         │
     └──────────────────┘          └──────────┬──────────┘
                                             │
                                             ▼
                                  ┌─────────────────────┐
                                  │ MoveIt / UR5e       │
                                  │ Motion Execution    │
                                  └──────────┬──────────┘
                                             │
                                             ▼
                                  ┌─────────────────────┐
                                  │ Suction Manager     │
                                  │ Attach / Detach     │
                                  └─────────────────────┘


     Gazebo Safety GUI
             │
             ▼
     Hinged Safety Door
             │
             ▼
      4 PNP + 4 Targets
             │
             ▼
       ROS-Gazebo Bridge
             │
             ▼
      Safety Supervisor
             │
       ┌─────┴─────┐
       ▼           ▼
 Robot permission  Conveyor permission
```

---

# Camera and Perception

The cell uses an RGB camera aimed at the conveyor pickup area.

The camera field of view is configured with approximately **2× optical-style zoom** so the pickup region occupies a larger part of the image.

The perception system:

1. receives the Gazebo camera image,
2. converts the image to HSV,
3. detects red, green and blue objects,
4. identifies individual box contours,
5. determines each box center in image coordinates,
6. projects the image location into the Gazebo world frame,
7. determines whether a box is inside the pickup zone,
8. waits for the box to settle,
9. publishes the authoritative pickup event and pose.

Important perception topics include:

```text
/sorting_camera/image
/sorting_camera/debug

/perception/box_pose
/perception/detected_color
/perception/object_in_pickup_zone
```

The detector supports all three sorting colors:

```text
RED
GREEN
BLUE
```

---

# Conveyor Control

The detector owns conveyor movement.

The operating logic is:

```text
Pickup area clear
        │
        ▼
Conveyor runs
        │
        ▼
Box enters pickup zone
        │
        ▼
Box settles
        │
        ▼
Conveyor stops
        │
        ▼
Robot removes box
        │
        ▼
Pickup area confirmed clear
        │
        ▼
Conveyor starts again
```

The conveyor does not restart from a single missing frame. Multiple clear observations are used to prevent false restart commands caused by brief detection loss.

---

# Continuous Box Feeder

The current feeder provides:

```text
16 RED
16 GREEN
16 BLUE
---------
48 boxes total
```

The remaining colors are shuffled so the sorting sequence is mixed rather than grouped by color.

Each replacement box is also spawned at a randomized lateral position across the conveyor.

A replacement is generated only after the previous pickup location has been confirmed physically clear.

The feeder does **not** control the conveyor.

---

# Parallel Pickup IK

Terminal 8 runs the pickup IK preprocessor.

Its purpose is to prepare the next pickup while the robot is busy completing the current sorting cycle.

It observes settled RED, GREEN and BLUE pickup events but does not move the robot.

The calculated pickup information is stored in:

```text
/tmp/safety_fence_prepared_pickup.json
```

This allows the unified sorting coordinator to consume already prepared pickup information when possible.

---

# Unified Robot Sorting

The current V1.2 runtime uses one persistent sorting coordinator for all three colors.

```text
RED   → red bin
GREEN → green bin
BLUE  → blue bin
```

The coordinator uses:

- the live camera-derived pickup position,
- prepared pickup IK,
- MoveIt,
- commissioned/cached transfer information,
- suction acknowledgement,
- current safety permission.

The coordinator remains active across multiple sorting cycles rather than starting a separate robot program for every box.

An optional cycle limit can be supplied with:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/07_sorting_coordinator.sh --max-cycles N
```

For example:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/07_sorting_coordinator.sh --max-cycles 10
```

---

# MoveIt

MoveIt provides the robot planning and trajectory execution interfaces used by the current coordinator.

The normal project launcher is headless:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_moveit.sh
```

This intentionally starts MoveIt without RViz because the automation communicates directly with MoveGroup services and actions.

Simulation time is enabled so MoveIt and the Gazebo controllers operate from the same clock.

## Optional MoveIt + RViz

RViz can still be opened manually when visualization is required:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ur_gz_ws/install/setup.bash
source ~/Safety_Fence_Project/Safety_Fence_ws/install/setup.bash

ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  use_sim_time:=true \
  launch_rviz:=true
```

Do not run this at the same time as another MoveIt instance.

---

# MoveIt Planning Scene

After MoveIt starts, the fixed sorting-cell geometry can be loaded into the planning scene with:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ur_gz_ws/install/setup.bash
source ~/Safety_Fence_Project/Safety_Fence_ws/install/setup.bash

python3 \
  ~/Safety_Fence_Project/Safety_Fence_ws/src/sorting_cell_control/scripts/load_sorting_scene.py
```

---

# Suction System

Terminal 10 contains the dedicated suction manager.

Responsibilities are separated from the detector and coordinator.

The suction manager:

- identifies the exact physical box being picked,
- creates the Gazebo detachable joint dynamically,
- attaches the selected box to the suction tool,
- confirms attachment,
- removes the joint when requested,
- confirms detachment.

A suction joint exists only while an actual box is being held.

---

# Safety System

V1.2 contains four main safety states:

```text
RUNNING
MANUAL_PAUSE
PROTECTIVE_STOP
E_STOP
```

Safety priority is:

```text
E_STOP
   ↓
PROTECTIVE_STOP
   ↓
MANUAL_PAUSE
   ↓
RUNNING
```

The safety supervisor starts in:

```text
MANUAL_PAUSE
```

A fresh simulation therefore requires an explicit **Resume** before normal robot operation is permitted.

---

# Manual Pause

Manual Pause is a controlled operator stop.

When Manual Pause is requested while the robot is moving:

1. the safety state changes immediately,
2. new robot motions are blocked,
3. the conveyor is inhibited,
4. the active robot motion decelerates,
5. controller speed scaling ramps from 1.0 to 0.0,
6. the robot comes to a controlled stop.

Configured controlled-stop duration:

```text
1.5 seconds of simulation time
```

After a Manual Pause, normal operation can continue using:

```text
RESUME
```

---

# Protective Stop

Opening the safety door causes a **Protective Stop**.

The PNP sensors, rather than the GUI command itself, determine whether the physical door is safely closed.

When the door begins opening and the valid PNP pattern is lost:

```text
RUNNING
   │
   ▼
PROTECTIVE_STOP
   │
   ▼
controlled deceleration
```

The door must be closed before the protective condition can be reset.

Recovery sequence:

```text
CLOSE DOOR
    ↓
PNP = 1111
    ↓
SAFETY RESET
    ↓
MANUAL_PAUSE
    ↓
RESUME
    ↓
RUNNING
```

---

# Emergency Stop

Emergency Stop has the highest priority.

When E-Stop is activated, the safety supervisor sends an immediate controller speed-scaling factor of:

```text
0.0
```

This stops commanded robot trajectory motion without waiting for the controlled 1.5 second Pause / Protective Stop ramp.

Recovery sequence:

```text
EMERGENCY STOP
      ↓
    E_STOP
      ↓
SAFETY RESET
      ↓
MANUAL_PAUSE
      ↓
   RESUME
      ↓
   RUNNING
```

If the safety door is still open when the E-Stop is reset, the machine remains in Protective Stop.

---

# Hinged Safety Door

The safety gate is implemented as a hinged door rather than a sliding gate.

The hinge is located on the left side of the opening.

Closed position:

```text
0 degrees
```

Open position:

```text
approximately -90 degrees
```

The door swings outward / backwards from the robot cell.

The left hinge position remains fixed while the complete door rotates around it.

The operator controls physical door movement through the Gazebo safety panel.

---

# 20 mm Safety-Door Sensor Clearance

A dedicated physical clearance was created on the latch side of the door.

The door itself was not shortened.

Instead, material was removed from the stationary right fence post.

Geometry:

```text
Closed door edge:        x = 0.600 m
Fixed fence inner face:  x = 0.620 m

Structural clearance:      0.020 m
                         = 20 mm
```

The outer face of the fixed post remains unchanged.

This space contains the proximity-sensing arrangement.

---

# Four PNP Proximity Sensors

The safety door uses four simulated **PNP inductive proximity sensors**.

The sensor bodies are attached to the stationary fence.

Each sensor faces its own small metallic target mounted on the moving door.

```text
FIXED FENCE                     MOVING DOOR

[ PNP sensor ]  → air gap →  [ metal target ]
```

The sensor does not physically touch the target.

The current latch geometry uses approximately:

```text
PNP body depth:       10 mm
closed sensing gap:    5 mm
metal target:          4 mm
structural clearance: 20 mm
```

---

# PNP Sensor Distribution

The four sensors are intentionally **not mounted directly underneath one another**.

They remain at four different vertical heights, but they are also distributed across the width of the closed door / fixed-post overlap.

The outer pair is placed near opposite edges and the two middle sensors are distributed evenly between them.

Conceptually:

```text
one edge                               opposite edge
   │                                         │
   ●-------------●-------------●-------------●
 PNP1          PNP2          PNP3          PNP4
```

Their corresponding metal targets use exactly matching positions on the door.

This gives four separate closed-position checks rather than four sensors observing exactly the same point.

---

# PNP Safety Logic

The four signals are:

```text
/safety/pnp1
/safety/pnp2
/safety/pnp3
/safety/pnp4
```

The only valid safely-closed pattern is:

```text
PNP1 = 1
PNP2 = 1
PNP3 = 1
PNP4 = 1

1111 = CLOSED
```

Any other combination is considered unsafe:

```text
1110
1101
1011
0111
0000
etc.
```

and is treated as:

```text
OPEN / UNSAFE
```

The safety supervisor also monitors PNP feedback freshness.

Loss of sensor feedback is handled fail-safe and causes the door to be treated as open.

The GUI door command therefore does **not** prove that the door reached the commanded position.

The physical sequence is:

```text
GUI OPEN / CLOSE command
          │
          ▼
Gazebo SafetyRuntime moves door
          │
          ▼
Metal targets physically move
          │
          ▼
Four virtual PNP sensors evaluate position
          │
          ▼
Gazebo → ROS bridge
          │
          ▼
Safety Supervisor evaluates 1111
```

---

# Safety GUI

The Gazebo simulation contains a dedicated machine-safety operator panel.

Available controls include:

```text
START / RESUME
MANUAL PAUSE
SAFETY RESET
EMERGENCY STOP
OPEN DOOR
CLOSE DOOR
```

The GUI also shows:

- current safety state,
- current door state,
- operator warnings,
- reset / resume requirements.

Examples of operator warnings include:

```text
YOU NEED TO CLOSE THE DOOR FIRST.
```

and:

```text
YOU NEED TO RESET FIRST.
```

---

# Stack Light

The simulated industrial stack light follows the safety state.

```text
RUNNING          → GREEN

MANUAL_PAUSE     → YELLOW

PROTECTIVE_STOP  → blinking YELLOW

E_STOP           → RED
```

---

# ROS-Gazebo Bridge

Terminal 2 currently bridges:

```text
/sorting_camera/image

/conveyor/cmd_vel

/safety/gui/command
/safety/state
/safety/gate_visual_open

/safety/pnp1
/safety/pnp2
/safety/pnp3
/safety/pnp4
```

Main directions are:

```text
Camera:
Gazebo → ROS

Conveyor:
ROS → Gazebo

PNP sensors:
Gazebo → ROS

GUI command:
Gazebo → ROS

Safety state:
ROS → Gazebo
```

---

# Project Structure

Main project structure:

```text
Safety_Fence_Project/
│
├── README.md
│
└── Safety_Fence_ws/
    │
    ├── run/
    │   ├── 00_reset.sh
    │   ├── 01_simulation.sh
    │   ├── 02_bridge.sh
    │   ├── 03_detector.sh
    │   ├── 04_moveit.sh
    │   ├── 07_sorting_coordinator.sh
    │   ├── 08_pickup_ik_preprocessor.sh
    │   ├── 09_continuous_box_feeder.sh
    │   ├── 10_suction_manager.sh
    │   ├── 11_safety_supervisor.sh
    │   └── _common.sh
    │
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

The `run/` directory also contains older commissioning, diagnostic and V1 per-color scripts documented later in this README.

Files with names such as:

```text
.before_...
```

are historical backups and are **not** part of the normal startup procedure.

---

# Build the Workspace

```bash
cd ~/Safety_Fence_Project/Safety_Fence_ws

source /opt/ros/lyrical/setup.bash
source ~/ur_gz_ws/install/setup.bash

colcon build --symlink-install
```

---

# Run the Complete V1.2 System

Each persistent component should normally run in its own terminal.

## 0 — Reset Previous Simulation

Run once before a clean startup:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/00_reset.sh
```

This stops previous Gazebo, MoveIt, detector, coordinator, feeder, suction and safety processes and clears temporary runtime state.

---

## 1 — Gazebo Simulation

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/01_simulation.sh
```

This starts:

- Gazebo
- UR5e simulation
- controllers
- sorting-cell world
- safety fence
- safety door
- safety GUI
- stack light

RViz is intentionally not started here.

Wait for Gazebo and the robot controllers to finish loading.

---

## 2 — ROS-Gazebo Bridge

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/02_bridge.sh
```

This bridges the camera, conveyor, GUI, safety state and four PNP channels.

---

## 3 — Detector and Conveyor

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/03_detector.sh
```

Responsibilities:

```text
camera detection
RGB classification
pickup-zone detection
settled-box detection
pickup generation
conveyor stop/restart
```

This terminal does not control suction or box spawning.

---

## 4 — MoveIt

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_moveit.sh
```

MoveIt starts headless.

---

## 5 — Load MoveIt Planning Scene

After MoveIt is ready:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ur_gz_ws/install/setup.bash
source ~/Safety_Fence_Project/Safety_Fence_ws/install/setup.bash

python3 \
  ~/Safety_Fence_Project/Safety_Fence_ws/src/sorting_cell_control/scripts/load_sorting_scene.py
```

---

## 6 — Parallel Pickup IK

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/08_pickup_ik_preprocessor.sh
```

This terminal prepares future pickup IK solutions and never commands robot motion.

---

## 7 — Continuous Box Feeder

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/09_continuous_box_feeder.sh
```

This provides the shuffled 48-box RGB inventory.

---

## 8 — Suction Manager

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/10_suction_manager.sh
```

This owns exact physical attachment and detachment.

---

## 9 — Safety Supervisor

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/11_safety_supervisor.sh
```

The initial safety state is Manual Pause.

Verify the safety system is healthy before allowing robot movement.

---

## 10 — Unified Sorting Coordinator

Unlimited operation:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/07_sorting_coordinator.sh
```

Limited number of cycles:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/07_sorting_coordinator.sh --max-cycles N
```

Example:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/07_sorting_coordinator.sh --max-cycles 10
```

---

# Camera Window

To open the live ROS camera viewer:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ur_gz_ws/install/setup.bash
source ~/Safety_Fence_Project/Safety_Fence_ws/install/setup.bash

ros2 run rqt_image_view rqt_image_view
```

Useful image topics include:

```text
/sorting_camera/image
/sorting_camera/debug
```

For perception debugging, select:

```text
/sorting_camera/debug
```

---

# Recommended Operator Startup

A normal startup is:

```text
1. Reset
2. Start Gazebo
3. Start ROS-Gazebo bridge
4. Start detector
5. Start MoveIt
6. Load planning scene
7. Start pickup IK preprocessor
8. Start continuous box feeder
9. Start suction manager
10. Start safety supervisor
11. Start sorting coordinator
12. Confirm door closed / PNP 1111
13. Press START / RESUME in the safety GUI
```

The camera viewer is optional and can be opened after the bridge and detector are running.

---

# Normal Safety Operation

## Start

```text
Door physically closed
        ↓
PNP = 1111
        ↓
Safety state = MANUAL_PAUSE
        ↓
START / RESUME
        ↓
RUNNING
```

## Manual Pause

```text
MANUAL PAUSE
     ↓
controlled deceleration
     ↓
MANUAL_PAUSE
     ↓
RESUME
     ↓
RUNNING
```

## Door Open

```text
OPEN DOOR
    ↓
door begins moving
    ↓
PNP 1111 is lost
    ↓
PROTECTIVE_STOP
    ↓
controlled deceleration
```

Recovery:

```text
CLOSE DOOR
    ↓
PNP = 1111
    ↓
SAFETY RESET
    ↓
RESUME
```

## Emergency Stop

```text
EMERGENCY STOP
      ↓
    E_STOP
```

Recovery:

```text
SAFETY RESET
     ↓
MANUAL_PAUSE
     ↓
RESUME
```

---

# Monitoring Commands

Source the environment first:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ur_gz_ws/install/setup.bash
source ~/Safety_Fence_Project/Safety_Fence_ws/install/setup.bash
```

## Detected Box Pose

```bash
ros2 topic echo /perception/box_pose
```

## Detected Color

```bash
ros2 topic echo /perception/detected_color
```

## Pickup-Zone State

```bash
ros2 topic echo /perception/object_in_pickup_zone
```

## Conveyor Command

```bash
ros2 topic echo /conveyor/cmd_vel
```

## Safety State

```bash
ros2 topic echo /safety/state
```

## PNP 1

```bash
ros2 topic echo /safety/pnp1
```

## PNP 2

```bash
ros2 topic echo /safety/pnp2
```

## PNP 3

```bash
ros2 topic echo /safety/pnp3
```

## PNP 4

```bash
ros2 topic echo /safety/pnp4
```

A correctly closed door should report:

```text
PNP1 = true
PNP2 = true
PNP3 = true
PNP4 = true
```

---

# Legacy and Diagnostic Run Commands

The following scripts still exist in the repository but are not required for the normal unified V1.2 runtime.

They are retained for testing, commissioning and earlier V1/V1.1 workflows.

---

## Original Autonomous Red Wrapper

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_autonomous.sh
```

This forwards to the original red V1 automation.

---

## Saved Pose Test

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_pose_test.sh
```

Moves to the saved test pose and returns home.

---

## Robot Joint Smoke Test

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_robot_test.sh
```

Moves `wrist_3_joint` by a small amount and returns to its exact starting position.

---

# Red Development / Commissioning Commands

Generate original cached path:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_red_automation.sh plan
```

Validate original V1 pickup IK without robot movement:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_red_automation.sh check
```

Run original V1 direct-trajectory automation:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_red_automation.sh run
```

Validate MoveIt pickup inputs:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_red_automation.sh moveit-check
```

Commission MoveIt trajectory cache:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_red_automation.sh commission
```

Use commissioned MoveIt trajectories:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_red_automation.sh cached
```

Run MoveIt with normal replanning:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_red_automation.sh moveit
```

---

# Green Development / Commissioning Commands

Generate original cached path:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/05_green_automation.sh plan
```

Validate V1 pickup IK:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/05_green_automation.sh check
```

Run original V1 automation:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/05_green_automation.sh run
```

Validate MoveIt pickup inputs:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/05_green_automation.sh moveit-check
```

Commission MoveIt trajectories:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/05_green_automation.sh commission
```

Use commissioned MoveIt trajectories:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/05_green_automation.sh cached
```

Run MoveIt with normal replanning:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/05_green_automation.sh moveit
```

---

# Blue Development / Commissioning Commands

Generate original cached path:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/06_blue_automation.sh plan
```

Validate V1 pickup IK:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/06_blue_automation.sh check
```

Run original V1 automation:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/06_blue_automation.sh run
```

Validate MoveIt pickup inputs:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/06_blue_automation.sh moveit-check
```

Commission MoveIt trajectories:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/06_blue_automation.sh commission
```

Use commissioned MoveIt trajectories:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/06_blue_automation.sh cached
```

Run MoveIt with normal replanning:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/06_blue_automation.sh moveit
```

---

# Red Pickup IK Test

Dry run:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_red_ik_approach.sh dry
```

Move robot:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_red_ik_approach.sh move
```

---

# Red Near-Contact Test

Dry run:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_red_contact_test.sh dry
```

Move robot:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/04_red_contact_test.sh move
```

This test does not activate suction.

---

# Internal Run Helpers

The following files support the run scripts and normally should not be executed manually:

```text
run/_common.sh
run/wait_for_ros_endpoint.py
```

`_common.sh` prepares the ROS, UR Gazebo and Safety Fence workspace environments.

`wait_for_ros_endpoint.py` is a utility for waiting until a required ROS publisher or subscriber is available.

Historical files containing:

```text
.before_...
```

are backup snapshots and should not be used as normal runtime commands.

---

# Resetting the Project

To stop the complete running stack:

```bash
~/Safety_Fence_Project/Safety_Fence_ws/run/00_reset.sh
```

The reset script stops the major ROS, Gazebo, MoveIt, perception, automation, feeder, suction and safety processes and clears temporary runtime files.

It does not close the terminal windows themselves.

---

# V1.2 Current Capabilities

The current project includes:

- complete Gazebo sorting-cell environment
- UR5e simulation
- RGB box perception
- approximately 2× camera zoom
- image-to-world pickup localization
- multiple same-color contour handling
- automatic pickup-zone detection
- detector-owned conveyor logic
- automatic conveyor stop and restart
- red, green and blue sorting
- unified persistent MoveIt sorting coordinator
- dynamic camera-based pickup
- parallel pickup IK preprocessing
- commissioned/cached transfer execution
- continuous shuffled 48-box feeder
- randomized box spawn position
- exact physical-box suction
- dynamic Gazebo attachment and detachment
- machine-safety GUI
- Manual Pause
- Protective Stop
- Emergency Stop
- controller speed-scaling E-Stop
- controlled 1.5 second safety deceleration
- safety-state priority handling
- stack-light visualization
- hinged safety door
- smooth door opening and closing
- 20 mm latch-side sensor clearance
- four stationary PNP proximity sensors
- four moving metal targets
- distributed PNP sensor locations
- `1111 = CLOSED` gate validation
- fail-safe PNP heartbeat monitoring
- door-controlled Protective Stop
- Safety Reset / Resume recovery logic

---

# Safety Scope

This project is an automation and safety **simulation prototype**.

The PNP sensors, Emergency Stop, safety supervisor, controlled deceleration and other safety functions are intended to model and study industrial machine-safety behavior.

They are **not certified functional-safety hardware or software** and must not be treated as a certified safety system for a real industrial machine.

---

# Version

Current development branch:

```text
v1.2
```

This README describes the current integrated V1.2 Safety Fence Project.
