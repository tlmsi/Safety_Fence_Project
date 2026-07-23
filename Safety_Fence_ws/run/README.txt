SAFETY FENCE PROJECT RUN ORDER
================================

Reset:
~/Safety_Fence_Project/Safety_Fence_ws/run/00_reset.sh

Terminal 1 - Gazebo:
~/Safety_Fence_Project/Safety_Fence_ws/run/01_simulation.sh

Terminal 2 - ROS/Gazebo bridge:
~/Safety_Fence_Project/Safety_Fence_ws/run/02_bridge.sh

Terminal 3 - box detach, detector and conveyor:
~/Safety_Fence_Project/Safety_Fence_ws/run/03_detector.sh

Terminal 4 - generate and validate red path:
~/Safety_Fence_Project/Safety_Fence_ws/run/04_red_automation.sh plan

Terminal 4 - run red pickup/drop automation:
~/Safety_Fence_Project/Safety_Fence_ws/run/04_red_automation.sh run

Terminals 1, 2 and 3 remain running.

Current verified functionality:
Red-box pickup and placement in the red bin.
