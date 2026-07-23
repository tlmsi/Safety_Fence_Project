SAFETY FENCE PROJECT RUN ORDER
==============================

Reset:
    ~/Safety_Fence_Project/Safety_Fence_ws/run/00_reset.sh

Terminal 1 - Gazebo:
    ~/Safety_Fence_Project/Safety_Fence_ws/run/01_simulation.sh

Terminal 2 - bridge:
    ~/Safety_Fence_Project/Safety_Fence_ws/run/02_bridge.sh

Terminal 3 - detach boxes, detector and conveyor:
    ~/Safety_Fence_Project/Safety_Fence_ws/run/03_detector.sh

Terminal 4 - autonomous red and green pickup/drop:
    ~/Safety_Fence_Project/Safety_Fence_ws/run/04_autonomous.sh

Terminals 1, 2 and 3 remain running.
Terminal 4 performs the complete red and green sorting cycle.
