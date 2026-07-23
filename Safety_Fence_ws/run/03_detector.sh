#!/usr/bin/env bash

RUN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$RUN_DIR/_common.sh"

DETECTOR="$REAL_WS/src/sorting_cell_perception/sorting_cell_perception/color_sort_detector.py"

if [[ ! -f "$DETECTOR" ]]; then
    echo "ERROR: Detector missing:"
    echo "  $DETECTOR"
    exit 1
fi

echo "========================================"
echo "TERMINAL 3: DETACH + DETECTOR + CONVEYOR"
echo "========================================"
echo
echo "Waiting for suction topics..."

deadline=$((SECONDS + 120))

for color in red green blue; do
    for action in attach detach; do
        topic="/suction/${color}/${action}"

        while ! gz topic -l 2>/dev/null | grep -Fxq "$topic"; do
            if (( SECONDS >= deadline )); then
                echo
                echo "ERROR: Timed out waiting for:"
                echo "  $topic"
                echo "Make sure Terminal 1 is fully running."
                exit 1
            fi

            sleep 0.25
        done

        echo "Ready: $topic"
    done
done

echo
echo "Detaching all boxes..."

for color in red green blue; do
    echo "Detaching ${color^^}..."

    for _ in $(seq 1 10); do
        gz topic \
            -t "/suction/${color}/detach" \
            -m gz.msgs.Empty \
            -p 'unused: true' \
            >/dev/null 2>&1 || true

        sleep 0.05
    done
done

echo
echo "All boxes are detached."
echo
echo "Waiting for the camera bridge..."

if ! python3 "$RUN_DIR/wait_for_ros_endpoint.py" \
    publisher \
    /sorting_camera/image \
    60
then
    echo
    echo "ERROR: Camera bridge was not detected."
    echo "Make sure Terminal 2 is running."
    exit 1
fi

echo "Camera bridge is ready."
echo
echo "Starting Safety Fence detector..."
echo "The detector starts the conveyor on its first camera frame."
echo

DETECTOR_LOG="/tmp/safety_fence_detector.log"

python3 -u "$DETECTOR" 2>&1 | tee "$DETECTOR_LOG"
detector_status=${PIPESTATUS[0]}

echo
echo "ERROR: Detector exited with status:"
echo "  $detector_status"
echo
echo "Detector log:"
echo "  $DETECTOR_LOG"

exit "$detector_status"
