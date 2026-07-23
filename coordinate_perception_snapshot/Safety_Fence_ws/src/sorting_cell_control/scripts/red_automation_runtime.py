#!/usr/bin/env python3

import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List

import rclpy
from std_msgs.msg import Bool

from move_through_poses import (
    ContinuousPoseExecutor,
    load_pose,
)


HERE = Path(__file__).resolve().parent
CONTROL_DIR = HERE.parent

PATH_FILE = (
    CONTROL_DIR
    / 'config'
    / 'red_automation_path.json'
)

REQUIRED_POSES = (
    'pickup_approach',
    'pickup_touch',
    'middle',
    'drop_approach',
    'drop_release',
)

# Minimal Gazebo processing pause after attach/detach.
SUCTION_SETTLE_SECONDS = 0.10


def resolve_pose_path(
    value: object,
) -> Path:
    if not isinstance(value, str):
        raise RuntimeError(
            f'Invalid pose path value: {value!r}'
        )

    path = Path(value).expanduser()

    if not path.is_absolute():
        path = (
            PATH_FILE.parent
            / path
        ).resolve()

    return path


def load_cached_poses() -> Dict[str, List[float]]:
    if not PATH_FILE.is_file():
        raise RuntimeError(
            'No cached automation path exists. '
            'Run 04_red_automation.sh plan first.'
        )

    try:
        plan = json.loads(
            PATH_FILE.read_text()
        )

    except Exception as error:
        raise RuntimeError(
            f'Cannot read cached path: {error}'
        ) from error

    pose_files = plan.get(
        'pose_files'
    )

    if not isinstance(pose_files, dict):
        raise RuntimeError(
            f'Invalid pose_files section in '
            f'{PATH_FILE}'
        )

    poses: Dict[str, List[float]] = {}

    for key in REQUIRED_POSES:
        if key not in pose_files:
            raise RuntimeError(
                f'Cached path is missing pose: {key}'
            )

        pose_path = resolve_pose_path(
            pose_files[key]
        )

        _, positions = load_pose(
            pose_path
        )

        poses[key] = positions

    return poses


def wait_for_box(
    node: ContinuousPoseExecutor,
    timeout: float = 120.0,
) -> None:
    state = {
        'present': False,
    }

    def callback(
        message: Bool,
    ) -> None:
        state['present'] = bool(
            message.data
        )

    subscription = node.create_subscription(
        Bool,
        '/perception/object_in_pickup_zone',
        callback,
        10,
    )

    node.get_logger().info(
        'Waiting for box-present=true...'
    )

    deadline = (
        time.monotonic()
        + timeout
    )

    while (
        rclpy.ok()
        and not state['present']
        and time.monotonic() < deadline
    ):
        rclpy.spin_once(
            node,
            timeout_sec=0.1,
        )

    node.destroy_subscription(
        subscription
    )

    if not state['present']:
        raise RuntimeError(
            'Timed out waiting for the red box.'
        )

    node.get_logger().info(
        'Box detected in pickup zone.'
    )


def suction(
    node: ContinuousPoseExecutor,
    action: str,
) -> None:
    if action not in (
        'attach',
        'detach',
    ):
        raise RuntimeError(
            f'Invalid suction action: {action}'
        )

    node.get_logger().info(
        f'Suction: {action.upper()}'
    )

    try:
        result = subprocess.run(
            [
                'gz',
                'topic',
                '-t',
                f'/suction/red/{action}',
                '-m',
                'gz.msgs.Empty',
                '-p',
                'unused: true',
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            timeout=3.0,
            check=False,
        )

    except subprocess.TimeoutExpired as error:
        raise RuntimeError(
            f'Suction command timed out: {action}'
        ) from error

    if result.returncode != 0:
        raise RuntimeError(
            f'Suction {action} failed: '
            f'{result.stderr.strip()}'
        )

    time.sleep(
        SUCTION_SETTLE_SECONDS
    )


def execute_phase(
    node: ContinuousPoseExecutor,
    label: str,
    pose_names: List[str],
    poses: Dict[str, List[float]],
    segment_times: List[float],
) -> None:
    node.get_logger().info(
        label
    )

    node.execute(
        pose_names,
        [
            poses[name]
            for name in pose_names
        ],
        segment_times,
    )


def main() -> int:
    try:
        poses = load_cached_poses()

    except Exception as error:
        print(
            f'ERROR: {error}',
            file=sys.stderr,
        )
        return 1

    rclpy.init()

    # One node, subscription and action client remain
    # alive during the complete automation cycle.
    node = ContinuousPoseExecutor()

    try:
        node.get_logger().info(
            'Loaded cached red automation path.'
        )

        node.get_logger().info(
            'IK calculation skipped.'
        )

        node.get_logger().info(
            'Persistent execution node started.'
        )

        node.wait_for_controller(
            timeout=30.0
        )

        wait_for_box(
            node
        )

        # --------------------------------------------------
        # Phase 1
        #
        # Current position
        # -> pickup approach
        # -> pickup touch
        #
        # Stop only at pickup touch.
        # --------------------------------------------------

        execute_phase(
            node,
            'PHASE 1: continuous approach to pickup',
            [
                'pickup_approach',
                'pickup_touch',
            ],
            poses,
            [
                2.6,
                1.4,
            ],
        )

        suction(
            node,
            'attach',
        )

        # --------------------------------------------------
        # Phase 2
        #
        # Pickup touch
        # -> pickup lift
        # -> middle
        # -> drop approach
        # -> drop release
        #
        # Stop only at drop release.
        # --------------------------------------------------

        execute_phase(
            node,
            'PHASE 2: continuous transfer to red bin',
            [
                'pickup_approach',
                'middle',
                'drop_approach',
                'drop_release',
            ],
            poses,
            [
                1.4,
                2.3,
                2.3,
                1.4,
            ],
        )

        suction(
            node,
            'detach',
        )

        # --------------------------------------------------
        # Phase 3
        #
        # Drop release
        # -> drop lift
        # -> middle
        # -> final pickup approach
        # --------------------------------------------------

        execute_phase(
            node,
            'PHASE 3: continuous return above pickup',
            [
                'drop_approach',
                'middle',
                'pickup_approach',
            ],
            poses,
            [
                1.4,
                2.3,
                2.3,
            ],
        )

        node.get_logger().info(
            'RED AUTOMATION COMPLETE.'
        )

        node.get_logger().info(
            'Final position: above pickup zone.'
        )

        return 0

    except KeyboardInterrupt:
        node.get_logger().warning(
            'Automation interrupted.'
        )
        return 130

    except Exception as error:
        node.get_logger().error(
            f'RED AUTOMATION FAILED: {error}'
        )
        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(
        main()
    )
