#!/usr/bin/env python3

import argparse
import json
import math
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from move_through_poses import (
    ContinuousPoseExecutor,
    load_pose,
)
from blue_automation import load_ik, solve_fixed


HERE = Path(__file__).resolve().parent
CONTROL_DIR = HERE.parent
WORKSPACE = CONTROL_DIR.parent.parent

PATH_FILE = (
    CONTROL_DIR
    / 'config'
    / 'blue_automation_path.json'
)

WORLD_FILE = (
    WORKSPACE
    / 'src'
    / 'sorting_cell_gazebo'
    / 'worlds'
    / 'sorting_cell_world.sdf'
)

STATIC_POSES = (
    'middle',
    'drop_approach',
    'drop_release',
)

PICKUP_SEED_POSE = 'pickup_approach'

# Suction tip clearance above the detected top surface.
PICKUP_CLEARANCE_METRES = 0.15

# Reject clearly invalid perception coordinates before moving the robot.
MAX_HORIZONTAL_SHIFT_METRES = 0.10
MAX_VERTICAL_SHIFT_METRES = 0.025

# Average several ready pose messages to reduce pixel-level jitter.
POSE_SAMPLE_COUNT = 8

# Minimal Gazebo processing pause after attach/detach.
SUCTION_SETTLE_SECONDS = 0.10


class CachedPlan:
    def __init__(
        self,
        poses: Dict[str, List[float]],
        pickup_seed: List[float],
        tool_yaw: float,
        expected_pickup_touch: np.ndarray,
    ) -> None:
        self.poses = poses
        self.pickup_seed = pickup_seed
        self.tool_yaw = tool_yaw
        self.expected_pickup_touch = expected_pickup_touch


def resolve_pose_path(value: object) -> Path:
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


def load_cached_plan() -> CachedPlan:
    if not PATH_FILE.is_file():
        raise RuntimeError(
            'No cached automation path exists. '
            'Run 06_blue_automation.sh plan first.'
        )

    try:
        plan = json.loads(
            PATH_FILE.read_text()
        )

    except Exception as error:
        raise RuntimeError(
            f'Cannot read cached path: {error}'
        ) from error

    pose_files = plan.get('pose_files')

    if not isinstance(pose_files, dict):
        raise RuntimeError(
            f'Invalid pose_files section in {PATH_FILE}'
        )

    required = (
        PICKUP_SEED_POSE,
        *STATIC_POSES,
    )

    loaded: Dict[str, List[float]] = {}

    for key in required:
        if key not in pose_files:
            raise RuntimeError(
                f'Cached path is missing pose: {key}'
            )

        pose_path = resolve_pose_path(
            pose_files[key]
        )

        _, positions = load_pose(pose_path)
        loaded[key] = positions

    geometry = plan.get('geometry')

    if not isinstance(geometry, dict):
        raise RuntimeError(
            'Cached path has no geometry section.'
        )

    touch_value = geometry.get('pickup_touch_xyz')

    if not (
        isinstance(touch_value, list)
        and len(touch_value) == 3
    ):
        raise RuntimeError(
            'Cached path has no valid pickup_touch_xyz.'
        )

    tool_yaw = float(plan.get('tool_yaw_rad'))

    return CachedPlan(
        poses={
            key: loaded[key]
            for key in STATIC_POSES
        },
        pickup_seed=loaded[PICKUP_SEED_POSE],
        tool_yaw=tool_yaw,
        expected_pickup_touch=np.asarray(
            touch_value,
            dtype=float,
        ),
    )


def blue_box_half_height() -> float:
    if not WORLD_FILE.is_file():
        raise RuntimeError(
            f'World file is missing: {WORLD_FILE}'
        )

    root = ET.parse(WORLD_FILE).getroot()

    for model in root.iter('model'):
        if model.attrib.get('name') != 'blue_box':
            continue

        for query in (
            './link/collision/geometry/box/size',
            './link/visual/geometry/box/size',
        ):
            text = model.findtext(query)

            if not text:
                continue

            values = [
                float(value)
                for value in text.split()
            ]

            if len(values) != 3:
                continue

            return values[2] / 2.0

    raise RuntimeError(
        'Could not read the blue-box height from the world.'
    )


def wait_for_blue_box_pose(
    node: ContinuousPoseExecutor,
    timeout: float = 120.0,
) -> np.ndarray:
    state = {
        'ready': False,
        'color': None,
    }

    samples: Deque[np.ndarray] = deque(
        maxlen=POSE_SAMPLE_COUNT
    )

    def ready_callback(message: Bool) -> None:
        ready = bool(message.data)

        if not ready:
            samples.clear()

        state['ready'] = ready

    def color_callback(message: String) -> None:
        color = message.data.strip().lower()
        state['color'] = color

        if color != 'blue':
            samples.clear()

    def pose_callback(message: PoseStamped) -> None:
        if not state['ready']:
            return

        if state['color'] != 'blue':
            return

        if message.header.frame_id not in ('', 'world'):
            node.get_logger().warning(
                'Ignoring box pose in unexpected frame: '
                f'{message.header.frame_id}'
            )
            return

        point = np.array(
            [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ],
            dtype=float,
        )

        if not np.all(np.isfinite(point)):
            return

        samples.append(point)

    subscriptions = [
        node.create_subscription(
            Bool,
            '/perception/object_in_pickup_zone',
            ready_callback,
            10,
        ),
        node.create_subscription(
            String,
            '/perception/detected_color',
            color_callback,
            10,
        ),
        node.create_subscription(
            PoseStamped,
            '/perception/box_pose',
            pose_callback,
            10,
        ),
    ]

    node.get_logger().info(
        'Waiting for a settled BLUE box and live /perception/box_pose...'
    )

    deadline = time.monotonic() + timeout

    while (
        rclpy.ok()
        and len(samples) < POSE_SAMPLE_COUNT
        and time.monotonic() < deadline
    ):
        rclpy.spin_once(
            node,
            timeout_sec=0.1,
        )

    for subscription in subscriptions:
        node.destroy_subscription(subscription)

    if len(samples) < POSE_SAMPLE_COUNT:
        color = state['color'] or 'unknown'
        raise RuntimeError(
            'Timed out waiting for a settled blue-box pose. '
            f'Last ready state={state["ready"]}, color={color}, '
            f'pose samples={len(samples)}/{POSE_SAMPLE_COUNT}.'
        )

    stacked = np.vstack(list(samples))
    mean = np.mean(stacked, axis=0)
    spread = np.max(stacked, axis=0) - np.min(stacked, axis=0)

    node.get_logger().info(
        'Averaged detected blue-box centre: '
        f'x={mean[0]:.4f}, y={mean[1]:.4f}, z={mean[2]:.4f}'
    )

    node.get_logger().info(
        'Pose sample spread: '
        f'dx={spread[0] * 1000.0:.2f} mm, '
        f'dy={spread[1] * 1000.0:.2f} mm, '
        f'dz={spread[2] * 1000.0:.2f} mm'
    )

    return mean


def validate_detected_pose(
    box_center: np.ndarray,
    expected_touch: np.ndarray,
    half_height: float,
) -> None:
    expected_center = expected_touch.copy()
    expected_center[2] -= half_height

    horizontal_shift = float(
        np.linalg.norm(
            box_center[:2]
            - expected_center[:2]
        )
    )

    vertical_shift = abs(
        float(
            box_center[2]
            - expected_center[2]
        )
    )

    if horizontal_shift > MAX_HORIZONTAL_SHIFT_METRES:
        raise RuntimeError(
            'Detected box is too far from the pickup workspace: '
            f'horizontal shift={horizontal_shift:.3f} m, '
            f'limit={MAX_HORIZONTAL_SHIFT_METRES:.3f} m.'
        )

    if vertical_shift > MAX_VERTICAL_SHIFT_METRES:
        raise RuntimeError(
            'Detected box height is inconsistent with the conveyor: '
            f'vertical shift={vertical_shift:.3f} m, '
            f'limit={MAX_VERTICAL_SHIFT_METRES:.3f} m.'
        )


def solve_dynamic_pickup(
    node: ContinuousPoseExecutor,
    cached: CachedPlan,
    box_center: np.ndarray,
    half_height: float,
) -> Tuple[List[float], List[float], np.ndarray, np.ndarray]:
    validate_detected_pose(
        box_center,
        cached.expected_pickup_touch,
        half_height,
    )

    pickup_touch = box_center.copy()
    pickup_touch[2] += half_height

    pickup_approach = pickup_touch.copy()
    pickup_approach[2] += PICKUP_CLEARANCE_METRES

    node.get_logger().info(
        'Dynamic suction-tip targets:'
    )
    node.get_logger().info(
        f'  approach: x={pickup_approach[0]:.4f}, '
        f'y={pickup_approach[1]:.4f}, '
        f'z={pickup_approach[2]:.4f}'
    )
    node.get_logger().info(
        f'  touch:    x={pickup_touch[0]:.4f}, '
        f'y={pickup_touch[1]:.4f}, '
        f'z={pickup_touch[2]:.4f}'
    )

    ik = load_ik()

    chain, lower, upper = ik.build_chain(
        ik.generate_robot_urdf()
    )

    # Preserve the shoulder range already used by the verified planner.
    lower[1] = max(
        lower[1],
        math.radians(-135.0),
    )
    upper[1] = min(
        upper[1],
        math.radians(-45.0),
    )

    seed = np.asarray(
        cached.pickup_seed,
        dtype=float,
    )

    node.get_logger().info(
        'Solving live IK for detected pickup approach...'
    )

    (
        approach_joints,
        approach_position_error,
        approach_orientation_error,
    ) = solve_fixed(
        ik,
        chain,
        lower,
        upper,
        seed,
        pickup_approach,
        cached.tool_yaw,
    )

    node.get_logger().info(
        'Solving live IK for detected pickup touch...'
    )

    (
        touch_joints,
        touch_position_error,
        touch_orientation_error,
    ) = solve_fixed(
        ik,
        chain,
        lower,
        upper,
        approach_joints,
        pickup_touch,
        cached.tool_yaw,
    )

    node.get_logger().info(
        'Live pickup IK validated: '
        f'approach error={approach_position_error * 1000.0:.2f} mm, '
        f'touch error={touch_position_error * 1000.0:.2f} mm, '
        f'approach orientation={math.degrees(approach_orientation_error):.2f} deg, '
        f'touch orientation={math.degrees(touch_orientation_error):.2f} deg'
    )

    return (
        approach_joints.tolist(),
        touch_joints.tolist(),
        pickup_approach,
        pickup_touch,
    )


def suction(
    node: ContinuousPoseExecutor,
    action: str,
) -> None:
    if action not in ('attach', 'detach'):
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
                f'/suction/blue/{action}',
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

    time.sleep(SUCTION_SETTLE_SECONDS)


def execute_phase(
    node: ContinuousPoseExecutor,
    label: str,
    pose_names: List[str],
    poses: Dict[str, List[float]],
    segment_times: List[float],
) -> None:
    node.get_logger().info(label)

    node.execute(
        pose_names,
        [poses[name] for name in pose_names],
        segment_times,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Run blue automation using the live detected box pose.'
        )
    )
    parser.add_argument(
        '--solve-only',
        action='store_true',
        help=(
            'Wait for the blue box and validate live pickup IK '
            'without moving the robot.'
        ),
    )
    arguments = parser.parse_args()

    try:
        cached = load_cached_plan()
        half_height = blue_box_half_height()

    except Exception as error:
        print(
            f'ERROR: {error}',
            file=sys.stderr,
        )
        return 1

    rclpy.init()
    node = ContinuousPoseExecutor()

    try:
        node.get_logger().info(
            'Loaded cached transfer/drop path.'
        )
        node.get_logger().info(
            'Pickup joint poses will be solved from live perception.'
        )
        node.get_logger().info(
            'Persistent execution node started.'
        )

        node.wait_for_controller(timeout=30.0)

        box_center = wait_for_blue_box_pose(node)

        (
            pickup_approach_joints,
            pickup_touch_joints,
            pickup_approach,
            pickup_touch,
        ) = solve_dynamic_pickup(
            node,
            cached,
            box_center,
            half_height,
        )

        poses = dict(cached.poses)
        poses['pickup_approach'] = pickup_approach_joints
        poses['pickup_touch'] = pickup_touch_joints

        if arguments.solve_only:
            node.get_logger().info(
                'LIVE PICKUP CHECK PASSED. Robot was not moved.'
            )
            node.get_logger().info(
                'Run 06_blue_automation.sh run to execute the pickup.'
            )
            return 0

        execute_phase(
            node,
            'PHASE 1: continuous approach to live detected pickup',
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

        suction(node, 'attach')

        execute_phase(
            node,
            'PHASE 2: continuous transfer to blue bin',
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

        suction(node, 'detach')

        execute_phase(
            node,
            'PHASE 3: continuous return above live pickup',
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
            'BLUE AUTOMATION COMPLETE.'
        )
        node.get_logger().info(
            'Final position: above the detected pickup point.'
        )
        node.get_logger().info(
            'Used detected box centre: '
            f'x={box_center[0]:.4f}, '
            f'y={box_center[1]:.4f}, '
            f'z={box_center[2]:.4f}'
        )
        node.get_logger().info(
            'Used suction touch target: '
            f'x={pickup_touch[0]:.4f}, '
            f'y={pickup_touch[1]:.4f}, '
            f'z={pickup_touch[2]:.4f}'
        )
        node.get_logger().info(
            'Used pickup approach target: '
            f'x={pickup_approach[0]:.4f}, '
            f'y={pickup_approach[1]:.4f}, '
            f'z={pickup_approach[2]:.4f}'
        )

        return 0

    except KeyboardInterrupt:
        node.get_logger().warning(
            'Automation interrupted.'
        )
        return 130

    except Exception as error:
        node.get_logger().error(
            f'BLUE AUTOMATION FAILED: {error}'
        )
        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
