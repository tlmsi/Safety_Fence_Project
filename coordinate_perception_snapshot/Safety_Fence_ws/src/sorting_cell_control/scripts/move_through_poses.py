#!/usr/bin/env python3

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)


JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

ACTION_NAME = (
    '/joint_trajectory_controller/'
    'follow_joint_trajectory'
)


def make_duration(seconds: float) -> Duration:
    whole_seconds = int(seconds)

    nanoseconds = int(
        round(
            (seconds - whole_seconds)
            * 1_000_000_000
        )
    )

    if nanoseconds >= 1_000_000_000:
        whole_seconds += 1
        nanoseconds -= 1_000_000_000

    return Duration(
        sec=whole_seconds,
        nanosec=nanoseconds,
    )


def load_pose(path: Path) -> tuple[str, List[float]]:
    if not path.is_file():
        raise RuntimeError(
            f'Pose file not found: {path}'
        )

    data = json.loads(
        path.read_text()
    )

    name = str(
        data.get(
            'name',
            path.stem,
        )
    )

    if 'positions_by_name' in data:
        by_name = data[
            'positions_by_name'
        ]

    else:
        by_name = dict(
            zip(
                data['joint_names'],
                data['positions'],
            )
        )

    missing = [
        joint
        for joint in JOINT_NAMES
        if joint not in by_name
    ]

    if missing:
        raise RuntimeError(
            'Pose is missing joints: '
            + ', '.join(missing)
        )

    positions = [
        float(by_name[joint])
        for joint in JOINT_NAMES
    ]

    return name, positions


class ContinuousPoseExecutor(Node):

    def __init__(self) -> None:
        super().__init__(
            'continuous_pose_executor'
        )

        self.current_positions: Optional[
            List[float]
        ] = None

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data,
        )

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            ACTION_NAME,
        )

    def joint_state_callback(
        self,
        message: JointState,
    ) -> None:
        available: Dict[str, float] = dict(
            zip(
                message.name,
                message.position,
            )
        )

        if not all(
            joint in available
            for joint in JOINT_NAMES
        ):
            return

        self.current_positions = [
            float(available[joint])
            for joint in JOINT_NAMES
        ]

    def wait_for_controller(
        self,
        timeout: float,
    ) -> None:
        self.get_logger().info(
            f'Waiting for {ACTION_NAME}...'
        )

        if not self.action_client.wait_for_server(
            timeout_sec=timeout
        ):
            raise RuntimeError(
                'Trajectory controller unavailable.'
            )

    def read_current_positions(
        self,
        timeout: float,
    ) -> List[float]:
        self.current_positions = None
        deadline = time.monotonic() + timeout

        while (
            rclpy.ok()
            and self.current_positions is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.2,
            )

        if self.current_positions is None:
            raise RuntimeError(
                'No complete /joint_states message received.'
            )

        return list(
            self.current_positions
        )

    def execute(
        self,
        pose_names: List[str],
        pose_positions: List[List[float]],
        segment_times: List[float],
    ) -> None:
        current = self.read_current_positions(
            timeout=15.0
        )

        trajectory = JointTrajectory()
        trajectory.joint_names = list(
            JOINT_NAMES
        )

        # The first point anchors the trajectory to the
        # robot's actual current position.
        start_point = JointTrajectoryPoint()
        start_point.positions = current
        start_point.time_from_start = (
            make_duration(0.15)
        )

        trajectory.points.append(
            start_point
        )

        cumulative_time = 0.15

        for (
            pose_name,
            positions,
            segment_time,
        ) in zip(
            pose_names,
            pose_positions,
            segment_times,
        ):
            cumulative_time += segment_time

            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = make_duration(
                cumulative_time
            )

            # Velocities are intentionally omitted.
            # This allows smooth interpolation through
            # intermediate points instead of commanding
            # zero velocity at the middle waypoint.
            trajectory.points.append(
                point
            )

            self.get_logger().info(
                f'Pass-through waypoint: '
                f'{pose_name} at '
                f'{cumulative_time:.2f} s'
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = (
            make_duration(2.0)
        )

        self.get_logger().info(
            'Sending continuous multi-waypoint trajectory...'
        )

        send_future = (
            self.action_client.send_goal_async(
                goal
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=10.0,
        )

        if not send_future.done():
            raise RuntimeError(
                'Timed out sending trajectory.'
            )

        goal_handle = send_future.result()

        if goal_handle is None:
            raise RuntimeError(
                'Controller returned no goal handle.'
            )

        if not goal_handle.accepted:
            raise RuntimeError(
                'Controller rejected the trajectory.'
            )

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=cumulative_time + 10.0,
        )

        if not result_future.done():
            raise RuntimeError(
                'Timed out waiting for trajectory result.'
            )

        wrapped_result = result_future.result()

        if wrapped_result is None:
            raise RuntimeError(
                'Controller returned no result.'
            )

        if (
            wrapped_result.status
            != GoalStatus.STATUS_SUCCEEDED
        ):
            raise RuntimeError(
                'Continuous trajectory did not succeed. '
                f'Action status: '
                f'{wrapped_result.status}'
            )

        result = wrapped_result.result

        if result.error_code != 0:
            error_text = (
                result.error_string.strip()
                or 'No controller explanation.'
            )

            raise RuntimeError(
                f'Controller error '
                f'{result.error_code}: '
                f'{error_text}'
            )

        self.get_logger().info(
            'Continuous trajectory completed.'
        )


def parse_times(
    text: str,
    expected_count: int,
) -> List[float]:
    try:
        values = [
            float(value.strip())
            for value in text.split(',')
        ]

    except ValueError as error:
        raise RuntimeError(
            'Invalid --segment-times value.'
        ) from error

    if len(values) != expected_count:
        raise RuntimeError(
            '--segment-times must contain one value '
            'for each pose file.'
        )

    if any(
        value < 0.8 or value > 10.0
        for value in values
    ):
        raise RuntimeError(
            'Each segment time must be between '
            '0.8 and 10.0 seconds.'
        )

    return values


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Move continuously through multiple '
            'saved robot joint poses.'
        )
    )

    parser.add_argument(
        'pose_files',
        nargs='+',
        type=Path,
    )

    parser.add_argument(
        '--segment-times',
        required=True,
        help=(
            'Comma-separated duration for each segment, '
            'for example: 2.7,2.7'
        ),
    )

    arguments = parser.parse_args()

    try:
        segment_times = parse_times(
            arguments.segment_times,
            len(arguments.pose_files),
        )

        pose_names: List[str] = []
        pose_positions: List[List[float]] = []

        for pose_file in arguments.pose_files:
            name, positions = load_pose(
                pose_file.resolve()
            )

            pose_names.append(name)
            pose_positions.append(positions)

    except Exception as error:
        print(
            f'ERROR: {error}',
            file=sys.stderr,
        )
        return 1

    rclpy.init()
    node = ContinuousPoseExecutor()

    try:
        node.wait_for_controller(
            timeout=20.0
        )

        node.execute(
            pose_names,
            pose_positions,
            segment_times,
        )

        return 0

    except KeyboardInterrupt:
        node.get_logger().warning(
            'Continuous movement interrupted.'
        )
        return 130

    except Exception as error:
        node.get_logger().error(
            f'CONTINUOUS MOVEMENT FAILED: {error}'
        )
        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
