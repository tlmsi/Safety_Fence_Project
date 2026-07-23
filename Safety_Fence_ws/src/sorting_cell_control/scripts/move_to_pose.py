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
            f'Pose file does not exist: {path}'
        )

    data = json.loads(path.read_text())

    pose_name = str(
        data.get('name', path.stem)
    )

    if 'positions_by_name' in data:
        positions_by_name = data['positions_by_name']

        missing = [
            joint
            for joint in JOINT_NAMES
            if joint not in positions_by_name
        ]

        if missing:
            raise RuntimeError(
                'Pose is missing joints: '
                + ', '.join(missing)
            )

        positions = [
            float(positions_by_name[joint])
            for joint in JOINT_NAMES
        ]

    elif (
        'joint_names' in data
        and 'positions' in data
    ):
        names = list(data['joint_names'])
        values = list(data['positions'])

        if len(names) != len(values):
            raise RuntimeError(
                'joint_names and positions '
                'have different lengths.'
            )

        positions_by_name = {
            str(name): float(value)
            for name, value in zip(names, values)
        }

        missing = [
            joint
            for joint in JOINT_NAMES
            if joint not in positions_by_name
        ]

        if missing:
            raise RuntimeError(
                'Pose is missing joints: '
                + ', '.join(missing)
            )

        positions = [
            positions_by_name[joint]
            for joint in JOINT_NAMES
        ]

    else:
        raise RuntimeError(
            'Pose file must contain either '
            'positions_by_name or '
            'joint_names plus positions.'
        )

    if len(positions) != 6:
        raise RuntimeError(
            'A UR pose must contain six positions.'
        )

    return pose_name, positions


class PoseExecutor(Node):

    def __init__(self) -> None:
        super().__init__('safety_fence_pose_executor')

        self.current_positions: Optional[
            List[float]
        ] = None

        self.joint_subscription = (
            self.create_subscription(
                JointState,
                '/joint_states',
                self.joint_state_callback,
                qos_profile_sensor_data,
            )
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
        values: Dict[str, float] = dict(
            zip(message.name, message.position)
        )

        if not all(
            joint in values
            for joint in JOINT_NAMES
        ):
            return

        self.current_positions = [
            float(values[joint])
            for joint in JOINT_NAMES
        ]

    def wait_for_controller(
        self,
        timeout: float,
    ) -> None:
        self.get_logger().info(
            f'Waiting for {ACTION_NAME}...'
        )

        available = (
            self.action_client.wait_for_server(
                timeout_sec=timeout
            )
        )

        if not available:
            raise RuntimeError(
                'Joint trajectory controller '
                'action server is unavailable.'
            )

        self.get_logger().info(
            'Trajectory controller is ready.'
        )

    def read_current_positions(
        self,
        timeout: float,
    ) -> List[float]:
        self.get_logger().info(
            'Reading current joint positions...'
        )

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
                'No complete /joint_states '
                'message was received.'
            )

        return list(self.current_positions)

    def execute(
        self,
        pose_name: str,
        target: List[float],
        duration: float,
    ) -> None:
        current = self.read_current_positions(
            timeout=15.0
        )

        trajectory = JointTrajectory()
        trajectory.joint_names = list(JOINT_NAMES)

        start_point = JointTrajectoryPoint()
        start_point.positions = current
        start_point.velocities = [0.0] * 6
        start_point.time_from_start = make_duration(
            0.2
        )

        target_point = JointTrajectoryPoint()
        target_point.positions = target
        target_point.velocities = [0.0] * 6
        target_point.time_from_start = make_duration(
            duration
        )

        trajectory.points = [
            start_point,
            target_point,
        ]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = make_duration(
            2.0
        )

        self.get_logger().info(
            f'Moving to pose: {pose_name}'
        )

        for joint, current_value, target_value in zip(
            JOINT_NAMES,
            current,
            target,
        ):
            self.get_logger().info(
                f'  {joint}: '
                f'{current_value:.6f} '
                f'-> {target_value:.6f}'
            )

        send_future = (
            self.action_client.send_goal_async(goal)
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=10.0,
        )

        if not send_future.done():
            raise RuntimeError(
                'Timed out while sending trajectory.'
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

        self.get_logger().info(
            'Trajectory accepted.'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=duration + 15.0,
        )

        if not result_future.done():
            raise RuntimeError(
                'Timed out waiting for movement result.'
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
                'Trajectory action did not finish '
                f'successfully. Status: '
                f'{wrapped_result.status}'
            )

        result = wrapped_result.result

        if result.error_code != 0:
            error_text = (
                result.error_string.strip()
                or 'No explanation provided.'
            )

            raise RuntimeError(
                f'Controller error '
                f'{result.error_code}: '
                f'{error_text}'
            )

        self.get_logger().info(
            f'Pose reached successfully: '
            f'{pose_name}'
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Move the Safety Fence UR robot '
            'to a saved joint pose.'
        )
    )

    parser.add_argument(
        'pose_file',
        type=Path,
        help='JSON pose file.',
    )

    parser.add_argument(
        '--duration',
        type=float,
        default=3.0,
        help='Movement duration in seconds.',
    )

    arguments = parser.parse_args()

    if not 1.0 <= arguments.duration <= 20.0:
        print(
            'ERROR: Duration must be between '
            '1 and 20 seconds.',
            file=sys.stderr,
        )
        return 2

    try:
        pose_name, target = load_pose(
            arguments.pose_file.resolve()
        )

    except Exception as error:
        print(
            f'ERROR: Could not load pose: {error}',
            file=sys.stderr,
        )
        return 1

    rclpy.init()
    node = PoseExecutor()

    try:
        node.wait_for_controller(
            timeout=20.0
        )

        node.execute(
            pose_name=pose_name,
            target=target,
            duration=arguments.duration,
        )

        return 0

    except KeyboardInterrupt:
        node.get_logger().warning(
            'Movement interrupted.'
        )
        return 130

    except Exception as error:
        node.get_logger().error(
            f'MOVEMENT FAILED: {error}'
        )
        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
