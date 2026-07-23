#!/usr/bin/env python3

import argparse
import sys
import time
from typing import Dict, List, Optional

import rclpy

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


def duration_message(seconds: float) -> Duration:
    whole_seconds = int(seconds)
    nanoseconds = int(
        round((seconds - whole_seconds) * 1_000_000_000)
    )

    if nanoseconds >= 1_000_000_000:
        whole_seconds += 1
        nanoseconds -= 1_000_000_000

    return Duration(
        sec=whole_seconds,
        nanosec=nanoseconds,
    )


class RobotJointSmokeTest(Node):

    def __init__(self) -> None:
        super().__init__('robot_joint_smoke_test')

        self.current_positions: Optional[List[float]] = None

        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data,
        )

        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            ACTION_NAME,
        )

    def joint_state_callback(
        self,
        message: JointState,
    ) -> None:
        position_by_name: Dict[str, float] = dict(
            zip(message.name, message.position)
        )

        if not all(
            name in position_by_name
            for name in JOINT_NAMES
        ):
            return

        self.current_positions = [
            float(position_by_name[name])
            for name in JOINT_NAMES
        ]

    def wait_for_joint_state(
        self,
        timeout_seconds: float,
    ) -> List[float]:
        self.get_logger().info(
            'Waiting for /joint_states...'
        )

        deadline = time.monotonic() + timeout_seconds

        while (
            rclpy.ok()
            and time.monotonic() < deadline
            and self.current_positions is None
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.2,
            )

        if self.current_positions is None:
            raise RuntimeError(
                'No complete UR joint state was received.'
            )

        positions = list(self.current_positions)

        self.get_logger().info(
            'Current robot joint positions received.'
        )

        for name, value in zip(
            JOINT_NAMES,
            positions,
        ):
            self.get_logger().info(
                f'  {name}: {value:.6f} rad'
            )

        return positions

    def wait_for_controller(
        self,
        timeout_seconds: float,
    ) -> None:
        self.get_logger().info(
            f'Waiting for action server: {ACTION_NAME}'
        )

        if not self.trajectory_client.wait_for_server(
            timeout_sec=timeout_seconds
        ):
            raise RuntimeError(
                'The joint trajectory action server '
                'is unavailable.'
            )

        self.get_logger().info(
            'Robot trajectory controller is ready.'
        )

    def execute_positions(
        self,
        positions: List[float],
        movement_seconds: float,
        description: str,
    ) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = [
            float(value)
            for value in positions
        ]

        point.velocities = [0.0] * len(JOINT_NAMES)
        point.time_from_start = duration_message(
            movement_seconds
        )

        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = duration_message(2.0)

        self.get_logger().info(description)

        send_future = (
            self.trajectory_client.send_goal_async(goal)
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=10.0,
        )

        if not send_future.done():
            raise RuntimeError(
                'Timed out while sending the trajectory.'
            )

        goal_handle = send_future.result()

        if goal_handle is None:
            raise RuntimeError(
                'The controller returned no goal handle.'
            )

        if not goal_handle.accepted:
            raise RuntimeError(
                'The controller rejected the trajectory.'
            )

        self.get_logger().info(
            'Trajectory accepted.'
        )

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=movement_seconds + 10.0,
        )

        if not result_future.done():
            raise RuntimeError(
                'Timed out waiting for trajectory result.'
            )

        wrapped_result = result_future.result()

        if wrapped_result is None:
            raise RuntimeError(
                'The trajectory returned no result.'
            )

        result = wrapped_result.result

        if result.error_code != 0:
            error_text = result.error_string.strip()

            if not error_text:
                error_text = 'No controller explanation provided.'

            raise RuntimeError(
                f'Trajectory failed with error code '
                f'{result.error_code}: {error_text}'
            )

        self.get_logger().info(
            'Trajectory completed successfully.'
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Perform a small wrist_3 movement and '
            'return to the exact starting pose.'
        )
    )

    parser.add_argument(
        '--delta',
        type=float,
        default=0.12,
        help='Wrist movement in radians.',
    )

    parser.add_argument(
        '--duration',
        type=float,
        default=2.5,
        help='Seconds for each movement.',
    )

    arguments = parser.parse_args()

    if not 0.02 <= abs(arguments.delta) <= 0.25:
        print(
            'ERROR: --delta must be between '
            '0.02 and 0.25 radians.',
            file=sys.stderr,
        )
        return 2

    if not 1.0 <= arguments.duration <= 10.0:
        print(
            'ERROR: --duration must be between '
            '1.0 and 10.0 seconds.',
            file=sys.stderr,
        )
        return 2

    rclpy.init()
    node = RobotJointSmokeTest()

    try:
        node.wait_for_controller(20.0)

        start_positions = node.wait_for_joint_state(
            20.0
        )

        test_positions = list(start_positions)

        wrist_index = JOINT_NAMES.index(
            'wrist_3_joint'
        )

        current_wrist = start_positions[wrist_index]

        # Move toward zero, reducing the chance of approaching
        # a joint limit.
        direction = -1.0 if current_wrist > 0.0 else 1.0

        test_positions[wrist_index] = (
            current_wrist
            + direction * abs(arguments.delta)
        )

        node.get_logger().info(
            'Phase 1 test movement:'
        )

        node.get_logger().info(
            f'  wrist_3 start: '
            f'{current_wrist:.6f} rad'
        )

        node.get_logger().info(
            f'  wrist_3 target: '
            f'{test_positions[wrist_index]:.6f} rad'
        )

        node.execute_positions(
            test_positions,
            arguments.duration,
            'Moving wrist_3_joint slightly...',
        )

        time.sleep(0.5)

        node.execute_positions(
            start_positions,
            arguments.duration,
            'Returning to the original joint pose...',
        )

        node.get_logger().info(
            'PHASE 1 PASSED: '
            'robot moved and returned successfully.'
        )

        return 0

    except KeyboardInterrupt:
        node.get_logger().warning(
            'Robot movement test interrupted.'
        )
        return 130

    except Exception as error:
        node.get_logger().error(
            f'PHASE 1 FAILED: {error}'
        )
        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
