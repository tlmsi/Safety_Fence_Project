#!/usr/bin/env python3

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict

import rclpy

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


GROUP_NAME = 'ur_manipulator'

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


def load_pose(path: Path) -> tuple[str, Dict[str, float]]:
    if not path.is_file():
        raise RuntimeError(
            f'Pose file does not exist: {path}'
        )

    data = json.loads(
        path.read_text()
    )

    pose_name = str(
        data.get('name', path.stem)
    )

    if 'positions_by_name' in data:
        positions_by_name = {
            str(name): float(value)
            for name, value
            in data['positions_by_name'].items()
        }

    elif (
        'joint_names' in data
        and 'positions' in data
    ):
        names = list(
            data['joint_names']
        )

        positions = list(
            data['positions']
        )

        if len(names) != len(positions):
            raise RuntimeError(
                'joint_names and positions have '
                'different lengths.'
            )

        positions_by_name = {
            str(name): float(value)
            for name, value
            in zip(names, positions)
        }

    else:
        raise RuntimeError(
            'Pose JSON must contain positions_by_name '
            'or joint_names plus positions.'
        )

    missing = [
        name
        for name in JOINT_NAMES
        if name not in positions_by_name
    ]

    if missing:
        raise RuntimeError(
            'Pose is missing joints: '
            + ', '.join(missing)
        )

    return pose_name, positions_by_name


class MoveItPoseExecutor(Node):

    def __init__(
        self,
        pose_name: str,
        target: Dict[str, float],
        plan_only: bool,
        velocity: float,
        acceleration: float,
    ) -> None:
        super().__init__(
            'sorting_cell_moveit_pose_executor'
        )

        self.pose_name = pose_name
        self.target = target
        self.plan_only = plan_only
        self.velocity = velocity
        self.acceleration = acceleration

        self.current_positions: Dict[
            str,
            float,
        ] = {}

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data,
        )

        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
        )

    def joint_state_callback(
        self,
        message: JointState,
    ) -> None:
        self.current_positions.update(
            {
                name: float(position)
                for name, position
                in zip(
                    message.name,
                    message.position,
                )
            }
        )

    def wait_for_joint_state(
        self,
        timeout: float = 10.0,
    ) -> bool:
        deadline = (
            time.monotonic()
            + timeout
        )

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.2,
            )

            if all(
                name in self.current_positions
                for name in JOINT_NAMES
            ):
                return True

        return False

    def create_goal(
        self,
    ) -> MoveGroup.Goal:
        constraints = Constraints()
        constraints.name = self.pose_name

        for name in JOINT_NAMES:
            joint_constraint = JointConstraint()

            joint_constraint.joint_name = name
            joint_constraint.position = (
                self.target[name]
            )

            joint_constraint.tolerance_above = 0.001
            joint_constraint.tolerance_below = 0.001
            joint_constraint.weight = 1.0

            constraints.joint_constraints.append(
                joint_constraint
            )

        goal = MoveGroup.Goal()

        request = goal.request
        request.group_name = GROUP_NAME
        request.pipeline_id = 'ompl'
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0

        request.max_velocity_scaling_factor = (
            self.velocity
        )

        request.max_acceleration_scaling_factor = (
            self.acceleration
        )

        # Use MoveIt's live monitored state.
        request.start_state.is_diff = True

        request.goal_constraints.append(
            constraints
        )

        goal.planning_options.plan_only = (
            self.plan_only
        )

        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        return goal

    def print_target(self) -> None:
        print(
            f'\nTarget pose: {self.pose_name}'
        )

        print(
            '\nCurrent and target joint positions:'
        )

        for name in JOINT_NAMES:
            current = self.current_positions[name]
            target = self.target[name]

            print(
                f'  {name:24s}'
                f' current={current: .6f}'
                f' target={target: .6f}'
                f' change={target - current: .6f}'
            )

    def run(self) -> bool:
        self.get_logger().info(
            'Waiting for /joint_states...'
        )

        if not self.wait_for_joint_state():
            self.get_logger().error(
                'No complete UR joint state received.'
            )
            return False

        self.print_target()

        self.get_logger().info(
            'Waiting for MoveIt /move_action...'
        )

        if not (
            self.move_group_client.wait_for_server(
                timeout_sec=15.0
            )
        ):
            self.get_logger().error(
                'MoveIt /move_action is unavailable.'
            )
            return False

        mode = (
            'PLAN ONLY'
            if self.plan_only
            else 'PLAN AND EXECUTE'
        )

        self.get_logger().info(
            f'Sending {mode} request '
            f'for pose "{self.pose_name}".'
        )

        send_future = (
            self.move_group_client.send_goal_async(
                self.create_goal()
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
        )

        goal_handle = send_future.result()

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            self.get_logger().error(
                'MoveIt rejected the request.'
            )
            return False

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error(
                'MoveIt returned no result.'
            )
            return False

        result = wrapped_result.result

        error_code = (
            result.error_code.val
        )

        trajectory = (
            result.planned_trajectory
            .joint_trajectory
        )

        point_count = len(
            trajectory.points
        )

        duration = 0.0

        if trajectory.points:
            final_time = (
                trajectory.points[-1]
                .time_from_start
            )

            duration = (
                float(final_time.sec)
                + float(final_time.nanosec)
                / 1_000_000_000.0
            )

        print('\n=== MoveIt result ===')
        print(
            f'Error code:          {error_code}'
        )
        print(
            f'Planning time:       '
            f'{result.planning_time:.4f} s'
        )
        print(
            f'Trajectory points:   {point_count}'
        )
        print(
            f'Trajectory duration: {duration:.4f} s'
        )

        if (
            error_code
            != MoveItErrorCodes.SUCCESS
        ):
            self.get_logger().error(
                f'MoveIt failed with error '
                f'code {error_code}.'
            )
            return False

        if (
            point_count <= 1
            or duration <= 0.0
        ):
            self.get_logger().error(
                'MoveIt returned a zero-motion '
                'trajectory.'
            )
            return False

        if self.plan_only:
            self.get_logger().info(
                'Planning succeeded. '
                'The robot was not moved.'
            )
        else:
            self.get_logger().info(
                'Planning and execution succeeded.'
            )

        return True


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Plan or execute a saved joint pose '
            'through MoveIt.'
        )
    )

    parser.add_argument(
        '--pose',
        required=True,
        type=Path,
    )

    parser.add_argument(
        '--plan-only',
        action='store_true',
    )

    parser.add_argument(
        '--velocity',
        type=float,
        default=0.10,
    )

    parser.add_argument(
        '--acceleration',
        type=float,
        default=0.10,
    )

    arguments, _ = (
        parser.parse_known_args()
    )

    for name in (
        'velocity',
        'acceleration',
    ):
        value = getattr(
            arguments,
            name,
        )

        if not 0.0 < value <= 1.0:
            parser.error(
                f'--{name} must be greater '
                'than 0 and at most 1.'
            )

    return arguments


def main() -> int:
    arguments = parse_arguments()

    try:
        pose_name, target = load_pose(
            arguments.pose.expanduser().resolve()
        )

    except Exception as error:
        print(
            f'ERROR: {error}',
            file=sys.stderr,
        )
        return 1

    rclpy.init()

    node = MoveItPoseExecutor(
        pose_name=pose_name,
        target=target,
        plan_only=arguments.plan_only,
        velocity=arguments.velocity,
        acceleration=arguments.acceleration,
    )

    try:
        success = node.run()
        return 0 if success else 1

    except KeyboardInterrupt:
        node.get_logger().warning(
            'Interrupted.'
        )
        return 130

    except Exception as error:
        node.get_logger().exception(
            f'MoveIt pose command failed: {error}'
        )
        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
