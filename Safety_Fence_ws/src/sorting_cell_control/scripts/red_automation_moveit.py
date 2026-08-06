#!/usr/bin/env python3

import argparse
import sys
import time
from typing import Dict, List

import rclpy

from moveit_msgs.action import (
    MoveGroup,
    MoveGroupSequence,
)
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    MotionSequenceItem,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from moveit_attached_box import CarriedBoxSceneManager

from red_automation_runtime import (
    load_cached_plan,
    red_box_half_height,
    solve_dynamic_pickup,
    suction,
    wait_for_red_box_pose,
)


GROUP_NAME = 'ur_manipulator'
SEQUENCE_BLEND_RADIUS = 0.03

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


class RedMoveItRuntime(Node):

    def __init__(
        self,
        velocity: float,
        acceleration: float,
    ) -> None:
        super().__init__(
            'red_automation_moveit_runtime'
        )

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

        self.sequence_client = ActionClient(
            self,
            MoveGroupSequence,
            '/sequence_move_group',
        )

        self.carried_box = (
            CarriedBoxSceneManager(self)
        )

    def joint_state_callback(
        self,
        message: JointState,
    ) -> None:
        self.current_positions.update(
            {
                name: float(position)
                for name, position in zip(
                    message.name,
                    message.position,
                )
            }
        )

    def wait_for_joint_state(
        self,
        timeout: float = 15.0,
    ) -> None:
        deadline = time.monotonic() + timeout

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
                return

        raise RuntimeError(
            'No complete UR /joint_states message received.'
        )

    def wait_for_moveit(
        self,
        timeout: float = 30.0,
    ) -> None:
        self.get_logger().info(
            'Waiting for MoveIt /move_action...'
        )

        if not self.move_group_client.wait_for_server(
            timeout_sec=timeout
        ):
            raise RuntimeError(
                'MoveIt /move_action is unavailable.'
            )

        self.get_logger().info(
            'MoveIt action server is ready.'
        )

        self.get_logger().info(
            'Waiting for MoveIt /sequence_move_group...'
        )

        if not self.sequence_client.wait_for_server(
            timeout_sec=timeout
        ):
            raise RuntimeError(
                'MoveIt /sequence_move_group is unavailable.'
            )

        self.get_logger().info(
            'MoveIt sequence action server is ready.'
        )

    def create_goal(
        self,
        pose_name: str,
        positions: List[float],
    ) -> MoveGroup.Goal:
        if len(positions) != len(JOINT_NAMES):
            raise RuntimeError(
                f'Pose "{pose_name}" does not contain '
                'six joint positions.'
            )

        constraints = Constraints()
        constraints.name = pose_name

        for name, position in zip(
            JOINT_NAMES,
            positions,
        ):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(position)
            constraint.tolerance_above = 0.001
            constraint.tolerance_below = 0.001
            constraint.weight = 1.0

            constraints.joint_constraints.append(
                constraint
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

        # Use the live monitored robot position as start state.
        request.start_state.is_diff = True

        request.goal_constraints.append(
            constraints
        )

        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        return goal

    def execute_pose(
        self,
        pose_name: str,
        positions: List[float],
    ) -> None:
        self.wait_for_joint_state()

        self.get_logger().info(
            f'Planning and executing pose: {pose_name}'
        )

        send_future = (
            self.move_group_client.send_goal_async(
                self.create_goal(
                    pose_name,
                    positions,
                )
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
            raise RuntimeError(
                f'MoveIt rejected pose: {pose_name}'
            )

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        wrapped_result = result_future.result()

        if wrapped_result is None:
            raise RuntimeError(
                f'MoveIt returned no result for: {pose_name}'
            )

        result = wrapped_result.result
        error_code = result.error_code.val

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

        self.get_logger().info(
            f'MoveIt result for {pose_name}: '
            f'error={error_code}, '
            f'points={point_count}, '
            f'duration={duration:.3f} s'
        )

        if error_code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f'MoveIt failed for "{pose_name}" '
                f'with error code {error_code}.'
            )

        if point_count <= 1 or duration <= 0.0:
            raise RuntimeError(
                f'MoveIt returned a zero-motion trajectory '
                f'for "{pose_name}".'
            )

    def execute_sequence(
        self,
        label: str,
        pose_names: List[str],
        poses: Dict[str, List[float]],
    ) -> None:
        self.get_logger().info(label)

        # The pickup phase remains unchanged because suction must
        # be activated only after reaching pickup_touch.
        if 'middle' not in pose_names:
            for pose_name in pose_names:
                self.execute_pose(
                    pose_name,
                    poses[pose_name],
                )

            return

        self.wait_for_joint_state()

        self.get_logger().info(
            'Planning and executing one blended sequence.'
        )

        goal = MoveGroupSequence.Goal()

        final_index = len(pose_names) - 1

        for index, pose_name in enumerate(pose_names):
            move_goal = self.create_goal(
                pose_name,
                poses[pose_name],
            )

            item = MotionSequenceItem()
            item.req = move_goal.request

            # Blended motion sequences are provided by the
            # Pilz industrial motion-planning pipeline.
            item.req.pipeline_id = (
                'pilz_industrial_motion_planner'
            )
            item.req.planner_id = 'PTP'

            # Only the first request may specify the start state.
            # Every later segment begins at the preceding goal.
            if index > 0:
                item.req.start_state.is_diff = False

            if index < final_index:
                item.blend_radius = (
                    SEQUENCE_BLEND_RADIUS
                )
                waypoint_type = 'BYPASS'
            else:
                item.blend_radius = 0.0
                waypoint_type = 'STOP'

            self.get_logger().info(
                f'  {pose_name}: {waypoint_type}, '
                f'blend={item.blend_radius:.3f} m'
            )

            goal.request.items.append(item)

        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        send_future = (
            self.sequence_client.send_goal_async(goal)
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
            raise RuntimeError(
                f'MoveIt rejected sequence: {label}'
            )

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        wrapped_result = result_future.result()

        if wrapped_result is None:
            raise RuntimeError(
                f'MoveIt returned no sequence result: {label}'
            )

        response = wrapped_result.result.response
        error_code = response.error_code.val

        trajectory_count = len(
            response.planned_trajectories
        )

        point_count = sum(
            len(
                trajectory.joint_trajectory.points
            )
            for trajectory in (
                response.planned_trajectories
            )
        )

        self.get_logger().info(
            f'MoveIt sequence result: '
            f'error={error_code}, '
            f'trajectories={trajectory_count}, '
            f'points={point_count}, '
            f'planning_time={response.planning_time:.3f} s'
        )

        if error_code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f'MoveIt sequence failed with '
                f'error code {error_code}.'
            )



def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Execute the red sorting cycle through MoveIt.'
        )
    )

    parser.add_argument(
        '--solve-only',
        action='store_true',
        help=(
            'Validate perception and dynamic pickup IK '
            'without moving the robot.'
        ),
    )

    parser.add_argument(
        '--velocity',
        type=float,
        default=0.15,
    )

    parser.add_argument(
        '--acceleration',
        type=float,
        default=0.15,
    )

    arguments = parser.parse_args()

    for name in (
        'velocity',
        'acceleration',
    ):
        value = getattr(arguments, name)

        if not 0.0 < value <= 1.0:
            parser.error(
                f'--{name} must be greater than 0 '
                'and no greater than 1.'
            )

    return arguments


def main() -> int:
    arguments = parse_arguments()

    try:
        cached = load_cached_plan()
        half_height = red_box_half_height()

    except Exception as error:
        print(
            f'ERROR: {error}',
            file=sys.stderr,
        )
        return 1

    rclpy.init()

    node = RedMoveItRuntime(
        velocity=arguments.velocity,
        acceleration=arguments.acceleration,
    )

    try:
        node.wait_for_joint_state()
        node.wait_for_moveit()
        node.carried_box.wait_for_services()

        node.get_logger().info(
            'Waiting for the live red-box pose.'
        )

        box_center = wait_for_red_box_pose(
            node
        )

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

        poses = dict(
            cached.poses
        )

        poses['pickup_approach'] = (
            pickup_approach_joints
        )

        poses['pickup_touch'] = (
            pickup_touch_joints
        )

        if arguments.solve_only:
            node.get_logger().info(
                'MOVEIT PICKUP CHECK PASSED. '
                'The robot was not moved.'
            )
            return 0

        node.execute_sequence(
            'PHASE 1: MoveIt pickup approach and touch',
            [
                'pickup_approach',
                'pickup_touch',
            ],
            poses,
        )

        suction(
            node,
            'attach',
        )

        node.carried_box.attach_box(
            half_height=half_height
        )

        node.execute_sequence(
            'PHASE 2: MoveIt transfer to red bin',
            [
                'pickup_approach',
                'drop_approach',
                'drop_release',
            ],
            poses,
        )

        suction(
            node,
            'detach',
        )

        node.carried_box.detach_box()

        node.execute_sequence(
            'PHASE 3: MoveIt return to pickup area',
            [
                'drop_approach',
                'pickup_approach',
            ],
            poses,
        )

        node.get_logger().info(
            'RED MOVEIT AUTOMATION COMPLETE.'
        )

        node.get_logger().info(
            'Detected box centre: '
            f'x={box_center[0]:.4f}, '
            f'y={box_center[1]:.4f}, '
            f'z={box_center[2]:.4f}'
        )

        node.get_logger().info(
            'Pickup touch target: '
            f'x={pickup_touch[0]:.4f}, '
            f'y={pickup_touch[1]:.4f}, '
            f'z={pickup_touch[2]:.4f}'
        )

        node.get_logger().info(
            'Pickup approach target: '
            f'x={pickup_approach[0]:.4f}, '
            f'y={pickup_approach[1]:.4f}, '
            f'z={pickup_approach[2]:.4f}'
        )

        return 0

    except KeyboardInterrupt:
        node.get_logger().warning(
            'MoveIt automation interrupted.'
        )
        return 130

    except Exception as error:
        node.get_logger().error(
            f'RED MOVEIT AUTOMATION FAILED: {error}'
        )
        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
