#!/usr/bin/env python3

from typing import List, Tuple

import rclpy

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    RobotTrajectory,
)
from rclpy.action import ActionClient

from moveit_trajectory_cache import (
    trajectory_duration,
)


GROUP_NAME = 'ur_manipulator'

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

CARRIED_OBJECT_IDS = (
    'red_box_carried',
    'green_box_carried',
    'blue_box_carried',
)


class PreparedPickupTrajectoryPlanner:

    def __init__(
        self,
        node,
        velocity: float = 0.15,
        acceleration: float = 0.15,
    ) -> None:
        self.node = node
        self.velocity = float(velocity)
        self.acceleration = float(acceleration)

        self.move_group_client = ActionClient(
            node,
            MoveGroup,
            '/move_action',
        )

    def wait_for_moveit(
        self,
        timeout: float = 10.0,
    ) -> None:
        if not self.move_group_client.wait_for_server(
            timeout_sec=timeout
        ):
            raise RuntimeError(
                'MoveIt /move_action is unavailable '
                'for pickup trajectory preplanning.'
            )

    def create_plan_only_goal(
        self,
        *,
        label: str,
        start_positions: List[float],
        goal_positions: List[float],
    ) -> MoveGroup.Goal:

        if len(start_positions) != 6:
            raise RuntimeError(
                f'{label}: start state does not contain '
                'six joints.'
            )

        if len(goal_positions) != 6:
            raise RuntimeError(
                f'{label}: goal state does not contain '
                'six joints.'
            )

        constraints = Constraints()
        constraints.name = label

        for name, position in zip(
            JOINT_NAMES,
            goal_positions,
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

        # IMPORTANT:
        # Do not use the robot's current position.
        #
        # The robot may currently be sorting the previous
        # box. We explicitly plan from the state at which
        # the previous cycle will finish.
        request.start_state.joint_state.name = list(
            JOINT_NAMES
        )

        request.start_state.joint_state.position = [
            float(value)
            for value in start_positions
        ]

        request.start_state.is_diff = False

        request.goal_constraints.append(
            constraints
        )

        # Build a temporary planning-scene diff representing
        # an EMPTY suction tool.
        #
        # While this future pickup is being planned, MoveIt
        # may still have the previous box attached. That box
        # must not be considered attached during the NEXT
        # pickup trajectory.
        scene = (
            goal.planning_options
            .planning_scene_diff
        )

        scene.is_diff = True
        scene.robot_state.is_diff = True

        for object_id in CARRIED_OBJECT_IDS:
            attached_remove = (
                AttachedCollisionObject()
            )

            attached_remove.object.id = object_id
            attached_remove.object.operation = (
                CollisionObject.REMOVE
            )

            scene.robot_state.attached_collision_objects.append(
                attached_remove
            )

            world_remove = CollisionObject()
            world_remove.id = object_id
            world_remove.operation = (
                CollisionObject.REMOVE
            )

            scene.world.collision_objects.append(
                world_remove
            )

        # PLAN ONLY. Absolutely no robot movement from T8.
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        return goal

    def plan_segment(
        self,
        *,
        label: str,
        start_positions: List[float],
        goal_positions: List[float],
    ) -> RobotTrajectory:

        self.wait_for_moveit()

        self.node.get_logger().info(
            f'PLAN ONLY: {label}'
        )

        send_future = (
            self.move_group_client.send_goal_async(
                self.create_plan_only_goal(
                    label=label,
                    start_positions=start_positions,
                    goal_positions=goal_positions,
                )
            )
        )

        rclpy.spin_until_future_complete(
            self.node,
            send_future,
        )

        goal_handle = send_future.result()

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            raise RuntimeError(
                f'MoveIt rejected plan-only request: '
                f'{label}'
            )

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self.node,
            result_future,
        )

        wrapped_result = result_future.result()

        if wrapped_result is None:
            raise RuntimeError(
                f'MoveIt returned no result for '
                f'plan-only request: {label}'
            )

        result = wrapped_result.result

        if (
            result.error_code.val
            != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(
                f'MoveIt plan-only request failed for '
                f'"{label}" with error '
                f'{result.error_code.val}.'
            )

        trajectory = (
            result.planned_trajectory
        )

        point_count = len(
            trajectory.joint_trajectory.points
        )

        duration = trajectory_duration(
            trajectory
        )

        if (
            point_count <= 1
            or duration <= 0.0
        ):
            raise RuntimeError(
                f'MoveIt returned an invalid trajectory '
                f'for "{label}".'
            )

        self.node.get_logger().info(
            f'Prepared trajectory segment: {label}: '
            f'{point_count} points, '
            f'{duration:.3f} s'
        )

        return trajectory

    def plan_pickup(
        self,
        *,
        pickup_exit_joints: List[float],
        pickup_approach_joints: List[float],
        pickup_touch_joints: List[float],
    ) -> Tuple[
        RobotTrajectory,
        RobotTrajectory,
    ]:

        approach_trajectory = self.plan_segment(
            label=(
                'pickup_exit -> pickup_approach'
            ),
            start_positions=pickup_exit_joints,
            goal_positions=pickup_approach_joints,
        )

        touch_trajectory = self.plan_segment(
            label=(
                'pickup_approach -> pickup_touch'
            ),
            start_positions=pickup_approach_joints,
            goal_positions=pickup_touch_joints,
        )

        return (
            approach_trajectory,
            touch_trajectory,
        )
