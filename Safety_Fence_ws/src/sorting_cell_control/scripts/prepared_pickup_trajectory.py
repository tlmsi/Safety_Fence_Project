#!/usr/bin/env python3

from typing import List, Optional, Tuple

import rclpy

from geometry_msgs.msg import Pose
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
from shape_msgs.msg import SolidPrimitive

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

ATTACH_LINK = 'suction_tip'

TOUCH_LINKS = [
    'suction_tip',
    'suction_cup_link',
    'suction_tool_link',
]

BOX_WIDTH = 0.06
CONTACT_CLEARANCE = 0.002


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

    def create_carried_box(
        self,
        *,
        color: str,
        half_height: float,
    ) -> AttachedCollisionObject:

        color = color.strip().lower()

        object_id = (
            f'{color}_box_carried'
        )

        if object_id not in CARRIED_OBJECT_IDS:
            raise RuntimeError(
                f'Unsupported carried-box color: {color}'
            )

        full_height = float(
            2.0 * half_height
        )

        collision_height = max(
            0.001,
            full_height - CONTACT_CLEARANCE,
        )

        attached = AttachedCollisionObject()

        attached.link_name = ATTACH_LINK

        attached.touch_links = list(
            TOUCH_LINKS
        )

        attached.object.header.frame_id = (
            ATTACH_LINK
        )

        attached.object.id = object_id

        attached.object.operation = (
            CollisionObject.ADD
        )

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX

        primitive.dimensions = [
            BOX_WIDTH,
            BOX_WIDTH,
            collision_height,
        ]

        pose = Pose()

        # Exact same representation used by the real
        # MoveIt carried-box scene managers.
        pose.position.z = (
            collision_height / 2.0
        )

        pose.orientation.w = 1.0

        attached.object.primitives.append(
            primitive
        )

        attached.object.primitive_poses.append(
            pose
        )

        return attached

    def create_plan_only_goal(
        self,
        *,
        label: str,
        start_positions: List[float],
        goal_positions: List[float],
        attached_color: Optional[str] = None,
        attached_half_height: Optional[float] = None,
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

        carried_object_id = None

        if attached_color is not None:
            if attached_half_height is None:
                raise RuntimeError(
                    f'{label}: attached box height '
                    'was not provided.'
                )

            carried_object = self.create_carried_box(
                color=attached_color,
                half_height=attached_half_height,
            )

            carried_object_id = (
                carried_object.object.id
            )

            # The explicit start state must also know that
            # the box is already attached at pickup_touch.
            request.start_state.attached_collision_objects.append(
                carried_object
            )

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

            # Remove any OTHER box that may currently be
            # carried by the robot while Terminal 8 plans
            # the next cycle.
            if object_id != carried_object_id:
                attached_remove = (
                    AttachedCollisionObject()
                )

                attached_remove.object.id = (
                    object_id
                )

                attached_remove.object.operation = (
                    CollisionObject.REMOVE
                )

                scene.robot_state.attached_collision_objects.append(
                    attached_remove
                )

            # No carried-box representation should remain
            # in the world collision-object list.
            world_remove = CollisionObject()

            world_remove.id = object_id

            world_remove.operation = (
                CollisionObject.REMOVE
            )

            scene.world.collision_objects.append(
                world_remove
            )

        if attached_color is not None:
            scene.robot_state.attached_collision_objects.append(
                self.create_carried_box(
                    color=attached_color,
                    half_height=attached_half_height,
                )
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
        attached_color: Optional[str] = None,
        attached_half_height: Optional[float] = None,
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
                    attached_color=attached_color,
                    attached_half_height=attached_half_height,
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

    def plan_attached_exit(
        self,
        *,
        color: str,
        half_height: float,
        pickup_touch_joints: List[float],
        pickup_exit_joints: List[float],
    ) -> RobotTrajectory:

        return self.plan_segment(
            label=(
                'pickup_touch -> pickup_exit '
                f'({color.upper()} box attached)'
            ),
            start_positions=pickup_touch_joints,
            goal_positions=pickup_exit_joints,
            attached_color=color,
            attached_half_height=half_height,
        )
