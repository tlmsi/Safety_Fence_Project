#!/usr/bin/env python3

from typing import List, Optional

import rclpy

from geometry_msgs.msg import Pose
from moveit_msgs.action import (
    MoveGroup,
    MoveGroupSequence,
)
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionSequenceItem,
    MoveItErrorCodes,
    RobotTrajectory,
)
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
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

SEQUENCE_BLEND_RADIUS = 0.03

ATTACH_LINK = 'suction_tip'

TOUCH_LINKS = [
    'suction_tip',
    'suction_cup_link',
    'suction_tool_link',
]

CARRIED_OBJECT_IDS = (
    'red_box_carried',
    'green_box_carried',
    'blue_box_carried',
)

CARRIED_BOX_WIDTH = 0.06
CONTACT_CLEARANCE = 0.002


class PreparedBinTrajectoryPlanner:

    def __init__(
        self,
        node,
        velocity: float = 0.15,
        acceleration: float = 0.15,
    ) -> None:
        self.node = node

        self.velocity = float(
            velocity
        )

        self.acceleration = float(
            acceleration
        )

        self.move_group_client = ActionClient(
            node,
            MoveGroup,
            '/move_action',
        )

        self.sequence_client = ActionClient(
            node,
            MoveGroupSequence,
            '/sequence_move_group',
        )

        # IMPORTANT:
        # Never use rclpy's global executor here.
        #
        # This node plans in a background thread while the
        # sorting coordinator simultaneously executes robot
        # trajectories on its own executor.
        self.executor = SingleThreadedExecutor()

        self.executor.add_node(
            node
        )

    # ========================================================
    # Common helpers
    # ========================================================

    def wait_for_moveit(
        self,
        timeout: float = 10.0,
    ) -> None:

        if not self.move_group_client.wait_for_server(
            timeout_sec=timeout
        ):
            raise RuntimeError(
                'MoveIt /move_action is unavailable '
                'for bin-path preplanning.'
            )

        if not self.sequence_client.wait_for_server(
            timeout_sec=timeout
        ):
            raise RuntimeError(
                'MoveIt /sequence_move_group is unavailable '
                'for bin-path preplanning.'
            )

    def create_constraints(
        self,
        label: str,
        positions: List[float],
    ) -> Constraints:

        if len(positions) != 6:
            raise RuntimeError(
                f'{label}: expected six joint positions.'
            )

        constraints = Constraints()
        constraints.name = label

        for name, position in zip(
            JOINT_NAMES,
            positions,
        ):
            constraint = JointConstraint()

            constraint.joint_name = name
            constraint.position = float(
                position
            )

            constraint.tolerance_above = 0.001
            constraint.tolerance_below = 0.001
            constraint.weight = 1.0

            constraints.joint_constraints.append(
                constraint
            )

        return constraints

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

        full_height = (
            2.0 * float(half_height)
        )

        collision_height = max(
            0.001,
            full_height
            - CONTACT_CLEARANCE,
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
            CARRIED_BOX_WIDTH,
            CARRIED_BOX_WIDTH,
            collision_height,
        ]

        pose = Pose()

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

    def create_placed_box(
        self,
        *,
        color: str,
        slot,
        half_height: float,
        bin_surface_z: float,
        box_size_x: float,
        box_size_y: float,
    ) -> CollisionObject:

        collision_object = (
            CollisionObject()
        )

        collision_object.header.frame_id = (
            'world'
        )

        collision_object.id = (
            f'{color}_box_slot_'
            f'{slot.index + 1:02d}'
        )

        collision_object.operation = (
            CollisionObject.ADD
        )

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX

        primitive.dimensions = [
            float(box_size_x),
            float(box_size_y),
            2.0 * float(half_height),
        ]

        pose = Pose()

        pose.position.x = float(
            slot.x
        )

        pose.position.y = float(
            slot.y
        )

        pose.position.z = (
            float(bin_surface_z)
            + float(half_height)
        )

        pose.orientation.w = 1.0

        collision_object.primitives.append(
            primitive
        )

        collision_object.primitive_poses.append(
            pose
        )

        return collision_object

    def configure_scene(
        self,
        scene,
        *,
        attached_color: Optional[str] = None,
        attached_half_height: Optional[float] = None,
        placed_box: Optional[CollisionObject] = None,
    ) -> None:

        scene.is_diff = True
        scene.robot_state.is_diff = True

        carried_object_id = None

        if attached_color is not None:
            carried_object_id = (
                f'{attached_color}_box_carried'
            )

        for object_id in CARRIED_OBJECT_IDS:

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

            world_remove = CollisionObject()
            world_remove.id = object_id

            world_remove.operation = (
                CollisionObject.REMOVE
            )

            scene.world.collision_objects.append(
                world_remove
            )

        if attached_color is not None:

            if attached_half_height is None:
                raise RuntimeError(
                    'Attached-box height was not provided.'
                )

            scene.robot_state.attached_collision_objects.append(
                self.create_carried_box(
                    color=attached_color,
                    half_height=attached_half_height,
                )
            )

        if placed_box is not None:
            scene.world.collision_objects.append(
                placed_box
            )

    # ========================================================
    # Normal MoveGroup plan-only segment
    # ========================================================

    def create_plan_only_goal(
        self,
        *,
        label: str,
        start_positions: List[float],
        goal_positions: List[float],
        attached_color: Optional[str] = None,
        attached_half_height: Optional[float] = None,
        placed_box: Optional[CollisionObject] = None,
    ) -> MoveGroup.Goal:

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

        request.start_state.joint_state.name = list(
            JOINT_NAMES
        )

        request.start_state.joint_state.position = [
            float(value)
            for value in start_positions
        ]

        request.start_state.is_diff = False

        if attached_color is not None:

            request.start_state.attached_collision_objects.append(
                self.create_carried_box(
                    color=attached_color,
                    half_height=attached_half_height,
                )
            )

        request.goal_constraints.append(
            self.create_constraints(
                label,
                goal_positions,
            )
        )

        self.configure_scene(
            goal.planning_options.planning_scene_diff,
            attached_color=attached_color,
            attached_half_height=attached_half_height,
            placed_box=placed_box,
        )

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
        placed_box: Optional[CollisionObject] = None,
    ) -> RobotTrajectory:

        self.wait_for_moveit()

        self.node.get_logger().info(
            f'BIN PLAN ONLY: {label}'
        )

        goal = self.create_plan_only_goal(
            label=label,
            start_positions=start_positions,
            goal_positions=goal_positions,
            attached_color=attached_color,
            attached_half_height=attached_half_height,
            placed_box=placed_box,
        )

        send_future = (
            self.move_group_client.send_goal_async(
                goal
            )
        )

        self.executor.spin_until_future_complete(
            send_future
        )

        goal_handle = send_future.result()

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            raise RuntimeError(
                f'MoveIt rejected bin plan: {label}'
            )

        result_future = (
            goal_handle.get_result_async()
        )

        self.executor.spin_until_future_complete(
            result_future
        )

        wrapped_result = (
            result_future.result()
        )

        if wrapped_result is None:
            raise RuntimeError(
                f'MoveIt returned no bin plan: {label}'
            )

        result = wrapped_result.result

        if (
            result.error_code.val
            != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(
                f'MoveIt bin plan failed for '
                f'"{label}" with error '
                f'{result.error_code.val}.'
            )

        trajectory = (
            result.planned_trajectory
        )

        self.validate_trajectory(
            trajectory,
            label,
        )

        return trajectory

    # ========================================================
    # Pilz blended dynamic drop plan-only sequence
    # ========================================================

    def plan_drop_sequence(
        self,
        *,
        color: str,
        half_height: float,
        staging_joints: List[float],
        drop_approach_joints: List[float],
        drop_release_joints: List[float],
    ) -> RobotTrajectory:

        self.wait_for_moveit()

        label = (
            f'{color.upper()} attached '
            'bin staging -> drop approach -> release'
        )

        self.node.get_logger().info(
            f'BIN PLAN ONLY: {label}'
        )

        goal = MoveGroupSequence.Goal()

        targets = [
            (
                'drop_approach',
                drop_approach_joints,
            ),
            (
                'drop_release',
                drop_release_joints,
            ),
        ]

        for index, (
            pose_name,
            positions,
        ) in enumerate(targets):

            request_goal = (
                self.create_plan_only_goal(
                    label=pose_name,
                    start_positions=staging_joints,
                    goal_positions=positions,
                    attached_color=color,
                    attached_half_height=half_height,
                )
            )

            item = MotionSequenceItem()

            item.req = (
                request_goal.request
            )

            item.req.pipeline_id = (
                'pilz_industrial_motion_planner'
            )

            item.req.planner_id = 'PTP'

            if index > 0:

                item.req.start_state.joint_state.name = []
                item.req.start_state.joint_state.position = []

                item.req.start_state.attached_collision_objects = []

                item.req.start_state.is_diff = False

            if index == 0:
                item.blend_radius = (
                    SEQUENCE_BLEND_RADIUS
                )
            else:
                item.blend_radius = 0.0

            goal.request.items.append(
                item
            )

        self.configure_scene(
            goal.planning_options.planning_scene_diff,
            attached_color=color,
            attached_half_height=half_height,
        )

        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        send_future = (
            self.sequence_client.send_goal_async(
                goal
            )
        )

        self.executor.spin_until_future_complete(
            send_future
        )

        goal_handle = send_future.result()

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            raise RuntimeError(
                f'MoveIt rejected bin sequence: '
                f'{label}'
            )

        result_future = (
            goal_handle.get_result_async()
        )

        self.executor.spin_until_future_complete(
            result_future
        )

        wrapped_result = (
            result_future.result()
        )

        if wrapped_result is None:
            raise RuntimeError(
                f'MoveIt returned no bin sequence '
                f'result: {label}'
            )

        response = (
            wrapped_result.result.response
        )

        if (
            response.error_code.val
            != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(
                f'MoveIt bin sequence failed for '
                f'"{label}" with error '
                f'{response.error_code.val}.'
            )

        trajectories = list(
            response.planned_trajectories
        )

        if len(trajectories) != 1:
            raise RuntimeError(
                f'Expected one executable trajectory '
                f'from bin sequence "{label}", '
                f'got {len(trajectories)}.'
            )

        trajectory = trajectories[0]

        self.validate_trajectory(
            trajectory,
            label,
        )

        return trajectory

    # ========================================================
    # Whole future bin section
    # ========================================================

    def plan_bin_cycle(
        self,
        *,
        color: str,
        slot,
        half_height: float,
        staging_joints: List[float],
        drop_approach_joints: List[float],
        drop_release_joints: List[float],
        bin_surface_z: float,
        box_size_x: float,
        box_size_y: float,
    ):

        self.node.get_logger().info(
            '========================================'
        )

        self.node.get_logger().info(
            f'PREPLANNING COMPLETE '
            f'{color.upper()} BIN SIDE'
        )

        self.node.get_logger().info(
            f'Slot {slot.index + 1}/{slot.capacity}'
        )

        self.node.get_logger().info(
            '========================================'
        )

        # ----------------------------------------------------
        # 1. Attached box:
        # staging -> dynamic drop.
        # ----------------------------------------------------

        if slot.index == 0:

            drop_trajectory = self.plan_segment(
                label=(
                    f'{color.upper()} attached '
                    'bin staging -> drop release'
                ),
                start_positions=staging_joints,
                goal_positions=drop_release_joints,
                attached_color=color,
                attached_half_height=half_height,
            )

        else:

            drop_trajectory = (
                self.plan_drop_sequence(
                    color=color,
                    half_height=half_height,
                    staging_joints=staging_joints,
                    drop_approach_joints=(
                        drop_approach_joints
                    ),
                    drop_release_joints=(
                        drop_release_joints
                    ),
                )
            )

        # ----------------------------------------------------
        # 2. Immediately after DETACH:
        #
        # There is intentionally NO world box here.
        # At the release pose, the suction tip is touching
        # the physical box. This matches the current tested
        # runtime and avoids a start-state collision.
        # ----------------------------------------------------

        retreat_trajectory = self.plan_segment(
            label=(
                f'{color.upper()} empty retreat: '
                'drop_release -> drop_approach'
            ),
            start_positions=drop_release_joints,
            goal_positions=drop_approach_joints,
        )

        # ----------------------------------------------------
        # 3. After the retreat, the released box is inserted
        # into the real MoveIt world. Plan the staging motion
        # with the same box represented in this request.
        # ----------------------------------------------------

        staging_trajectory = None

        if slot.index != 0:

            placed_box = self.create_placed_box(
                color=color,
                slot=slot,
                half_height=half_height,
                bin_surface_z=bin_surface_z,
                box_size_x=box_size_x,
                box_size_y=box_size_y,
            )

            staging_trajectory = self.plan_segment(
                label=(
                    f'{color.upper()} empty '
                    'drop_approach -> bin_staging '
                    'with placed box'
                ),
                start_positions=drop_approach_joints,
                goal_positions=staging_joints,
                placed_box=placed_box,
            )

        self.node.get_logger().info(
            f'PREPARED {color.upper()} BIN PATHS READY.'
        )

        return {
            'drop_trajectory': (
                drop_trajectory
            ),
            'retreat_trajectory': (
                retreat_trajectory
            ),
            'staging_trajectory': (
                staging_trajectory
            ),
        }

    def shutdown(
        self,
    ) -> None:

        self.executor.remove_node(
            self.node
        )

        self.executor.shutdown()


    def validate_trajectory(
        self,
        trajectory: RobotTrajectory,
        label: str,
    ) -> None:

        point_count = len(
            trajectory
            .joint_trajectory
            .points
        )

        duration = trajectory_duration(
            trajectory
        )

        if (
            point_count <= 1
            or duration <= 0.0
        ):
            raise RuntimeError(
                f'Invalid prepared bin trajectory: '
                f'{label}'
            )

        self.node.get_logger().info(
            f'Prepared bin trajectory: {label}: '
            f'{point_count} points, '
            f'{duration:.3f} s'
        )
