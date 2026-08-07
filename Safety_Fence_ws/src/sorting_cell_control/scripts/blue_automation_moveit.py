#!/usr/bin/env python3

import argparse
import sys
import time
from typing import Dict, List

import rclpy

from moveit_msgs.action import (
    ExecuteTrajectory,
    MoveGroup,
    MoveGroupSequence,
)
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    MotionSequenceItem,
    RobotTrajectory,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from blue_moveit_attached_box import CarriedBoxSceneManager
from blue_bin_placement import (
    BIN_SURFACE_Z,
    mark_blue_slot_occupied,
    restore_occupied_blue_boxes,
    solve_dynamic_blue_drop,
)

from prepared_pickup import (
    PREPARED_PICKUP_APPROACH_TRAJECTORY,
    PREPARED_PICKUP_TOUCH_TRAJECTORY,
    load_prepared_pickup,
)

from moveit_trajectory_cache import (
    BLUE_RETURN_CACHE,
    BLUE_TRANSFER_CACHE,
    load_trajectory,
    save_trajectory,
    trajectory_duration,
    trajectory_start_error,
)

from blue_automation_runtime import (
    load_cached_plan,
    blue_box_half_height,
    solve_dynamic_pickup,
    suction,
    wait_for_blue_box_pose,
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


class BlueMoveItRuntime(Node):

    def __init__(
        self,
        velocity: float,
        acceleration: float,
    ) -> None:
        super().__init__(
            'blue_automation_moveit_runtime'
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

        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory',
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

    def wait_for_execute_trajectory(
        self,
        timeout: float = 30.0,
    ) -> None:
        self.get_logger().info(
            'Waiting for MoveIt /execute_trajectory...'
        )

        if not self.execute_trajectory_client.wait_for_server(
            timeout_sec=timeout
        ):
            raise RuntimeError(
                'MoveIt /execute_trajectory '
                'action server is unavailable.'
            )

        self.get_logger().info(
            'MoveIt trajectory execution '
            'action server is ready.'
        )

    def execute_pose(
        self,
        pose_name: str,
        positions: List[float],
    ) -> RobotTrajectory:
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

        return result.planned_trajectory

    def execute_cached_trajectory(
        self,
        label: str,
        cache_path,
        start_tolerance: float = 0.05,
    ) -> None:
        self.wait_for_joint_state()

        trajectory, document = load_trajectory(
            cache_path
        )

        start_error = trajectory_start_error(
            trajectory,
            self.current_positions,
        )

        self.get_logger().info(
            f'Cached trajectory start-state error '
            f'for {label}: '
            f'{start_error:.6f} rad'
        )

        if start_error > start_tolerance:
            raise RuntimeError(
                f'Cached trajectory "{label}" cannot '
                'start safely. Maximum joint error is '
                f'{start_error:.6f} rad; allowed error is '
                f'{start_tolerance:.6f} rad.'
            )

        point_count = len(
            trajectory.joint_trajectory.points
        )

        duration = trajectory_duration(
            trajectory
        )

        self.get_logger().info(
            f'Executing cached trajectory: {label} '
            f'({point_count} points, '
            f'{duration:.3f} s)'
        )

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = (
            self.execute_trajectory_client
            .send_goal_async(goal)
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
                f'MoveIt rejected cached trajectory: '
                f'{label}'
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
                f'MoveIt returned no cached-trajectory '
                f'result for: {label}'
            )

        result = wrapped_result.result
        error_code = result.error_code.val

        self.get_logger().info(
            f'Cached trajectory result for {label}: '
            f'error={error_code}, '
            f'action_status={wrapped_result.status}, '
            f'message={result.error_code.message}, '
            f'source={result.error_code.source}'
        )

        if error_code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f'Cached trajectory "{label}" failed '
                f'with error code {error_code}.'
            )

    def execute_sequence(
        self,
        label: str,
        pose_names: List[str],
        poses: Dict[str, List[float]],
        blended: bool = False,
    ) -> None:
        self.get_logger().info(label)

        # The pickup phase remains unchanged because suction must
        # be activated only after reaching pickup_touch.
        if not blended:
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
            'Execute the blue sorting cycle through MoveIt.'
        )
    )

    mode_group = (
        parser.add_mutually_exclusive_group()
    )

    mode_group.add_argument(
        '--commission-cache',
        action='store_true',
        help=(
            'Plan, execute, and save the fixed '
            'blue transfer and return trajectories.'
        ),
    )

    mode_group.add_argument(
        '--use-cache',
        action='store_true',
        help=(
            'Execute the commissioned fixed '
            'trajectories without replanning them.'
        ),
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
        half_height = blue_box_half_height()

        if arguments.commission_cache:
            BLUE_TRANSFER_CACHE.unlink(
                missing_ok=True
            )

            BLUE_RETURN_CACHE.unlink(
                missing_ok=True
            )

        if arguments.use_cache:
            load_trajectory(
                BLUE_TRANSFER_CACHE
            )

            load_trajectory(
                BLUE_RETURN_CACHE
            )

    except Exception as error:
        print(
            f'ERROR: {error}',
            file=sys.stderr,
        )
        return 1

    rclpy.init()

    node = BlueMoveItRuntime(
        velocity=arguments.velocity,
        acceleration=arguments.acceleration,
    )

    try:
        node.wait_for_joint_state()
        node.wait_for_moveit()

        if arguments.use_cache:
            node.wait_for_execute_trajectory()

        node.carried_box.wait_for_services()

        restore_occupied_blue_boxes(
            node,
            node.carried_box,
            half_height,
        )

        prepared_pickup = load_prepared_pickup(
            expected_color='blue',
            max_age_seconds=120.0,
        )

        use_prepared_pickup_trajectory = False

        if prepared_pickup is not None:
            prepared_age = max(
                0.0,
                time.time()
                - float(
                    prepared_pickup[
                        'created_unix_time'
                    ]
                ),
            )

            prepared_generation = int(
                prepared_pickup[
                    'generation'
                ]
            )

            box_center = [
                float(value)
                for value in prepared_pickup[
                    'box_center'
                ]
            ]

            pickup_approach_joints = [
                float(value)
                for value in prepared_pickup[
                    'pickup_approach_joints'
                ]
            ]

            pickup_touch_joints = [
                float(value)
                for value in prepared_pickup[
                    'pickup_touch_joints'
                ]
            ]

            pickup_approach = [
                float(value)
                for value in prepared_pickup[
                    'pickup_approach'
                ]
            ]

            pickup_touch = [
                float(value)
                for value in prepared_pickup[
                    'pickup_touch'
                ]
            ]

            node.get_logger().info(
                'Using PRECOMPUTED BLUE pickup IK: '
                f'generation={prepared_generation}, '
                f'age={prepared_age:.2f} s.'
            )

            node.get_logger().info(
                'Live pickup IK calculation skipped.'
            )

            if (
                arguments.use_cache
                and bool(
                    prepared_pickup.get(
                        'trajectory_ready',
                        False,
                    )
                )
            ):
                try:
                    (
                        prepared_approach_trajectory,
                        prepared_approach_document,
                    ) = load_trajectory(
                        PREPARED_PICKUP_APPROACH_TRAJECTORY
                    )

                    (
                        prepared_touch_trajectory,
                        prepared_touch_document,
                    ) = load_trajectory(
                        PREPARED_PICKUP_TOUCH_TRAJECTORY
                    )

                    for (
                        trajectory_label,
                        trajectory_document,
                    ) in (
                        (
                            'approach',
                            prepared_approach_document,
                        ),
                        (
                            'touch',
                            prepared_touch_document,
                        ),
                    ):
                        metadata = (
                            trajectory_document.get(
                                'metadata',
                                {},
                            )
                        )

                        trajectory_generation = int(
                            metadata.get(
                                'generation',
                                -1,
                            )
                        )

                        trajectory_color = str(
                            metadata.get(
                                'color',
                                '',
                            )
                        ).strip().lower()

                        if (
                            trajectory_generation
                            != prepared_generation
                            or trajectory_color
                            != 'blue'
                        ):
                            raise RuntimeError(
                                f'Prepared '
                                f'{trajectory_label} '
                                'trajectory belongs to '
                                'a different pickup event.'
                            )

                    prepared_start_error = (
                        trajectory_start_error(
                            prepared_approach_trajectory,
                            node.current_positions,
                        )
                    )

                    node.get_logger().info(
                        'Prepared BLUE pickup '
                        'trajectory start-state error: '
                        f'{prepared_start_error:.6f} rad'
                    )

                    if prepared_start_error <= 0.05:
                        use_prepared_pickup_trajectory = True

                        node.get_logger().info(
                            'PREPARED BLUE PICKUP '
                            'TRAJECTORY ACCEPTED.'
                        )

                    else:
                        node.get_logger().warning(
                            'Prepared BLUE pickup '
                            'trajectory start state does '
                            'not match the current robot. '
                            'Falling back to normal '
                            'MoveIt pickup planning.'
                        )

                except Exception as error:
                    node.get_logger().warning(
                        'Prepared BLUE pickup '
                        'trajectory cannot be used: '
                        f'{error}'
                    )

                    node.get_logger().warning(
                        'Falling back to normal '
                        'MoveIt pickup planning.'
                    )

        else:
            node.get_logger().warning(
                'No valid precomputed BLUE pickup '
                'IK is available. Falling back to '
                'live perception and live IK.'
            )

            box_center = wait_for_blue_box_pose(
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

        (
            blue_slot,
            drop_approach_joints,
            drop_release_joints,
            drop_approach,
            drop_release,
        ) = solve_dynamic_blue_drop(
            node,
            cached,
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

        # Fixed interface between dynamic pickup and the
        # reusable commissioned transfer trajectory.
        poses['pickup_exit'] = list(
            cached.pickup_seed
        )

        # Fixed interface between the reusable transfer and
        # the future dynamic placement section.
        poses['blue_bin_staging'] = list(
            cached.poses['drop_approach']
        )

        poses['drop_approach'] = (
            drop_approach_joints
        )

        poses['drop_release'] = (
            drop_release_joints
        )

        if arguments.solve_only:
            node.get_logger().info(
                'MOVEIT DYNAMIC PICKUP/DROP CHECK PASSED. '
                'The robot was not moved.'
            )
            return 0

        if use_prepared_pickup_trajectory:
            node.get_logger().info(
                'PHASE 1: Executing PREPARED '
                'BLUE pickup trajectory'
            )

            node.execute_cached_trajectory(
                (
                    'blue prepared pickup: '
                    'pickup_exit to pickup_approach'
                ),
                PREPARED_PICKUP_APPROACH_TRAJECTORY,
            )

            node.execute_cached_trajectory(
                (
                    'blue prepared pickup: '
                    'pickup_approach to pickup_touch'
                ),
                PREPARED_PICKUP_TOUCH_TRAJECTORY,
            )

        else:
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

        # Reach the fixed interface pose using a
        # dynamically planned movement from pickup_touch.
        node.execute_pose(
            'pickup_exit',
            poses['pickup_exit'],
        )

        # Fixed attached-box transfer:
        # pickup_exit -> blue_bin_staging
        if arguments.commission_cache:
            node.get_logger().info(
                'Commissioning fixed attached-box '
                'blue transfer trajectory.'
            )

            transfer_trajectory = node.execute_pose(
                'blue_bin_staging',
                poses['blue_bin_staging'],
            )

            save_trajectory(
                BLUE_TRANSFER_CACHE,
                transfer_trajectory,
                label=(
                    'blue attached transfer: '
                    'pickup_exit to blue_bin_staging'
                ),
                metadata={
                    'start_pose': 'pickup_exit',
                    'goal_pose': 'blue_bin_staging',
                    'carried_object': 'blue_box_carried',
                },
            )

            node.get_logger().info(
                'Saved commissioned blue transfer: '
                f'{BLUE_TRANSFER_CACHE}'
            )

        elif arguments.use_cache:
            node.execute_cached_trajectory(
                (
                    'blue attached transfer: '
                    'pickup_exit to blue_bin_staging'
                ),
                BLUE_TRANSFER_CACHE,
            )

        else:
            node.execute_pose(
                'blue_bin_staging',
                poses['blue_bin_staging'],
            )

        # Dynamic placement from the fixed staging pose.
        if blue_slot.index == 0:
            node.execute_pose(
                'drop_release',
                poses['drop_release'],
            )

        else:
            node.execute_sequence(
                'PHASE 2: Dynamic blue-bin placement',
                [
                    'drop_approach',
                    'drop_release',
                ],
                poses,
                blended=True,
            )

        suction(
            node,
            'detach',
        )

        node.carried_box.detach_box()

        node.get_logger().info(
            'Retreating from the released blue box.'
        )

        node.execute_pose(
            'drop_approach',
            poses['drop_approach'],
        )

        placed_object_id = (
            f'blue_box_slot_{blue_slot.index + 1:02d}'
        )

        node.carried_box.add_world_box(
            object_id=placed_object_id,
            center_xyz=(
                blue_slot.x,
                blue_slot.y,
                BIN_SURFACE_Z + half_height,
            ),
            size_xyz=(
                0.06,
                0.06,
                2.0 * half_height,
            ),
            frame_id='world',
        )

        mark_blue_slot_occupied(
            node,
            blue_slot,
        )

        # Reach the fixed empty-tool return interface.
        if blue_slot.index != 0:
            node.execute_pose(
                'blue_bin_staging',
                poses['blue_bin_staging'],
            )

        # Fixed empty-tool return:
        # blue_bin_staging -> pickup_exit
        if arguments.commission_cache:
            node.get_logger().info(
                'Commissioning fixed empty-tool '
                'blue return trajectory.'
            )

            return_trajectory = node.execute_pose(
                'pickup_exit',
                poses['pickup_exit'],
            )

            save_trajectory(
                BLUE_RETURN_CACHE,
                return_trajectory,
                label=(
                    'blue empty return: '
                    'blue_bin_staging to pickup_exit'
                ),
                metadata={
                    'start_pose': 'blue_bin_staging',
                    'goal_pose': 'pickup_exit',
                    'carried_object': None,
                },
            )

            node.get_logger().info(
                'Saved commissioned blue return: '
                f'{BLUE_RETURN_CACHE}'
            )

        elif arguments.use_cache:
            node.execute_cached_trajectory(
                (
                    'blue empty return: '
                    'blue_bin_staging to pickup_exit'
                ),
                BLUE_RETURN_CACHE,
            )

        else:
            node.execute_pose(
                'pickup_exit',
                poses['pickup_exit'],
            )

        node.get_logger().info(
            'BLUE MOVEIT AUTOMATION COMPLETE.'
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

        node.get_logger().info(
            f'Blue-bin slot used: '
            f'{blue_slot.index + 1}/'
            f'{blue_slot.capacity}'
        )

        node.get_logger().info(
            'Drop release target: '
            f'x={drop_release[0]:.4f}, '
            f'y={drop_release[1]:.4f}, '
            f'z={drop_release[2]:.4f}'
        )

        return 0

    except KeyboardInterrupt:
        node.get_logger().warning(
            'MoveIt automation interrupted.'
        )
        return 130

    except Exception as error:
        node.get_logger().error(
            f'BLUE MOVEIT AUTOMATION FAILED: {error}'
        )
        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
