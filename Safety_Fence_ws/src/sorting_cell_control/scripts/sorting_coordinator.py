#!/usr/bin/env python3

import argparse
import math
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, Optional

import rclpy

from action_msgs.msg import GoalStatus

from moveit_msgs.action import (
    ExecuteTrajectory,
    MoveGroup,
    MoveGroupSequence,
)
from moveit_msgs.msg import (
    MoveItErrorCodes,
    RobotTrajectory,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, UInt64

import red_automation_moveit as moveit_base

from prepared_bin_trajectory import (
    PreparedBinTrajectoryPlanner,
)

from prepared_pickup import (
    PREPARED_PICKUP_APPROACH_TRAJECTORY,
    PREPARED_PICKUP_TOUCH_TRAJECTORY,
    PREPARED_PICKUP_EXIT_ATTACHED_TRAJECTORY,
    load_prepared_pickup,
)

from moveit_trajectory_cache import (
    BLUE_RETURN_CACHE,
    BLUE_TRANSFER_CACHE,
    GREEN_RETURN_CACHE,
    GREEN_TRANSFER_CACHE,
    RED_RETURN_CACHE,
    RED_TRANSFER_CACHE,
    load_trajectory,
    trajectory_duration,
    trajectory_start_error,
)

from moveit_attached_box import (
    CarriedBoxSceneManager as RedSceneManager,
)

from green_moveit_attached_box import (
    CarriedBoxSceneManager as GreenSceneManager,
)

from blue_moveit_attached_box import (
    CarriedBoxSceneManager as BlueSceneManager,
)

from red_automation_runtime import (
    load_cached_plan as load_red_plan,
    red_box_half_height,
    solve_dynamic_pickup as solve_red_pickup,
    suction as red_suction,
    wait_for_red_box_pose,
)

from green_automation_runtime import (
    load_cached_plan as load_green_plan,
    green_box_half_height,
    solve_dynamic_pickup as solve_green_pickup,
    suction as green_suction,
    wait_for_green_box_pose,
)

from blue_automation_runtime import (
    load_cached_plan as load_blue_plan,
    blue_box_half_height,
    solve_dynamic_pickup as solve_blue_pickup,
    suction as blue_suction,
    wait_for_blue_box_pose,
)

from red_bin_placement import (
    BIN_SURFACE_Z as RED_BIN_SURFACE_Z,
    BOX_SIZE_X as RED_BOX_SIZE_X,
    BOX_SIZE_Y as RED_BOX_SIZE_Y,
    mark_red_slot_occupied,
    restore_occupied_red_boxes,
    solve_dynamic_red_drop,
)

from green_bin_placement import (
    BIN_SURFACE_Z as GREEN_BIN_SURFACE_Z,
    BOX_SIZE_X as GREEN_BOX_SIZE_X,
    BOX_SIZE_Y as GREEN_BOX_SIZE_Y,
    mark_green_slot_occupied,
    restore_occupied_green_boxes,
    solve_dynamic_green_drop,
)

from blue_bin_placement import (
    BIN_SURFACE_Z as BLUE_BIN_SURFACE_Z,
    BOX_SIZE_X as BLUE_BOX_SIZE_X,
    BOX_SIZE_Y as BLUE_BOX_SIZE_Y,
    mark_blue_slot_occupied,
    restore_occupied_blue_boxes,
    solve_dynamic_blue_drop,
)


VALID_COLORS = {
    'red',
    'green',
    'blue',
}


class SortingCoordinator(
    moveit_base.RedMoveItRuntime
):

    def __init__(
        self,
        velocity: float,
        acceleration: float,
    ) -> None:

        # Deliberately initialise Node directly rather than
        # RedMoveItRuntime.__init__ because this is now the
        # shared persistent executor for all three colours.
        Node.__init__(
            self,
            'sorting_coordinator',
        )

        self.velocity = float(
            velocity
        )

        self.acceleration = float(
            acceleration
        )

        # One persistent background worker computes the
        # current cycle's dynamic drop IK while the robot
        # executes its pickup trajectory.
        self.drop_ik_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix='sorting_drop_ik',
        )

        self.bin_planner_node = Node(
            'sorting_bin_path_preplanner'
        )

        self.bin_planner = (
            PreparedBinTrajectoryPlanner(
                self.bin_planner_node,
                velocity=self.velocity,
                acceleration=self.acceleration,
            )
        )

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

        # ----------------------------------------------------
        # Safety supervisor
        # ----------------------------------------------------

        # Fail safe until the supervisor heartbeat explicitly
        # grants motion permission.
        self.safety_motion_allowed = False
        self.safety_state = 'UNKNOWN'
        self.safety_last_message_monotonic = 0.0

        self.active_goal_handle = None
        self.active_goal_label = None
        self.active_goal_cancel_requested = False

        self.create_subscription(
            Bool,
            '/safety/motion_allowed',
            self.safety_motion_callback,
            10,
        )

        # The state topic blocks NEW goals immediately during
        # a controlled stop. The current active goal is allowed
        # to decelerate until motion_allowed becomes false.
        self.create_subscription(
            String,
            '/safety/state',
            self.safety_state_callback,
            10,
        )

        # If the supervisor disappears, motion permission
        # expires automatically.
        self.create_timer(
            0.25,
            self.safety_watchdog_callback,
        )

        # ----------------------------------------------------
        # Detector state
        # ----------------------------------------------------

        self.object_ready = False

        self.detected_color: Optional[
            str
        ] = None

        self.pickup_generation = 0

        # Direct next-pickup handoff.
        #
        # If the previous cycle moved directly to the next
        # dynamic pickup_approach, these fields identify the
        # exact detector event now waiting there.
        self.direct_handoff_generation = None
        self.direct_handoff_color = None

        # The detector owns pickup-event identity.
        # Both T7 and T8 consume exactly the same generation.
        self.create_subscription(
            UInt64,
            '/perception/pickup_generation',
            self.generation_callback,
            10,
        )

        # Depth 10 is deliberate. We do not want to lose a
        # pickup-state transition while executing a robot
        # action.
        self.create_subscription(
            Bool,
            '/perception/object_in_pickup_zone',
            self.ready_callback,
            10,
        )

        self.create_subscription(
            String,
            '/perception/detected_color',
            self.color_callback,
            10,
        )

        # ----------------------------------------------------
        # One persistent planning-scene manager per colour.
        # ----------------------------------------------------

        self.scene_managers = {
            'red': RedSceneManager(self),
            'green': GreenSceneManager(self),
            'blue': BlueSceneManager(self),
        }

        self.configs = {
            'red': {
                'load_plan': load_red_plan,
                'half_height_fn': red_box_half_height,
                'pickup_solver': solve_red_pickup,
                'wait_box_pose': wait_for_red_box_pose,
                'suction': red_suction,
                'drop_solver': solve_dynamic_red_drop,
                'restore': restore_occupied_red_boxes,
                'mark_slot': mark_red_slot_occupied,
                'transfer_cache': RED_TRANSFER_CACHE,
                'return_cache': RED_RETURN_CACHE,
                'bin_surface_z': RED_BIN_SURFACE_Z,
                'box_size_x': RED_BOX_SIZE_X,
                'box_size_y': RED_BOX_SIZE_Y,
            },

            'green': {
                'load_plan': load_green_plan,
                'half_height_fn': green_box_half_height,
                'pickup_solver': solve_green_pickup,
                'wait_box_pose': wait_for_green_box_pose,
                'suction': green_suction,
                'drop_solver': solve_dynamic_green_drop,
                'restore': restore_occupied_green_boxes,
                'mark_slot': mark_green_slot_occupied,
                'transfer_cache': GREEN_TRANSFER_CACHE,
                'return_cache': GREEN_RETURN_CACHE,
                'bin_surface_z': GREEN_BIN_SURFACE_Z,
                'box_size_x': GREEN_BOX_SIZE_X,
                'box_size_y': GREEN_BOX_SIZE_Y,
            },

            'blue': {
                'load_plan': load_blue_plan,
                'half_height_fn': blue_box_half_height,
                'pickup_solver': solve_blue_pickup,
                'wait_box_pose': wait_for_blue_box_pose,
                'suction': blue_suction,
                'drop_solver': solve_dynamic_blue_drop,
                'restore': restore_occupied_blue_boxes,
                'mark_slot': mark_blue_slot_occupied,
                'transfer_cache': BLUE_TRANSFER_CACHE,
                'return_cache': BLUE_RETURN_CACHE,
                'bin_surface_z': BLUE_BIN_SURFACE_Z,
                'box_size_x': BLUE_BOX_SIZE_X,
                'box_size_y': BLUE_BOX_SIZE_Y,
            },
        }

        self.get_logger().info(
            'Persistent sorting coordinator started.'
        )


    # ========================================================
    # Safety
    # ========================================================

    SAFETY_HEARTBEAT_TIMEOUT = 1.50

    def safety_state_callback(
        self,
        message: String,
    ) -> None:

        state = (
            message.data
            .strip()
            .upper()
        )

        if state == self.safety_state:
            return

        previous = self.safety_state
        self.safety_state = state

        self.get_logger().info(
            'SAFETY: supervisor state '
            f'{previous} -> {state}'
        )

    def safety_motion_callback(
        self,
        message: Bool,
    ) -> None:

        self.safety_last_message_monotonic = (
            time.monotonic()
        )

        allowed = bool(
            message.data
        )

        previous = (
            self.safety_motion_allowed
        )

        self.safety_motion_allowed = allowed

        if allowed and not previous:

            self.get_logger().info(
                'SAFETY: motion permission granted.'
            )

        elif not allowed and previous:

            self.get_logger().warning(
                'SAFETY: motion permission removed.'
            )

        if not allowed:
            self.cancel_active_motion(
                'safety permission removed'
            )

    def safety_watchdog_callback(
        self,
    ) -> None:

        if (
            self.safety_last_message_monotonic
            <= 0.0
        ):
            return

        age = (
            time.monotonic()
            - self.safety_last_message_monotonic
        )

        if (
            age
            <= self.SAFETY_HEARTBEAT_TIMEOUT
        ):
            return

        if self.safety_motion_allowed:

            self.get_logger().error(
                'SAFETY SUPERVISOR HEARTBEAT LOST. '
                'Motion is being inhibited.'
            )

        self.safety_motion_allowed = False

        self.cancel_active_motion(
            'safety heartbeat lost'
        )

    def safety_message_is_fresh(
        self,
    ) -> bool:

        if (
            self.safety_last_message_monotonic
            <= 0.0
        ):
            return False

        return (
            time.monotonic()
            - self.safety_last_message_monotonic
            <= self.SAFETY_HEARTBEAT_TIMEOUT
        )

    def cancel_active_motion(
        self,
        reason: str,
    ) -> None:

        if self.active_goal_handle is None:
            return

        if self.active_goal_cancel_requested:
            return

        self.active_goal_cancel_requested = True

        self.get_logger().warning(
            'SAFETY: cancelling active MoveIt goal: '
            f'{self.active_goal_label} '
            f'[{reason}]'
        )

        try:
            self.active_goal_handle.cancel_goal_async()

        except Exception as error:

            self.get_logger().error(
                'Could not request MoveIt goal '
                f'cancellation: {error}'
            )

    def wait_until_safety_allows_motion(
        self,
        context: str,
    ) -> None:

        announced = False

        while rclpy.ok():

            if (
                self.safety_motion_allowed
                and self.safety_state == 'RUNNING'
                and self.safety_message_is_fresh()
            ):
                if announced:
                    self.get_logger().info(
                        'SAFETY: continuing: '
                        f'{context}'
                    )

                return

            if not announced:

                self.get_logger().warning(
                    'SAFETY: waiting in PAUSED / '
                    f'E-STOP state before: {context}'
                )

                announced = True

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

        raise KeyboardInterrupt

    def wait_for_robot_after_safety_stop(
        self,
        seconds: float = 0.20,
    ) -> None:

        deadline = (
            time.monotonic()
            + seconds
        )

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

    def joint_target_error(
        self,
        positions,
    ) -> float:

        names = list(
            moveit_base.JOINT_NAMES
        )

        if len(positions) != len(names):
            return float('inf')

        if not all(
            name in self.current_positions
            for name in names
        ):
            return float('inf')

        errors = []

        for name, target in zip(
            names,
            positions,
        ):
            current = float(
                self.current_positions[name]
            )

            difference = math.atan2(
                math.sin(
                    current - float(target)
                ),
                math.cos(
                    current - float(target)
                ),
            )

            errors.append(
                abs(difference)
            )

        return max(
            errors,
            default=0.0,
        )

    def trajectory_final_positions(
        self,
        trajectory: RobotTrajectory,
    ):

        joint_trajectory = (
            trajectory.joint_trajectory
        )

        if not joint_trajectory.points:
            raise RuntimeError(
                'Trajectory contains no points.'
            )

        names = list(
            joint_trajectory.joint_names
        )

        final = list(
            joint_trajectory.points[
                -1
            ].positions
        )

        by_name = dict(
            zip(
                names,
                final,
            )
        )

        expected = list(
            moveit_base.JOINT_NAMES
        )

        missing = [
            name
            for name in expected
            if name not in by_name
        ]

        if missing:
            raise RuntimeError(
                'Trajectory final state is missing '
                'joint(s): '
                + ', '.join(missing)
            )

        return [
            float(by_name[name])
            for name in expected
        ]

    def run_action_goal_with_safety(
        self,
        *,
        client,
        goal,
        label: str,
    ):

        self.wait_until_safety_allows_motion(
            label
        )

        send_future = (
            client.send_goal_async(
                goal
            )
        )

        while (
            rclpy.ok()
            and not send_future.done()
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

        goal_handle = (
            send_future.result()
        )

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            raise RuntimeError(
                f'MoveIt rejected goal: {label}'
            )

        self.active_goal_handle = (
            goal_handle
        )

        self.active_goal_label = (
            label
        )

        self.active_goal_cancel_requested = (
            False
        )

        # Safety may have changed while the goal request
        # itself was being accepted.
        if not (
            self.safety_motion_allowed
            and self.safety_message_is_fresh()
        ):
            self.cancel_active_motion(
                'safety changed while goal was accepted'
            )

        result_future = (
            goal_handle.get_result_async()
        )

        while (
            rclpy.ok()
            and not result_future.done()
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

            if not (
                self.safety_motion_allowed
                and self.safety_message_is_fresh()
            ):
                self.cancel_active_motion(
                    'safety stop during MoveIt execution'
                )

        wrapped_result = (
            result_future.result()
        )

        cancel_was_requested = (
            self.active_goal_cancel_requested
        )

        self.active_goal_handle = None
        self.active_goal_label = None
        self.active_goal_cancel_requested = False

        if wrapped_result is None:
            raise RuntimeError(
                f'MoveIt returned no result: {label}'
            )

        action_succeeded = (
            wrapped_result.status
            == GoalStatus.STATUS_SUCCEEDED
        )

        # A cancellation request can race with normal action
        # completion. Only treat the trajectory as interrupted
        # when the action did NOT finish successfully.
        interrupted_by_safety = (
            cancel_was_requested
            and not action_succeeded
        )

        if (
            cancel_was_requested
            and action_succeeded
        ):

            self.get_logger().warning(
                'SAFETY: cancellation was requested, '
                'but the MoveIt goal completed before '
                'cancellation took effect: '
                f'{label}'
            )

            # Even though the trajectory completed, never
            # continue the automation while safety permission
            # is absent.
            self.wait_until_safety_allows_motion(
                f'continue after safety stop: {label}'
            )

        elif interrupted_by_safety:

            self.get_logger().warning(
                'SAFETY STOP CONFIRMED. '
                f'Interrupted goal: {label}'
            )

            self.wait_until_safety_allows_motion(
                f'resume {label}'
            )

            self.wait_for_robot_after_safety_stop()

        return (
            wrapped_result,
            interrupted_by_safety,
        )

    # --------------------------------------------------------
    # Override the inherited MoveGroup executor so every
    # online movement is safety cancellable and resumes by
    # replanning from the LIVE robot state.
    # --------------------------------------------------------

    def execute_pose(
        self,
        pose_name: str,
        positions,
    ) -> RobotTrajectory:

        safety_retry = False

        while rclpy.ok():

            self.wait_until_safety_allows_motion(
                f'move to {pose_name}'
            )

            self.wait_for_joint_state()

            target_error = (
                self.joint_target_error(
                    positions
                )
            )

            # Reaching an already-satisfied target is a valid
            # no-op. This is especially important when a
            # safety cancellation races with trajectory
            # completion.
            if target_error <= 0.010:

                if safety_retry:

                    self.get_logger().info(
                        'SAFETY RESUME: target was '
                        'already reached before the '
                        'cancel completed: '
                        f'{pose_name}'
                    )

                else:

                    self.get_logger().info(
                        'Target is already reached: '
                        f'{pose_name} '
                        f'(joint error '
                        f'{target_error:.6f} rad).'
                    )

                return RobotTrajectory()

            self.get_logger().info(
                'Planning and executing pose: '
                f'{pose_name}'
            )

            goal = self.create_goal(
                pose_name,
                positions,
            )

            (
                wrapped_result,
                interrupted,
            ) = self.run_action_goal_with_safety(
                client=self.move_group_client,
                goal=goal,
                label=(
                    f'MoveGroup -> {pose_name}'
                ),
            )

            if interrupted:

                safety_retry = True

                self.get_logger().info(
                    'SAFETY RESUME: replanning '
                    'from the actual current robot '
                    f'state to {pose_name}.'
                )

                continue

            result = (
                wrapped_result.result
            )

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
                    + float(
                        final_time.nanosec
                    )
                    / 1_000_000_000.0
                )

            self.get_logger().info(
                f'MoveIt result for {pose_name}: '
                f'error={error_code}, '
                f'points={point_count}, '
                f'duration={duration:.3f} s'
            )

            if (
                error_code
                != MoveItErrorCodes.SUCCESS
            ):
                raise RuntimeError(
                    f'MoveIt failed for '
                    f'"{pose_name}" with '
                    f'error code {error_code}.'
                )

            if (
                point_count <= 1
                or duration <= 0.0
            ):
                raise RuntimeError(
                    'MoveIt returned a zero-motion '
                    f'trajectory for "{pose_name}".'
                )

            return (
                result.planned_trajectory
            )

        raise KeyboardInterrupt

    def execute_sequence(
        self,
        label: str,
        pose_names,
        poses,
        blended: bool = False,
    ) -> None:

        self.get_logger().info(
            label
        )

        # Safety resume needs deterministic intermediate
        # targets. Execute each required waypoint through the
        # safety-aware MoveGroup executor. This also preserves
        # exact approach / release waypoints.
        if blended:

            self.get_logger().info(
                'Safety-aware sequence execution: '
                'required waypoints will be executed '
                'individually.'
            )

        for pose_name in pose_names:

            self.execute_pose(
                pose_name,
                poses[pose_name],
            )

    def execute_cached_trajectory(
        self,
        label: str,
        cache_path,
        start_tolerance: float = 0.05,
    ) -> None:

        trajectory, _ = load_trajectory(
            cache_path
        )

        self.execute_prepared_trajectory(
            label,
            trajectory,
            start_tolerance=start_tolerance,
        )


    # ========================================================
    # Perception
    # ========================================================

    def generation_callback(
        self,
        message: UInt64,
    ) -> None:

        generation = int(
            message.data
        )

        if generation <= 0:
            return

        if generation == self.pickup_generation:
            return

        self.pickup_generation = generation
        self.detected_color = None

        self.get_logger().info(
            'Authoritative pickup event available: '
            f'generation {self.pickup_generation}.'
        )

    def ready_callback(
        self,
        message: Bool,
    ) -> None:

        ready = bool(
            message.data
        )

        self.object_ready = ready

        if not ready:
            self.detected_color = None

    def color_callback(
        self,
        message: String,
    ) -> None:

        if not self.object_ready:
            return

        color = (
            message.data
            .strip()
            .lower()
        )

        if color not in VALID_COLORS:
            return

        self.detected_color = color

    def wait_for_box(
        self,
        after_generation: int,
    ):
        self.get_logger().info(
            'Waiting for a settled box...'
        )

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            if (
                self.object_ready
                and self.detected_color
                in VALID_COLORS
                and self.pickup_generation
                > after_generation
            ):
                self.wait_until_safety_allows_motion(
                    'start next sorting cycle'
                )

                color = (
                    self.detected_color
                )

                generation = (
                    self.pickup_generation
                )

                self.get_logger().info(
                    f'{color.upper()} box is ready '
                    f'for sorting '
                    f'(generation {generation}).'
                )

                return (
                    color,
                    generation,
                )

        raise KeyboardInterrupt

    # ========================================================
    # One-time startup
    # ========================================================

    def initialize_executor(
        self,
    ) -> None:

        self.get_logger().info(
            'Loading all RED / GREEN / BLUE '
            'automation configurations once.'
        )

        for color, config in (
            self.configs.items()
        ):
            config['cached'] = (
                config['load_plan']()
            )

            config['half_height'] = (
                config['half_height_fn']()
            )

            # Validate commissioned trajectories once.
            load_trajectory(
                config['transfer_cache']
            )

            load_trajectory(
                config['return_cache']
            )

            self.get_logger().info(
                f'{color.upper()} configuration '
                'and trajectory caches loaded.'
            )

        self.get_logger().info(
            'Connecting persistent executor '
            'to MoveIt...'
        )

        self.wait_for_joint_state()

        self.wait_for_moveit()

        self.wait_for_execute_trajectory()

        for color in (
            'red',
            'green',
            'blue',
        ):
            self.scene_managers[
                color
            ].wait_for_services()

        self.get_logger().info(
            'Restoring persisted bin occupancy '
            'into MoveIt once.'
        )

        for color, config in (
            self.configs.items()
        ):
            config['restore'](
                self,
                self.scene_managers[color],
                config['half_height'],
            )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'PERSISTENT MOVEIT EXECUTOR READY.'
        )

        self.get_logger().info(
            'MoveIt will NOT be reinitialised '
            'between boxes.'
        )

        self.get_logger().info(
            '========================================'
        )

    # ========================================================
    # Prepared trajectory execution
    # ========================================================

    def execute_prepared_trajectory(
        self,
        label: str,
        trajectory: RobotTrajectory,
        start_tolerance: float = 0.05,
        safety_resume_targets=None,
    ) -> None:

        self.wait_until_safety_allows_motion(
            f'prepared trajectory {label}'
        )

        self.wait_for_joint_state()

        start_error = (
            trajectory_start_error(
                trajectory,
                self.current_positions,
            )
        )

        self.get_logger().info(
            'Prepared trajectory '
            f'start-state error for {label}: '
            f'{start_error:.6f} rad'
        )

        if start_error > start_tolerance:
            raise RuntimeError(
                f'Prepared trajectory "{label}" '
                'cannot start safely. '
                f'Maximum joint error is '
                f'{start_error:.6f} rad; '
                f'allowed error is '
                f'{start_tolerance:.6f} rad.'
            )

        point_count = len(
            trajectory
            .joint_trajectory
            .points
        )

        duration = trajectory_duration(
            trajectory
        )

        self.get_logger().info(
            'Executing PREPARED trajectory: '
            f'{label} '
            f'({point_count} points, '
            f'{duration:.3f} s)'
        )

        if safety_resume_targets is None:

            safety_resume_targets = [
                (
                    f'{label} final target',
                    self.trajectory_final_positions(
                        trajectory
                    ),
                )
            ]

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        (
            wrapped_result,
            interrupted,
        ) = self.run_action_goal_with_safety(
            client=(
                self.execute_trajectory_client
            ),
            goal=goal,
            label=(
                f'ExecuteTrajectory -> {label}'
            ),
        )

        if interrupted:

            self.get_logger().warning(
                'SAFETY RESUME: the interrupted '
                'prepared trajectory will NOT be '
                'replayed from its old start state.'
            )

            for (
                target_name,
                target_positions,
            ) in safety_resume_targets:

                self.get_logger().info(
                    'SAFETY RESUME TARGET: '
                    f'{target_name}'
                )

                self.execute_pose(
                    target_name,
                    list(target_positions),
                )

            return

        result = (
            wrapped_result.result
        )

        error_code = (
            result.error_code.val
        )

        self.get_logger().info(
            'Prepared trajectory result '
            f'for {label}: '
            f'error={error_code}, '
            f'action_status='
            f'{wrapped_result.status}'
        )

        if (
            error_code
            != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(
                f'Prepared trajectory '
                f'"{label}" failed with '
                f'error code {error_code}.'
            )

    def get_pickup_solution(
        self,
        color: str,
        config,
        expected_generation: int,
    ):

        prepared = (
            load_prepared_pickup(
                expected_color=color,
                max_age_seconds=120.0,
            )
        )

        prepared_approach_trajectory = None
        prepared_touch_trajectory = None
        prepared_attached_exit_trajectory = None

        direct_handoff_active = (
            self.direct_handoff_generation
            == expected_generation
            and self.direct_handoff_color
            == color
        )

        if prepared is not None:

            prepared_generation = int(
                prepared.get(
                    'generation',
                    -1,
                )
            )

            if (
                prepared_generation
                != expected_generation
            ):

                self.get_logger().warning(
                    f'Prepared {color.upper()} pickup '
                    f'belongs to generation '
                    f'{prepared_generation}, while '
                    f'generation {expected_generation} '
                    'is required.'
                )

                prepared = None

        if prepared is not None:

            prepared_age = max(
                0.0,
                time.time()
                - float(
                    prepared[
                        'created_unix_time'
                    ]
                ),
            )

            generation = int(
                prepared[
                    'generation'
                ]
            )

            box_center = [
                float(value)
                for value in prepared[
                    'box_center'
                ]
            ]

            pickup_approach_joints = [
                float(value)
                for value in prepared[
                    'pickup_approach_joints'
                ]
            ]

            pickup_touch_joints = [
                float(value)
                for value in prepared[
                    'pickup_touch_joints'
                ]
            ]

            pickup_approach = [
                float(value)
                for value in prepared[
                    'pickup_approach'
                ]
            ]

            pickup_touch = [
                float(value)
                for value in prepared[
                    'pickup_touch'
                ]
            ]

            self.get_logger().info(
                f'Using PRECOMPUTED '
                f'{color.upper()} pickup IK: '
                f'generation={generation}, '
                f'age={prepared_age:.2f} s.'
            )

            self.get_logger().info(
                'Live pickup IK calculation skipped.'
            )

            if bool(
                prepared.get(
                    'trajectory_ready',
                    False,
                )
            ):
                try:
                    (
                        approach_trajectory,
                        approach_document,
                    ) = load_trajectory(
                        PREPARED_PICKUP_APPROACH_TRAJECTORY
                    )

                    (
                        touch_trajectory,
                        touch_document,
                    ) = load_trajectory(
                        PREPARED_PICKUP_TOUCH_TRAJECTORY
                    )

                    for (
                        label,
                        document,
                    ) in (
                        (
                            'approach',
                            approach_document,
                        ),
                        (
                            'touch',
                            touch_document,
                        ),
                    ):
                        metadata = (
                            document.get(
                                'metadata',
                                {},
                            )
                        )

                        file_generation = int(
                            metadata.get(
                                'generation',
                                -1,
                            )
                        )

                        file_color = str(
                            metadata.get(
                                'color',
                                '',
                            )
                        ).strip().lower()

                        if (
                            file_generation
                            != generation
                            or file_color
                            != color
                        ):
                            raise RuntimeError(
                                f'Prepared {label} '
                                'trajectory belongs '
                                'to another pickup event.'
                            )

                    self.wait_for_joint_state()

                    if direct_handoff_active:

                        start_error = (
                            trajectory_start_error(
                                touch_trajectory,
                                self.current_positions,
                            )
                        )

                        self.get_logger().info(
                            f'Direct-handoff '
                            f'{color.upper()} pickup-touch '
                            'start-state error: '
                            f'{start_error:.6f} rad'
                        )

                        if start_error <= 0.05:

                            prepared_touch_trajectory = (
                                touch_trajectory
                            )

                            self.get_logger().info(
                                f'PREPARED '
                                f'{color.upper()} '
                                'PICKUP-TOUCH TRAJECTORY '
                                'ACCEPTED FROM DIRECT '
                                'HANDOFF.'
                            )

                        else:

                            self.get_logger().warning(
                                f'Direct {color.upper()} '
                                'handoff does not match '
                                'the prepared pickup-touch '
                                'start state. Online '
                                'pickup-touch planning '
                                'will be used.'
                            )

                    else:

                        start_error = (
                            trajectory_start_error(
                                approach_trajectory,
                                self.current_positions,
                            )
                        )

                        self.get_logger().info(
                            f'Prepared '
                            f'{color.upper()} pickup '
                            'trajectory start-state error: '
                            f'{start_error:.6f} rad'
                        )

                        if start_error <= 0.05:

                            prepared_approach_trajectory = (
                                approach_trajectory
                            )

                            prepared_touch_trajectory = (
                                touch_trajectory
                            )

                            self.get_logger().info(
                                f'PREPARED '
                                f'{color.upper()} PICKUP '
                                'TRAJECTORY ACCEPTED.'
                            )

                        else:

                            self.get_logger().warning(
                                f'Prepared '
                                f'{color.upper()} pickup '
                                'trajectory does not match '
                                'the current robot state. '
                                'Normal MoveIt pickup '
                                'planning will be used.'
                            )

                except Exception as error:
                    self.get_logger().warning(
                        f'Prepared '
                        f'{color.upper()} pickup '
                        'trajectory cannot be used: '
                        f'{error}'
                    )

            if bool(
                prepared.get(
                    'attached_exit_trajectory_ready',
                    False,
                )
            ):
                try:
                    (
                        attached_exit_trajectory,
                        attached_exit_document,
                    ) = load_trajectory(
                        PREPARED_PICKUP_EXIT_ATTACHED_TRAJECTORY
                    )

                    metadata = (
                        attached_exit_document.get(
                            'metadata',
                            {},
                        )
                    )

                    file_generation = int(
                        metadata.get(
                            'generation',
                            -1,
                        )
                    )

                    file_color = str(
                        metadata.get(
                            'color',
                            '',
                        )
                    ).strip().lower()

                    start_pose = str(
                        metadata.get(
                            'start_pose',
                            '',
                        )
                    )

                    goal_pose = str(
                        metadata.get(
                            'goal_pose',
                            '',
                        )
                    )

                    carried_object = str(
                        metadata.get(
                            'carried_object',
                            '',
                        )
                    )

                    expected_carried_object = (
                        f'{color}_box_carried'
                    )

                    if (
                        file_generation
                        != generation
                        or file_color
                        != color
                        or start_pose
                        != 'pickup_touch'
                        or goal_pose
                        != 'pickup_exit'
                        or carried_object
                        != expected_carried_object
                    ):
                        raise RuntimeError(
                            'Prepared attached pickup-exit '
                            'trajectory metadata does not '
                            'match this pickup event.'
                        )

                    prepared_attached_exit_trajectory = (
                        attached_exit_trajectory
                    )

                    self.get_logger().info(
                        f'PREPARED '
                        f'{color.upper()} ATTACHED '
                        'PICKUP-EXIT TRAJECTORY LOADED.'
                    )

                except Exception as error:
                    self.get_logger().warning(
                        f'Prepared '
                        f'{color.upper()} attached '
                        'pickup-exit trajectory '
                        f'cannot be used: {error}'
                    )

                    self.get_logger().warning(
                        'Normal MoveIt pickup_exit '
                        'planning will be used.'
                    )

        else:
            self.get_logger().warning(
                f'No valid precomputed '
                f'{color.upper()} pickup is '
                'available. Falling back to '
                'live perception and IK.'
            )

            box_center = (
                config[
                    'wait_box_pose'
                ](
                    self
                )
            )

            (
                pickup_approach_joints,
                pickup_touch_joints,
                pickup_approach,
                pickup_touch,
            ) = config[
                'pickup_solver'
            ](
                self,
                config['cached'],
                box_center,
                config['half_height'],
            )

        return {
            'box_center': box_center,
            'pickup_approach_joints': (
                pickup_approach_joints
            ),
            'pickup_touch_joints': (
                pickup_touch_joints
            ),
            'pickup_approach': (
                pickup_approach
            ),
            'pickup_touch': (
                pickup_touch
            ),
            'approach_trajectory': (
                prepared_approach_trajectory
            ),
            'touch_trajectory': (
                prepared_touch_trajectory
            ),
            'attached_exit_trajectory': (
                prepared_attached_exit_trajectory
            ),
        }

    # ========================================================
    # Generic colour cycle
    # ========================================================

    def execute_cycle(
        self,
        color: str,
        generation: int,
        allow_next_handoff: bool = True,
    ) -> None:

        self.wait_until_safety_allows_motion(
            f'begin {color.upper()} sorting cycle'
        )

        config = self.configs[
            color
        ]

        cached = config[
            'cached'
        ]

        half_height = config[
            'half_height'
        ]

        manager = self.scene_managers[
            color
        ]

        self.get_logger().info(
            f'Executing {color.upper()} '
            'inside the persistent MoveIt process.'
        )

        direct_handoff_active = (
            self.direct_handoff_generation
            == generation
            and self.direct_handoff_color
            == color
        )

        pickup = (
            self.get_pickup_solution(
                color,
                config,
                generation,
            )
        )

        # The special start state is now owned by this cycle.
        if direct_handoff_active:
            self.direct_handoff_generation = None
            self.direct_handoff_color = None

        self.get_logger().info(
            f'Starting {color.upper()} dynamic '
            'drop IK in parallel with pickup.'
        )

        drop_future = (
            self.drop_ik_executor.submit(
                config['drop_solver'],
                self,
                cached,
                half_height,
            )
        )

        poses = dict(
            cached.poses
        )

        poses[
            'pickup_approach'
        ] = pickup[
            'pickup_approach_joints'
        ]

        poses[
            'pickup_touch'
        ] = pickup[
            'pickup_touch_joints'
        ]

        poses[
            'pickup_exit'
        ] = list(
            cached.pickup_seed
        )

        # Normal runtime intentionally has no bin-staging
        # waypoint. Dynamic bin motion starts directly from
        # pickup_exit and ends at the selected drop pose.

        # Dynamic drop poses are being solved in the
        # background and will be inserted after pickup.

        # ----------------------------------------------------
        # PICKUP
        # ----------------------------------------------------

        if direct_handoff_active:

            self.get_logger().info(
                f'PHASE 1: {color.upper()} cycle '
                'already starts at dynamic '
                'pickup_approach.'
            )

            self.get_logger().info(
                'Skipping pickup_exit -> '
                'pickup_approach.'
            )

            if (
                pickup[
                    'touch_trajectory'
                ]
                is not None
            ):

                self.execute_prepared_trajectory(
                    (
                        f'{color} prepared pickup: '
                        'pickup_approach -> '
                        'pickup_touch'
                    ),
                    pickup[
                        'touch_trajectory'
                    ],
                )

            else:

                self.get_logger().info(
                    'Planning directly from current '
                    'pickup_approach to pickup_touch.'
                )

                self.execute_pose(
                    'pickup_touch',
                    poses[
                        'pickup_touch'
                    ],
                )

        elif (
            pickup[
                'approach_trajectory'
            ]
            is not None
            and pickup[
                'touch_trajectory'
            ]
            is not None
        ):
            self.get_logger().info(
                'PHASE 1: Executing PREPARED '
                f'{color.upper()} pickup trajectory'
            )

            self.execute_prepared_trajectory(
                (
                    f'{color} prepared pickup: '
                    'pickup_exit -> pickup_approach'
                ),
                pickup[
                    'approach_trajectory'
                ],
            )

            self.execute_prepared_trajectory(
                (
                    f'{color} prepared pickup: '
                    'pickup_approach -> pickup_touch'
                ),
                pickup[
                    'touch_trajectory'
                ],
            )

        else:
            self.execute_sequence(
                (
                    'PHASE 1: MoveIt pickup '
                    'approach and touch'
                ),
                [
                    'pickup_approach',
                    'pickup_touch',
                ],
                poses,
            )

        # ----------------------------------------------------
        # COLLECT BACKGROUND DROP IK
        #
        # Do this before suction attach. If drop IK failed,
        # the robot has reached pickup_touch but has not yet
        # taken possession of the box.
        # ----------------------------------------------------

        drop_wait_started = time.monotonic()

        try:
            (
                slot,
                drop_approach_joints,
                drop_release_joints,
                drop_approach,
                drop_release,
            ) = drop_future.result()

        except Exception as error:
            raise RuntimeError(
                f'{color.upper()} background '
                f'drop IK failed: {error}'
            ) from error

        drop_wait_seconds = (
            time.monotonic()
            - drop_wait_started
        )

        poses[
            'drop_approach'
        ] = drop_approach_joints

        poses[
            'drop_release'
        ] = drop_release_joints

        # Per-color physical box sequence follows the bin slot:
        #
        # slot 1 -> red_box / green_box / blue_box
        # slot 2 -> red_box_02 / green_box_02 / blue_box_02
        # ...
        box_instance_index = (
            int(slot.index) + 1
        )

        self.get_logger().info(
            f'Physical {color.upper()} box '
            f'instance: {box_instance_index}/16'
        )

        self.get_logger().info(
            f'BACKGROUND {color.upper()} DROP IK READY. '
            f'Wait at pickup_touch: '
            f'{drop_wait_seconds:.3f} s.'
        )

        # ----------------------------------------------------
        # ATTACH + PICKUP EXIT
        # ----------------------------------------------------

        self.wait_until_safety_allows_motion(
            'before suction ATTACH'
        )

        config[
            'suction'
        ](
            self,
            'attach',
            box_index=box_instance_index,
        )

        manager.attach_box(
            half_height=half_height
        )


        # Suction and MoveIt attachment are now fully
        # confirmed. Start future bin planning only after
        # this reliability-critical handshake has finished.

        self.get_logger().info(
            f'Starting DIRECT '
            f'{color.upper()} drop + retreat '
            'trajectory preplanning in background.'
        )

        bin_plan_future = (
            self.drop_ik_executor.submit(
                self.bin_planner.plan_bin_cycle,
                color=color,
                slot=slot,
                half_height=half_height,
                pickup_exit_joints=list(
                    poses[
                        'pickup_exit'
                    ]
                ),
                drop_approach_joints=list(
                    drop_approach_joints
                ),
                drop_release_joints=list(
                    drop_release_joints
                ),

            )
        )


        attached_exit_trajectory = pickup[
            'attached_exit_trajectory'
        ]

        use_prepared_attached_exit = False

        if attached_exit_trajectory is not None:
            self.wait_for_joint_state()

            attached_exit_start_error = (
                trajectory_start_error(
                    attached_exit_trajectory,
                    self.current_positions,
                )
            )

            self.get_logger().info(
                f'Prepared {color.upper()} attached '
                'pickup-exit start-state error: '
                f'{attached_exit_start_error:.6f} rad'
            )

            if attached_exit_start_error <= 0.05:
                use_prepared_attached_exit = True

            else:
                self.get_logger().warning(
                    f'Prepared {color.upper()} attached '
                    'pickup-exit trajectory does not '
                    'match the current pickup_touch '
                    'state. Normal MoveIt pickup_exit '
                    'planning will be used.'
                )

        if use_prepared_attached_exit:
            self.get_logger().info(
                f'Executing PREPARED '
                f'{color.upper()} attached '
                'pickup_touch -> pickup_exit.'
            )

            self.execute_prepared_trajectory(
                (
                    f'{color} prepared attached pickup: '
                    'pickup_touch -> pickup_exit'
                ),
                attached_exit_trajectory,
            )

        else:
            self.execute_pose(
                'pickup_exit',
                poses['pickup_exit'],
            )

        # ----------------------------------------------------
        # DIRECT DYNAMIC DROP
        #
        # Robot is currently at pickup_exit. The prepared
        # trajectory starts HERE and travels directly to the
        # selected drop_approach and then drop_release.
        # ----------------------------------------------------
        # ----------------------------------------------------

        bin_plan = None

        bin_plan_wait_started = (
            time.monotonic()
        )

        try:
            bin_plan = (
                bin_plan_future.result()
            )

            bin_plan_wait_seconds = (
                time.monotonic()
                - bin_plan_wait_started
            )

            self.get_logger().info(
                f'BACKGROUND '
                f'{color.upper()} BIN PATHS READY. '
                'Wait after pickup_exit: '
                f'{bin_plan_wait_seconds:.3f} s.'
            )

        except Exception as error:
            self.get_logger().warning(
                f'Background '
                f'{color.upper()} bin-path '
                f'preplanning failed: {error}'
            )

            self.get_logger().warning(
                'Using existing online MoveIt '
                'bin planning as fallback.'
            )

        use_prepared_drop = False

        if bin_plan is not None:

            drop_trajectory = (
                bin_plan[
                    'drop_trajectory'
                ]
            )

            self.wait_for_joint_state()

            drop_start_error = (
                trajectory_start_error(
                    drop_trajectory,
                    self.current_positions,
                )
            )

            self.get_logger().info(
                f'Prepared '
                f'{color.upper()} dynamic-drop '
                'start-state error: '
                f'{drop_start_error:.6f} rad'
            )

            if drop_start_error <= 0.05:
                use_prepared_drop = True

            else:
                self.get_logger().warning(
                    f'Prepared {color.upper()} '
                    'dynamic drop does not match '
                    'the actual pickup_exit state. '
                    'Online fallback will be used.'
                )

        if use_prepared_drop:

            self.get_logger().info(
                f'Executing PREPARED '
                f'{color.upper()} dynamic drop.'
            )

            self.execute_prepared_trajectory(
                (
                    f'{color} prepared dynamic '
                    'bin placement'
                ),
                bin_plan[
                    'drop_trajectory'
                ],
                safety_resume_targets=[
                    (
                        'drop_approach',
                        poses[
                            'drop_approach'
                        ],
                    ),
                    (
                        'drop_release',
                        poses[
                            'drop_release'
                        ],
                    ),
                ],
            )

        else:

            self.get_logger().info(
                f'Online {color.upper()} direct-drop '
                'fallback: pickup_exit -> '
                'drop_approach -> drop_release.'
            )

            self.execute_pose(
                'drop_approach',
                poses[
                    'drop_approach'
                ],
            )

            self.execute_pose(
                'drop_release',
                poses[
                    'drop_release'
                ],
            )

        self.wait_until_safety_allows_motion(
            'before suction DETACH'
        )

        config[
            'suction'
        ](
            self,
            'detach',
            box_index=box_instance_index,
        )

        manager.detach_box()

        self.get_logger().info(
            'Retreating from the released '
            f'{color} box.'
        )

        use_prepared_retreat = False

        if bin_plan is not None:

            retreat_trajectory = (
                bin_plan[
                    'retreat_trajectory'
                ]
            )

            self.wait_for_joint_state()

            retreat_start_error = (
                trajectory_start_error(
                    retreat_trajectory,
                    self.current_positions,
                )
            )

            self.get_logger().info(
                f'Prepared '
                f'{color.upper()} retreat '
                'start-state error: '
                f'{retreat_start_error:.6f} rad'
            )

            if retreat_start_error <= 0.05:
                use_prepared_retreat = True

            else:
                self.get_logger().warning(
                    f'Prepared {color.upper()} retreat '
                    'does not match the actual '
                    'drop_release state. '
                    'Online fallback will be used.'
                )

        if use_prepared_retreat:

            self.get_logger().info(
                f'Executing PREPARED '
                f'{color.upper()} '
                'post-drop retreat.'
            )

            self.execute_prepared_trajectory(
                (
                    f'{color} prepared empty retreat: '
                    'drop_release -> drop_approach'
                ),
                bin_plan[
                    'retreat_trajectory'
                ],
            )

        else:

            self.execute_pose(
                'drop_approach',
                poses[
                    'drop_approach'
                ],
            )

        placed_object_id = (
            f'{color}_box_slot_'
            f'{slot.index + 1:02d}'
        )

        manager.add_world_box(
            object_id=placed_object_id,
            center_xyz=(
                slot.x,
                slot.y,
                (
                    config[
                        'bin_surface_z'
                    ]
                    + half_height
                ),
            ),
            size_xyz=(
                config[
                    'box_size_x'
                ],
                config[
                    'box_size_y'
                ],
                2.0 * half_height,
            ),
            frame_id='world',
        )

        config[
            'mark_slot'
        ](
            self,
            slot,
        )

        # ----------------------------------------------------
        # DIRECT EXIT FROM DROP APPROACH
        #
        # The vertical retreat has already brought the robot
        # to drop_approach. From here it goes directly either
        # to the next pickup_approach or to pickup_exit.
        # ----------------------------------------------------

        direct_handoff_used = False

        if allow_next_handoff:

            direct_handoff_used = (
                self.try_direct_next_pickup_handoff(
                    current_color=color,
                    current_generation=generation,
                    start_name='drop_approach',
                    start_joints=poses[
                        'drop_approach'
                    ],
                )
            )

        if not direct_handoff_used:

            self.return_directly_to_pickup_exit(
                color=color,
                start_name='drop_approach',
                start_joints=poses[
                    'drop_approach'
                ],
                pickup_exit_joints=poses[
                    'pickup_exit'
                ],
            )

        else:

            self.get_logger().info(
                'Direct return to pickup_exit '
                'SKIPPED because direct '
                'next-pickup handoff succeeded.'
            )

        self.get_logger().info(
            f'{color.upper()} MOVEIT '
            'AUTOMATION COMPLETE.'
        )

        self.get_logger().info(
            f'{color.upper()}-bin slot used: '
            f'{slot.index + 1}/'
            f'{slot.capacity}'
        )

        self.get_logger().info(
            'Drop release target: '
            f'x={drop_release[0]:.4f}, '
            f'y={drop_release[1]:.4f}, '
            f'z={drop_release[2]:.4f}'
        )



    def return_directly_to_pickup_exit(
        self,
        *,
        color: str,
        start_name: str,
        start_joints,
        pickup_exit_joints,
    ) -> None:

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'DIRECT EMPTY RETURN: '
            f'{color.upper()} '
            f'{start_name} -> pickup_exit'
        )

        self.get_logger().info(
            'No bin-staging waypoint will be used.'
        )

        self.get_logger().info(
            '========================================'
        )

        try:

            trajectory = (
                self.bin_planner.plan_segment(
                    label=(
                        f'{color.upper()} '
                        f'{start_name} -> '
                        'pickup_exit'
                    ),
                    start_positions=list(
                        start_joints
                    ),
                    goal_positions=list(
                        pickup_exit_joints
                    ),
                )
            )

        except Exception as error:

            self.get_logger().warning(
                'Direct pickup_exit return '
                f'preplanning failed: {error}'
            )

            self.get_logger().warning(
                'Using normal MoveIt planning '
                'directly to pickup_exit. '
                'Bin staging remains bypassed.'
            )

            self.execute_pose(
                'pickup_exit',
                list(
                    pickup_exit_joints
                ),
            )

            return

        self.wait_for_joint_state()

        start_error = (
            trajectory_start_error(
                trajectory,
                self.current_positions,
            )
        )

        self.get_logger().info(
            'Direct pickup_exit return '
            'start-state error: '
            f'{start_error:.6f} rad'
        )

        if start_error > 0.05:

            self.get_logger().warning(
                'Direct pickup_exit return '
                'start state does not match '
                'the actual robot state. '
                'Using normal MoveIt planning '
                'directly to pickup_exit.'
            )

            self.execute_pose(
                'pickup_exit',
                list(
                    pickup_exit_joints
                ),
            )

            return

        self.execute_prepared_trajectory(
            (
                f'{color} direct empty return: '
                f'{start_name} -> pickup_exit'
            ),
            trajectory,
        )


    def try_direct_next_pickup_handoff(
        self,
        *,
        current_color: str,
        current_generation: int,
        start_name: str,
        start_joints,
    ) -> bool:

        # A newer detector event must already exist.
        if (
            not self.object_ready
            or self.detected_color
            not in VALID_COLORS
            or self.pickup_generation
            <= current_generation
        ):
            return False

        next_generation = int(
            self.pickup_generation
        )

        next_color = str(
            self.detected_color
        ).strip().lower()

        prepared = (
            load_prepared_pickup(
                expected_color=next_color,
                max_age_seconds=120.0,
            )
        )

        if prepared is None:

            self.get_logger().info(
                'Next box exists, but T8 has not '
                'finished preparing it. Using '
                'pickup_exit fallback.'
            )

            return False

        prepared_generation = int(
            prepared.get(
                'generation',
                -1,
            )
        )

        if (
            prepared_generation
            != next_generation
        ):

            self.get_logger().info(
                'Next detector generation and T8 '
                'generation do not match yet. '
                'Using pickup_exit fallback.'
            )

            return False

        if not bool(
            prepared.get(
                'trajectory_ready',
                False,
            )
        ):

            self.get_logger().info(
                'Next pickup trajectory is not '
                'ready. Using pickup_exit fallback.'
            )

            return False

        next_pickup_approach = [
            float(value)
            for value in prepared[
                'pickup_approach_joints'
            ]
        ]

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'DIRECT NEXT-PICKUP HANDOFF: '
            f'{current_color.upper()} -> '
            f'{next_color.upper()}'
        )

        self.get_logger().info(
            f'Planning {start_name} directly '
            f'to generation {next_generation} '
            'pickup_approach.'
        )

        self.get_logger().info(
            'pickup_exit will be bypassed.'
        )

        self.get_logger().info(
            '========================================'
        )

        try:

            trajectory = (
                self.bin_planner.plan_segment(
                    label=(
                        f'{current_color.upper()} '
                        f'{start_name} -> next '
                        f'{next_color.upper()} '
                        'pickup_approach'
                    ),
                    start_positions=list(
                        start_joints
                    ),
                    goal_positions=(
                        next_pickup_approach
                    ),
                )
            )

        except Exception as error:

            self.get_logger().warning(
                'Direct next-pickup handoff '
                f'planning failed: {error}'
            )

            self.get_logger().warning(
                'Using direct return to '
                'pickup_exit instead.'
            )

            return False

        self.wait_for_joint_state()

        start_error = (
            trajectory_start_error(
                trajectory,
                self.current_positions,
            )
        )

        self.get_logger().info(
            'Direct handoff start-state error: '
            f'{start_error:.6f} rad'
        )

        if start_error > 0.05:

            self.get_logger().warning(
                'Direct handoff start state does '
                'not match the robot. Using '
                'pickup_exit fallback.'
            )

            return False

        self.execute_prepared_trajectory(
            (
                f'{current_color} direct handoff: '
                f'{start_name} -> '
                f'{next_color} pickup_approach'
            ),
            trajectory,
        )

        self.direct_handoff_generation = (
            next_generation
        )

        self.direct_handoff_color = (
            next_color
        )

        self.get_logger().info(
            'DIRECT HANDOFF COMPLETE. '
            f'Robot is already at '
            f'{next_color.upper()} '
            'pickup_approach.'
        )

        return True


    def shutdown_background_workers(
        self,
    ) -> None:
        self.drop_ik_executor.shutdown(
            wait=True,
            cancel_futures=True,
        )

        self.bin_planner.shutdown()
        self.bin_planner_node.destroy_node()


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            'Persistent unified RED / GREEN / BLUE '
            'MoveIt sorting executor.'
        )
    )

    parser.add_argument(
        '--max-cycles',
        type=int,
        default=0,
        help=(
            'Stop after this many successful cycles. '
            '0 means run continuously.'
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

    arguments = (
        parser.parse_args()
    )

    if arguments.max_cycles < 0:
        parser.error(
            '--max-cycles cannot be negative.'
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
                f'--{name} must be > 0 '
                'and <= 1.'
            )

    return arguments


def main() -> int:

    arguments = (
        parse_arguments()
    )

    rclpy.init()

    node = SortingCoordinator(
        velocity=arguments.velocity,
        acceleration=arguments.acceleration,
    )

    completed_cycles = 0
    last_handled_generation = 0

    try:
        node.initialize_executor()

        while rclpy.ok():

            (
                color,
                generation,
            ) = node.wait_for_box(
                after_generation=(
                    last_handled_generation
                )
            )

            node.get_logger().info(
                '========================================'
            )

            node.get_logger().info(
                f'AUTOMATIC CYCLE '
                f'{completed_cycles + 1}: '
                f'{color.upper()}'
            )

            node.get_logger().info(
                '========================================'
            )

            allow_next_handoff = (
                arguments.max_cycles == 0
                or (
                    completed_cycles + 1
                    < arguments.max_cycles
                )
            )

            node.execute_cycle(
                color,
                generation,
                allow_next_handoff=(
                    allow_next_handoff
                ),
            )

            completed_cycles += 1

            last_handled_generation = (
                generation
            )

            node.get_logger().info(
                f'{color.upper()} sorting cycle '
                'completed successfully.'
            )

            # Detector callbacks were processed while the
            # robot was executing. If the next box is already
            # ready, wait_for_box() returns immediately.
            if (
                node.pickup_generation
                > last_handled_generation
                and node.object_ready
            ):
                node.get_logger().info(
                    'Next pickup event is already '
                    'available. Continuing immediately.'
                )

            else:
                node.get_logger().info(
                    'Waiting for the next conveyor box.'
                )

            if (
                arguments.max_cycles > 0
                and completed_cycles
                >= arguments.max_cycles
            ):
                node.get_logger().info(
                    f'Requested '
                    f'{completed_cycles} automatic '
                    'sorting cycles completed.'
                )

                return 0

        return 0

    except KeyboardInterrupt:
        node.get_logger().warning(
            'Persistent sorting coordinator '
            'interrupted.'
        )

        return 130

    except Exception as error:
        node.get_logger().error(
            'SORTING COORDINATOR FAULT: '
            f'{error}'
        )

        return 1

    finally:
        node.shutdown_background_workers()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
