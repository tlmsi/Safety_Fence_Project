#!/usr/bin/env python3

import argparse
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, Optional

import rclpy

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
from std_msgs.msg import Bool, String

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
        # Detector state
        # ----------------------------------------------------

        self.object_ready = False

        self.detected_color: Optional[
            str
        ] = None

        self.pickup_generation = 0

        # Depth 10 is deliberate. We do not want to lose a
        # False -> True pickup transition while executing a
        # robot action.
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
    # Perception
    # ========================================================

    def ready_callback(
        self,
        message: Bool,
    ) -> None:

        ready = bool(
            message.data
        )

        previous_ready = (
            self.object_ready
        )

        self.object_ready = ready

        if (
            ready
            and not previous_ready
        ):
            self.pickup_generation += 1
            self.detected_color = None

            self.get_logger().info(
                'New pickup event available: '
                f'generation '
                f'{self.pickup_generation}.'
            )

        elif not ready:
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
    ) -> None:

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

        goal = (
            ExecuteTrajectory.Goal()
        )

        goal.trajectory = trajectory

        send_future = (
            self.execute_trajectory_client
            .send_goal_async(goal)
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
        )

        goal_handle = (
            send_future.result()
        )

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            raise RuntimeError(
                'MoveIt rejected prepared '
                f'trajectory: {label}'
            )

        result_future = (
            goal_handle
            .get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        wrapped_result = (
            result_future.result()
        )

        if wrapped_result is None:
            raise RuntimeError(
                'MoveIt returned no result '
                'for prepared trajectory: '
                f'{label}'
            )

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
    ) -> None:

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

        pickup = (
            self.get_pickup_solution(
                color,
                config,
            )
        )

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

        staging_name = (
            f'{color}_bin_staging'
        )

        poses[
            staging_name
        ] = list(
            cached.poses[
                'drop_approach'
            ]
        )

        # Dynamic drop poses are being solved in the
        # background and will be inserted after pickup.

        # ----------------------------------------------------
        # PICKUP
        # ----------------------------------------------------

        if (
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

        self.get_logger().info(
            f'BACKGROUND {color.upper()} DROP IK READY. '
            f'Wait at pickup_touch: '
            f'{drop_wait_seconds:.3f} s.'
        )

        # ----------------------------------------------------
        # ATTACH + PICKUP EXIT
        # ----------------------------------------------------

        config[
            'suction'
        ](
            self,
            'attach',
        )

        manager.attach_box(
            half_height=half_height
        )


        # Suction and MoveIt attachment are now fully
        # confirmed. Start future bin planning only after
        # this reliability-critical handshake has finished.

        self.get_logger().info(
            f'Starting COMPLETE '
            f'{color.upper()} bin-side '
            'trajectory preplanning in background.'
        )

        bin_plan_future = (
            self.drop_ik_executor.submit(
                self.bin_planner.plan_bin_cycle,
                color=color,
                slot=slot,
                half_height=half_height,
                staging_joints=list(
                    poses[
                        staging_name
                    ]
                ),
                drop_approach_joints=list(
                    drop_approach_joints
                ),
                drop_release_joints=list(
                    drop_release_joints
                ),

                # Current RED / GREEN / BLUE bin geometry.
                # All three use the same surface height and
                # 60 x 60 mm boxes.
                bin_surface_z=1.02,
                box_size_x=0.06,
                box_size_y=0.06,
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
        # FIXED ATTACHED TRANSFER
        # ----------------------------------------------------

        self.execute_cached_trajectory(
            (
                f'{color} attached transfer: '
                f'pickup_exit to '
                f'{staging_name}'
            ),
            config[
                'transfer_cache'
            ],
        )

        # ----------------------------------------------------
        # DYNAMIC DROP
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
                'Wait after cached transfer: '
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
                    'the actual bin-staging state. '
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
            )

        elif slot.index == 0:

            self.execute_pose(
                'drop_release',
                poses[
                    'drop_release'
                ],
            )

        else:

            self.execute_sequence(
                (
                    'PHASE 2: Dynamic '
                    f'{color}-bin placement'
                ),
                [
                    'drop_approach',
                    'drop_release',
                ],
                poses,
                blended=True,
            )

        config[
            'suction'
        ](
            self,
            'detach',
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
        # FIXED EMPTY RETURN INTERFACE
        # ----------------------------------------------------

        if slot.index != 0:

            use_prepared_staging = False

            if (
                bin_plan is not None
                and bin_plan[
                    'staging_trajectory'
                ] is not None
            ):

                staging_trajectory = (
                    bin_plan[
                        'staging_trajectory'
                    ]
                )

                self.wait_for_joint_state()

                staging_start_error = (
                    trajectory_start_error(
                        staging_trajectory,
                        self.current_positions,
                    )
                )

                self.get_logger().info(
                    f'Prepared '
                    f'{color.upper()} staging '
                    'start-state error: '
                    f'{staging_start_error:.6f} rad'
                )

                if staging_start_error <= 0.05:
                    use_prepared_staging = True

                else:
                    self.get_logger().warning(
                        f'Prepared '
                        f'{color.upper()} staging '
                        'trajectory does not match '
                        'the actual drop_approach '
                        'state. Online fallback '
                        'will be used.'
                    )

            if use_prepared_staging:

                self.get_logger().info(
                    f'Executing PREPARED '
                    f'{color.upper()} '
                    'drop_approach -> '
                    f'{staging_name}.'
                )

                self.execute_prepared_trajectory(
                    (
                        f'{color} prepared empty '
                        'return interface: '
                        'drop_approach -> '
                        f'{staging_name}'
                    ),
                    bin_plan[
                        'staging_trajectory'
                    ],
                )

            else:

                self.execute_pose(
                    staging_name,
                    poses[
                        staging_name
                    ],
                )

        self.execute_cached_trajectory(
            (
                f'{color} empty return: '
                f'{staging_name} to '
                'pickup_exit'
            ),
            config[
                'return_cache'
            ],
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

            node.execute_cycle(
                color
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
