#!/usr/bin/env python3

import sys
from collections import deque
from typing import Deque, Dict, Optional

import numpy as np
import rclpy

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String

from prepared_pickup import (
    PREPARED_PICKUP_FILE,
    PREPARED_PICKUP_APPROACH_TRAJECTORY,
    PREPARED_PICKUP_TOUCH_TRAJECTORY,
    PREPARED_PICKUP_EXIT_ATTACHED_TRAJECTORY,
    clear_prepared_pickup,
    save_prepared_pickup,
)

from prepared_pickup_trajectory import (
    PreparedPickupTrajectoryPlanner,
)

from moveit_trajectory_cache import (
    save_trajectory,
)

from red_automation_runtime import (
    load_cached_plan as load_red_plan,
    red_box_half_height,
    solve_dynamic_pickup as solve_red_pickup,
)

from green_automation_runtime import (
    load_cached_plan as load_green_plan,
    green_box_half_height,
    solve_dynamic_pickup as solve_green_pickup,
)

from blue_automation_runtime import (
    load_cached_plan as load_blue_plan,
    blue_box_half_height,
    solve_dynamic_pickup as solve_blue_pickup,
)


POSE_SAMPLE_COUNT = 8
VALID_COLORS = {
    'red',
    'green',
    'blue',
}


class PickupIKPreprocessor(Node):

    def __init__(self) -> None:
        super().__init__(
            'pickup_ik_preprocessor'
        )

        self.ready = False
        self.color: Optional[str] = None

        self.generation = 0
        self.attempted_generation: Optional[int] = None

        self.samples: Deque[np.ndarray] = deque(
            maxlen=POSE_SAMPLE_COUNT
        )

        self.trajectory_planner = (
            PreparedPickupTrajectoryPlanner(
                self
            )
        )

        self.configs: Dict[str, Dict] = {
            'red': {
                'cached': load_red_plan(),
                'half_height': red_box_half_height(),
                'solver': solve_red_pickup,
            },
            'green': {
                'cached': load_green_plan(),
                'half_height': green_box_half_height(),
                'solver': solve_green_pickup,
            },
            'blue': {
                'cached': load_blue_plan(),
                'half_height': blue_box_half_height(),
                'solver': solve_blue_pickup,
            },
        }

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

        self.create_subscription(
            PoseStamped,
            '/perception/box_pose',
            self.pose_callback,
            10,
        )

        clear_prepared_pickup()

        self.get_logger().info(
            'Parallel pickup IK preprocessor started.'
        )

        self.get_logger().info(
            'Waiting for settled RED / GREEN / BLUE boxes.'
        )

        self.get_logger().info(
            f'Prepared pickup cache: '
            f'{PREPARED_PICKUP_FILE}'
        )

    def ready_callback(
        self,
        message: Bool,
    ) -> None:
        new_ready = bool(
            message.data
        )

        if new_ready and not self.ready:
            self.generation += 1
            self.samples.clear()
            self.color = None
            self.attempted_generation = None

            self.get_logger().info(
                f'New pickup event detected: '
                f'generation {self.generation}.'
            )

        elif not new_ready and self.ready:
            self.samples.clear()
            self.color = None
            self.attempted_generation = None

            clear_prepared_pickup()

            self.get_logger().info(
                'Pickup position cleared. '
                'Prepared solution invalidated.'
            )

        self.ready = new_ready

    def color_callback(
        self,
        message: String,
    ) -> None:
        if not self.ready:
            return

        color = (
            message.data
            .strip()
            .lower()
        )

        if color not in VALID_COLORS:
            return

        if self.color != color:
            self.samples.clear()

            self.get_logger().info(
                f'Generation {self.generation}: '
                f'detected color {color.upper()}.'
            )

        self.color = color

    def pose_callback(
        self,
        message: PoseStamped,
    ) -> None:
        if not self.ready:
            return

        if self.color not in VALID_COLORS:
            return

        if message.header.frame_id not in (
            '',
            'world',
        ):
            return

        point = np.asarray(
            [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ],
            dtype=float,
        )

        if not np.all(
            np.isfinite(point)
        ):
            return

        self.samples.append(
            point
        )

    def prepare_if_ready(self):
        if not self.ready:
            return None

        if self.color not in VALID_COLORS:
            return None

        if (
            len(self.samples)
            < POSE_SAMPLE_COUNT
        ):
            return None

        if (
            self.attempted_generation
            == self.generation
        ):
            return None

        generation = self.generation
        color = self.color

        self.attempted_generation = generation

        stacked = np.vstack(
            list(self.samples)
        )

        box_center = np.mean(
            stacked,
            axis=0,
        )

        spread = (
            np.max(stacked, axis=0)
            - np.min(stacked, axis=0)
        )

        config = self.configs[
            color
        ]

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'PRECOMPUTING {color.upper()} PICKUP IK '
            f'FOR GENERATION {generation}'
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'Averaged next-box centre: '
            f'x={box_center[0]:.4f}, '
            f'y={box_center[1]:.4f}, '
            f'z={box_center[2]:.4f}'
        )

        self.get_logger().info(
            'Pose sample spread: '
            f'dx={spread[0] * 1000.0:.2f} mm, '
            f'dy={spread[1] * 1000.0:.2f} mm, '
            f'dz={spread[2] * 1000.0:.2f} mm'
        )

        try:
            (
                pickup_approach_joints,
                pickup_touch_joints,
                pickup_approach,
                pickup_touch,
            ) = config[
                'solver'
            ](
                self,
                config['cached'],
                box_center,
                config['half_height'],
            )

        except Exception as error:
            self.get_logger().error(
                f'{color.upper()} pickup IK '
                f'precomputation failed: {error}'
            )

            return None

        pickup_exit_joints = list(
            config['cached'].pickup_seed
        )

        trajectory_ready = False
        approach_trajectory = None
        touch_trajectory = None

        attached_exit_trajectory_ready = False
        attached_exit_trajectory = None

        self.get_logger().info(
            f'Preplanning complete '
            f'{color.upper()} pickup trajectory '
            'from fixed pickup_exit.'
        )

        try:
            (
                approach_trajectory,
                touch_trajectory,
            ) = self.trajectory_planner.plan_pickup(
                pickup_exit_joints=(
                    pickup_exit_joints
                ),
                pickup_approach_joints=(
                    pickup_approach_joints
                ),
                pickup_touch_joints=(
                    pickup_touch_joints
                ),
            )

            trajectory_ready = True

        except Exception as error:
            self.get_logger().warning(
                f'{color.upper()} pickup trajectory '
                f'preplanning failed: {error}'
            )

            self.get_logger().warning(
                'Keeping prepared IK available; '
                'the robot runtime can fall back '
                'to normal MoveIt planning.'
            )

        self.get_logger().info(
            f'Preplanning {color.upper()} '
            'ATTACHED pickup_touch -> pickup_exit '
            'trajectory.'
        )

        try:
            attached_exit_trajectory = (
                self.trajectory_planner.plan_attached_exit(
                    color=color,
                    half_height=config['half_height'],
                    pickup_touch_joints=(
                        pickup_touch_joints
                    ),
                    pickup_exit_joints=(
                        pickup_exit_joints
                    ),
                )
            )

            attached_exit_trajectory_ready = True

        except Exception as error:
            self.get_logger().warning(
                f'{color.upper()} attached '
                'pickup-exit trajectory '
                f'preplanning failed: {error}'
            )

            self.get_logger().warning(
                'Existing prepared pickup '
                'trajectories remain available.'
            )

        return {
            'generation': generation,
            'color': color,
            'box_center': box_center,
            'pose_spread': spread,
            'pickup_exit_joints': (
                pickup_exit_joints
            ),
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
            'trajectory_ready': (
                trajectory_ready
            ),
            'approach_trajectory': (
                approach_trajectory
            ),
            'touch_trajectory': (
                touch_trajectory
            ),
            'attached_exit_trajectory_ready': (
                attached_exit_trajectory_ready
            ),
            'attached_exit_trajectory': (
                attached_exit_trajectory
            ),
        }

    def commit(
        self,
        solution,
    ) -> None:
        if solution is None:
            return

        if not self.ready:
            return

        if (
            self.generation
            != solution['generation']
        ):
            return

        if (
            self.color
            != solution['color']
        ):
            return

        if solution['trajectory_ready']:
            save_trajectory(
                PREPARED_PICKUP_APPROACH_TRAJECTORY,
                solution['approach_trajectory'],
                label=(
                    f'{solution["color"]} prepared '
                    'pickup_exit to pickup_approach'
                ),
                metadata={
                    'generation': (
                        solution['generation']
                    ),
                    'color': solution['color'],
                    'start_pose': 'pickup_exit',
                    'goal_pose': 'pickup_approach',
                },
            )

            save_trajectory(
                PREPARED_PICKUP_TOUCH_TRAJECTORY,
                solution['touch_trajectory'],
                label=(
                    f'{solution["color"]} prepared '
                    'pickup_approach to pickup_touch'
                ),
                metadata={
                    'generation': (
                        solution['generation']
                    ),
                    'color': solution['color'],
                    'start_pose': 'pickup_approach',
                    'goal_pose': 'pickup_touch',
                },
            )

        if solution[
            'attached_exit_trajectory_ready'
        ]:
            save_trajectory(
                PREPARED_PICKUP_EXIT_ATTACHED_TRAJECTORY,
                solution[
                    'attached_exit_trajectory'
                ],
                label=(
                    f'{solution["color"]} prepared '
                    'pickup_touch to pickup_exit '
                    'with carried box'
                ),
                metadata={
                    'generation': (
                        solution['generation']
                    ),
                    'color': solution['color'],
                    'start_pose': 'pickup_touch',
                    'goal_pose': 'pickup_exit',
                    'carried_object': (
                        f'{solution["color"]}_box_carried'
                    ),
                },
            )

        save_prepared_pickup(
            generation=solution['generation'],
            color=solution['color'],
            box_center=solution['box_center'],
            pickup_approach_joints=(
                solution['pickup_approach_joints']
            ),
            pickup_touch_joints=(
                solution['pickup_touch_joints']
            ),
            pickup_exit_joints=(
                solution['pickup_exit_joints']
            ),
            pickup_approach=(
                solution['pickup_approach']
            ),
            pickup_touch=(
                solution['pickup_touch']
            ),
            pose_spread=(
                solution['pose_spread']
            ),
            trajectory_ready=(
                solution['trajectory_ready']
            ),
            attached_exit_trajectory_ready=(
                solution[
                    'attached_exit_trajectory_ready'
                ]
            ),
        )

        self.get_logger().info(
            f'PREPARED {solution["color"].upper()} '
            'PICKUP IK IS READY.'
        )

        if solution['trajectory_ready']:
            self.get_logger().info(
                f'PREPARED '
                f'{solution["color"].upper()} '
                'PICKUP TRAJECTORY IS READY.'
            )

        if solution[
            'attached_exit_trajectory_ready'
        ]:
            self.get_logger().info(
                f'PREPARED '
                f'{solution["color"].upper()} '
                'ATTACHED PICKUP-EXIT '
                'TRAJECTORY IS READY.'
            )

        self.get_logger().info(
            f'Saved generation '
            f'{solution["generation"]}: '
            f'{PREPARED_PICKUP_FILE}'
        )


def main() -> int:
    rclpy.init()

    try:
        node = PickupIKPreprocessor()

    except Exception as error:
        print(
            f'ERROR: Cannot initialize pickup IK preprocessor: '
            f'{error}',
            file=sys.stderr,
        )

        if rclpy.ok():
            rclpy.shutdown()

        return 1

    try:
        while rclpy.ok():
            rclpy.spin_once(
                node,
                timeout_sec=0.05,
            )

            solution = (
                node.prepare_if_ready()
            )

            if solution is None:
                continue

            # Refresh queued perception messages before
            # committing a potentially expensive IK result.
            for _ in range(30):
                rclpy.spin_once(
                    node,
                    timeout_sec=0.0,
                )

            node.commit(
                solution
            )

        return 0

    except KeyboardInterrupt:
        node.get_logger().warning(
            'Pickup IK preprocessor interrupted.'
        )

        return 130

    finally:
        clear_prepared_pickup()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
