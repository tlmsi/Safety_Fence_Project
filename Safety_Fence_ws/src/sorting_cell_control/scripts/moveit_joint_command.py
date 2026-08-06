#!/usr/bin/env python3

import argparse
import sys
import time
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


GROUP_NAME = "ur_manipulator"

GROUP_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class MoveItJointCommand(Node):

    def __init__(
        self,
        joint_name: str,
        delta: float,
        plan_only: bool,
    ) -> None:
        super().__init__("sorting_cell_moveit_joint_command")

        self.joint_name = joint_name
        self.delta = delta
        self.plan_only = plan_only

        self.current_positions: Dict[str, float] = {}

        self.joint_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            qos_profile_sensor_data,
        )

        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            "/move_action",
        )

    def joint_state_callback(
        self,
        message: JointState,
    ) -> None:
        self.current_positions.update(
            dict(zip(message.name, message.position))
        )

    def wait_for_complete_joint_state(
        self,
        timeout_seconds: float = 10.0,
    ) -> bool:
        deadline = time.monotonic() + timeout_seconds

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)

            if all(
                name in self.current_positions
                for name in GROUP_JOINTS
            ):
                return True

        return False

    def create_goal(self) -> MoveGroup.Goal:
        target_positions = {
            name: self.current_positions[name]
            for name in GROUP_JOINTS
        }

        target_positions[self.joint_name] += self.delta

        constraints = Constraints()
        constraints.name = "sorting_cell_joint_target"

        for name in GROUP_JOINTS:
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = target_positions[name]
            constraint.tolerance_above = 0.001
            constraint.tolerance_below = 0.001
            constraint.weight = 1.0

            constraints.joint_constraints.append(constraint)

        goal = MoveGroup.Goal()

        request = goal.request
        request.group_name = GROUP_NAME
        request.pipeline_id = "ompl"
        request.planner_id = ""
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.10
        request.max_acceleration_scaling_factor = 0.10

        # An empty differential start state means:
        # use MoveIt's current monitored robot state.
        request.start_state.is_diff = True
        request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = self.plan_only
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        print("\nCurrent and target values:")

        for name in GROUP_JOINTS:
            current = self.current_positions[name]
            target = target_positions[name]

            print(
                f"  {name:24s}"
                f" current={current: .6f}"
                f" target={target: .6f}"
            )

        return goal

    def run(self) -> bool:
        if self.joint_name not in GROUP_JOINTS:
            self.get_logger().error(
                f"Unknown group joint: {self.joint_name}"
            )
            return False

        self.get_logger().info(
            "Waiting for a complete /joint_states message..."
        )

        if not self.wait_for_complete_joint_state():
            self.get_logger().error(
                "Timed out waiting for all six UR joints."
            )
            return False

        self.get_logger().info(
            "Waiting for MoveIt's /move_action server..."
        )

        if not self.move_group_client.wait_for_server(
            timeout_sec=15.0
        ):
            self.get_logger().error(
                "/move_action is unavailable. "
                "Make sure move_group is running."
            )
            return False

        goal = self.create_goal()

        mode = "PLAN ONLY" if self.plan_only else "PLAN AND EXECUTE"

        self.get_logger().info(
            f"Sending {mode} request: "
            f"{self.joint_name} delta={self.delta:.4f} rad"
        )

        send_future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                "MoveIt rejected the goal request."
            )
            return False

        self.get_logger().info("MoveIt accepted the request.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error(
                "No result was returned by MoveIt."
            )
            return False

        result = wrapped_result.result
        error_code = result.error_code.val

        trajectory = result.planned_trajectory.joint_trajectory
        point_count = len(trajectory.points)

        duration = 0.0

        if trajectory.points:
            final_time = trajectory.points[-1].time_from_start
            duration = (
                float(final_time.sec)
                + float(final_time.nanosec) / 1_000_000_000.0
            )

        print("\n=== MoveIt result ===")
        print(f"Error code:          {error_code}")
        print(f"Planning time:       {result.planning_time:.4f} s")
        print(f"Trajectory points:   {point_count}")
        print(f"Trajectory duration: {duration:.4f} s")

        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"MoveIt failed with error code {error_code}."
            )
            return False

        if point_count <= 1 or duration <= 0.0:
            self.get_logger().error(
                "MoveIt returned an empty or zero-duration motion."
            )
            return False

        if self.plan_only:
            self.get_logger().info(
                "Collision-aware planning succeeded. "
                "No execution was requested."
            )
        else:
            self.get_logger().info(
                "Collision-aware planning and execution succeeded."
            )

        return True


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Move one UR joint through the running "
            "MoveIt move_group action server."
        )
    )

    parser.add_argument(
        "--joint",
        default="shoulder_pan_joint",
        choices=GROUP_JOINTS,
    )

    parser.add_argument(
        "--delta",
        type=float,
        default=0.10,
        help="Relative target change in radians.",
    )

    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="Plan without executing.",
    )

    arguments, _ = parser.parse_known_args()
    return arguments


def main() -> int:
    arguments = parse_arguments()

    rclpy.init()

    node = MoveItJointCommand(
        joint_name=arguments.joint,
        delta=arguments.delta,
        plan_only=arguments.plan_only,
    )

    try:
        succeeded = node.run()
        return 0 if succeeded else 1

    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted.")
        return 130

    except Exception as error:
        node.get_logger().exception(
            f"MoveIt command failed: {error}"
        )
        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
