#!/usr/bin/env python3

import json
import time
from pathlib import Path
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


class HomePoseSaver(Node):

    def __init__(self) -> None:
        super().__init__('save_home_pose')

        self.positions: Optional[Dict[str, float]] = None

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data,
        )

    def joint_state_callback(
        self,
        message: JointState,
    ) -> None:
        available = dict(
            zip(message.name, message.position)
        )

        if not all(
            joint in available
            for joint in JOINT_NAMES
        ):
            return

        self.positions = {
            joint: float(available[joint])
            for joint in JOINT_NAMES
        }


def main() -> int:
    script_dir = Path(__file__).resolve().parent
    config_dir = script_dir.parent / 'config'
    output_file = config_dir / 'home_pose.json'

    config_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    rclpy.init()
    node = HomePoseSaver()

    try:
        node.get_logger().info(
            'Waiting for the current robot pose...'
        )

        deadline = time.monotonic() + 15.0

        while (
            rclpy.ok()
            and node.positions is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.2,
            )

        if node.positions is None:
            node.get_logger().error(
                'No complete UR joint state was received.'
            )
            return 1

        data = {
            'name': 'home',
            'description': (
                'Verified startup pose captured from '
                'the Safety Fence simulation.'
            ),
            'joint_names': JOINT_NAMES,
            'positions': [
                node.positions[joint]
                for joint in JOINT_NAMES
            ],
            'positions_by_name': node.positions,
        }

        output_file.write_text(
            json.dumps(
                data,
                indent=2,
            )
            + '\n'
        )

        node.get_logger().info(
            'Home pose saved successfully:'
        )

        node.get_logger().info(
            f'  {output_file}'
        )

        for joint in JOINT_NAMES:
            node.get_logger().info(
                f'  {joint}: '
                f'{node.positions[joint]:.6f} rad'
            )

        return 0

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
