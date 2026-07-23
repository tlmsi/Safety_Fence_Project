#!/usr/bin/env python3

import argparse
import time

import rclpy

from rclpy.node import Node
from std_msgs.msg import Bool


class BoxPresenceWaiter(Node):

    def __init__(self) -> None:
        super().__init__('red_box_presence_waiter')

        self.box_present = False

        self.create_subscription(
            Bool,
            '/perception/object_in_pickup_zone',
            self.callback,
            10,
        )

    def callback(
        self,
        message: Bool,
    ) -> None:
        self.box_present = bool(message.data)


def main() -> int:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--timeout',
        type=float,
        default=20.0,
    )

    arguments = parser.parse_args()

    rclpy.init()
    node = BoxPresenceWaiter()

    try:
        node.get_logger().info(
            'Waiting for a box in the pickup zone...'
        )

        deadline = (
            time.monotonic()
            + arguments.timeout
        )

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.2,
            )

            if node.box_present:
                node.get_logger().info(
                    'Box-present signal received: true'
                )
                return 0

        node.get_logger().error(
            'No box-present=true signal was received.'
        )

        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
