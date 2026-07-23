#!/usr/bin/env python3

import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "endpoint",
        choices=("publisher", "subscriber"),
    )
    parser.add_argument("topic")
    parser.add_argument(
        "timeout",
        nargs="?",
        type=float,
        default=60.0,
    )

    args = parser.parse_args()

    rclpy.init()

    node = Node(
        f"safety_fence_endpoint_waiter_{os.getpid()}"
    )

    deadline = time.monotonic() + args.timeout

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(
                node,
                timeout_sec=0.2,
            )

            if args.endpoint == "publisher":
                count = node.count_publishers(args.topic)
            else:
                count = node.count_subscribers(args.topic)

            if count > 0:
                print(
                    f"Ready: {args.endpoint} on "
                    f"{args.topic}"
                )
                return 0

        print(
            f"Timed out waiting for {args.endpoint} "
            f"on {args.topic}",
            file=sys.stderr,
        )
        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
