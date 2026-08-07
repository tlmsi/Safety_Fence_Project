#!/usr/bin/env python3

import argparse
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, Optional

import rclpy

from rclpy.node import Node
from std_msgs.msg import Bool, String


HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[2]
RUN_DIR = WORKSPACE / 'run'

AUTOMATION_RUNNERS: Dict[str, Path] = {
    'red': RUN_DIR / '04_red_automation.sh',
    'green': RUN_DIR / '05_green_automation.sh',
    'blue': RUN_DIR / '06_blue_automation.sh',
}

VALID_COLORS = set(
    AUTOMATION_RUNNERS
)


class SortingCoordinator(Node):

    def __init__(self) -> None:
        super().__init__(
            'sorting_coordinator'
        )

        self.object_ready = False
        self.ready_state_received = False
        self.detected_color: Optional[str] = None

        self.create_subscription(
            Bool,
            '/perception/object_in_pickup_zone',
            self.ready_callback,
            1,
        )

        self.create_subscription(
            String,
            '/perception/detected_color',
            self.color_callback,
            1,
        )

        self.get_logger().info(
            'Unified sorting coordinator started.'
        )

    def ready_callback(
        self,
        message: Bool,
    ) -> None:
        ready = bool(message.data)
        previous_ready = self.object_ready

        self.ready_state_received = True
        self.object_ready = ready

        # A new False -> True transition represents a new
        # pickup event. Require a fresh color message for it.
        if not ready or (
            ready
            and not previous_ready
        ):
            self.detected_color = None

    def color_callback(
        self,
        message: String,
    ) -> None:
        color = message.data.strip().lower()

        if color not in VALID_COLORS:
            self.get_logger().warning(
                f'Ignoring unsupported detected color: '
                f'{color!r}'
            )
            return

        # A color is meaningful to the coordinator only
        # while the detector says a box is actually ready.
        if not self.object_ready:
            return

        self.detected_color = color

    def wait_for_box(
        self,
    ) -> str:
        self.get_logger().info(
            'Waiting for a settled box...'
        )

        while rclpy.ok():
            rclpy.spin_once(
                self,
                timeout_sec=0.10,
            )

            if (
                self.object_ready
                and self.detected_color
                in VALID_COLORS
            ):
                color = self.detected_color

                self.get_logger().info(
                    f'{color.upper()} box is ready '
                    'for sorting.'
                )

                return color

        raise KeyboardInterrupt

    def refresh_after_cycle(
        self,
        refresh_seconds: float = 0.75,
    ) -> None:
        self.get_logger().info(
            'Refreshing detector state for '
            'the next box.'
        )

        # Clear the decision that launched the previous
        # automation.
        self.object_ready = False
        self.detected_color = None
        self.ready_state_received = False

        # Drain messages accumulated while the coordinator
        # was blocked running the MoveIt subprocess and obtain
        # the detector's current state.
        deadline = (
            time.monotonic()
            + refresh_seconds
        )

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

        # Even when a new box is already ready, require one
        # fresh color message after the refresh. The detector
        # publishes that color continuously while ready.
        self.detected_color = None

        if self.object_ready:
            self.get_logger().info(
                'A new box is already present '
                'at the pickup position.'
            )
        else:
            self.get_logger().info(
                'No box is currently ready. '
                'Waiting for the conveyor.'
            )

    def run_automation(
        self,
        color: str,
    ) -> None:
        runner = AUTOMATION_RUNNERS[color]

        if not runner.is_file():
            raise RuntimeError(
                f'{color} automation runner '
                f'is missing: {runner}'
            )

        self.get_logger().info(
            f'Starting cached '
            f'{color.upper()} MoveIt automation.'
        )

        result = subprocess.run(
            [
                str(runner),
                'cached',
            ],
            cwd=str(WORKSPACE),
            check=False,
        )

        if result.returncode != 0:
            raise RuntimeError(
                f'{color.upper()} automation failed '
                f'with exit code '
                f'{result.returncode}.'
            )

        self.get_logger().info(
            f'{color.upper()} sorting cycle '
            'completed successfully.'
        )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Automatically select and execute '
            'the cached red, green, or blue '
            'MoveIt sorting cycle.'
        )
    )

    parser.add_argument(
        '--max-cycles',
        type=int,
        default=0,
        help=(
            'Stop after this many successful '
            'cycles. 0 means run continuously.'
        ),
    )

    arguments = parser.parse_args()

    if arguments.max_cycles < 0:
        parser.error(
            '--max-cycles cannot be negative.'
        )

    return arguments


def validate_runners() -> None:
    missing = [
        str(path)
        for path in AUTOMATION_RUNNERS.values()
        if not path.is_file()
    ]

    if missing:
        raise RuntimeError(
            'Missing automation runner(s): '
            + ', '.join(missing)
        )


def main() -> int:
    arguments = parse_arguments()

    try:
        validate_runners()

    except Exception as error:
        print(
            f'ERROR: {error}',
            file=sys.stderr,
        )
        return 1

    rclpy.init()

    node = SortingCoordinator()

    completed_cycles = 0

    try:
        while rclpy.ok():
            color = node.wait_for_box()

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

            node.run_automation(
                color
            )

            completed_cycles += 1

            node.refresh_after_cycle()

            if (
                arguments.max_cycles > 0
                and completed_cycles
                >= arguments.max_cycles
            ):
                node.get_logger().info(
                    f'Requested {completed_cycles} '
                    'automatic sorting cycles completed.'
                )

                return 0

        return 0

    except KeyboardInterrupt:
        node.get_logger().warning(
            'Sorting coordinator interrupted.'
        )

        return 130

    except Exception as error:
        node.get_logger().error(
            f'SORTING COORDINATOR FAULT: '
            f'{error}'
        )

        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
