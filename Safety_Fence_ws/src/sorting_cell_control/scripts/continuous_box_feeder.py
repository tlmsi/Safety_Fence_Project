#!/usr/bin/env python3

import json
import random
import subprocess
import time
import xml.etree.ElementTree as ET

from copy import deepcopy
from pathlib import Path

import rclpy

from rclpy.node import Node
from std_msgs.msg import String


WORLD_NAME = 'sorting_cell_world'

CREATE_SERVICE = (
    f'/world/{WORLD_NAME}/create'
)

SCENE_SERVICE = (
    f'/world/{WORLD_NAME}/scene/info'
)

MAX_BOX_INDEX = 16

SPAWN_X = -1.235

SPAWN_Y_CENTER = 0.450
SPAWN_Y_RANGE = 0.120

SPAWN_Z = 0.940

VALID_COLORS = (
    'red',
    'green',
    'blue',
)

DUPLICATE_CLEAR_GUARD_SECONDS = 8.0

STATE_FILE = Path(
    '/tmp/safety_fence_box_feeder_state.json'
)


class ContinuousBoxFeeder(Node):

    def __init__(self):

        super().__init__(
            'continuous_box_feeder'
        )

        self.workspace = (
            Path(__file__).resolve().parents[3]
        )

        self.world_file = (
            self.workspace
            / 'src'
            / 'sorting_cell_gazebo'
            / 'worlds'
            / 'sorting_cell_world.sdf'
        )

        if not self.world_file.is_file():

            raise RuntimeError(
                f'World missing: '
                f'{self.world_file}'
            )

        self.next_index = {
            'red': 2,
            'green': 2,
            'blue': 2,
        }

        self.last_clear_time = {
            'red': float('-inf'),
            'green': float('-inf'),
            'blue': float('-inf'),
        }

        self.load_state()

        self.subscription = (
            self.create_subscription(
                String,
                '/perception/pickup_cleared_color',
                self.clear_callback,
                10,
            )
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'CONTINUOUS BOX FEEDER READY'
        )

        self.get_logger().info(
            '16 RED + 16 GREEN + 16 BLUE = 48'
        )

        self.get_logger().info(
            'Spawned boxes begin FREE.'
        )

        self.get_logger().info(
            'Feeder performs NO suction commands.'
        )

        self.get_logger().info(
            '========================================'
        )

    @staticmethod
    def model_name(
        color,
        index,
    ):

        if index == 1:

            return (
                f'{color}_box'
            )

        return (
            f'{color}_box_'
            f'{index:02d}'
        )

    @staticmethod
    def random_spawn_y():

        return random.uniform(
            SPAWN_Y_CENTER - SPAWN_Y_RANGE,
            SPAWN_Y_CENTER + SPAWN_Y_RANGE,
        )

    def remaining_color_bag(self):

        bag = []

        for color in VALID_COLORS:

            remaining = max(
                0,
                MAX_BOX_INDEX
                - self.next_index[color]
                + 1,
            )

            bag.extend(
                [color] * remaining
            )

        return bag

    def load_state(self):

        if not STATE_FILE.exists():
            return

        try:

            payload = json.loads(
                STATE_FILE.read_text()
            )

            for color in VALID_COLORS:

                value = int(
                    payload.get(
                        color,
                        2,
                    )
                )

                self.next_index[color] = max(
                    2,
                    min(
                        MAX_BOX_INDEX + 1,
                        value,
                    ),
                )

            self.get_logger().info(
                f'Restored feeder state: '
                f'{self.next_index}'
            )

        except Exception as error:

            self.get_logger().warning(
                f'Ignoring feeder state: '
                f'{error}'
            )

    def save_state(self):

        temporary = Path(
            str(STATE_FILE)
            + '.tmp'
        )

        temporary.write_text(
            json.dumps(
                self.next_index,
                indent=2,
                sort_keys=True,
            )
        )

        temporary.replace(
            STATE_FILE
        )

    def entity_exists(
        self,
        name,
    ):

        result = subprocess.run(
            [
                'gz',
                'service',
                '-s',
                SCENE_SERVICE,
                '--reqtype',
                'gz.msgs.Empty',
                '--reptype',
                'gz.msgs.Scene',
                '--timeout',
                '3000',
                '--req',
                '',
            ],
            capture_output=True,
            text=True,
            check=False,
        )

        output = (
            result.stdout
            + '\n'
            + result.stderr
        )

        return (
            f'name: "{name}"'
            in output
        )

    def create_sdf(
        self,
        color,
        index,
        spawn_y,
    ):

        tree = ET.parse(
            self.world_file
        )

        root = tree.getroot()

        world = root.find(
            'world'
        )

        if world is None:

            raise RuntimeError(
                'World element not found'
            )

        source_name = (
            f'{color}_box'
        )

        source = world.find(
            f"./model[@name='{source_name}']"
        )

        if source is None:

            raise RuntimeError(
                f'{source_name} missing '
                'from world'
            )

        model = deepcopy(
            source
        )

        name = self.model_name(
            color,
            index,
        )

        model.set(
            'name',
            name,
        )

        pose = model.find(
            'pose'
        )

        if pose is None:

            pose = ET.Element(
                'pose'
            )

            model.insert(
                0,
                pose,
            )

        pose.text = (
            f'{SPAWN_X:.6f} '
            f'{spawn_y:.6f} '
            f'{SPAWN_Z:.6f} '
            '0 0 0'
        )

        sdf = ET.Element(
            'sdf',
            {
                'version': (
                    root.attrib.get(
                        'version',
                        '1.10',
                    )
                )
            },
        )

        sdf.append(
            model
        )

        ET.indent(
            sdf,
            space='  ',
        )

        output = Path(
            f'/tmp/'
            f'safety_fence_{name}.sdf'
        )

        ET.ElementTree(
            sdf
        ).write(
            output,
            encoding='unicode',
        )

        return output

    def spawn(
        self,
        color,
        index,
    ):

        name = self.model_name(
            color,
            index,
        )

        if self.entity_exists(
            name
        ):

            self.get_logger().error(
                f'Refusing duplicate entity: '
                f'{name} already exists.'
            )

            return False

        spawn_y = self.random_spawn_y()

        sdf = self.create_sdf(
            color,
            index,
            spawn_y,
        )

        request = (
            f'sdf_filename: "{sdf}", '
            'allow_renaming: false'
        )

        for attempt in range(
            1,
            4,
        ):

            self.get_logger().info(
                f'Spawning FREE box '
                f'{name} at conveyor entrance '
                f'y={spawn_y:+.3f} m '
                f'({attempt}/3).'
            )

            result = subprocess.run(
                [
                    'gz',
                    'service',
                    '-s',
                    CREATE_SERVICE,
                    '--reqtype',
                    'gz.msgs.EntityFactory',
                    '--reptype',
                    'gz.msgs.Boolean',
                    '--timeout',
                    '5000',
                    '--req',
                    request,
                ],
                capture_output=True,
                text=True,
                check=False,
            )

            output = (
                result.stdout
                + '\n'
                + result.stderr
            ).strip()

            normalized = (
                output
                .lower()
                .replace(' ', '')
            )

            created = (
                result.returncode == 0
                and 'data:true'
                in normalized
            )

            if not created:

                time.sleep(
                    0.20
                )

                created = (
                    self.entity_exists(
                        name
                    )
                )

            if created:

                self.get_logger().info(
                    f'SPAWN COMPLETE: '
                    f'{name} is FREE '
                    'on the conveyor at '
                    f'y={spawn_y:+.3f} m.'
                )

                return True

            self.get_logger().warning(
                f'Spawn attempt failed '
                f'for {name}: {output}'
            )

            time.sleep(
                0.50
            )

        return False

    def clear_callback(
        self,
        message,
    ):

        cleared_color = (
            message.data
            .strip()
            .lower()
        )

        if cleared_color not in VALID_COLORS:
            return

        now = time.monotonic()

        elapsed = (
            now
            - self.last_clear_time[
                cleared_color
            ]
        )

        if (
            elapsed
            < DUPLICATE_CLEAR_GUARD_SECONDS
        ):

            self.get_logger().warning(
                f'IGNORING duplicate '
                f'{cleared_color.upper()} '
                'pickup-clear event after '
                f'{elapsed:.2f} s.'
            )

            return

        self.last_clear_time[
            cleared_color
        ] = now

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'{cleared_color.upper()} PICKUP '
            'PHYSICALLY CLEARED'
        )

        remaining_bag = (
            self.remaining_color_bag()
        )

        if not remaining_bag:

            self.get_logger().info(
                'All 48 boxes have been introduced.'
            )

            self.get_logger().info(
                'No replacement spawned.'
            )

            self.get_logger().info(
                '========================================'
            )

            return

        spawn_color = random.choice(
            remaining_bag
        )

        index = self.next_index[
            spawn_color
        ]

        name = self.model_name(
            spawn_color,
            index,
        )

        self.get_logger().info(
            'SHUFFLED NEXT COLOR: '
            f'{spawn_color.upper()}'
        )

        self.get_logger().info(
            f'Introducing exactly ONE '
            f'FREE shuffled replacement: '
            f'{name}'
        )

        if not self.spawn(
            spawn_color,
            index,
        ):

            self.get_logger().error(
                f'FAILED TO SPAWN {name}'
            )

            self.get_logger().error(
                'Counter was NOT advanced.'
            )

            return

        self.next_index[
            spawn_color
        ] = (
            index + 1
        )

        self.save_state()

        remaining = {
            color: max(
                0,
                MAX_BOX_INDEX
                - self.next_index[color]
                + 1,
            )
            for color in VALID_COLORS
        }

        self.get_logger().info(
            f'{spawn_color.upper()} introduced: '
            f'{index}/16'
        )

        self.get_logger().info(
            'Remaining unintroduced boxes: '
            f'RED={remaining["red"]}, '
            f'GREEN={remaining["green"]}, '
            f'BLUE={remaining["blue"]}'
        )

        self.get_logger().info(
            'Three-box conveyor pipeline '
            'restored with shuffled color.'
        )

        self.get_logger().info(
            '========================================'
        )


def main():

    rclpy.init()

    node = ContinuousBoxFeeder()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':
    main()
