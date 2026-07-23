#!/usr/bin/env python3

import argparse
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
import xacro

from ament_index_python.packages import (
    get_package_share_directory,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState


JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

HERE = Path(__file__).resolve().parent
CONTROL = HERE.parent
WS = CONTROL.parent.parent

WORLD = (
    WS
    / 'src/sorting_cell_gazebo/worlds/'
    / 'sorting_cell_world.sdf'
)

CONVEYOR = (
    WS
    / 'src/sorting_cell_gazebo/models/'
    / 'conveyor/model.sdf'
)

ROBOT_XACRO = (
    WS
    / 'src/sorting_cell_description/urdf/'
    / 'sorting_cell_ur.urdf.xacro'
)

DEFAULT_OUTPUT = (
    CONTROL
    / 'config/red_pickup_approach_ik.json'
)


def numbers(text, expected_count):
    values = [
        float(value)
        for value in text.split()
    ]

    if len(values) != expected_count:
        raise RuntimeError(
            f'Expected {expected_count} values: {text}'
        )

    return values


def element_pose(element):
    pose = element.find('pose')

    if pose is None or not pose.text:
        return [0.0] * 6

    return numbers(
        pose.text,
        6,
    )


def pose_matrix(values):
    x, y, z, roll, pitch, yaw = values

    transform = np.eye(4)

    transform[:3, :3] = Rotation.from_euler(
        'xyz',
        [roll, pitch, yaw],
    ).as_matrix()

    transform[:3, 3] = [
        x,
        y,
        z,
    ]

    return transform


def calculate_pickup_targets(
    clearance,
    contact_offset,
):
    world_root = ET.parse(
        WORLD
    ).getroot()

    conveyor_root = ET.parse(
        CONVEYOR
    ).getroot()

    conveyor_include = next(
        (
            include
            for include in world_root.iter('include')
            if (
                include.findtext('uri')
                or ''
            ).strip().rstrip('/').endswith(
                'models/conveyor'
            )
        ),
        None,
    )

    if conveyor_include is None:
        raise RuntimeError(
            'Conveyor include was not found '
            'in the world file.'
        )

    pickup_marker = next(
        (
            visual
            for visual in conveyor_root.iter('visual')
            if 'pickup_zone' in visual.attrib.get(
                'name',
                '',
            )
        ),
        None,
    )

    belt_collision = next(
        (
            collision
            for collision
            in conveyor_root.iter('collision')
            if 'belt' in collision.attrib.get(
                'name',
                '',
            )
        ),
        None,
    )

    if pickup_marker is None:
        raise RuntimeError(
            'Pickup-zone marker was not found.'
        )

    if belt_collision is None:
        raise RuntimeError(
            'Conveyor belt collision was not found.'
        )

    belt_size_element = belt_collision.find(
        './geometry/box/size'
    )

    if (
        belt_size_element is None
        or not belt_size_element.text
    ):
        raise RuntimeError(
            'Conveyor belt dimensions are missing.'
        )

    belt_size = numbers(
        belt_size_element.text,
        3,
    )

    belt_pose = element_pose(
        belt_collision
    )

    belt_top_local_z = (
        belt_pose[2]
        + belt_size[2] / 2.0
    )

    red_box_height = 0.06

    for model in world_root.iter('model'):
        if model.attrib.get('name') != 'red_box':
            continue

        size_element = model.find(
            './link/visual/geometry/box/size'
        )

        if (
            size_element is not None
            and size_element.text
        ):
            red_box_height = numbers(
                size_element.text,
                3,
            )[2]

        break

    marker_pose = element_pose(
        pickup_marker
    )

    # Four millimetres above the theoretical box surface.
    local_contact = np.array(
        [
            marker_pose[0],
            marker_pose[1],
            belt_top_local_z
            + red_box_height
            + contact_offset,
            1.0,
        ],
        dtype=float,
    )

    world_from_conveyor = pose_matrix(
        element_pose(conveyor_include)
    )

    contact_world = (
        world_from_conveyor
        @ local_contact
    )[:3]

    approach_world = contact_world.copy()
    approach_world[2] += clearance

    return (
        approach_world,
        contact_world,
    )


def origin_matrix(joint):
    origin = joint.find('origin')

    xyz = [0.0] * 3
    rpy = [0.0] * 3

    if origin is not None:
        if origin.attrib.get('xyz'):
            xyz = numbers(
                origin.attrib['xyz'],
                3,
            )

        if origin.attrib.get('rpy'):
            rpy = numbers(
                origin.attrib['rpy'],
                3,
            )

    transform = np.eye(4)

    transform[:3, :3] = Rotation.from_euler(
        'xyz',
        rpy,
    ).as_matrix()

    transform[:3, 3] = xyz

    return transform


def generate_robot_urdf():
    simulation_share = Path(
        get_package_share_directory(
            'ur_simulation_gz'
        )
    )

    controller_file = (
        simulation_share
        / 'config/ur_controllers.yaml'
    )

    document = xacro.process_file(
        str(ROBOT_XACRO),
        mappings={
            'name': 'ur',
            'ur_type': 'ur5e',
            'simulation_controllers': str(
                controller_file
            ),
        },
    )

    return document.toxml()


def build_chain(urdf_text):
    root = ET.fromstring(
        urdf_text
    )

    joints_by_child = {}

    for joint in root.findall('joint'):
        parent = joint.find('parent')
        child = joint.find('child')

        if parent is None or child is None:
            continue

        child_link = child.attrib['link']

        joints_by_child[child_link] = {
            'name': joint.attrib['name'],
            'type': joint.attrib.get(
                'type',
                'fixed',
            ),
            'parent': parent.attrib['link'],
            'origin': origin_matrix(joint),
            'element': joint,
        }

    reversed_chain = []
    current_link = 'suction_tip'

    while current_link != 'world':
        if current_link not in joints_by_child:
            raise RuntimeError(
                'No parent joint was found for '
                f'link: {current_link}'
            )

        joint = joints_by_child[
            current_link
        ]

        reversed_chain.append(
            joint
        )

        current_link = joint['parent']

    chain = list(
        reversed(reversed_chain)
    )

    lower = []
    upper = []

    for joint_name in JOINTS:
        matching = [
            joint
            for joint in chain
            if joint['name'] == joint_name
        ]

        if len(matching) != 1:
            raise RuntimeError(
                f'Joint {joint_name} is missing '
                'from the tool chain.'
            )

        joint = matching[0]
        element = joint['element']

        axis_element = element.find('axis')

        axis_text = '1 0 0'

        if axis_element is not None:
            axis_text = axis_element.attrib.get(
                'xyz',
                axis_text,
            )

        axis = np.array(
            numbers(
                axis_text,
                3,
            ),
            dtype=float,
        )

        axis_norm = float(
            np.linalg.norm(axis)
        )

        if axis_norm < 1e-12:
            raise RuntimeError(
                f'Invalid joint axis: {joint_name}'
            )

        joint['axis'] = axis / axis_norm

        limit = element.find('limit')

        if (
            joint['type'] == 'continuous'
            or limit is None
        ):
            lower.append(
                -2.0 * math.pi
            )

            upper.append(
                2.0 * math.pi
            )

        else:
            lower.append(
                float(
                    limit.attrib.get(
                        'lower',
                        -2.0 * math.pi,
                    )
                )
            )

            upper.append(
                float(
                    limit.attrib.get(
                        'upper',
                        2.0 * math.pi,
                    )
                )
            )

    return (
        chain,
        np.array(lower),
        np.array(upper),
    )


def forward_kinematics(
    chain,
    joint_positions,
):
    position_by_name = dict(
        zip(
            JOINTS,
            joint_positions,
        )
    )

    transform = np.eye(4)

    for joint in chain:
        transform = (
            transform
            @ joint['origin']
        )

        if joint['type'] not in (
            'revolute',
            'continuous',
        ):
            continue

        motion = np.eye(4)

        motion[:3, :3] = (
            Rotation.from_rotvec(
                joint['axis']
                * position_by_name[
                    joint['name']
                ]
            ).as_matrix()
        )

        transform = (
            transform
            @ motion
        )

    return transform


def wrapped_difference(
    target,
    start,
):
    difference = target - start

    return np.arctan2(
        np.sin(difference),
        np.cos(difference),
    )


def downward_orientation(yaw):
    # Suction-tip local positive Z points downward.
    tool_z_down = np.diag(
        [
            1.0,
            -1.0,
            -1.0,
        ]
    )

    return (
        Rotation.from_euler(
            'z',
            yaw,
        ).as_matrix()
        @ tool_z_down
    )


def solve_ik(
    chain,
    current,
    lower,
    upper,
    target_position,
):
    seeds = [
        current.copy(),
        np.array([
            0.8,
            -1.4,
            1.7,
            -1.9,
            -1.57,
            0.0,
        ]),
        np.array([
            1.2,
            -1.2,
            1.4,
            -1.8,
            -1.57,
            0.0,
        ]),
        np.array([
            0.5,
            -1.6,
            1.9,
            -1.9,
            -1.57,
            0.0,
        ]),
        np.array([
            -0.6,
            -1.4,
            1.7,
            -1.9,
            -1.57,
            0.0,
        ]),
    ]

    random_generator = (
        np.random.default_rng(
            20260723
        )
    )

    for scale in (
        0.15,
        0.30,
        0.50,
    ):
        for _ in range(8):
            seeds.append(
                current
                + random_generator.normal(
                    0.0,
                    scale,
                    6,
                )
            )

    posture_weights = np.array([
        2.5,
        1.2,
        1.4,
        0.7,
        0.5,
        0.3,
    ])

    best = None

    for yaw in (
        0.0,
        math.pi / 2.0,
        -math.pi / 2.0,
        math.pi,
    ):
        target_rotation = (
            downward_orientation(yaw)
        )

        for raw_seed in seeds:
            seed = np.clip(
                raw_seed,
                lower + 1e-6,
                upper - 1e-6,
            )

            def residual(candidate):
                transform = (
                    forward_kinematics(
                        chain,
                        candidate,
                    )
                )

                position_error = (
                    transform[:3, 3]
                    - target_position
                )

                orientation_error = (
                    Rotation.from_matrix(
                        target_rotation.T
                        @ transform[:3, :3]
                    ).as_rotvec()
                )

                posture_error = (
                    wrapped_difference(
                        candidate,
                        current,
                    )
                )

                return np.concatenate([
                    16.0 * position_error,
                    2.0 * orientation_error,
                    0.008
                    * posture_weights
                    * posture_error,
                ])

            result = least_squares(
                residual,
                seed,
                bounds=(
                    lower,
                    upper,
                ),
                max_nfev=4000,
                ftol=1e-11,
                xtol=1e-11,
                gtol=1e-11,
                x_scale='jac',
            )

            transform = forward_kinematics(
                chain,
                result.x,
            )

            position_error = float(
                np.linalg.norm(
                    transform[:3, 3]
                    - target_position
                )
            )

            orientation_error = float(
                np.linalg.norm(
                    Rotation.from_matrix(
                        target_rotation.T
                        @ transform[:3, :3]
                    ).as_rotvec()
                )
            )

            joint_difference = (
                wrapped_difference(
                    result.x,
                    current,
                )
            )

            maximum_joint_change = float(
                np.max(
                    np.abs(
                        joint_difference
                    )
                )
            )

            if (
                maximum_joint_change
                > math.radians(135.0)
            ):
                continue

            weighted_travel = float(
                np.linalg.norm(
                    posture_weights
                    * joint_difference
                )
            )

            score = (
                position_error
                + 0.20 * orientation_error
                + 0.012 * weighted_travel
            )

            candidate = (
                score,
                result.x,
                position_error,
                orientation_error,
                yaw,
                maximum_joint_change,
            )

            if (
                best is None
                or candidate[0] < best[0]
            ):
                best = candidate

    if best is None:
        raise RuntimeError(
            'No acceptable IK solution was found.'
        )

    return best[1:]


class JointReader(Node):

    def __init__(self):
        super().__init__(
            'red_pickup_ik_solver'
        )

        self.positions = None

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile_sensor_data,
        )

    def joint_state_callback(
        self,
        message,
    ):
        available = dict(
            zip(
                message.name,
                message.position,
            )
        )

        if all(
            joint in available
            for joint in JOINTS
        ):
            self.positions = np.array(
                [
                    available[joint]
                    for joint in JOINTS
                ],
                dtype=float,
            )

    def read_positions(
        self,
        timeout=15.0,
    ):
        deadline = (
            time.monotonic()
            + timeout
        )

        while (
            rclpy.ok()
            and self.positions is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.2,
            )

        if self.positions is None:
            raise RuntimeError(
                'No complete /joint_states '
                'message was received.'
            )

        return self.positions.copy()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--clearance',
        type=float,
        default=0.15,
    )

    parser.add_argument(
        '--output',
        type=Path,
        default=DEFAULT_OUTPUT,
    )

    parser.add_argument(
        '--target',
        choices=(
            'approach',
            'contact',
        ),
        default='approach',
    )

    parser.add_argument(
        '--contact-offset',
        type=float,
        default=0.012,
        help=(
            'Vertical distance above the calculated '
            'red-box surface in metres.'
        ),
    )

    arguments = parser.parse_args()

    if not (
        0.08
        <= arguments.clearance
        <= 0.30
    ):
        raise RuntimeError(
            'Clearance must be between '
            '0.08 and 0.30 metres.'
        )

    if not (
        0.002
        <= arguments.contact_offset
        <= 0.050
    ):
        raise RuntimeError(
            'Contact offset must be between '
            '0.002 and 0.050 metres.'
        )

    for required_file in (
        WORLD,
        CONVEYOR,
        ROBOT_XACRO,
    ):
        if not required_file.is_file():
            raise RuntimeError(
                'Required file is missing: '
                f'{required_file}'
            )

    approach, contact = (
        calculate_pickup_targets(
            arguments.clearance,
            arguments.contact_offset,
        )
    )

    selected_target = (
        approach
        if arguments.target == 'approach'
        else contact
    )

    print(
        'Geometry-derived red pickup targets:'
    )

    print(
        f'  contact:  '
        f'x={contact[0]:.4f}, '
        f'y={contact[1]:.4f}, '
        f'z={contact[2]:.4f}'
    )

    print(
        f'  approach: '
        f'x={approach[0]:.4f}, '
        f'y={approach[1]:.4f}, '
        f'z={approach[2]:.4f}'
    )

    chain, lower, upper = build_chain(
        generate_robot_urdf()
    )

    # Keep the shoulder in the normal working branch.
    lower[1] = max(
        lower[1],
        math.radians(-135.0),
    )

    upper[1] = min(
        upper[1],
        math.radians(-45.0),
    )

    rclpy.init()
    node = JointReader()

    try:
        current = node.read_positions()

        print()
        print('Current joints:')

        for name, value in zip(
            JOINTS,
            current,
        ):
            print(
                f'  {name:22s}: '
                f'{value: .6f}'
            )

        print()
        print(
            'Solving inverse kinematics...'
        )

        (
            solution,
            position_error,
            orientation_error,
            yaw,
            maximum_joint_change,
        ) = solve_ik(
            chain,
            current,
            lower,
            upper,
            selected_target,
        )

        print()
        print('IK solution:')

        for name, value in zip(
            JOINTS,
            solution,
        ):
            print(
                f'  {name:22s}: '
                f'{value: .6f}'
            )

        print()
        print(
            f'Position error: '
            f'{position_error * 1000.0:.2f} mm'
        )

        print(
            f'Orientation error: '
            f'{math.degrees(orientation_error):.2f} deg'
        )

        print(
            f'Tool yaw: '
            f'{math.degrees(yaw):.1f} deg'
        )

        print(
            f'Largest joint change: '
            f'{math.degrees(maximum_joint_change):.1f} deg'
        )

        if position_error > 0.005:
            raise RuntimeError(
                'Position error exceeds 5 mm.'
            )

        if (
            orientation_error
            > math.radians(5.0)
        ):
            raise RuntimeError(
                'Orientation error exceeds '
                '5 degrees.'
            )

        output_file = (
            arguments.output.resolve()
        )

        output_file.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        data = {
            'name': (
                f'red_pickup_{arguments.target}_ik'
            ),
            'description': (
                f'IK solution for the red pickup '
                f'{arguments.target} target, calculated '
                f'from Safety Fence environment geometry.'
            ),
            'target_frame': 'world',
            'target_suction_tip_xyz': (
                selected_target.tolist()
            ),
            'red_contact_xyz': (
                contact.tolist()
            ),
            'joint_names': JOINTS,
            'positions': solution.tolist(),
            'positions_by_name': dict(
                zip(
                    JOINTS,
                    solution.tolist(),
                )
            ),
            'validation': {
                'position_error_m': (
                    position_error
                ),
                'orientation_error_rad': (
                    orientation_error
                ),
                'tool_yaw_rad': yaw,
            },
        }

        output_file.write_text(
            json.dumps(
                data,
                indent=2,
            )
            + '\n'
        )

        print()
        print(
            'Validated IK pose saved:'
        )

        print(
            f'  {output_file}'
        )

        return 0

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
