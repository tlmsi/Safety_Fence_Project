#!/usr/bin/env python3

import argparse
import importlib.util
import json
import math
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from pathlib import Path

import numpy as np

from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


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
CONFIG = CONTROL / 'config'
WAYPOINTS = CONFIG / 'red_automation'

IK_FILE = HERE / 'red_pickup_ik_solver.py'
EXECUTOR = HERE / 'move_to_pose.py'
THROUGH_EXECUTOR = HERE / 'move_through_poses.py'
WAITER = HERE / 'wait_for_box_present.py'

HOME_FILE = CONFIG / 'home_pose.json'

# This is the already tested pose above the pickup zone.
APPROACH_FILE = (
    CONFIG
    / 'red_pickup_approach_ik.json'
)

PATH_FILE = (
    CONFIG
    / 'red_automation_path.json'
)

WORLD = (
    WS
    / 'src/sorting_cell_gazebo/worlds'
    / 'sorting_cell_world.sdf'
)

BINS = (
    WS
    / 'src/sorting_cell_gazebo/models'
    / 'sorting_bins/model.sdf'
)


def nums(
    text,
    count,
):
    values = [
        float(value)
        for value in text.split()
    ]

    if len(values) != count:
        raise RuntimeError(
            f'Expected {count} values: {text}'
        )

    return values


def element_pose(
    element,
):
    pose = element.find('pose')

    if pose is None or not pose.text:
        return [0.0] * 6

    return nums(
        pose.text,
        6,
    )


def matrix(
    pose,
):
    x, y, z, roll, pitch, yaw = pose

    output = np.eye(4)

    output[:3, :3] = (
        Rotation.from_euler(
            'xyz',
            [
                roll,
                pitch,
                yaw,
            ],
        ).as_matrix()
    )

    output[:3, 3] = [
        x,
        y,
        z,
    ]

    return output


def load_ik():
    spec = importlib.util.spec_from_file_location(
        'red_pickup_ik',
        IK_FILE,
    )

    if spec is None or spec.loader is None:
        raise RuntimeError(
            f'Cannot load {IK_FILE}'
        )

    module = importlib.util.module_from_spec(
        spec
    )

    spec.loader.exec_module(
        module
    )

    return module


def load_joint_pose(
    path,
):
    data = json.loads(
        path.read_text()
    )

    by_name = data.get(
        'positions_by_name'
    )

    if by_name is None:
        by_name = dict(
            zip(
                data['joint_names'],
                data['positions'],
            )
        )

    positions = np.array(
        [
            float(by_name[name])
            for name in JOINTS
        ],
        dtype=float,
    )

    return positions, data


def find_include(
    root,
    suffix,
):
    for include in root.iter('include'):
        uri = (
            include.findtext('uri')
            or ''
        ).strip().rstrip('/')

        if uri.endswith(
            suffix.rstrip('/')
        ):
            return include

    raise RuntimeError(
        f'World include not found: {suffix}'
    )


def red_box_size(
    world_root,
):
    for model in world_root.iter('model'):

        if (
            model.attrib.get('name')
            != 'red_box'
        ):
            continue

        for query in (
            './link/collision/geometry/box/size',
            './link/visual/geometry/box/size',
        ):
            item = model.find(query)

            if (
                item is not None
                and item.text
            ):
                return np.array(
                    nums(
                        item.text,
                        3,
                    ),
                    dtype=float,
                )

    raise RuntimeError(
        'Cannot read red-box dimensions.'
    )


def drop_geometry(
    clearance=0.15,
    gap=0.001,
):
    world_root = ET.parse(
        WORLD
    ).getroot()

    bins_root = ET.parse(
        BINS
    ).getroot()

    bins_include = find_include(
        world_root,
        'models/sorting_bins',
    )

    world_from_bins = matrix(
        element_pose(
            bins_include
        )
    )

    red_bottom = next(
        (
            collision
            for collision
            in bins_root.iter('collision')
            if (
                collision.attrib.get('name')
                == 'red_bottom_collision'
            )
        ),
        None,
    )

    if red_bottom is None:
        raise RuntimeError(
            'red_bottom_collision not found.'
        )

    size_item = red_bottom.find(
        './geometry/box/size'
    )

    if (
        size_item is None
        or not size_item.text
    ):
        raise RuntimeError(
            'Red bin size is missing.'
        )

    size = nums(
        size_item.text,
        3,
    )

    pose = element_pose(
        red_bottom
    )

    surface_local_z = (
        pose[2]
        + size[2] / 2.0
    )

    red_size = red_box_size(
        world_root
    )

    # Place the box in the inner corner of the red bin,
    # closest to the robot.
    #
    # Keep the complete box inside the bin footprint and
    # leave 10 mm clearance from each selected edge.
    edge_clearance = 0.010

    corner_offset_x = -(
        size[0] / 2.0
        - red_size[0] / 2.0
        - edge_clearance
    )

    corner_offset_y = (
        size[1] / 2.0
        - red_size[1] / 2.0
        - edge_clearance
    )

    if (
        corner_offset_x >= 0.0
        or corner_offset_y <= 0.0
    ):
        raise RuntimeError(
            'The red box does not fit inside the '
            'requested red-bin corner clearance.'
        )

    # The suction tip remains attached to the top surface
    # of the box while carrying it.
    #
    # At release:
    #
    # box bottom =
    # bin surface + 1 mm
    #
    # suction-tip Z =
    # bin surface + box height + 1 mm
    release_local = np.array(
        [
            pose[0]
            + corner_offset_x,

            pose[1]
            + corner_offset_y,

            surface_local_z
            + red_size[2]
            + gap,

            1.0,
        ],
        dtype=float,
    )

    release = (
        world_from_bins
        @ release_local
    )[:3]

    approach = release.copy()
    approach[2] += clearance

    bin_surface_point = np.array(
        [
            pose[0],
            pose[1],
            surface_local_z,
            1.0,
        ],
        dtype=float,
    )

    surface_z = float(
        (
            world_from_bins
            @ bin_surface_point
        )[2]
    )

    return (
        approach,
        release,
        surface_z,
    )


def solve_fixed(
    ik,
    chain,
    lower,
    upper,
    start,
    xyz,
    yaw,
):
    target_rotation = (
        ik.downward_orientation(
            yaw
        )
    )

    weights = np.array(
        [
            2.5,
            1.2,
            1.4,
            0.7,
            0.5,
            0.3,
        ]
    )

    random_generator = (
        np.random.default_rng(
            20260723
        )
    )

    seeds = [
        start.copy()
    ]

    for scale in (
        0.03,
        0.08,
        0.16,
    ):
        for _ in range(5):
            seeds.append(
                start
                + random_generator.normal(
                    0.0,
                    scale,
                    6,
                )
            )

    best = None

    for raw_seed in seeds:

        seed = np.clip(
            raw_seed,
            lower + 1e-7,
            upper - 1e-7,
        )

        def residual(
            candidate,
        ):
            transform = (
                ik.forward_kinematics(
                    chain,
                    candidate,
                )
            )

            position_error = (
                transform[:3, 3]
                - xyz
            )

            orientation_error = (
                Rotation.from_matrix(
                    target_rotation.T
                    @ transform[:3, :3]
                ).as_rotvec()
            )

            posture_error = (
                ik.wrapped_difference(
                    candidate,
                    start,
                )
            )

            return np.concatenate(
                [
                    16.0
                    * position_error,

                    2.0
                    * orientation_error,

                    0.007
                    * weights
                    * posture_error,
                ]
            )

        result = least_squares(
            residual,
            seed,
            bounds=(
                lower,
                upper,
            ),
            max_nfev=3500,
            ftol=1e-11,
            xtol=1e-11,
            gtol=1e-11,
            x_scale='jac',
        )

        transform = (
            ik.forward_kinematics(
                chain,
                result.x,
            )
        )

        position_error = float(
            np.linalg.norm(
                transform[:3, 3]
                - xyz
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

        joint_change = (
            ik.wrapped_difference(
                result.x,
                start,
            )
        )

        maximum_change = float(
            np.max(
                np.abs(
                    joint_change
                )
            )
        )

        if (
            maximum_change
            > math.radians(135.0)
        ):
            continue

        score = (
            position_error
            + 0.20
            * orientation_error
            + 0.01
            * float(
                np.linalg.norm(
                    weights
                    * joint_change
                )
            )
        )

        candidate = (
            score,
            result.x.copy(),
            position_error,
            orientation_error,
        )

        if (
            best is None
            or candidate[0] < best[0]
        ):
            best = candidate

    if best is None:
        raise RuntimeError(
            f'No IK solution for {xyz}'
        )

    (
        _,
        solution,
        position_error,
        orientation_error,
    ) = best

    # The drop gap is only 1 mm, so use a much
    # stricter accuracy limit than the earlier tests.
    if (
        position_error > 0.001
        or orientation_error
        > math.radians(3.0)
    ):
        raise RuntimeError(
            'IK validation failed '
            '(position limit 1 mm): '
            f'target='
            f'({xyz[0]:.4f}, '
            f'{xyz[1]:.4f}, '
            f'{xyz[2]:.4f}), '
            f'position error='
            f'{position_error * 1000.0:.2f} mm, '
            f'orientation error='
            f'{math.degrees(orientation_error):.2f} deg'
        )

    return (
        solution,
        position_error,
        orientation_error,
    )


def save_pose(
    filename,
    name,
    xyz,
    joints,
):
    WAYPOINTS.mkdir(
        parents=True,
        exist_ok=True,
    )

    path = (
        WAYPOINTS
        / filename
    )

    data = {
        'name': name,
        'target_frame': 'world',
        'target_suction_tip_xyz': (
            xyz.tolist()
        ),
        'joint_names': JOINTS,
        'positions': joints.tolist(),
        'positions_by_name': dict(
            zip(
                JOINTS,
                joints.tolist(),
            )
        ),
    }

    path.write_text(
        json.dumps(
            data,
            indent=2,
        )
        + '\n'
    )

    return path


def generate_plan():
    for required_file in (
        IK_FILE,
        EXECUTOR,
        THROUGH_EXECUTOR,
        WAITER,
        HOME_FILE,
        WORLD,
        BINS,
    ):
        if not required_file.is_file():
            raise RuntimeError(
                'Missing required file: '
                f'{required_file}'
            )

    ik = load_ik()

    home_joints, _ = (
        load_joint_pose(
            HOME_FILE
        )
    )

    # Reuse the pickup-approach pose that has
    # already been physically tested.
    if APPROACH_FILE.is_file():

        (
            pickup_approach_joints,
            approach_data,
        ) = load_joint_pose(
            APPROACH_FILE
        )

        pickup_approach = np.array(
            approach_data[
                'target_suction_tip_xyz'
            ],
            dtype=float,
        )

        yaw = float(
            approach_data[
                'validation'
            ][
                'tool_yaw_rad'
            ]
        )

    else:
        (
            pickup_approach,
            _,
        ) = ik.calculate_pickup_targets(
            0.15,
            0.0,
        )

        (
            chain,
            lower,
            upper,
        ) = ik.build_chain(
            ik.generate_robot_urdf()
        )

        lower[1] = max(
            lower[1],
            math.radians(-135.0),
        )

        upper[1] = min(
            upper[1],
            math.radians(-45.0),
        )

        (
            pickup_approach_joints,
            position_error,
            orientation_error,
            yaw,
            _,
        ) = ik.solve_ik(
            chain,
            home_joints,
            lower,
            upper,
            pickup_approach,
        )

        if (
            position_error > 0.001
            or orientation_error
            > math.radians(3.0)
        ):
            raise RuntimeError(
                'Pickup approach validation failed.'
            )

    # Exact box-top position:
    # no positive contact offset.
    (
        _,
        pickup_touch,
    ) = ik.calculate_pickup_targets(
        0.15,
        0.0,
    )

    (
        drop_approach,
        drop_release,
        bin_surface_z,
    ) = drop_geometry(
        clearance=0.15,
        gap=0.001,
    )

    # The literal Cartesian midpoint lies almost directly
    # over the robot base and produces a near-singular pose.
    #
    # Instead, construct the middle transfer waypoint halfway
    # around a safe arc centred on the robot base.
    robot_base_xy = np.array(
        [
            0.15,
            0.00,
        ],
        dtype=float,
    )

    pickup_vector = (
        pickup_approach[:2]
        - robot_base_xy
    )

    drop_vector = (
        drop_approach[:2]
        - robot_base_xy
    )

    pickup_radius = float(
        np.linalg.norm(
            pickup_vector
        )
    )

    drop_radius = float(
        np.linalg.norm(
            drop_vector
        )
    )

    pickup_angle = math.atan2(
        pickup_vector[1],
        pickup_vector[0],
    )

    drop_angle = math.atan2(
        drop_vector[1],
        drop_vector[0],
    )

    angle_difference = math.atan2(
        math.sin(
            drop_angle
            - pickup_angle
        ),
        math.cos(
            drop_angle
            - pickup_angle
        ),
    )

    middle_angle = (
        pickup_angle
        + 0.5 * angle_difference
    )

    # Stay clear of the base singularity without extending
    # unnecessarily close to the robot's maximum reach.
    middle_radius = max(
        0.62,
        0.5
        * (
            pickup_radius
            + drop_radius
        ),
    )

    middle_height = (
        max(
            pickup_approach[2],
            drop_approach[2],
        )
        + 0.03
    )

    middle = np.array(
        [
            robot_base_xy[0]
            + middle_radius
            * math.cos(
                middle_angle
            ),

            robot_base_xy[1]
            + middle_radius
            * math.sin(
                middle_angle
            ),

            middle_height,
        ],
        dtype=float,
    )

    print()
    print(
        'Safe middle transfer waypoint:'
    )

    print(
        f'  x={middle[0]:.4f}, '
        f'y={middle[1]:.4f}, '
        f'z={middle[2]:.4f}'
    )

    print(
        f'  distance from base axis: '
        f'{middle_radius:.4f} m'
    )

    (
        chain,
        lower,
        upper,
    ) = ik.build_chain(
        ik.generate_robot_urdf()
    )

    lower[1] = max(
        lower[1],
        math.radians(-135.0),
    )

    upper[1] = min(
        upper[1],
        math.radians(-45.0),
    )

    print()
    print(
        'Solving IK: pickup touch'
    )

    (
        pickup_touch_joints,
        pickup_touch_position_error,
        pickup_touch_orientation_error,
    ) = solve_fixed(
        ik,
        chain,
        lower,
        upper,
        pickup_approach_joints,
        pickup_touch,
        yaw,
    )

    print()
    print(
        'Solving IK: safe middle waypoint'
    )

    (
        middle_joints,
        middle_position_error,
        middle_orientation_error,
    ) = solve_fixed(
        ik,
        chain,
        lower,
        upper,
        pickup_approach_joints,
        middle,
        yaw,
    )

    print()
    print(
        'Solving IK: above red drop zone'
    )

    (
        drop_approach_joints,
        drop_approach_position_error,
        drop_approach_orientation_error,
    ) = solve_fixed(
        ik,
        chain,
        lower,
        upper,
        middle_joints,
        drop_approach,
        yaw,
    )

    print()
    print(
        'Solving IK: red release at 1 mm'
    )

    (
        drop_release_joints,
        drop_release_position_error,
        drop_release_orientation_error,
    ) = solve_fixed(
        ik,
        chain,
        lower,
        upper,
        drop_approach_joints,
        drop_release,
        yaw,
    )

    files = {
        'home': str(
            HOME_FILE
        ),

        'pickup_approach': str(
            save_pose(
                '01_pickup_approach.json',
                'red_pickup_approach',
                pickup_approach,
                pickup_approach_joints,
            )
        ),

        'pickup_touch': str(
            save_pose(
                '02_pickup_touch.json',
                'red_pickup_touch',
                pickup_touch,
                pickup_touch_joints,
            )
        ),

        'middle': str(
            save_pose(
                '03_middle.json',
                'red_middle_waypoint',
                middle,
                middle_joints,
            )
        ),

        'drop_approach': str(
            save_pose(
                '04_drop_approach.json',
                'red_drop_approach',
                drop_approach,
                drop_approach_joints,
            )
        ),

        'drop_release': str(
            save_pose(
                '05_drop_release_1mm.json',
                'red_drop_release_1mm',
                drop_release,
                drop_release_joints,
            )
        ),
    }

    plan = {
        'name': 'red_automation',

        'sequence': [
            'home',
            'pickup_approach',
            'pickup_touch',
            'attach_red',
            'pickup_approach',
            'middle',
            'drop_approach',
            'drop_release',
            'detach_red',
            'drop_approach',
            'middle',
            'pickup_approach',
        ],

        'tool_yaw_rad': yaw,

        'geometry': {
            'pickup_approach_xyz': (
                pickup_approach.tolist()
            ),

            'pickup_touch_xyz': (
                pickup_touch.tolist()
            ),

            'middle_xyz': (
                middle.tolist()
            ),

            'drop_approach_xyz': (
                drop_approach.tolist()
            ),

            'drop_release_tip_xyz': (
                drop_release.tolist()
            ),

            'red_bin_surface_z': (
                bin_surface_z
            ),

            'drop_gap_box_bottom_to_bin_m': (
                0.001
            ),

            'drop_location': (
                'red_bin_inner_corner_closest_to_robot'
            ),

            'drop_edge_clearance_m': (
                0.010
            ),
        },

        'pose_files': files,

        'validation': {
            'pickup_touch': {
                'position_error_m': (
                    pickup_touch_position_error
                ),

                'orientation_error_rad': (
                    pickup_touch_orientation_error
                ),
            },

            'middle': {
                'position_error_m': (
                    middle_position_error
                ),

                'orientation_error_rad': (
                    middle_orientation_error
                ),
            },

            'drop_approach': {
                'position_error_m': (
                    drop_approach_position_error
                ),

                'orientation_error_rad': (
                    drop_approach_orientation_error
                ),
            },

            'drop_release': {
                'position_error_m': (
                    drop_release_position_error
                ),

                'orientation_error_rad': (
                    drop_release_orientation_error
                ),
            },
        },
    }

    PATH_FILE.write_text(
        json.dumps(
            plan,
            indent=2,
        )
        + '\n'
    )

    return plan


def run_command(
    command,
    label,
):
    print()
    print(
        f'=== {label} ==='
    )

    result = subprocess.run(
        command,
        check=False,
    )

    if result.returncode != 0:
        raise RuntimeError(
            f'Failed: {label}'
        )


def move(
    path,
    duration,
    label,
):
    run_command(
        [
            sys.executable,
            '-u',
            str(EXECUTOR),
            path,
            '--duration',
            str(duration),
        ],
        label,
    )


def move_through(
    paths,
    segment_times,
    label,
):
    if len(paths) != len(segment_times):
        raise RuntimeError(
            'Continuous path and timing lengths differ.'
        )

    run_command(
        [
            sys.executable,
            '-u',
            str(THROUGH_EXECUTOR),
            '--segment-times',
            ','.join(
                str(value)
                for value in segment_times
            ),
            *paths,
        ],
        label,
    )


def suction(
    action,
):
    if action not in (
        'attach',
        'detach',
    ):
        raise RuntimeError(
            f'Invalid suction action: {action}'
        )

    topic = f'/suction/red/{action}'

    print()
    print(
        f'Suction command: {action.upper()}'
    )

    result = subprocess.run(
        [
            'gz',
            'topic',
            '-t',
            topic,
            '-m',
            'gz.msgs.Empty',
            '-p',
            'unused: true',
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )

    if result.returncode != 0:
        raise RuntimeError(
            f'Suction {action} failed: '
            f'{result.stderr.strip()}'
        )

    # Minimal time for Gazebo to process the command.
    time.sleep(0.10)




def load_saved_plan():
    if not PATH_FILE.is_file():
        raise RuntimeError(
            'No saved automation path exists. Run plan first:\n'
            f'  {PATH_FILE}'
        )

    try:
        plan = json.loads(
            PATH_FILE.read_text()
        )
    except Exception as error:
        raise RuntimeError(
            f'Could not read saved path: {error}'
        ) from error

    pose_files = plan.get(
        'pose_files'
    )

    if not isinstance(pose_files, dict):
        raise RuntimeError(
            'Saved path has no valid pose_files section.'
        )

    for name, filename in pose_files.items():
        pose_path = Path(
            filename
        ).expanduser()

        if not pose_path.is_absolute():
            pose_path = (
                PATH_FILE.parent
                / pose_path
            ).resolve()

        if not pose_path.is_file():
            raise RuntimeError(
                f'Saved pose file missing: '
                f'{name}: {pose_path}'
            )

        pose_files[name] = str(
            pose_path
        )

    print()
    print(
        'Loaded previously validated path.'
    )
    print(
        'IK calculation skipped.'
    )

    return plan


def execute(
    plan,
):
    files = plan[
        'pose_files'
    ]

    run_command(
        [
            sys.executable,
            '-u',
            str(WAITER),
            '--timeout',
            '120',
        ],
        'Wait for box-present=true',
    )

    # Continuous motion from the robot's current
    # initial position to the box.
    #
    # The pickup approach is only a pass-through
    # waypoint. The robot stops at pickup_touch
    # because attachment is required there.
    move_through(
        [
            files['pickup_approach'],
            files['pickup_touch'],
        ],
        [
            2.6,
            1.4,
        ],
        'Continuous motion to red-box pickup',
    )

    suction(
        'attach'
    )

    # Continuous carrying trajectory.
    #
    # Pickup approach, middle and drop approach
    # are all pass-through waypoints.
    # The robot stops only at drop_release.
    move_through(
        [
            files['pickup_approach'],
            files['middle'],
            files['drop_approach'],
            files['drop_release'],
        ],
        [
            1.4,
            2.3,
            2.3,
            1.4,
        ],
        'Continuous red-box transfer',
    )

    suction(
        'detach'
    )

    # Continuous return trajectory.
    #
    # Drop approach and middle are pass-through
    # waypoints. The final stop is above pickup.
    move_through(
        [
            files['drop_approach'],
            files['middle'],
            files['pickup_approach'],
        ],
        [
            1.4,
            2.3,
            2.3,
        ],
        'Continuous return above pickup zone',
    )

    print()
    print(
        'Robot final position: '
        'above the pickup zone.'
    )


def print_plan(
    plan,
):
    print()
    print(
        'RED AUTOMATION PATH GENERATED'
    )

    print(
        '============================='
    )

    for label, key in (
        (
            'Pickup approach',
            'pickup_approach_xyz',
        ),
        (
            'Pickup touch',
            'pickup_touch_xyz',
        ),
        (
            'Middle waypoint',
            'middle_xyz',
        ),
        (
            'Drop approach',
            'drop_approach_xyz',
        ),
        (
            'Drop release',
            'drop_release_tip_xyz',
        ),
    ):
        x, y, z = (
            plan[
                'geometry'
            ][key]
        )

        print(
            f'{label:18s}: '
            f'x={x:.4f}, '
            f'y={y:.4f}, '
            f'z={z:.4f}'
        )

    print()
    print(
        'Drop location: '
        'inner red-bin corner closest to robot'
    )

    print(
        'Bin-edge clearance: 10.0 mm'
    )

    print(
        'Drop gap: 1.0 mm'
    )

    print(
        'Final robot waypoint: '
        'above pickup zone'
    )

    print(
        f'Path file: {PATH_FILE}'
    )

    print()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        'mode',
        choices=(
            'plan',
            'run',
        ),
        nargs='?',
        default='plan',
    )

    arguments = parser.parse_args()

    try:
        if arguments.mode == 'plan':
            plan = generate_plan()
        else:
            plan = load_saved_plan()
        print_plan(
            plan
        )

        if arguments.mode == 'run':
            execute(
                plan
            )

            print()
            print(
                '========================================'
            )

            print(
                'RED AUTOMATION COMPLETED SUCCESSFULLY'
            )

            print(
                '========================================'
            )

        else:
            print(
                'PLAN PASSED. '
                'No robot or suction command was sent.'
            )

        return 0

    except Exception as error:
        print(
            f'ERROR: {error}',
            file=sys.stderr,
        )

        return 1


if __name__ == '__main__':
    raise SystemExit(
        main()
    )
