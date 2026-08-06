#!/usr/bin/env python3

import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Optional, Tuple

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


CACHE_VERSION = 1

CACHE_DIRECTORY = (
    Path(__file__).resolve().parents[1]
    / 'config'
    / 'moveit_cache'
)

RED_TRANSFER_CACHE = (
    CACHE_DIRECTORY
    / 'red_transfer_attached.json'
)

RED_RETURN_CACHE = (
    CACHE_DIRECTORY
    / 'red_return_empty.json'
)


def trajectory_duration(
    trajectory: RobotTrajectory,
) -> float:
    points = trajectory.joint_trajectory.points

    if not points:
        return 0.0

    final_time = points[-1].time_from_start

    return (
        float(final_time.sec)
        + float(final_time.nanosec)
        / 1_000_000_000.0
    )


def save_trajectory(
    path: Path,
    trajectory: RobotTrajectory,
    label: str,
    metadata: Optional[Dict] = None,
) -> None:
    joint_trajectory = (
        trajectory.joint_trajectory
    )

    if not joint_trajectory.joint_names:
        raise RuntimeError(
            f'Cannot save "{label}": '
            'the trajectory has no joint names.'
        )

    if len(joint_trajectory.points) <= 1:
        raise RuntimeError(
            f'Cannot save "{label}": '
            'the trajectory has no motion.'
        )

    if trajectory.multi_dof_joint_trajectory.points:
        raise RuntimeError(
            f'Cannot save "{label}": '
            'multi-DOF trajectories are not supported.'
        )

    joint_count = len(
        joint_trajectory.joint_names
    )

    serialized_points = []

    for index, point in enumerate(
        joint_trajectory.points
    ):
        if len(point.positions) != joint_count:
            raise RuntimeError(
                f'Cannot save "{label}": point {index} '
                'has an invalid position count.'
            )

        serialized_points.append(
            {
                'positions': [
                    float(value)
                    for value in point.positions
                ],
                'velocities': [
                    float(value)
                    for value in point.velocities
                ],
                'accelerations': [
                    float(value)
                    for value in point.accelerations
                ],
                'effort': [
                    float(value)
                    for value in point.effort
                ],
                'time_from_start': {
                    'sec': int(
                        point.time_from_start.sec
                    ),
                    'nanosec': int(
                        point.time_from_start.nanosec
                    ),
                },
            }
        )

    document = {
        'version': CACHE_VERSION,
        'label': label,
        'created_utc': datetime.now(
            timezone.utc
        ).isoformat(),
        'frame_id': (
            joint_trajectory.header.frame_id
        ),
        'joint_names': list(
            joint_trajectory.joint_names
        ),
        'points': serialized_points,
        'metadata': dict(metadata or {}),
    }

    path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    temporary_path = path.with_suffix(
        path.suffix + '.tmp'
    )

    temporary_path.write_text(
        json.dumps(
            document,
            indent=2,
        )
        + '\n'
    )

    temporary_path.replace(path)


def load_trajectory(
    path: Path,
) -> Tuple[RobotTrajectory, Dict]:
    if not path.is_file():
        raise RuntimeError(
            f'Cached trajectory does not exist: {path}'
        )

    try:
        document = json.loads(
            path.read_text()
        )

    except Exception as error:
        raise RuntimeError(
            f'Could not read trajectory cache '
            f'"{path}": {error}'
        ) from error

    if document.get('version') != CACHE_VERSION:
        raise RuntimeError(
            f'Unsupported trajectory-cache version '
            f'in "{path}".'
        )

    joint_names = document.get(
        'joint_names',
        [],
    )

    raw_points = document.get(
        'points',
        [],
    )

    if not joint_names:
        raise RuntimeError(
            f'Trajectory cache has no joints: {path}'
        )

    if len(raw_points) <= 1:
        raise RuntimeError(
            f'Trajectory cache has no motion: {path}'
        )

    trajectory = RobotTrajectory()

    trajectory.joint_trajectory.header.frame_id = (
        str(document.get('frame_id', ''))
    )

    trajectory.joint_trajectory.joint_names = [
        str(name)
        for name in joint_names
    ]

    joint_count = len(joint_names)
    previous_time = -1.0

    for index, raw_point in enumerate(raw_points):
        point = JointTrajectoryPoint()

        point.positions = [
            float(value)
            for value in raw_point.get(
                'positions',
                [],
            )
        ]

        point.velocities = [
            float(value)
            for value in raw_point.get(
                'velocities',
                [],
            )
        ]

        point.accelerations = [
            float(value)
            for value in raw_point.get(
                'accelerations',
                [],
            )
        ]

        point.effort = [
            float(value)
            for value in raw_point.get(
                'effort',
                [],
            )
        ]

        if len(point.positions) != joint_count:
            raise RuntimeError(
                f'Invalid position count at point '
                f'{index} in "{path}".'
            )

        for field_name, values in (
            ('velocities', point.velocities),
            (
                'accelerations',
                point.accelerations,
            ),
            ('effort', point.effort),
        ):
            if values and len(values) != joint_count:
                raise RuntimeError(
                    f'Invalid {field_name} count at '
                    f'point {index} in "{path}".'
                )

        raw_duration = raw_point.get(
            'time_from_start',
            {},
        )

        point.time_from_start.sec = int(
            raw_duration.get('sec', 0)
        )

        point.time_from_start.nanosec = int(
            raw_duration.get('nanosec', 0)
        )

        current_time = (
            float(point.time_from_start.sec)
            + float(
                point.time_from_start.nanosec
            )
            / 1_000_000_000.0
        )

        if current_time < previous_time:
            raise RuntimeError(
                f'Non-monotonic trajectory time at '
                f'point {index} in "{path}".'
            )

        previous_time = current_time

        trajectory.joint_trajectory.points.append(
            point
        )

    if trajectory_duration(trajectory) <= 0.0:
        raise RuntimeError(
            f'Cached trajectory has zero duration: {path}'
        )

    return trajectory, document


def trajectory_start_error(
    trajectory: RobotTrajectory,
    current_positions: Dict[str, float],
) -> float:
    joint_names = (
        trajectory.joint_trajectory.joint_names
    )

    points = (
        trajectory.joint_trajectory.points
    )

    if not points:
        raise RuntimeError(
            'Cached trajectory has no start point.'
        )

    missing = [
        name
        for name in joint_names
        if name not in current_positions
    ]

    if missing:
        raise RuntimeError(
            'Current joint state is missing: '
            + ', '.join(missing)
        )

    start_positions = points[0].positions

    errors = []

    for name, target in zip(
        joint_names,
        start_positions,
    ):
        current = float(
            current_positions[name]
        )

        difference = (
            current
            - float(target)
            + math.pi
        ) % (2.0 * math.pi) - math.pi

        errors.append(abs(difference))

    return max(errors, default=0.0)
