#!/usr/bin/env python3

import json
import math
import os
import time
from pathlib import Path
from typing import Dict, Optional


PREPARED_PICKUP_FILE = Path(
    '/tmp/safety_fence_prepared_pickup.json'
)

PREPARED_PICKUP_APPROACH_TRAJECTORY = Path(
    '/tmp/safety_fence_prepared_pickup_approach.json'
)

PREPARED_PICKUP_TOUCH_TRAJECTORY = Path(
    '/tmp/safety_fence_prepared_pickup_touch.json'
)

PREPARED_PICKUP_EXIT_ATTACHED_TRAJECTORY = Path(
    '/tmp/safety_fence_prepared_pickup_touch_to_exit_attached.json'
)

VALID_COLORS = {
    'red',
    'green',
    'blue',
}


def clear_prepared_pickup() -> None:
    for path in (
        PREPARED_PICKUP_FILE,
        PREPARED_PICKUP_APPROACH_TRAJECTORY,
        PREPARED_PICKUP_TOUCH_TRAJECTORY,
        PREPARED_PICKUP_EXIT_ATTACHED_TRAJECTORY,
    ):
        path.unlink(
            missing_ok=True
        )


def save_prepared_pickup(
    *,
    generation: int,
    color: str,
    box_center,
    pickup_approach_joints,
    pickup_touch_joints,
    pickup_approach,
    pickup_touch,
    pose_spread,
    pickup_exit_joints,
    trajectory_ready: bool,
    attached_exit_trajectory_ready: bool,
) -> None:
    color = color.strip().lower()

    if color not in VALID_COLORS:
        raise RuntimeError(
            f'Unsupported pickup color: {color}'
        )

    document = {
        'version': 3,
        'generation': int(generation),
        'color': color,
        'created_unix_time': time.time(),

        'box_center': [
            float(value)
            for value in box_center
        ],

        'pickup_approach_joints': [
            float(value)
            for value in pickup_approach_joints
        ],

        'pickup_touch_joints': [
            float(value)
            for value in pickup_touch_joints
        ],

        'pickup_exit_joints': [
            float(value)
            for value in pickup_exit_joints
        ],

        'pickup_approach': [
            float(value)
            for value in pickup_approach
        ],

        'pickup_touch': [
            float(value)
            for value in pickup_touch
        ],

        'pose_spread': [
            float(value)
            for value in pose_spread
        ],

        'trajectory_ready': bool(
            trajectory_ready
        ),

        'pickup_approach_trajectory_file': str(
            PREPARED_PICKUP_APPROACH_TRAJECTORY
        ),

        'pickup_touch_trajectory_file': str(
            PREPARED_PICKUP_TOUCH_TRAJECTORY
        ),

        'attached_exit_trajectory_ready': bool(
            attached_exit_trajectory_ready
        ),

        'pickup_exit_attached_trajectory_file': str(
            PREPARED_PICKUP_EXIT_ATTACHED_TRAJECTORY
        ),
    }

    temporary = PREPARED_PICKUP_FILE.with_name(
        f'.{PREPARED_PICKUP_FILE.name}.'
        f'{os.getpid()}.tmp'
    )

    temporary.write_text(
        json.dumps(
            document,
            indent=2,
        )
        + '\n'
    )

    os.replace(
        temporary,
        PREPARED_PICKUP_FILE,
    )


def load_prepared_pickup(
    expected_color: Optional[str] = None,
    max_age_seconds: float = 120.0,
) -> Optional[Dict]:

    if not PREPARED_PICKUP_FILE.is_file():
        return None

    try:
        document = json.loads(
            PREPARED_PICKUP_FILE.read_text()
        )

    except Exception:
        return None

    version = int(
        document.get('version', 0)
    )

    if version not in (2, 3):
        return None

    color = str(
        document.get('color', '')
    ).strip().lower()

    if color not in VALID_COLORS:
        return None

    if (
        expected_color is not None
        and color != expected_color.strip().lower()
    ):
        return None

    try:
        created = float(
            document['created_unix_time']
        )

        age = (
            time.time()
            - created
        )

        if (
            not math.isfinite(age)
            or age < -5.0
            or age > max_age_seconds
        ):
            return None

        vector_lengths = {
            'box_center': 3,
            'pickup_approach_joints': 6,
            'pickup_touch_joints': 6,
            'pickup_exit_joints': 6,
            'pickup_approach': 3,
            'pickup_touch': 3,
            'pose_spread': 3,
        }

        for key, expected_length in (
            vector_lengths.items()
        ):
            values = document[key]

            if (
                not isinstance(values, list)
                or len(values) != expected_length
            ):
                return None

            for value in values:
                if not math.isfinite(
                    float(value)
                ):
                    return None

        trajectory_ready = bool(
            document.get(
                'trajectory_ready',
                False,
            )
        )

        if trajectory_ready:
            if not (
                PREPARED_PICKUP_APPROACH_TRAJECTORY.is_file()
                and PREPARED_PICKUP_TOUCH_TRAJECTORY.is_file()
            ):
                document[
                    'trajectory_ready'
                ] = False

        attached_exit_ready = bool(
            document.get(
                'attached_exit_trajectory_ready',
                False,
            )
        )

        if attached_exit_ready:
            if not (
                PREPARED_PICKUP_EXIT_ATTACHED_TRAJECTORY.is_file()
            ):
                document[
                    'attached_exit_trajectory_ready'
                ] = False

    except Exception:
        return None

    return document
