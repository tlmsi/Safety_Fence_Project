#!/usr/bin/env python3

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Set, Tuple

import numpy as np

from green_automation import load_ik, solve_fixed


STATE_FILE = Path(
    '/tmp/safety_fence_green_bin_slots.json'
)

# Green-bin geometry from sorting_bins/model.sdf and
# load_sorting_scene.py.
BIN_CENTER_X = 0.72
BIN_CENTER_Y = -0.05
BIN_SIZE_X = 0.38
BIN_SIZE_Y = 0.34
BIN_SURFACE_Z = 1.02

BOX_SIZE_X = 0.06
BOX_SIZE_Y = 0.06

EDGE_CLEARANCE = 0.01
BOX_GAP = 0.01
DROP_GAP = 0.001
DROP_APPROACH_CLEARANCE = 0.15


@dataclass(frozen=True)
class RedBinSlot:
    index: int
    row: int
    column: int
    x: float
    y: float
    capacity: int


def generate_green_slots() -> List[RedBinSlot]:
    pitch_x = BOX_SIZE_X + BOX_GAP
    pitch_y = BOX_SIZE_Y + BOX_GAP

    first_x = (
        BIN_CENTER_X
        - BIN_SIZE_X / 2.0
        + BOX_SIZE_X / 2.0
        + EDGE_CLEARANCE
    )

    last_x = (
        BIN_CENTER_X
        + BIN_SIZE_X / 2.0
        - BOX_SIZE_X / 2.0
        - EDGE_CLEARANCE
    )

    # Start at the bin edge closest to the robot.
    first_y = (
        BIN_CENTER_Y
        + BIN_SIZE_Y / 2.0
        - BOX_SIZE_Y / 2.0
        - EDGE_CLEARANCE
    )

    last_y = (
        BIN_CENTER_Y
        - BIN_SIZE_Y / 2.0
        + BOX_SIZE_Y / 2.0
        + EDGE_CLEARANCE
    )

    columns = (
        math.floor(
            (last_x - first_x) / pitch_x
            + 1e-9
        )
        + 1
    )

    rows = (
        math.floor(
            (first_y - last_y) / pitch_y
            + 1e-9
        )
        + 1
    )

    capacity = rows * columns
    slots: List[RedBinSlot] = []

    for row in range(rows):
        y = first_y - row * pitch_y

        for column in range(columns):
            x = first_x + column * pitch_x
            index = row * columns + column

            slots.append(
                RedBinSlot(
                    index=index,
                    row=row,
                    column=column,
                    x=float(x),
                    y=float(y),
                    capacity=capacity,
                )
            )

    return slots


def read_occupied_slots() -> Set[int]:
    if not STATE_FILE.is_file():
        return set()

    try:
        data = json.loads(
            STATE_FILE.read_text()
        )

    except Exception as error:
        raise RuntimeError(
            f'Cannot read green-bin slot state: {error}'
        ) from error

    occupied = data.get('occupied')

    if not isinstance(occupied, list):
        raise RuntimeError(
            'Green-bin state has no valid occupied list.'
        )

    slots = generate_green_slots()
    valid_indexes = {
        slot.index
        for slot in slots
    }

    result: Set[int] = set()

    for value in occupied:
        if (
            not isinstance(value, int)
            or value not in valid_indexes
        ):
            raise RuntimeError(
                f'Invalid green-bin slot index: {value!r}'
            )

        result.add(value)

    return result


def next_green_slot() -> RedBinSlot:
    occupied = read_occupied_slots()

    for slot in generate_green_slots():
        if slot.index not in occupied:
            return slot

    raise RuntimeError(
        'The green bin is full. '
        'No unused drop slots remain.'
    )


def mark_green_slot_occupied(
    node,
    slot: RedBinSlot,
) -> None:
    occupied = read_occupied_slots()
    occupied.add(slot.index)

    data = {
        'version': 1,
        'occupied': sorted(occupied),
    }

    temporary = STATE_FILE.with_suffix(
        '.tmp'
    )

    temporary.write_text(
        json.dumps(
            data,
            indent=2,
        )
        + '\n'
    )

    temporary.replace(
        STATE_FILE
    )

    node.get_logger().info(
        f'Green-bin slot {slot.index + 1}/'
        f'{slot.capacity} marked occupied.'
    )


def restore_occupied_green_boxes(
    node,
    scene_manager,
    half_height: float,
) -> None:
    occupied = read_occupied_slots()

    if not occupied:
        node.get_logger().info(
            'No occupied green-bin slots need restoration.'
        )
        return

    slots = {
        slot.index: slot
        for slot in generate_green_slots()
    }

    restored = 0

    for index in sorted(occupied):
        slot = slots[index]

        object_id = (
            f'green_box_slot_{slot.index + 1:02d}'
        )

        scene_manager.add_world_box(
            object_id=object_id,
            center_xyz=(
                slot.x,
                slot.y,
                BIN_SURFACE_Z + half_height,
            ),
            size_xyz=(
                BOX_SIZE_X,
                BOX_SIZE_Y,
                2.0 * half_height,
            ),
            frame_id='world',
        )

        restored += 1

    node.get_logger().info(
        f'Restored {restored} occupied green-bin '
        'box collision object(s) into MoveIt.'
    )


def solve_dynamic_green_drop(
    node,
    cached,
    half_height: float,
) -> Tuple[
    RedBinSlot,
    List[float],
    List[float],
    np.ndarray,
    np.ndarray,
]:
    slot = next_green_slot()

    release_tip = np.array(
        [
            slot.x,
            slot.y,
            (
                BIN_SURFACE_Z
                + 2.0 * half_height
                + DROP_GAP
            ),
        ],
        dtype=float,
    )

    drop_approach = release_tip.copy()
    drop_approach[2] += (
        DROP_APPROACH_CLEARANCE
    )

    node.get_logger().info(
        f'Selected green-bin slot '
        f'{slot.index + 1}/{slot.capacity}: '
        f'row={slot.row + 1}, '
        f'column={slot.column + 1}'
    )

    node.get_logger().info(
        'Dynamic green-bin targets:'
    )

    node.get_logger().info(
        f'  approach: x={drop_approach[0]:.4f}, '
        f'y={drop_approach[1]:.4f}, '
        f'z={drop_approach[2]:.4f}'
    )

    node.get_logger().info(
        f'  release:  x={release_tip[0]:.4f}, '
        f'y={release_tip[1]:.4f}, '
        f'z={release_tip[2]:.4f}'
    )

    ik = load_ik()

    chain, lower, upper = ik.build_chain(
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

    staging_seed = np.asarray(
        cached.poses['drop_approach'],
        dtype=float,
    )

    node.get_logger().info(
        'Solving IK for dynamic drop approach...'
    )

    (
        approach_joints,
        approach_position_error,
        approach_orientation_error,
    ) = solve_fixed(
        ik,
        chain,
        lower,
        upper,
        staging_seed,
        drop_approach,
        cached.tool_yaw,
    )

    node.get_logger().info(
        'Solving IK for dynamic drop release...'
    )

    (
        release_joints,
        release_position_error,
        release_orientation_error,
    ) = solve_fixed(
        ik,
        chain,
        lower,
        upper,
        approach_joints,
        release_tip,
        cached.tool_yaw,
    )

    node.get_logger().info(
        'Dynamic drop IK validated: '
        f'approach error='
        f'{approach_position_error * 1000.0:.2f} mm, '
        f'release error='
        f'{release_position_error * 1000.0:.2f} mm, '
        f'approach orientation='
        f'{math.degrees(approach_orientation_error):.2f} deg, '
        f'release orientation='
        f'{math.degrees(release_orientation_error):.2f} deg'
    )

    return (
        slot,
        approach_joints.tolist(),
        release_joints.tolist(),
        drop_approach,
        release_tip,
    )
