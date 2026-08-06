#!/usr/bin/env python3

import argparse
import sys
from typing import Dict, Sequence, Tuple

import rclpy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetPlanningScene,
)
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


Vector3 = Tuple[float, float, float]


# Only fixed cell geometry belongs in the world planning scene.
FIXED_OBJECTS: Dict[
    str,
    Tuple[Vector3, Vector3],
] = {
    "conveyor_belt": (
        (-0.65, 0.45, 0.875),
        (1.25, 0.42, 0.07),
    ),
    "conveyor_left_rail": (
        (-0.65, 0.695, 0.890),
        (1.25, 0.07, 0.04),
    ),
    "conveyor_right_rail": (
        (-0.65, 0.205, 0.890),
        (1.25, 0.07, 0.04),
    ),
    "red_bin_bottom": (
        (0.72, -0.53, 1.00),
        (0.38, 0.34, 0.04),
    ),
    "green_bin_bottom": (
        (0.72, -0.05, 1.00),
        (0.38, 0.34, 0.04),
    ),
    "blue_bin_bottom": (
        (0.72, 0.43, 1.00),
        (0.38, 0.34, 0.04),
    ),
}


# These IDs were created by the previous loader.
# They must be removed because the real boxes move in Gazebo.
LEGACY_MOVING_OBJECT_IDS = (
    "red_box",
    "green_box",
    "blue_box",
)


def make_box(
    object_id: str,
    centre: Sequence[float],
    size: Sequence[float],
    frame_id: str,
) -> CollisionObject:
    collision_object = CollisionObject()
    collision_object.header.frame_id = frame_id
    collision_object.id = object_id

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [
        float(size[0]),
        float(size[1]),
        float(size[2]),
    ]

    pose = Pose()
    pose.position.x = float(centre[0])
    pose.position.y = float(centre[1])
    pose.position.z = float(centre[2])
    pose.orientation.w = 1.0

    collision_object.primitives.append(
        primitive
    )
    collision_object.primitive_poses.append(
        pose
    )
    collision_object.operation = (
        CollisionObject.ADD
    )

    return collision_object


def make_remove(
    object_id: str,
    frame_id: str,
) -> CollisionObject:
    collision_object = CollisionObject()
    collision_object.header.frame_id = frame_id
    collision_object.id = object_id
    collision_object.operation = (
        CollisionObject.REMOVE
    )

    return collision_object


class SortingSceneLoader(Node):

    def __init__(
        self,
        frame_id: str,
    ) -> None:
        super().__init__(
            "sorting_cell_planning_scene_loader"
        )

        self.frame_id = frame_id

        self.apply_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )

        self.get_client = self.create_client(
            GetPlanningScene,
            "/get_planning_scene",
        )

    def wait_for_services(
        self,
        timeout: float = 15.0,
    ) -> bool:
        self.get_logger().info(
            "Waiting for MoveIt planning-scene services..."
        )

        if not self.apply_client.wait_for_service(
            timeout_sec=timeout
        ):
            self.get_logger().error(
                "/apply_planning_scene is unavailable."
            )
            return False

        if not self.get_client.wait_for_service(
            timeout_sec=timeout
        ):
            self.get_logger().error(
                "/get_planning_scene is unavailable."
            )
            return False

        return True

    def apply(
        self,
        remove: bool,
        existing_ids: Sequence[str],
    ) -> bool:
        scene = PlanningScene()
        scene.name = "sorting_cell"
        scene.is_diff = True

        existing = set(existing_ids)

        if remove:
            managed_ids = (
                set(FIXED_OBJECTS)
                | set(LEGACY_MOVING_OBJECT_IDS)
            )

            remove_ids = sorted(
                managed_ids & existing
            )

            for object_id in remove_ids:
                scene.world.collision_objects.append(
                    make_remove(
                        object_id,
                        self.frame_id,
                    )
                )

            description = (
                f"Removing {len(remove_ids)} existing "
                "managed planning-scene objects"
            )

        else:
            for object_id, (
                centre,
                size,
            ) in FIXED_OBJECTS.items():
                scene.world.collision_objects.append(
                    make_box(
                        object_id,
                        centre,
                        size,
                        self.frame_id,
                    )
                )

            # Remove only legacy objects that actually exist.
            legacy_to_remove = sorted(
                set(LEGACY_MOVING_OBJECT_IDS)
                & existing
            )

            for object_id in legacy_to_remove:
                scene.world.collision_objects.append(
                    make_remove(
                        object_id,
                        self.frame_id,
                    )
                )

            description = (
                "Adding fixed environment and removing "
                f"{len(legacy_to_remove)} existing legacy "
                "moving-box copies"
            )

        if not scene.world.collision_objects:
            self.get_logger().info(
                "No planning-scene changes are required."
            )
            return True

        self.get_logger().info(
            description
        )

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = self.apply_client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=15.0,
        )

        if not future.done():
            self.get_logger().error(
                "Planning-scene update timed out."
            )
            return False

        response = future.result()

        if (
            response is None
            or not response.success
        ):
            self.get_logger().error(
                "MoveIt rejected the planning-scene update."
            )
            return False

        return True

    def read_scene_ids(
        self,
    ) -> Sequence[str]:
        request = GetPlanningScene.Request()

        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
        )

        future = self.get_client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=15.0,
        )

        if not future.done():
            raise RuntimeError(
                "Timed out reading the planning scene."
            )

        response = future.result()

        if response is None:
            raise RuntimeError(
                "MoveIt returned no planning scene."
            )

        return sorted(
            collision_object.id
            for collision_object
            in response.scene.world.collision_objects
        )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Add or remove fixed Safety Fence "
            "collision geometry in MoveIt."
        )
    )

    parser.add_argument(
        "--frame",
        default="world",
    )

    parser.add_argument(
        "--remove",
        action="store_true",
    )

    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()

    rclpy.init()

    node = SortingSceneLoader(
        frame_id=arguments.frame,
    )

    try:
        if not node.wait_for_services():
            return 1

        existing_ids = node.read_scene_ids()

        if not node.apply(
            remove=arguments.remove,
            existing_ids=existing_ids,
        ):
            return 1

        scene_ids = set(
            node.read_scene_ids()
        )

        managed_ids = (
            set(FIXED_OBJECTS)
            | set(LEGACY_MOVING_OBJECT_IDS)
        )

        if arguments.remove:
            remaining = sorted(
                managed_ids & scene_ids
            )

            if remaining:
                print(
                    "\nERROR: Managed objects remain:"
                )

                for object_id in remaining:
                    print(f"  {object_id}")

                return 1

            print(
                "\nAll managed planning-scene "
                "objects were removed."
            )
            return 0

        loaded_fixed = sorted(
            set(FIXED_OBJECTS) & scene_ids
        )

        missing_fixed = sorted(
            set(FIXED_OBJECTS) - scene_ids
        )

        legacy_remaining = sorted(
            set(LEGACY_MOVING_OBJECT_IDS)
            & scene_ids
        )

        print(
            "\n=== Fixed sorting-cell scene ==="
        )
        print(f"Frame: {arguments.frame}")
        print(
            f"Loaded fixed objects: "
            f"{len(loaded_fixed)}/{len(FIXED_OBJECTS)}"
        )

        for object_id in loaded_fixed:
            print(f"  {object_id}")

        if missing_fixed:
            print(
                "\nERROR: Missing fixed objects:"
            )

            for object_id in missing_fixed:
                print(f"  {object_id}")

            return 1

        if legacy_remaining:
            print(
                "\nERROR: Static moving-box copies remain:"
            )

            for object_id in legacy_remaining:
                print(f"  {object_id}")

            return 1

        print(
            "Moving boxes are not stored as fixed "
            "world collision objects."
        )

        return 0

    except Exception as error:
        node.get_logger().exception(
            f"Planning-scene loading failed: {error}"
        )
        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
