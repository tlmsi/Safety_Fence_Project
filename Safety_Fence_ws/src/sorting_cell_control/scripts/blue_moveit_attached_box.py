#!/usr/bin/env python3

import time
from typing import Set

import rclpy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetPlanningScene,
)
from shape_msgs.msg import SolidPrimitive


class CarriedBoxSceneManager:

    def __init__(
        self,
        node,
        object_id: str = 'blue_box_carried',
        attach_link: str = 'suction_tip',
    ) -> None:
        self.node = node
        self.object_id = object_id
        self.attach_link = attach_link

        self.touch_links = [
            'suction_tip',
            'suction_cup_link',
            'suction_tool_link',
        ]

        self.apply_client = node.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene',
        )

        self.get_client = node.create_client(
            GetPlanningScene,
            '/get_planning_scene',
        )

    def wait_for_services(
        self,
        timeout: float = 30.0,
    ) -> None:
        self.node.get_logger().info(
            'Waiting for MoveIt planning-scene services...'
        )

        if not self.apply_client.wait_for_service(
            timeout_sec=timeout
        ):
            raise RuntimeError(
                '/apply_planning_scene is unavailable.'
            )

        if not self.get_client.wait_for_service(
            timeout_sec=timeout
        ):
            raise RuntimeError(
                '/get_planning_scene is unavailable.'
            )

        self.node.get_logger().info(
            'MoveIt planning-scene services are ready.'
        )

    def apply_scene(
        self,
        scene: PlanningScene,
        label: str,
    ) -> None:
        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = self.apply_client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=10.0,
        )

        if not future.done():
            raise RuntimeError(
                f'Planning-scene update timed out: {label}'
            )

        response = future.result()

        if (
            response is None
            or not response.success
        ):
            raise RuntimeError(
                f'MoveIt rejected planning-scene update: '
                f'{label}'
            )

    def attached_ids(
        self,
    ) -> Set[str]:
        request = GetPlanningScene.Request()

        components = PlanningSceneComponents()

        request.components.components = (
            components.ROBOT_STATE_ATTACHED_OBJECTS
        )

        future = self.get_client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=10.0,
        )

        if not future.done():
            raise RuntimeError(
                'Timed out reading attached objects.'
            )

        response = future.result()

        if response is None:
            raise RuntimeError(
                'MoveIt returned no planning scene.'
            )

        return {
            attached.object.id
            for attached in (
                response.scene.robot_state
                .attached_collision_objects
            )
        }

    def wait_for_state(
        self,
        required_attached: bool,
        timeout: float = 5.0,
    ) -> None:
        deadline = time.monotonic() + timeout

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            is_attached = (
                self.object_id in self.attached_ids()
            )

            if is_attached == required_attached:
                return

            time.sleep(0.1)

        state = (
            'attached'
            if required_attached
            else 'detached'
        )

        raise RuntimeError(
            f'{self.object_id} did not become '
            f'{state} in MoveIt.'
        )

    def attach_box(
        self,
        half_height: float,
        width: float = 0.06,
    ) -> None:
        if self.object_id in self.attached_ids():
            self.node.get_logger().warning(
                'The blue-box collision object '
                'is already attached.'
            )
            return

        attached = AttachedCollisionObject()
        attached.link_name = self.attach_link
        attached.touch_links = list(
            self.touch_links
        )

        attached.object.header.frame_id = (
            self.attach_link
        )

        attached.object.id = self.object_id
        attached.object.operation = (
            CollisionObject.ADD
        )

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX

        full_height = float(
            2.0 * half_height
        )

        # The physical box rests directly on the conveyor.
        # Shorten only the lower collision face by 2 mm to
        # prevent numerical start-state penetration.
        contact_clearance = 0.002
        collision_height = max(
            0.001,
            full_height - contact_clearance,
        )

        primitive.dimensions = [
            float(width),
            float(width),
            collision_height,
        ]

        pose = Pose()

        # Keep the top face aligned with suction_tip.
        pose.position.z = (
            collision_height / 2.0
        )
        pose.orientation.w = 1.0

        attached.object.primitives.append(
            primitive
        )

        attached.object.primitive_poses.append(
            pose
        )

        scene = PlanningScene()
        scene.name = 'sorting_cell'
        scene.is_diff = True
        scene.robot_state.is_diff = True

        scene.robot_state.attached_collision_objects.append(
            attached
        )

        self.apply_scene(
            scene,
            'attach carried blue box',
        )

        self.wait_for_state(
            required_attached=True
        )

        self.node.get_logger().info(
            'MoveIt attached the blue box to '
            f'{self.attach_link}.'
        )

    def add_world_box(
        self,
        object_id: str,
        center_xyz,
        size_xyz,
        frame_id: str = 'world',
    ) -> None:
        if len(center_xyz) != 3:
            raise RuntimeError(
                'World-box centre must contain three values.'
            )

        if len(size_xyz) != 3:
            raise RuntimeError(
                'World-box size must contain three values.'
            )

        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = object_id
        collision_object.operation = (
            CollisionObject.ADD
        )

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            float(value)
            for value in size_xyz
        ]

        pose = Pose()
        pose.position.x = float(center_xyz[0])
        pose.position.y = float(center_xyz[1])
        pose.position.z = float(center_xyz[2])
        pose.orientation.w = 1.0

        collision_object.primitives.append(
            primitive
        )

        collision_object.primitive_poses.append(
            pose
        )

        scene = PlanningScene()
        scene.name = 'sorting_cell'
        scene.is_diff = True
        scene.world.collision_objects.append(
            collision_object
        )

        self.apply_scene(
            scene,
            f'add placed object {object_id}',
        )

        self.node.get_logger().info(
            f'MoveIt added placed collision object: '
            f'{object_id}.'
        )

    def detach_box(
        self,
    ) -> None:
        if self.object_id not in self.attached_ids():
            self.node.get_logger().warning(
                'The blue-box collision object '
                'is not attached.'
            )
            return

        attached = AttachedCollisionObject()
        attached.link_name = self.attach_link
        attached.object.id = self.object_id
        attached.object.operation = (
            CollisionObject.REMOVE
        )

        scene = PlanningScene()
        scene.name = 'sorting_cell'
        scene.is_diff = True
        scene.robot_state.is_diff = True

        scene.robot_state.attached_collision_objects.append(
            attached
        )

        # Removing an attached object normally returns it
        # to the MoveIt collision world. Remove that temporary
        # world object in the same planning-scene update.
        remove_world = CollisionObject()
        remove_world.id = self.object_id
        remove_world.operation = (
            CollisionObject.REMOVE
        )

        scene.world.collision_objects.append(
            remove_world
        )

        self.apply_scene(
            scene,
            'detach and remove carried blue box',
        )

        self.wait_for_state(
            required_attached=False
        )

        self.node.get_logger().info(
            'MoveIt detached and removed the '
            'carried blue-box collision object.'
        )
