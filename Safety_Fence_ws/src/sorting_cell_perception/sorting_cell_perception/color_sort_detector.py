#!/usr/bin/env python3

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger


DetectionResult = Tuple[float, int, int, int, int]


def numbers(text: Optional[str], count: int) -> List[float]:
    if not text:
        return [0.0] * count

    values = [float(value) for value in text.split()]

    if len(values) != count:
        raise RuntimeError(
            f'Expected {count} values, received: {text}'
        )

    return values


def element_pose(element: Optional[ET.Element]) -> List[float]:
    if element is None:
        return [0.0] * 6

    pose = element.find('pose')
    return numbers(pose.text if pose is not None else None, 6)


def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    rotation_x = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cr, -sr],
            [0.0, sr, cr],
        ],
        dtype=float,
    )

    rotation_y = np.array(
        [
            [cp, 0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp],
        ],
        dtype=float,
    )

    rotation_z = np.array(
        [
            [cy, -sy, 0.0],
            [sy, cy, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )

    return rotation_z @ rotation_y @ rotation_x


def pose_matrix(pose: List[float]) -> np.ndarray:
    x, y, z, roll, pitch, yaw = pose

    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation_matrix(roll, pitch, yaw)
    transform[:3, 3] = [x, y, z]
    return transform


def find_include(root: ET.Element, suffix: str) -> ET.Element:
    for include in root.iter('include'):
        uri = (include.findtext('uri') or '').strip().rstrip('/')

        if uri.endswith(suffix.rstrip('/')):
            return include

    raise RuntimeError(f'World include not found: {suffix}')


class SceneGeometry:
    def __init__(self, workspace: Path) -> None:
        gazebo = workspace / 'src' / 'sorting_cell_gazebo'
        models = gazebo / 'models'

        world_path = gazebo / 'worlds' / 'sorting_cell_world.sdf'
        camera_path = models / 'camera_frame' / 'model.sdf'
        conveyor_path = models / 'conveyor' / 'model.sdf'

        for path in (world_path, camera_path, conveyor_path):
            if not path.is_file():
                raise RuntimeError(f'Required geometry file is missing: {path}')

        world_root = ET.parse(world_path).getroot()
        camera_root = ET.parse(camera_path).getroot()
        conveyor_root = ET.parse(conveyor_path).getroot()

        self._load_camera(world_root, camera_root)
        self._load_conveyor(world_root, conveyor_root)
        self._load_box_geometry(world_root)
        self._finish_object_plane()

    def _load_camera(
        self,
        world_root: ET.Element,
        camera_root: ET.Element,
    ) -> None:
        camera_include = find_include(world_root, 'models/camera_frame')
        world_from_model = pose_matrix(element_pose(camera_include))

        camera_model = camera_root.find('./model')
        if camera_model is None:
            raise RuntimeError('Camera model was not found.')

        world_from_model = (
            world_from_model
            @ pose_matrix(element_pose(camera_model))
        )

        link = camera_model.find('./link')
        if link is None:
            raise RuntimeError('Camera link was not found.')

        sensor = link.find("./sensor[@type='camera']")
        if sensor is None:
            raise RuntimeError('Camera sensor was not found.')

        camera = sensor.find('./camera')
        if camera is None:
            raise RuntimeError('Camera configuration was not found.')

        world_from_link = world_from_model @ pose_matrix(element_pose(link))
        world_from_sensor = world_from_link @ pose_matrix(element_pose(sensor))
        world_from_camera = world_from_sensor @ pose_matrix(element_pose(camera))

        self.camera_position = world_from_camera[:3, 3]
        self.world_from_camera_rotation = world_from_camera[:3, :3]

        self.horizontal_fov = float(camera.findtext('horizontal_fov') or '0')
        self.calibration_width = int(camera.findtext('./image/width') or '0')
        self.calibration_height = int(camera.findtext('./image/height') or '0')

        if (
            self.horizontal_fov <= 0.0
            or self.calibration_width <= 0
            or self.calibration_height <= 0
        ):
            raise RuntimeError('Camera FOV or image dimensions are invalid.')

        self.base_focal_pixels = self.calibration_width / (
            2.0 * math.tan(self.horizontal_fov / 2.0)
        )

    def _load_conveyor(
        self,
        world_root: ET.Element,
        conveyor_root: ET.Element,
    ) -> None:
        conveyor_include = find_include(world_root, 'models/conveyor')
        world_from_model = pose_matrix(element_pose(conveyor_include))

        conveyor_model = conveyor_root.find('./model')
        if conveyor_model is None:
            raise RuntimeError('Conveyor model was not found.')

        world_from_model = (
            world_from_model
            @ pose_matrix(element_pose(conveyor_model))
        )

        belt_link = conveyor_model.find("./link[@name='belt_link']")
        if belt_link is None:
            raise RuntimeError('Conveyor belt link was not found.')

        belt_collision = belt_link.find("./collision[@name='belt_collision']")
        if belt_collision is None:
            raise RuntimeError('Conveyor belt collision was not found.')

        size = numbers(belt_collision.findtext('./geometry/box/size'), 3)

        world_from_belt = (
            world_from_model
            @ pose_matrix(element_pose(belt_link))
            @ pose_matrix(element_pose(belt_collision))
        )

        self.belt_axis_x = world_from_belt[:3, 0]
        self.belt_axis_y = world_from_belt[:3, 1]
        self.belt_normal = world_from_belt[:3, 2]
        self.belt_half_width = size[1] / 2.0

        top_local = np.array([0.0, 0.0, size[2] / 2.0, 1.0])
        self.belt_top_point = (world_from_belt @ top_local)[:3]

        frame_link = conveyor_model.find("./link[@name='frame_link']")
        if frame_link is None:
            raise RuntimeError('Conveyor frame link was not found.')

        pickup_visual = frame_link.find("./visual[@name='pickup_zone_visual']")
        if pickup_visual is None:
            raise RuntimeError('Pickup-zone visual was not found.')

        world_from_pickup = (
            world_from_model
            @ pose_matrix(element_pose(frame_link))
            @ pose_matrix(element_pose(pickup_visual))
        )

        self.pickup_center = world_from_pickup[:3, 3]
        self.pickup_longitudinal = self.longitudinal(self.pickup_center)

        pickup_size = numbers(
            pickup_visual.findtext('./geometry/box/size'),
            3,
        )
        self.pickup_half_length = pickup_size[0] / 2.0

    def _load_box_geometry(self, world_root: ET.Element) -> None:
        for model in world_root.iter('model'):
            name = model.attrib.get('name', '')

            if not name.endswith('_box'):
                continue

            for query in (
                './link/collision/geometry/box/size',
                './link/visual/geometry/box/size',
            ):
                size_text = model.findtext(query)

                if size_text:
                    self.box_size = np.array(numbers(size_text, 3), dtype=float)
                    return

        raise RuntimeError('Colored-box dimensions were not found.')

    def _finish_object_plane(self) -> None:
        self.object_center_plane_point = (
            self.belt_top_point
            + self.belt_normal * (self.box_size[2] / 2.0)
        )

    def longitudinal(self, point: np.ndarray) -> float:
        return float(
            np.dot(point - self.belt_top_point, self.belt_axis_x)
        )

    def lateral(self, point: np.ndarray) -> float:
        return float(
            np.dot(point - self.belt_top_point, self.belt_axis_y)
        )

    def camera_intrinsics(
        self,
        width: int,
        height: int,
    ) -> Tuple[float, float, float, float]:
        scale_x = width / float(self.calibration_width)
        scale_y = height / float(self.calibration_height)

        focal_x = self.base_focal_pixels * scale_x
        focal_y = self.base_focal_pixels * scale_y
        center_x = 0.5 * self.calibration_width * scale_x
        center_y = 0.5 * self.calibration_height * scale_y

        return focal_x, focal_y, center_x, center_y

    def pixel_to_box_center(
        self,
        pixel_x: float,
        pixel_y: float,
        width: int,
        height: int,
    ) -> Optional[np.ndarray]:
        focal_x, focal_y, center_x, center_y = self.camera_intrinsics(
            width,
            height,
        )

        # Gazebo camera coordinates are +X forward, +Y left, +Z up.
        ray_camera = np.array(
            [
                1.0,
                -(pixel_x - center_x) / focal_x,
                -(pixel_y - center_y) / focal_y,
            ],
            dtype=float,
        )

        ray_world = self.world_from_camera_rotation @ ray_camera
        denominator = float(np.dot(self.belt_normal, ray_world))

        if abs(denominator) < 1.0e-9:
            return None

        distance = float(
            np.dot(
                self.belt_normal,
                self.object_center_plane_point - self.camera_position,
            )
            / denominator
        )

        if distance <= 0.0:
            return None

        return self.camera_position + distance * ray_world

    def world_to_pixel(
        self,
        point: np.ndarray,
        width: int,
        height: int,
    ) -> Optional[Tuple[int, int]]:
        focal_x, focal_y, center_x, center_y = self.camera_intrinsics(
            width,
            height,
        )

        camera_point = (
            self.world_from_camera_rotation.T
            @ (point - self.camera_position)
        )

        forward = float(camera_point[0])
        if forward <= 0.0:
            return None

        pixel_x = center_x - focal_x * float(camera_point[1]) / forward
        pixel_y = center_y - focal_y * float(camera_point[2]) / forward

        return int(round(pixel_x)), int(round(pixel_y))

    def pickup_line_world_points(self) -> Tuple[np.ndarray, np.ndarray]:
        center = (
            self.belt_top_point
            + self.belt_axis_x * self.pickup_longitudinal
            + self.belt_normal * (self.box_size[2] / 2.0)
        )

        return (
            center - self.belt_axis_y * self.belt_half_width,
            center + self.belt_axis_y * self.belt_half_width,
        )


class ColorSortDetector(Node):
    def __init__(self) -> None:
        super().__init__('color_sort_detector')

        self.declare_parameter('belt_speed', 0.12)
        self.declare_parameter('minimum_area', 250.0)
        self.declare_parameter('stop_lead_distance', 0.025)
        self.declare_parameter('pickup_tolerance', 0.060)
        self.declare_parameter('belt_lateral_margin', 0.050)
        self.declare_parameter('settle_frames_required', 4)
        self.declare_parameter('settle_motion_threshold', 0.0025)
        self.declare_parameter('clear_frames_required', 4)

        self.belt_speed = float(self.get_parameter('belt_speed').value)
        self.minimum_area = float(self.get_parameter('minimum_area').value)
        self.stop_lead_distance = float(
            self.get_parameter('stop_lead_distance').value
        )
        self.pickup_tolerance = float(
            self.get_parameter('pickup_tolerance').value
        )
        self.belt_lateral_margin = float(
            self.get_parameter('belt_lateral_margin').value
        )
        self.settle_frames_required = int(
            self.get_parameter('settle_frames_required').value
        )
        self.settle_motion_threshold = float(
            self.get_parameter('settle_motion_threshold').value
        )
        self.clear_frames_required = int(
            self.get_parameter('clear_frames_required').value
        )

        workspace = Path(__file__).resolve().parents[3]
        self.geometry = SceneGeometry(workspace)

        self.bridge = CvBridge()
        self.conveyor_running: Optional[bool] = None
        self.stopped_for_box = False
        self.object_latched = False
        self.clear_frames = 0
        self.stable_frames = 0
        self.previous_longitudinal: Optional[float] = None
        self.tracked_color: Optional[str] = None

        self.image_subscription = self.create_subscription(
            Image,
            '/sorting_camera/image',
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.conveyor_publisher = self.create_publisher(
            Float64,
            '/conveyor/cmd_vel',
            10,
        )

        self.color_publisher = self.create_publisher(
            String,
            '/perception/detected_color',
            10,
        )

        self.zone_publisher = self.create_publisher(
            Bool,
            '/perception/object_in_pickup_zone',
            10,
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/perception/box_pose',
            10,
        )

        self.debug_publisher = self.create_publisher(
            Image,
            '/sorting_camera/debug',
            10,
        )

        self.resume_service = self.create_service(
            Trigger,
            '/perception/resume_conveyor',
            self.resume_callback,
        )

        target = self.geometry.pickup_center
        camera = self.geometry.camera_position

        self.get_logger().info('Coordinate-based RGB detector started.')
        self.get_logger().info(
            'Camera world position: '
            f'x={camera[0]:.3f}, y={camera[1]:.3f}, z={camera[2]:.3f}'
        )
        self.get_logger().info(
            'Physical pickup target: '
            f'x={target[0]:.3f}, y={target[1]:.3f}'
        )

    def command_conveyor(self, running: bool) -> None:
        if self.conveyor_running is running:
            return

        message = Float64()
        message.data = self.belt_speed if running else 0.0
        self.conveyor_publisher.publish(message)
        self.conveyor_running = running

        state = 'RUNNING' if running else 'STOPPED'
        self.get_logger().info(f'Conveyor command: {state}')

    def resume_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        response.success = True
        response.message = 'Automatic coordinate-based conveyor control is active.'
        return response

    def create_masks(self, hsv: np.ndarray) -> Dict[str, np.ndarray]:
        red_low = cv2.inRange(
            hsv,
            np.array([0, 80, 60], dtype=np.uint8),
            np.array([10, 255, 255], dtype=np.uint8),
        )
        red_high = cv2.inRange(
            hsv,
            np.array([170, 80, 60], dtype=np.uint8),
            np.array([179, 255, 255], dtype=np.uint8),
        )

        return {
            'red': cv2.bitwise_or(red_low, red_high),
            'green': cv2.inRange(
                hsv,
                np.array([35, 80, 60], dtype=np.uint8),
                np.array([85, 255, 255], dtype=np.uint8),
            ),
            'blue': cv2.inRange(
                hsv,
                np.array([90, 80, 60], dtype=np.uint8),
                np.array([135, 255, 255], dtype=np.uint8),
            ),
        }

    def find_largest_object(self, mask: np.ndarray) -> Optional[DetectionResult]:
        kernel = np.ones((5, 5), dtype=np.uint8)
        cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            cleaned,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(largest))

        if area < self.minimum_area:
            return None

        x, y, width, height = cv2.boundingRect(largest)
        return area, x, y, width, height

    def publish_zone_state(self, active: bool) -> None:
        message = Bool()
        message.data = active
        self.zone_publisher.publish(message)

    def publish_box_pose(
        self,
        source_message: Image,
        point: np.ndarray,
    ) -> None:
        message = PoseStamped()
        message.header.stamp = source_message.header.stamp
        message.header.frame_id = 'world'
        message.pose.position.x = float(point[0])
        message.pose.position.y = float(point[1])
        message.pose.position.z = float(point[2])
        message.pose.orientation.w = 1.0
        self.pose_publisher.publish(message)

    def publish_debug_image(
        self,
        frame: np.ndarray,
        source_message: Image,
    ) -> None:
        try:
            debug = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            debug.header = source_message.header
            self.debug_publisher.publish(debug)
        except CvBridgeError as error:
            self.get_logger().error(f'Could not publish debug image: {error}')

    def draw_pickup_line(self, frame: np.ndarray) -> None:
        height, width = frame.shape[:2]
        point_a, point_b = self.geometry.pickup_line_world_points()
        pixel_a = self.geometry.world_to_pixel(point_a, width, height)
        pixel_b = self.geometry.world_to_pixel(point_b, width, height)

        if pixel_a is None or pixel_b is None:
            return

        cv2.line(frame, pixel_a, pixel_b, (0, 255, 255), 2)
        label_x = min(max(pixel_a[0], 5), width - 180)
        label_y = min(max(pixel_a[1] - 8, 20), height - 5)
        cv2.putText(
            frame,
            'PHYSICAL PICKUP LINE',
            (label_x, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            2,
        )

    def image_callback(self, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )
        except CvBridgeError as error:
            self.get_logger().error(f'Could not convert camera image: {error}')
            return

        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = self.create_masks(hsv)

        drawing_colors = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
        }

        candidates = []

        for color_name, mask in masks.items():
            result = self.find_largest_object(mask)
            if result is None:
                continue

            area, x, y, box_width, box_height = result
            pixel_x = x + box_width / 2.0
            pixel_y = y + box_height / 2.0

            world_point = self.geometry.pixel_to_box_center(
                pixel_x,
                pixel_y,
                width,
                height,
            )

            if world_point is None:
                continue

            longitudinal = self.geometry.longitudinal(world_point)
            lateral = self.geometry.lateral(world_point)
            on_belt = abs(lateral) <= (
                self.geometry.belt_half_width + self.belt_lateral_margin
            )

            color = drawing_colors[color_name]
            cv2.rectangle(
                frame,
                (x, y),
                (x + box_width, y + box_height),
                color,
                2,
            )
            cv2.circle(
                frame,
                (int(round(pixel_x)), int(round(pixel_y))),
                4,
                color,
                -1,
            )
            cv2.putText(
                frame,
                f'{color_name}  x={world_point[0]:+.3f} y={world_point[1]:+.3f}',
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                2,
            )

            if on_belt:
                candidates.append(
                    {
                        'color': color_name,
                        'point': world_point,
                        'longitudinal': longitudinal,
                        'lateral': lateral,
                        'area': area,
                    }
                )

        selected = max(
            candidates,
            key=lambda item: float(item['longitudinal']),
            default=None,
        )

        object_ready = False
        selected_color: Optional[str] = None
        selected_longitudinal: Optional[float] = None

        if selected is not None:
            selected_color = str(selected['color'])
            selected_point = np.asarray(selected['point'], dtype=float)
            selected_longitudinal = float(selected['longitudinal'])
            self.publish_box_pose(message, selected_point)

            trigger = (
                self.geometry.pickup_longitudinal
                - self.stop_lead_distance
            )
            near_pickup = selected_longitudinal >= trigger

            if not self.stopped_for_box and near_pickup:
                self.command_conveyor(False)
                self.stopped_for_box = True
                self.clear_frames = 0
                self.stable_frames = 0
                self.previous_longitudinal = selected_longitudinal
                self.tracked_color = selected_color

                self.get_logger().info(
                    f'{selected_color.upper()} reached stop trigger: '
                    f'x={selected_point[0]:.3f}, '
                    f'y={selected_point[1]:.3f}'
                )

            elif not self.stopped_for_box:
                self.command_conveyor(True)

            if self.stopped_for_box and near_pickup:
                self.clear_frames = 0

                if self.tracked_color != selected_color:
                    self.tracked_color = selected_color
                    self.stable_frames = 0
                    self.previous_longitudinal = selected_longitudinal

                elif self.previous_longitudinal is not None:
                    motion = abs(
                        selected_longitudinal - self.previous_longitudinal
                    )

                    if motion <= self.settle_motion_threshold:
                        self.stable_frames += 1
                    else:
                        self.stable_frames = 0

                    self.previous_longitudinal = selected_longitudinal

                target_error = abs(
                    selected_longitudinal
                    - self.geometry.pickup_longitudinal
                )

                object_ready = (
                    self.stable_frames >= self.settle_frames_required
                    and target_error <= self.pickup_tolerance
                )

            elif self.stopped_for_box:
                self.clear_frames += 1

        else:
            if self.stopped_for_box:
                self.clear_frames += 1
            else:
                self.command_conveyor(True)

        if (
            self.stopped_for_box
            and self.clear_frames >= self.clear_frames_required
        ):
            self.stopped_for_box = False
            self.object_latched = False
            self.clear_frames = 0
            self.stable_frames = 0
            self.previous_longitudinal = None
            self.tracked_color = None
            self.command_conveyor(True)
            self.get_logger().info(
                'Physical pickup position is clear. Conveyor restarted.'
            )

        self.publish_zone_state(object_ready)

        if object_ready and selected_color is not None:
            # Publish the ready box color continuously while the box remains
            # settled. This lets automation started after the stop event still
            # receive the current color state.
            color_message = String()
            color_message.data = selected_color
            self.color_publisher.publish(color_message)

            if not self.object_latched:
                self.object_latched = True

                self.get_logger().info(
                    f'{selected_color.upper()} box settled at the physical '
                    'pickup position. Box pose published.'
                )

        self.draw_pickup_line(frame)

        status = 'RUNNING' if self.conveyor_running else 'STOPPED'
        cv2.putText(
            frame,
            f'Conveyor: {status}',
            (15, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )

        if selected_longitudinal is not None:
            remaining = (
                self.geometry.pickup_longitudinal - selected_longitudinal
            )
            cv2.putText(
                frame,
                f'Distance to pickup: {remaining:+.3f} m',
                (15, 56),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
            )

        cv2.putText(
            frame,
            f'Box ready: {object_ready}',
            (15, 84),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )

        self.publish_debug_image(frame, message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorSortDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.command_conveyor(False)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
