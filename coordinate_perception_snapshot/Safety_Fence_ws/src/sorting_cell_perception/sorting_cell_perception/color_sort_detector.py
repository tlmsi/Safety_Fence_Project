#!/usr/bin/env python3

from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger


DetectionResult = Tuple[float, int, int, int, int]


class ColorSortDetector(Node):

    def __init__(self) -> None:
        super().__init__('color_sort_detector')

        self.declare_parameter('belt_speed', 0.12)
        self.declare_parameter('minimum_area', 250.0)

        self.declare_parameter('roi_center_x', 320)
        self.declare_parameter('roi_center_y', 240)
        self.declare_parameter('roi_half_width', 18)
        self.declare_parameter('roi_half_height', 18)
        self.declare_parameter('clear_frames_required', 3)

        self.belt_speed = float(
            self.get_parameter('belt_speed').value
        )

        self.minimum_area = float(
            self.get_parameter('minimum_area').value
        )

        self.roi_center_x = int(
            self.get_parameter('roi_center_x').value
        )

        self.roi_center_y = int(
            self.get_parameter('roi_center_y').value
        )

        self.roi_half_width = int(
            self.get_parameter('roi_half_width').value
        )

        self.roi_half_height = int(
            self.get_parameter('roi_half_height').value
        )
        self.clear_frames_required = int(
            self.get_parameter(
                'clear_frames_required'
            ).value
        )


        self.bridge = CvBridge()
        self.conveyor_started = False
        self.object_latched = False
        self.clear_frames = 0


        self.image_subscription = self.create_subscription(
            Image,
            '/sorting_camera/image',
            self.image_callback,
            10,
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

        self.get_logger().info(
            'RGB sorting detector started.'
        )

    def command_conveyor(self, velocity: float) -> None:
        message = Float64()
        message.data = float(velocity)
        self.conveyor_publisher.publish(message)

    def resume_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        response.success = True
        response.message = (
            'Automatic camera conveyor control is active.'
        )

        self.get_logger().info(
            'Resume request received. '
            'Conveyor state remains camera-controlled.'
        )

        return response

    def create_masks(

        self,
        hsv: np.ndarray,
    ) -> Dict[str, np.ndarray]:

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

        red = cv2.bitwise_or(red_low, red_high)

        green = cv2.inRange(
            hsv,
            np.array([35, 80, 60], dtype=np.uint8),
            np.array([85, 255, 255], dtype=np.uint8),
        )

        blue = cv2.inRange(
            hsv,
            np.array([90, 80, 60], dtype=np.uint8),
            np.array([135, 255, 255], dtype=np.uint8),
        )

        return {
            'red': red,
            'green': green,
            'blue': blue,
        }

    def find_largest_object(
        self,
        mask: np.ndarray,
    ) -> Optional[DetectionResult]:

        kernel = np.ones((5, 5), dtype=np.uint8)

        cleaned = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel,
        )

        cleaned = cv2.morphologyEx(
            cleaned,
            cv2.MORPH_CLOSE,
            kernel,
        )

        contours, _ = cv2.findContours(
            cleaned,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        if not contours:
            return None

        largest_contour = max(
            contours,
            key=cv2.contourArea,
        )

        area = float(
            cv2.contourArea(largest_contour)
        )

        if area < self.minimum_area:
            return None

        x, y, width, height = cv2.boundingRect(
            largest_contour
        )

        return area, x, y, width, height

    def publish_zone_state(
        self,
        active: bool,
    ) -> None:
        message = Bool()
        message.data = bool(active)
        self.zone_publisher.publish(message)

    def publish_debug_image(
        self,
        frame: np.ndarray,
        source_message: Image,
    ) -> None:
        try:
            debug_message = self.bridge.cv2_to_imgmsg(
                frame,
                encoding='bgr8',
            )

            debug_message.header = source_message.header
            self.debug_publisher.publish(debug_message)

        except CvBridgeError as error:
            self.get_logger().error(
                f'Could not publish debug image: {error}'
            )

    def image_callback(
        self,
        message: Image,
    ) -> None:

        try:
            frame = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )

        except CvBridgeError as error:
            self.get_logger().error(
                f'Could not convert camera image: {error}'
            )
            return

        height, width = frame.shape[:2]

        center_x = min(
            max(self.roi_center_x, 0),
            width - 1,
        )

        center_y = min(
            max(self.roi_center_y, 0),
            height - 1,
        )

        roi_left = max(
            0,
            center_x - self.roi_half_width,
        )

        roi_right = min(
            width - 1,
            center_x + self.roi_half_width,
        )

        roi_top = max(
            0,
            center_y - self.roi_half_height,
        )

        roi_bottom = min(
            height - 1,
            center_y + self.roi_half_height,
        )

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV,
        )

        masks = self.create_masks(hsv)

        drawing_colors = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
        }

        detected_in_zone: Optional[str] = None
        largest_in_zone_area = 0.0

        for color_name, mask in masks.items():

            result = self.find_largest_object(mask)

            if result is None:
                continue

            area, x, y, box_width, box_height = result

            object_center_x = x + box_width // 2
            object_center_y = y + box_height // 2

            drawing_color = drawing_colors[color_name]

            cv2.rectangle(
                frame,
                (x, y),
                (x + box_width, y + box_height),
                drawing_color,
                2,
            )

            cv2.circle(
                frame,
                (object_center_x, object_center_y),
                4,
                drawing_color,
                -1,
            )

            cv2.putText(
                frame,
                f'{color_name}: {area:.0f}',
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                drawing_color,
                2,
            )

            center_is_inside_roi = (
                roi_left <= object_center_x <= roi_right
                and roi_top <= object_center_y <= roi_bottom
            )

            if (
                center_is_inside_roi
                and area > largest_in_zone_area
            ):
                detected_in_zone = color_name
                largest_in_zone_area = area

        cv2.rectangle(
            frame,
            (roi_left, roi_top),
            (roi_right, roi_bottom),
            (0, 255, 255),
            2,
        )

        cv2.putText(
            frame,
            'PICKUP ZONE',
            (roi_left, max(20, roi_top - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2,
        )
        object_in_zone = (
            detected_in_zone is not None
        )

        self.publish_zone_state(
            object_in_zone
        )

        if object_in_zone:
            # Stop immediately whenever any red, green,
            # or blue box is inside the pickup zone.
            self.clear_frames = 0

            if self.conveyor_started:
                self.command_conveyor(0.0)
                self.conveyor_started = False

            if not self.object_latched:
                self.object_latched = True

                color_message = String()
                color_message.data = detected_in_zone
                self.color_publisher.publish(
                    color_message
                )

                self.get_logger().info(
                    f'{detected_in_zone.upper()} box reached '
                    f'pickup zone. Target bin: '
                    f'{detected_in_zone}_bin. '
                    f'Conveyor stopped.'
                )

        else:
            # Require several consecutive clear frames so
            # one noisy frame cannot restart the conveyor.
            self.clear_frames += 1

            if (
                self.clear_frames
                >= self.clear_frames_required
            ):
                self.object_latched = False

                if not self.conveyor_started:
                    self.command_conveyor(
                        self.belt_speed
                    )

                    self.conveyor_started = True

                    self.get_logger().info(
                        'Pickup zone clear. '
                        f'Conveyor running at '
                        f'{self.belt_speed:.2f} m/s.'
                    )

        status_text = (
            'RUNNING'
            if self.conveyor_started
            else 'STOPPED'
        )


        cv2.putText(
            frame,
            f'Conveyor: {status_text}',
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        if detected_in_zone is not None:
            cv2.putText(
                frame,
                f'Zone color: {detected_in_zone}',
                (15, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                drawing_colors[detected_in_zone],
                2,
            )

        self.publish_debug_image(
            frame,
            message,
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = ColorSortDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.command_conveyor(0.0)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
