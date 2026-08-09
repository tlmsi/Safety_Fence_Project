#!/usr/bin/env python3

import rclpy

from control_msgs.msg import SpeedScalingFactor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


STATE_RUNNING = 'RUNNING'
STATE_MANUAL_PAUSE = 'MANUAL_PAUSE'
STATE_PROTECTIVE_STOP = 'PROTECTIVE_STOP'
STATE_ESTOP = 'E_STOP'


class SafetySupervisor(Node):

    def __init__(self) -> None:
        super().__init__(
            'safety_supervisor'
        )

        # ----------------------------------------------------
        # SAFETY STATE
        # ----------------------------------------------------
        #
        # Startup is intentionally inhibited.
        # An explicit Resume is required.
        self.state = STATE_MANUAL_PAUSE

        # Gate is initially assumed closed in the simulation.
        # The Gazebo gate sensor will later own this input.
        self.gate_open = False

        # ----------------------------------------------------
        # OUTPUTS
        # ----------------------------------------------------

        self.state_publisher = (
            self.create_publisher(
                String,
                '/safety/state',
                10,
            )
        )

        self.motion_publisher = (
            self.create_publisher(
                Bool,
                '/safety/motion_allowed',
                10,
            )
        )

        self.conveyor_publisher = (
            self.create_publisher(
                Bool,
                '/safety/conveyor_allowed',
                10,
            )
        )

        self.estop_publisher = (
            self.create_publisher(
                Bool,
                '/safety/estop_latched',
                10,
            )
        )

        self.protective_stop_publisher = (
            self.create_publisher(
                Bool,
                '/safety/protective_stop_active',
                10,
            )
        )

        # ----------------------------------------------------
        # DIRECT E-STOP CONTROLLER PATH
        # ----------------------------------------------------
        #
        # Manual Pause and Protective Stop keep using the
        # normal MoveIt cancellation path.
        #
        # E_STOP additionally sends speed scaling = 0.0
        # directly to the active JointTrajectoryController.
        # This prevents trajectory progression immediately
        # while T7 performs its normal cancellation/recovery.

        speed_scaling_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.controller_speed_scale_publisher = (
            self.create_publisher(
                SpeedScalingFactor,
                (
                    '/joint_trajectory_controller/'
                    'speed_scaling_input'
                ),
                speed_scaling_qos,
            )
        )

        # ----------------------------------------------------
        # INPUTS
        # ----------------------------------------------------
        #
        # For now this can be tested manually:
        #
        #   data: true  -> gate opened
        #   data: false -> gate closed
        #
        # Later the Gazebo safety gate sensor will publish it.
        self.create_subscription(
            Bool,
            '/safety/gate_open',
            self.gate_callback,
            10,
        )

        # ----------------------------------------------------
        # OPERATOR SERVICES
        # ----------------------------------------------------

        self.create_service(
            Trigger,
            '/safety/pause',
            self.pause_callback,
        )

        self.create_service(
            Trigger,
            '/safety/resume',
            self.resume_callback,
        )

        self.create_service(
            Trigger,
            '/safety/emergency_stop',
            self.estop_callback,
        )

        self.create_service(
            Trigger,
            '/safety/reset',
            self.reset_callback,
        )

        # Repeated publication is intentional.
        # T7/T3 may start later and must still receive the
        # current safety permission.
        self.create_timer(
            0.10,
            self.publish_state,
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'SAFETY SUPERVISOR STARTED.'
        )

        self.get_logger().info(
            'States: RUNNING / MANUAL_PAUSE / '
            'PROTECTIVE_STOP / E_STOP'
        )

        self.get_logger().info(
            'Initial state: MANUAL_PAUSE.'
        )

        self.get_logger().info(
            'Explicit /safety/resume is required.'
        )

        self.get_logger().info(
            '========================================'
        )

        self.publish_state()

    # ========================================================
    # OUTPUT STATE
    # ========================================================

    def publish_state(self) -> None:

        state_message = String()
        state_message.data = self.state

        allowed_message = Bool()
        allowed_message.data = (
            self.state == STATE_RUNNING
        )

        estop_message = Bool()
        estop_message.data = (
            self.state == STATE_ESTOP
        )

        protective_message = Bool()
        protective_message.data = (
            self.state == STATE_PROTECTIVE_STOP
        )

        self.state_publisher.publish(
            state_message
        )

        self.motion_publisher.publish(
            allowed_message
        )

        self.conveyor_publisher.publish(
            allowed_message
        )

        self.estop_publisher.publish(
            estop_message
        )

        self.protective_stop_publisher.publish(
            protective_message
        )

    def transition(
        self,
        new_state: str,
        reason: str,
    ) -> None:

        if new_state == self.state:
            self.publish_state()
            return

        previous = self.state
        self.state = new_state

        self.get_logger().warning(
            'SAFETY STATE: '
            f'{previous} -> {new_state} '
            f'[{reason}]'
        )

        self.publish_state()

    # ========================================================
    # DIRECT CONTROLLER SPEED CONTROL
    # ========================================================

    def command_controller_speed_scale(
        self,
        factor: float,
        reason: str,
    ) -> None:

        message = SpeedScalingFactor()
        message.factor = float(factor)

        self.controller_speed_scale_publisher.publish(
            message
        )

        self.get_logger().warning(
            'CONTROLLER SPEED SCALE: '
            f'{factor:.3f} [{reason}]'
        )

    # ========================================================
    # GATE / PROTECTIVE STOP
    # ========================================================

    def gate_callback(
        self,
        message: Bool,
    ) -> None:

        opened = bool(
            message.data
        )

        if opened == self.gate_open:
            return

        self.gate_open = opened

        if opened:

            self.get_logger().warning(
                'SAFETY GATE OPENED.'
            )

            # Emergency stop has the highest priority.
            # Do not downgrade E_STOP to PROTECTIVE_STOP.
            if self.state != STATE_ESTOP:

                self.transition(
                    STATE_PROTECTIVE_STOP,
                    'safety gate opened',
                )

            else:

                self.get_logger().warning(
                    'Gate opened while E-STOP is active. '
                    'E-STOP remains the active '
                    'highest-priority state.'
                )

            return

        # Gate has closed.
        self.get_logger().info(
            'SAFETY GATE CLOSED.'
        )

        if self.state == STATE_PROTECTIVE_STOP:

            self.get_logger().warning(
                'Protective stop remains latched. '
                'SAFETY RESET is required.'
            )

        self.publish_state()

    # ========================================================
    # MANUAL PAUSE
    # ========================================================

    def pause_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:

        del request

        if self.state == STATE_ESTOP:

            response.success = False
            response.message = (
                'Manual Pause cannot replace an '
                'active Emergency Stop.'
            )

            return response

        if self.state == STATE_PROTECTIVE_STOP:

            response.success = False
            response.message = (
                'Manual Pause cannot replace an '
                'active Protective Stop.'
            )

            return response

        if self.state == STATE_MANUAL_PAUSE:

            response.success = True
            response.message = (
                'System is already manually paused.'
            )

            return response

        self.transition(
            STATE_MANUAL_PAUSE,
            'manual pause',
        )

        response.success = True
        response.message = (
            'Manual pause active. '
            'Resume may continue automation directly.'
        )

        return response

    # ========================================================
    # RESUME
    # ========================================================

    def resume_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:

        del request

        if self.state == STATE_ESTOP:

            response.success = False
            response.message = (
                'Resume denied: Emergency Stop '
                'is latched. Safety Reset is required.'
            )

            return response

        if self.state == STATE_PROTECTIVE_STOP:

            if self.gate_open:

                response.success = False
                response.message = (
                    'Resume denied: safety gate '
                    'is still open.'
                )

            else:

                response.success = False
                response.message = (
                    'Resume denied: Protective Stop '
                    'is latched. Safety Reset is '
                    'required first.'
                )

            return response

        if self.gate_open:

            # Defensive check. Normally gate_callback()
            # will already have entered PROTECTIVE_STOP.
            self.transition(
                STATE_PROTECTIVE_STOP,
                'resume attempted with gate open',
            )

            response.success = False
            response.message = (
                'Resume denied: safety gate is open.'
            )

            return response

        if self.state == STATE_RUNNING:

            response.success = True
            response.message = (
                'System is already running.'
            )

            return response

        # Restore trajectory progression only when Resume
        # is genuinely permitted. Reset alone never restores
        # robot motion.
        self.command_controller_speed_scale(
            1.0,
            'motion resume',
        )

        self.transition(
            STATE_RUNNING,
            'manual resume',
        )

        response.success = True
        response.message = (
            'Safety permission granted. '
            'Automation may continue.'
        )

        return response

    # ========================================================
    # EMERGENCY STOP
    # ========================================================

    def estop_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:

        del request

        # Emergency path deliberately bypasses the ordinary
        # MoveIt cancellation latency for the physical stop.
        #
        # Publish controller halt FIRST. The ordinary safety
        # transition then tells T7 to cancel its MoveIt goal
        # and prepare safe recovery.
        self.command_controller_speed_scale(
            0.0,
            'EMERGENCY STOP',
        )

        self.transition(
            STATE_ESTOP,
            'emergency stop',
        )

        response.success = True
        response.message = (
            'EMERGENCY STOP LATCHED. '
            'Safety Reset and Resume are required.'
        )

        return response

    # ========================================================
    # SAFETY RESET
    # ========================================================

    def reset_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:

        del request

        # ----------------------------------------------------
        # E-STOP RESET
        # ----------------------------------------------------

        if self.state == STATE_ESTOP:

            # If the gate is still open, clearing E-stop may
            # remove the highest-priority condition but cannot
            # grant ordinary pause/run status.
            if self.gate_open:

                self.transition(
                    STATE_PROTECTIVE_STOP,
                    'E-stop reset while gate remains open',
                )

                response.success = True
                response.message = (
                    'Emergency Stop reset, but the '
                    'safety gate remains open. '
                    'PROTECTIVE_STOP remains active.'
                )

                return response

            self.transition(
                STATE_MANUAL_PAUSE,
                'emergency stop reset',
            )

            response.success = True
            response.message = (
                'Emergency Stop reset. '
                'System remains in MANUAL_PAUSE. '
                'Press Resume to restore motion.'
            )

            return response

        # ----------------------------------------------------
        # PROTECTIVE STOP RESET
        # ----------------------------------------------------

        if self.state == STATE_PROTECTIVE_STOP:

            if self.gate_open:

                response.success = False
                response.message = (
                    'Safety Reset denied: '
                    'the safety gate is still open.'
                )

                return response

            self.transition(
                STATE_MANUAL_PAUSE,
                'protective stop reset',
            )

            response.success = True
            response.message = (
                'Protective Stop reset. '
                'System remains in MANUAL_PAUSE. '
                'Press Resume to restore motion.'
            )

            return response

        # ----------------------------------------------------
        # OTHER STATES
        # ----------------------------------------------------

        if self.state == STATE_RUNNING:

            response.success = True
            response.message = (
                'System is already running. '
                'No latched safety stop exists.'
            )

            return response

        response.success = True
        response.message = (
            'System is in MANUAL_PAUSE. '
            'No reset is required. '
            'Press Resume to continue.'
        )

        return response


def main() -> int:

    rclpy.init()

    node = SafetySupervisor()

    try:

        rclpy.spin(
            node
        )

        return 0

    except KeyboardInterrupt:

        node.get_logger().warning(
            'Safety supervisor interrupted.'
        )

        return 130

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
