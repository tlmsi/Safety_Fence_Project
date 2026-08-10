#!/usr/bin/env python3

import time

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

# Manual Pause and Protective Stop use the same direct
# controller path as E-stop, but ramp the controller speed
# smoothly to zero instead of applying an immediate halt.
CONTROLLED_STOP_DURATION_SECONDS = 1.50


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

        # Controlled safety stop state.
        #
        # The public safety state may report PAUSE /
        # PROTECTIVE_STOP immediately while the robot is still
        # performing its permitted deceleration motion.
        self.controlled_stop_active = False
        self.controlled_stop_started = 0.0
        self.controlled_stop_target_state = None
        self.controlled_stop_reason = ''

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

        # Current physical gate state.
        # This is bridged back into Gazebo so the gate model
        # always follows the supervisor's accepted state.
        self.gate_state_publisher = (
            self.create_publisher(
                Bool,
                '/safety/gate_visual_open',
                10,
            )
        )

        # ----------------------------------------------------
        # DIRECT E-STOP CONTROLLER PATH
        # ----------------------------------------------------
        #
        # Manual Pause and Protective Stop use a controlled
        # speed-scaling ramp before the normal MoveIt
        # cancellation / recovery path.
        #
        # E_STOP sends speed scaling = 0.0 immediately
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

        # Persistent Gazebo GUI command channel.
        # Unlike repeated ros2 service CLI calls, the Gazebo
        # panel stays connected for immediate commands.
        self.create_subscription(
            String,
            '/safety/gui/command',
            self.gui_command_callback,
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

        # The existing 10 Hz safety heartbeat also drives the
        # controlled deceleration ramp. No blocking sleep is
        # used, so Emergency Stop can still preempt it.
        self.update_controlled_stop()

        reported_state = self.state

        if (
            self.controlled_stop_active
            and self.controlled_stop_target_state
            is not None
        ):
            # Show the requested safety condition immediately
            # to the GUI / stack light even though the current
            # trajectory is still decelerating.
            reported_state = (
                self.controlled_stop_target_state
            )

        state_message = String()
        state_message.data = reported_state

        # During controlled deceleration the current robot
        # trajectory is temporarily permitted to continue.
        # T7 therefore does not cancel it until scale reaches
        # zero. New robot movements are blocked separately in
        # T7 by the non-RUNNING safety state.
        motion_message = Bool()
        motion_message.data = (
            self.state == STATE_RUNNING
        )

        # Conveyor motion stops as soon as the safety command
        # is received; it does not wait for robot deceleration.
        conveyor_message = Bool()
        conveyor_message.data = (
            self.state == STATE_RUNNING
            and not self.controlled_stop_active
        )

        estop_message = Bool()
        estop_message.data = (
            reported_state == STATE_ESTOP
        )

        protective_message = Bool()
        protective_message.data = (
            reported_state
            == STATE_PROTECTIVE_STOP
        )

        self.state_publisher.publish(
            state_message
        )

        self.motion_publisher.publish(
            motion_message
        )

        self.conveyor_publisher.publish(
            conveyor_message
        )

        self.estop_publisher.publish(
            estop_message
        )

        self.protective_stop_publisher.publish(
            protective_message
        )

        gate_message = Bool()
        gate_message.data = bool(
            self.gate_open
        )

        self.gate_state_publisher.publish(
            gate_message
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
    # CONTROLLED SAFETY STOP
    # ========================================================

    def start_controlled_stop(
        self,
        target_state: str,
        reason: str,
    ) -> None:

        if self.state == STATE_ESTOP:
            return

        if self.controlled_stop_active:

            # Protective Stop has priority over an ordinary
            # Manual Pause request.
            if (
                target_state
                == STATE_PROTECTIVE_STOP
                and self.controlled_stop_target_state
                != STATE_PROTECTIVE_STOP
            ):
                self.controlled_stop_target_state = (
                    STATE_PROTECTIVE_STOP
                )

                self.controlled_stop_reason = reason

                self.get_logger().warning(
                    'CONTROLLED STOP upgraded to '
                    'PROTECTIVE_STOP.'
                )

                self.publish_state()

            return

        # If the robot is already stopped, there is no reason
        # to execute another 1.5 second ramp.
        if self.state != STATE_RUNNING:

            self.transition(
                target_state,
                reason,
            )

            return

        self.controlled_stop_active = True

        self.controlled_stop_started = (
            time.monotonic()
        )

        self.controlled_stop_target_state = (
            target_state
        )

        self.controlled_stop_reason = reason

        self.get_logger().warning(
            'CONTROLLED SAFETY STOP STARTED: '
            'controller speed will ramp '
            f'1.000 -> 0.000 over '
            f'{CONTROLLED_STOP_DURATION_SECONDS:.2f} s '
            f'[{reason}]'
        )

        # Publishes the requested safety state and immediately
        # inhibits the conveyor.
        self.publish_state()


    def update_controlled_stop(
        self,
    ) -> None:

        if not self.controlled_stop_active:
            return

        elapsed = max(
            0.0,
            (
                time.monotonic()
                - self.controlled_stop_started
            ),
        )

        progress = min(
            1.0,
            (
                elapsed
                / CONTROLLED_STOP_DURATION_SECONDS
            ),
        )

        factor = max(
            0.0,
            1.0 - progress,
        )

        self.command_controller_speed_scale(
            factor,
            'controlled safety deceleration',
        )

        if progress < 1.0:
            return

        target_state = (
            self.controlled_stop_target_state
        )

        reason = (
            self.controlled_stop_reason
            or 'controlled safety stop'
        )

        self.controlled_stop_active = False
        self.controlled_stop_started = 0.0
        self.controlled_stop_target_state = None
        self.controlled_stop_reason = ''

        if target_state is None:
            target_state = STATE_MANUAL_PAUSE

        previous = self.state
        self.state = target_state

        self.get_logger().warning(
            'CONTROLLED SAFETY STOP COMPLETE: '
            'controller speed = 0.000.'
        )

        if previous != target_state:

            self.get_logger().warning(
                'SAFETY STATE: '
                f'{previous} -> {target_state} '
                f'[{reason}]'
            )


    def cancel_controlled_stop(
        self,
        reason: str,
    ) -> None:

        if not self.controlled_stop_active:
            return

        self.get_logger().warning(
            'CONTROLLED SAFETY STOP PREEMPTED: '
            f'{reason}'
        )

        self.controlled_stop_active = False
        self.controlled_stop_started = 0.0
        self.controlled_stop_target_state = None
        self.controlled_stop_reason = ''


    # ========================================================
    # GAZEBO SAFETY PANEL
    # ========================================================

    def gui_command_callback(
        self,
        message: String,
    ) -> None:

        command = (
            message.data
            .strip()
            .lower()
        )

        if command == 'gate_open':

            gate_message = Bool()
            gate_message.data = True

            self.gate_callback(
                gate_message
            )

            return

        if command == 'gate_close':

            gate_message = Bool()
            gate_message.data = False

            self.gate_callback(
                gate_message
            )

            return

        handlers = {
            'resume': self.resume_callback,
            'pause': self.pause_callback,
            'reset': self.reset_callback,
            'emergency_stop': self.estop_callback,
        }

        handler = handlers.get(
            command
        )

        if handler is None:

            self.get_logger().warning(
                'Unknown Gazebo safety command: '
                f'{command}'
            )

            return

        response = handler(
            Trigger.Request(),
            Trigger.Response(),
        )

        panel_message = (
            'GAZEBO PANEL: '
            f'{command} -> '
            f'{response.message}'
        )

        if response.success:
            self.get_logger().info(
                panel_message
            )
        else:
            self.get_logger().warning(
                panel_message
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

            # Emergency Stop remains highest priority.
            if self.state == STATE_ESTOP:

                self.get_logger().warning(
                    'Gate opened while E-STOP is active. '
                    'E-STOP remains the active '
                    'highest-priority state.'
                )

                self.publish_state()
                return

            # If motion is active, decelerate through the same
            # controller scaling path used by E-stop, but over
            # the configured controlled-stop duration.
            if (
                self.state == STATE_RUNNING
                or self.controlled_stop_active
            ):

                self.start_controlled_stop(
                    STATE_PROTECTIVE_STOP,
                    'safety gate opened',
                )

            else:

                # Robot is already stopped.
                self.transition(
                    STATE_PROTECTIVE_STOP,
                    'safety gate opened',
                )

            return

        # Gate has closed.
        self.get_logger().info(
            'SAFETY GATE CLOSED.'
        )

        if (
            self.controlled_stop_active
            and self.controlled_stop_target_state
            == STATE_PROTECTIVE_STOP
        ):

            self.get_logger().warning(
                'Protective stop remains requested. '
                'Controlled deceleration will finish '
                'and SAFETY RESET will be required.'
            )

        elif self.state == STATE_PROTECTIVE_STOP:

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

        if (
            self.state == STATE_PROTECTIVE_STOP
            or (
                self.controlled_stop_active
                and self.controlled_stop_target_state
                == STATE_PROTECTIVE_STOP
            )
        ):

            response.success = False
            response.message = (
                'Manual Pause cannot replace an '
                'active Protective Stop.'
            )

            return response

        if self.controlled_stop_active:

            response.success = True
            response.message = (
                'Controlled Manual Pause is already '
                'decelerating the robot.'
            )

            return response

        if self.state == STATE_MANUAL_PAUSE:

            response.success = True
            response.message = (
                'System is already manually paused.'
            )

            return response

        self.start_controlled_stop(
            STATE_MANUAL_PAUSE,
            'manual pause',
        )

        response.success = True
        response.message = (
            'Manual Pause requested. '
            'Robot is performing a controlled '
            f'{CONTROLLED_STOP_DURATION_SECONDS:.1f} s '
            'deceleration.'
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

        if self.controlled_stop_active:

            response.success = False
            response.message = (
                'Resume denied: controlled safety '
                'deceleration is still in progress.'
            )

            return response

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

        self.cancel_controlled_stop(
            'Emergency Stop requested'
        )

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

        if self.controlled_stop_active:

            response.success = False
            response.message = (
                'Safety Reset denied: controlled '
                'deceleration is still in progress.'
            )

            return response

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
