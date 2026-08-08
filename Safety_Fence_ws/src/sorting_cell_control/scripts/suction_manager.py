#!/usr/bin/env python3

import re
import subprocess
import threading
import time

import rclpy

from rclpy.node import Node
from std_srvs.srv import Trigger


WORLD_NAME = 'sorting_cell_world'

SYSTEM_ADD_SERVICE = (
    f'/world/{WORLD_NAME}/entity/system/add'
)

SCENE_SERVICE = (
    f'/world/{WORLD_NAME}/scene/info'
)

VALID_COLORS = (
    'red',
    'green',
    'blue',
)

MAX_BOX_INDEX = 16

PARENT_LINK = 'wrist_3_link'
CHILD_LINK = 'box_link'

# No multi-second artificial delays.
JOINT_CREATE_TIMEOUT = 2.0
STATE_TIMEOUT = 1.5
LISTENER_STARTUP = 0.15


def model_name(
    color,
    index,
):

    if index == 1:
        return f'{color}_box'

    return (
        f'{color}_box_'
        f'{index:02d}'
    )


def channel_name(
    color,
    index,
):

    if index == 1:
        return color

    return (
        f'{color}_'
        f'{index:02d}'
    )


class GazeboStateMonitor:

    def __init__(
        self,
        topic,
    ):

        self.topic = topic

        self.condition = (
            threading.Condition()
        )

        self.state = None
        self.sequence = 0

        self.process = subprocess.Popen(
            [
                'stdbuf',
                '-oL',
                '-eL',
                'gz',
                'topic',
                '-e',
                '-t',
                topic,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )

        self.thread = threading.Thread(
            target=self._reader,
            daemon=True,
        )

        self.thread.start()

    def _reader(
        self,
    ):

        stream = self.process.stdout

        if stream is None:
            return

        while True:

            line = stream.readline()

            if not line:

                if (
                    self.process.poll()
                    is not None
                ):
                    return

                continue

            lower = line.lower()

            detected = None

            if 'detached' in lower:
                detected = 'detached'

            elif 'attached' in lower:
                detected = 'attached'

            if detected is None:
                continue

            with self.condition:

                self.state = detected

                self.sequence += 1

                self.condition.notify_all()

    def snapshot(
        self,
    ):

        with self.condition:
            return (
                self.sequence,
                self.state,
            )

    def wait_after(
        self,
        sequence,
        expected,
        timeout,
    ):

        deadline = (
            time.monotonic()
            + timeout
        )

        with self.condition:

            while True:

                if (
                    self.sequence > sequence
                    and self.state == expected
                ):
                    return True

                remaining = (
                    deadline
                    - time.monotonic()
                )

                if remaining <= 0:
                    return False

                self.condition.wait(
                    timeout=remaining
                )

    def close(
        self,
    ):

        if (
            self.process.poll()
            is None
        ):

            self.process.terminate()

            try:

                self.process.wait(
                    timeout=0.5
                )

            except subprocess.TimeoutExpired:

                self.process.kill()

                self.process.wait(
                    timeout=0.5
                )


class SuctionManager(Node):

    def __init__(
        self,
    ):

        super().__init__(
            'suction_manager'
        )

        self.robot_entity = None
        self.robot_name = None

        self.monitors = {}

        self.installed_joints = set()

        self.service_handles = []

        # Advertise every exact physical-box interface once.
        for color in VALID_COLORS:

            for index in range(
                1,
                MAX_BOX_INDEX + 1,
            ):

                channel = channel_name(
                    color,
                    index,
                )

                for action in (
                    'attach',
                    'detach',
                ):

                    service_name = (
                        '/sorting/'
                        'suction_manager/'
                        f'{channel}/'
                        f'{action}'
                    )

                    callback = (
                        self._make_callback(
                            color,
                            index,
                            action,
                        )
                    )

                    service = (
                        self.create_service(
                            Trigger,
                            service_name,
                            callback,
                        )
                    )

                    self.service_handles.append(
                        service
                    )

        self.get_logger().info(
            '========================================'
        )

        self.prewarm_initial_monitors()

        self.get_logger().info(
            'FAST T10 SUCTION MANAGER READY'
        )

        self.get_logger().info(
            'ROS command interface: '
            'persistent services'
        )

        self.get_logger().info(
            'Gazebo state interface: '
            'persistent listeners'
        )

        self.get_logger().info(
            'Boxes remain FREE until pickup.'
        )

        self.get_logger().info(
            '========================================'
        )

    def _make_callback(
        self,
        color,
        index,
        action,
    ):

        def callback(
            request,
            response,
        ):

            del request

            return self.handle_request(
                response,
                color,
                index,
                action,
            )

        return callback

    def handle_request(
        self,
        response,
        color,
        index,
        action,
    ):

        name = model_name(
            color,
            index,
        )

        channel = channel_name(
            color,
            index,
        )

        started = time.monotonic()

        self.get_logger().info(
            f'{action.upper()} REQUEST: '
            f'{name} [{channel}]'
        )

        try:

            if action == 'attach':

                self.attach(
                    color,
                    index,
                )

                self.prewarm_next_monitor_async(
                    color,
                    index,
                )

            else:

                self.detach(
                    color,
                    index,
                )

        except Exception as error:

            elapsed = (
                time.monotonic()
                - started
            )

            response.success = False

            response.message = str(
                error
            )

            self.get_logger().error(
                f'{action.upper()} FAILED '
                f'[{channel}] after '
                f'{elapsed:.3f} s: '
                f'{error}'
            )

            return response

        elapsed = (
            time.monotonic()
            - started
        )

        response.success = True

        response.message = (
            f'{action} complete '
            f'in {elapsed:.3f} s'
        )

        self.get_logger().info(
            f'{action.upper()} COMPLETE '
            f'[{channel}] in '
            f'{elapsed:.3f} s.'
        )

        return response

    # ========================================================
    # ROBOT DISCOVERY
    # ========================================================

    @staticmethod
    def top_level_models(
        scene_text,
    ):

        blocks = []

        active = []
        depth = 0
        capturing = False

        for line in scene_text.splitlines():

            stripped = line.strip()

            if not capturing:

                if stripped == 'model {':

                    capturing = True
                    active = [line]

                    depth = (
                        line.count('{')
                        - line.count('}')
                    )

                continue

            active.append(
                line
            )

            depth += (
                line.count('{')
                - line.count('}')
            )

            if depth == 0:

                blocks.append(
                    '\n'.join(active)
                )

                active = []
                capturing = False

        return blocks

    def discover_robot(
        self,
    ):

        if (
            self.robot_entity
            is not None
        ):
            return

        result = subprocess.run(
            [
                'gz',
                'service',
                '-s',
                SCENE_SERVICE,
                '--reqtype',
                'gz.msgs.Empty',
                '--reptype',
                'gz.msgs.Scene',
                '--timeout',
                '5000',
                '--req',
                '',
            ],
            capture_output=True,
            text=True,
            timeout=6.0,
            check=False,
        )

        if result.returncode != 0:

            raise RuntimeError(
                'Gazebo scene query failed: '
                f'{result.stderr.strip()}'
            )

        matches = []

        for block in self.top_level_models(
            result.stdout
        ):

            if (
                'name: "wrist_3_link"'
                not in block
            ):
                continue

            id_match = re.search(
                r'^\s*id:\s*(\d+)',
                block,
                re.MULTILINE,
            )

            name_match = re.search(
                r'^\s*name:\s*"([^"]+)"',
                block,
                re.MULTILINE,
            )

            if (
                id_match is None
                or name_match is None
            ):
                continue

            matches.append(
                (
                    int(
                        id_match.group(1)
                    ),
                    name_match.group(1),
                )
            )

        if len(matches) != 1:

            raise RuntimeError(
                'Could not uniquely identify '
                'robot model. '
                f'Matches={matches}'
            )

        (
            self.robot_entity,
            self.robot_name,
        ) = matches[0]

        self.get_logger().info(
            'Gazebo robot cached: '
            f'{self.robot_name} '
            f'(entity={self.robot_entity})'
        )

    # ========================================================
    # PERSISTENT GAZEBO STATE
    # ========================================================

    def prewarm_initial_monitors(
        self,
    ):

        self.get_logger().info(
            'Prewarming initial Gazebo suction '
            'state listeners.'
        )

        for color in VALID_COLORS:

            monitor = self.get_monitor(
                color,
                1,
            )

            if monitor.process.poll() is not None:

                raise RuntimeError(
                    'Initial Gazebo state listener '
                    f'failed to stay alive for [{color}]'
                )

        self.get_logger().info(
            'Initial RED / GREEN / BLUE suction '
            'state listeners are ready.'
        )

    def prewarm_next_monitor_async(
        self,
        color,
        index,
    ):

        next_index = (
            int(index)
            + 1
        )

        if next_index > MAX_BOX_INDEX:
            return

        key = (
            color,
            next_index,
        )

        existing = self.monitors.get(
            key
        )

        if (
            existing is not None
            and existing.process.poll() is None
        ):
            return

        channel = channel_name(
            color,
            next_index,
        )

        def worker():

            try:

                monitor = self.get_monitor(
                    color,
                    next_index,
                )

                if monitor.process.poll() is not None:

                    raise RuntimeError(
                        'Gazebo listener exited '
                        'during prewarm.'
                    )

                self.get_logger().info(
                    'Prewarmed next suction '
                    'state listener: '
                    f'[{channel}]'
                )

            except Exception as error:

                self.get_logger().warning(
                    'Could not prewarm next suction '
                    f'listener [{channel}]: '
                    f'{error}'
                )

        threading.Thread(
            target=worker,
            daemon=True,
        ).start()

    def get_monitor(
        self,
        color,
        index,
    ):

        key = (
            color,
            index,
        )

        monitor = self.monitors.get(
            key
        )

        if monitor is not None:

            if monitor.process.poll() is None:
                return monitor

            self.get_logger().warning(
                f'Gazebo state monitor died for '
                f'{color} box {index}; restarting it.'
            )

            monitor.close()

            self.monitors.pop(
                key,
                None,
            )

        channel = channel_name(
            color,
            index,
        )

        topic = (
            f'/suction/'
            f'{channel}/state'
        )

        monitor = GazeboStateMonitor(
            topic
        )

        self.monitors[key] = (
            monitor
        )

        # Only once in the entire lifetime of this box.
        time.sleep(
            LISTENER_STARTUP
        )

        return monitor

    # ========================================================
    # GAZEBO COMMAND
    # ========================================================

    @staticmethod
    def publish_empty(
        topic,
    ):

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
            timeout=2.0,
            check=False,
        )

        if result.returncode != 0:

            raise RuntimeError(
                f'Gazebo command failed '
                f'on {topic}: '
                f'{result.stderr.strip()}'
            )

    # ========================================================
    # DYNAMIC JOINT
    # ========================================================

    def install_joint(
        self,
        color,
        index,
    ):

        key = (
            color,
            index,
        )

        if key in self.installed_joints:
            return

        self.discover_robot()

        name = model_name(
            color,
            index,
        )

        channel = channel_name(
            color,
            index,
        )

        attach_topic = (
            f'/suction/'
            f'{channel}/attach'
        )

        detach_topic = (
            f'/suction/'
            f'{channel}/detach'
        )

        state_topic = (
            f'/suction/'
            f'{channel}/state'
        )

        monitor = self.get_monitor(
            color,
            index,
        )

        sequence, _ = (
            monitor.snapshot()
        )

        innerxml = (
            f'<parent_link>'
            f'{PARENT_LINK}'
            f'</parent_link>'
            f'<child_model>'
            f'{name}'
            f'</child_model>'
            f'<child_link>'
            f'{CHILD_LINK}'
            f'</child_link>'
            f'<attach_topic>'
            f'{attach_topic}'
            f'</attach_topic>'
            f'<detach_topic>'
            f'{detach_topic}'
            f'</detach_topic>'
            f'<output_topic>'
            f'{state_topic}'
            f'</output_topic>'
            f'<suppress_child_warning>'
            f'false'
            f'</suppress_child_warning>'
        )

        request = (
            'entity: { '
            f'id: {self.robot_entity} '
            '} '
            'plugins: { '
            'name: '
            '"gz::sim::systems::DetachableJoint" '
            'filename: '
            '"gz-sim-detachable-joint-system" '
            f'innerxml: "{innerxml}" '
            '}'
        )

        started = (
            time.monotonic()
        )

        self.get_logger().info(
            'Creating exact joint NOW: '
            f'{name}'
        )

        result = subprocess.run(
            [
                'gz',
                'service',
                '-s',
                SYSTEM_ADD_SERVICE,
                '--reqtype',
                'gz.msgs.EntityPlugin_V',
                '--reptype',
                'gz.msgs.Boolean',
                '--timeout',
                '3000',
                '--req',
                request,
            ],
            capture_output=True,
            text=True,
            timeout=4.0,
            check=False,
        )

        output = (
            result.stdout
            + '\n'
            + result.stderr
        ).strip()

        normalized = (
            output
            .lower()
            .replace(' ', '')
        )

        if (
            result.returncode != 0
            or 'data:true'
            not in normalized
        ):

            raise RuntimeError(
                'Dynamic joint creation failed: '
                f'{output}'
            )

        if not monitor.wait_after(
            sequence,
            'attached',
            JOINT_CREATE_TIMEOUT,
        ):

            _, latest = (
                monitor.snapshot()
            )

            if latest != 'attached':

                self.get_logger().warning(
                    'Initial ATTACHED acknowledgement '
                    f'was missed for [{channel}]. '
                    'Running exact-box recovery.'
                )

                # The DetachableJoint may already be physically
                # attached even though its one-shot state message
                # was missed. Start a fresh listener before
                # forcing a deterministic transition.
                monitor.close()

                monitor = GazeboStateMonitor(
                    state_topic
                )

                self.monitors[key] = (
                    monitor
                )

                time.sleep(
                    0.10
                )

                recovery_sequence, _ = (
                    monitor.snapshot()
                )

                self.publish_empty(
                    detach_topic
                )

                detached = monitor.wait_after(
                    recovery_sequence,
                    'detached',
                    STATE_TIMEOUT,
                )

                if detached:

                    self.get_logger().info(
                        f'Recovery DETACH confirmed '
                        f'[{channel}].'
                    )

                    recovery_sequence, _ = (
                        monitor.snapshot()
                    )

                    self.publish_empty(
                        attach_topic
                    )

                    if not monitor.wait_after(
                        recovery_sequence,
                        'attached',
                        STATE_TIMEOUT,
                    ):

                        raise RuntimeError(
                            'Recovery ATTACH was not '
                            f'confirmed for [{channel}]'
                        )

                    self.get_logger().info(
                        f'Recovery ATTACH confirmed '
                        f'[{channel}].'
                    )

                else:

                    current_sequence, current_state = (
                        monitor.snapshot()
                    )

                    # The initial attachment may have happened
                    # while we were attempting the recovery
                    # detach. In that case the fresh listener
                    # has now seen it and no further transition
                    # is necessary.
                    if (
                        current_sequence
                        > recovery_sequence
                        and current_state == 'attached'
                    ):

                        self.get_logger().info(
                            'Fresh listener recovered '
                            f'ATTACHED state [{channel}].'
                        )

                    else:

                        recovery_sequence, _ = (
                            monitor.snapshot()
                        )

                        self.publish_empty(
                            attach_topic
                        )

                        if not monitor.wait_after(
                            recovery_sequence,
                            'attached',
                            STATE_TIMEOUT,
                        ):

                            raise RuntimeError(
                                'Dynamic joint exists, but '
                                'neither initial nor recovery '
                                'ATTACH was confirmed for '
                                f'[{channel}]'
                            )

                        self.get_logger().info(
                            f'Recovery ATTACH confirmed '
                            f'[{channel}].'
                        )

        self.installed_joints.add(
            key
        )

        elapsed = (
            time.monotonic()
            - started
        )

        self.get_logger().info(
            f'Joint creation + initial attach '
            f'completed in {elapsed:.3f} s.'
        )

    # ========================================================
    # ATTACH
    # ========================================================

    def attach(
        self,
        color,
        index,
    ):

        key = (
            color,
            index,
        )

        monitor = self.get_monitor(
            color,
            index,
        )

        # First pickup:
        # creating DetachableJoint performs the attachment.
        if key not in self.installed_joints:

            self.install_joint(
                color,
                index,
            )

            return

        _, current = (
            monitor.snapshot()
        )

        if current == 'attached':

            self.get_logger().info(
                'Box already reported attached.'
            )

            return

        channel = channel_name(
            color,
            index,
        )

        topic = (
            f'/suction/'
            f'{channel}/attach'
        )

        sequence, _ = (
            monitor.snapshot()
        )

        self.publish_empty(
            topic
        )

        if not monitor.wait_after(
            sequence,
            'attached',
            STATE_TIMEOUT,
        ):

            raise RuntimeError(
                'Gazebo did not confirm '
                f'ATTACH [{channel}]'
            )

    # ========================================================
    # DETACH
    # ========================================================

    def detach(
        self,
        color,
        index,
    ):

        key = (
            color,
            index,
        )

        if key not in self.installed_joints:

            raise RuntimeError(
                'Cannot detach: joint was '
                'never created for this box.'
            )

        monitor = self.get_monitor(
            color,
            index,
        )

        _, current = (
            monitor.snapshot()
        )

        if current == 'detached':

            self.get_logger().info(
                'Box already reported detached.'
            )

            return

        channel = channel_name(
            color,
            index,
        )

        topic = (
            f'/suction/'
            f'{channel}/detach'
        )

        sequence, _ = (
            monitor.snapshot()
        )

        started = (
            time.monotonic()
        )

        # First DETACH command.
        self.publish_empty(
            topic
        )

        # In the normal case Gazebo reports DETACHED almost
        # immediately after the gz command process returns.
        #
        # Do not spend the full STATE_TIMEOUT before retrying:
        # a transient Gazebo Transport command-delivery miss
        # should be recovered quickly.
        detached = monitor.wait_after(
            sequence,
            'detached',
            0.35,
        )

        if not detached:

            retry_sequence, retry_state = (
                monitor.snapshot()
            )

            # Guard against a state transition arriving exactly
            # around the short first wait boundary.
            if (
                retry_sequence > sequence
                and retry_state == 'detached'
            ):

                detached = True

            else:

                self.get_logger().warning(
                    'DETACH acknowledgement was not '
                    f'seen after the first command '
                    f'[{channel}]. Retrying once.'
                )

                # A second DETACH request is safe here. If the
                # first command was simply missed, this gives
                # Gazebo another delivery opportunity. If its
                # acknowledgement was merely delayed, the
                # persistent listener can still observe that
                # transition while this command is running.
                self.publish_empty(
                    topic
                )

                detached = monitor.wait_after(
                    retry_sequence,
                    'detached',
                    STATE_TIMEOUT,
                )

        if not detached:

            final_sequence, final_state = (
                monitor.snapshot()
            )

            raise RuntimeError(
                'Gazebo did not confirm '
                f'DETACH [{channel}] after one retry. '
                f'Last monitor state={final_state}, '
                f'sequence={final_sequence}'
            )

        elapsed = (
            time.monotonic()
            - started
        )

        self.get_logger().info(
            f'Gazebo DETACH acknowledgement '
            f'received in {elapsed:.3f} s.'
        )

    def close(
        self,
    ):

        for monitor in (
            self.monitors.values()
        ):

            monitor.close()

        self.monitors.clear()


def main():

    rclpy.init()

    node = SuctionManager()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:

        node.close()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':
    main()
