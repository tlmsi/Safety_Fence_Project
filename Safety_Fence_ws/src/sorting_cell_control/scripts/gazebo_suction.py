#!/usr/bin/env python3

import re
import select
import subprocess
import time
from typing import List, Optional


MAX_ATTEMPTS = 3

# Give the Gazebo transport subscriber enough time to be
# active before the command is published.
LISTENER_STARTUP_SECONDS = 0.50

# Wait for the normal acknowledgement after each command.
STATE_TIMEOUT_SECONDS = 3.00

# Before resending a command, allow a late state message to
# arrive on the SAME listener.
LATE_ACK_GRACE_SECONDS = 0.75

# At pickup_touch, allow Gazebo contact to settle before
# requesting attachment.
CONTACT_SETTLE_SECONDS = 0.25


STATE_PATTERN = re.compile(
    r'\b(attached|detached)\b',
    re.IGNORECASE,
)


def _stop_process(
    process: subprocess.Popen,
) -> None:

    if process.poll() is not None:
        return

    process.terminate()

    try:
        process.wait(
            timeout=0.5
        )

    except subprocess.TimeoutExpired:
        process.kill()

        process.wait(
            timeout=0.5
        )


def _state_from_line(
    line: str,
) -> Optional[str]:

    match = STATE_PATTERN.search(
        line
    )

    if match is None:
        return None

    return (
        match
        .group(1)
        .strip()
        .lower()
    )


def _drain_available_output(
    process: subprocess.Popen,
) -> List[str]:

    observed: List[str] = []

    streams = [
        stream
        for stream in (
            process.stdout,
            process.stderr,
        )
        if stream is not None
    ]

    while streams:

        readable, _, _ = select.select(
            streams,
            [],
            [],
            0.0,
        )

        if not readable:
            break

        for stream in readable:

            line = stream.readline()

            if not line:
                continue

            cleaned = line.strip()

            if cleaned:
                observed.append(
                    cleaned
                )

    return observed


def _wait_for_state(
    process: subprocess.Popen,
    expected_state: str,
    timeout: float,
) -> tuple[bool, List[str]]:

    deadline = (
        time.monotonic()
        + timeout
    )

    observed_lines: List[str] = []

    streams = [
        stream
        for stream in (
            process.stdout,
            process.stderr,
        )
        if stream is not None
    ]

    while (
        time.monotonic() < deadline
    ):

        remaining = max(
            0.0,
            deadline
            - time.monotonic(),
        )

        readable, _, _ = select.select(
            streams,
            [],
            [],
            min(
                0.20,
                remaining,
            ),
        )

        for stream in readable:

            line = stream.readline()

            if not line:
                continue

            cleaned = line.strip()

            if not cleaned:
                continue

            observed_lines.append(
                cleaned
            )

            # Only stdout contains actual Gazebo topic
            # messages. Never treat stderr diagnostics as
            # a state acknowledgement.
            if stream is process.stdout:

                state = _state_from_line(
                    cleaned
                )

                if state == expected_state:
                    return (
                        True,
                        observed_lines,
                    )

        if process.poll() is not None:

            for stream in streams:

                remainder = stream.read()

                if remainder:

                    observed_lines.extend(
                        line.strip()
                        for line
                        in remainder.splitlines()
                        if line.strip()
                    )

            break

    return (
        False,
        observed_lines,
    )


def _start_listener(
    state_topic: str,
) -> subprocess.Popen:

    return subprocess.Popen(
        [
            'stdbuf',
            '-oL',
            '-eL',
            'gz',
            'topic',
            '-e',
            '-t',
            state_topic,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )


def command_suction(
    node,
    colour: str,
    action: str,
    settle_seconds: float = 0.10,
) -> None:

    colour = (
        colour
        .strip()
        .lower()
    )

    action = (
        action
        .strip()
        .lower()
    )

    if colour not in (
        'red',
        'green',
        'blue',
    ):
        raise RuntimeError(
            f'Invalid suction colour: '
            f'{colour}'
        )

    if action not in (
        'attach',
        'detach',
    ):
        raise RuntimeError(
            f'Invalid suction action: '
            f'{action}'
        )

    expected_state = (
        'attached'
        if action == 'attach'
        else 'detached'
    )

    command_topic = (
        f'/suction/{colour}/'
        f'{action}'
    )

    state_topic = (
        f'/suction/{colour}/state'
    )

    last_error: Optional[str] = None

    all_observed: List[str] = []

    node.get_logger().info(
        f'Suction: {action.upper()}'
    )

    # --------------------------------------------------------
    # Start ONE listener and preserve it across retries.
    # --------------------------------------------------------

    listener = _start_listener(
        state_topic
    )

    try:

        time.sleep(
            LISTENER_STARTUP_SECONDS
        )

        # Remove any state output generated before this
        # particular suction command.
        all_observed.extend(
            _drain_available_output(
                listener
            )
        )

        # Allow physical contact to settle before ATTACH.
        if action == 'attach':

            node.get_logger().info(
                'Allowing suction contact to '
                f'settle for '
                f'{CONTACT_SETTLE_SECONDS:.2f} s.'
            )

            time.sleep(
                CONTACT_SETTLE_SECONDS
            )

        for attempt in range(
            1,
            MAX_ATTEMPTS + 1,
        ):

            # ----------------------------------------------
            # If this is a retry, first check whether the
            # acknowledgement merely arrived late.
            # Do NOT resend unnecessarily.
            # ----------------------------------------------

            if attempt > 1:

                confirmed, observed = (
                    _wait_for_state(
                        listener,
                        expected_state,
                        LATE_ACK_GRACE_SECONDS,
                    )
                )

                all_observed.extend(
                    observed
                )

                if confirmed:

                    node.get_logger().info(
                        'Gazebo suction state '
                        'confirmed late: '
                        f'{expected_state}. '
                        'No resend was required.'
                    )

                    time.sleep(
                        settle_seconds
                    )

                    return

            # ----------------------------------------------
            # If the listener somehow died, restart it
            # before issuing another command.
            # ----------------------------------------------

            if listener.poll() is not None:

                node.get_logger().warning(
                    'Gazebo suction-state listener '
                    'stopped unexpectedly; restarting.'
                )

                listener = _start_listener(
                    state_topic
                )

                time.sleep(
                    LISTENER_STARTUP_SECONDS
                )

                all_observed.extend(
                    _drain_available_output(
                        listener
                    )
                )

            # ----------------------------------------------
            # Send attach/detach command.
            # ----------------------------------------------

            command_error = None

            try:

                command = subprocess.run(
                    [
                        'gz',
                        'topic',
                        '-t',
                        command_topic,
                        '-m',
                        'gz.msgs.Empty',
                        '-p',
                        'unused: true',
                    ],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=3.0,
                    check=False,
                )

                if command.returncode != 0:

                    command_error = (
                        command.stderr.strip()
                        or (
                            'Gazebo topic command '
                            f'returned '
                            f'{command.returncode}'
                        )
                    )

            except subprocess.TimeoutExpired:

                command_error = (
                    'Gazebo suction command '
                    'timed out'
                )

            # Even if the CLI command reported a timeout,
            # the Gazebo message may already have been sent.
            # Always check state before retrying.
            confirmed, observed = (
                _wait_for_state(
                    listener,
                    expected_state,
                    STATE_TIMEOUT_SECONDS,
                )
            )

            all_observed.extend(
                observed
            )

            if confirmed:

                node.get_logger().info(
                    'Gazebo suction state '
                    'confirmed: '
                    f'{expected_state}.'
                )

                time.sleep(
                    settle_seconds
                )

                return

            if command_error:

                last_error = (
                    command_error
                )

                node.get_logger().warning(
                    f'Suction {action} command '
                    f'problem on attempt '
                    f'{attempt}/'
                    f'{MAX_ATTEMPTS}: '
                    f'{command_error}'
                )

            else:

                node.get_logger().warning(
                    f'No "{expected_state}" '
                    'acknowledgement on suction '
                    f'attempt {attempt}/'
                    f'{MAX_ATTEMPTS}; '
                    'keeping listener active.'
                )

        # ----------------------------------------------------
        # Final late-ack grace period.
        #
        # We do not fault immediately after attempt 3 if an
        # acknowledgement is already travelling through the
        # Gazebo transport.
        # ----------------------------------------------------

        confirmed, observed = (
            _wait_for_state(
                listener,
                expected_state,
                LATE_ACK_GRACE_SECONDS,
            )
        )

        all_observed.extend(
            observed
        )

        if confirmed:

            node.get_logger().info(
                'Gazebo suction state '
                'confirmed during final '
                f'grace period: '
                f'{expected_state}.'
            )

            time.sleep(
                settle_seconds
            )

            return

    finally:

        _stop_process(
            listener
        )

    details = ''

    if all_observed:

        details = (
            ' Last state output: '
            + ' | '.join(
                all_observed[-5:]
            )
        )

    elif last_error:

        details = (
            f' Last command error: '
            f'{last_error}'
        )

    raise RuntimeError(
        f'Gazebo did not confirm suction '
        f'{action} for {colour} after '
        f'{MAX_ATTEMPTS} attempts.'
        f'{details}'
    )
