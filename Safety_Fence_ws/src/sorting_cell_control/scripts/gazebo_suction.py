#!/usr/bin/env python3

import select
import subprocess
import time
from typing import List, Optional


MAX_ATTEMPTS = 3
LISTENER_STARTUP_SECONDS = 0.30
STATE_TIMEOUT_SECONDS = 2.50


def _stop_process(
    process: subprocess.Popen,
) -> None:
    if process.poll() is not None:
        return

    process.terminate()

    try:
        process.wait(timeout=0.5)

    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=0.5)


def _wait_for_state(
    process: subprocess.Popen,
    expected_state: str,
    timeout: float,
) -> tuple[bool, List[str]]:
    deadline = time.monotonic() + timeout
    observed_lines: List[str] = []

    streams = [
        stream
        for stream in (
            process.stdout,
            process.stderr,
        )
        if stream is not None
    ]

    while time.monotonic() < deadline:
        remaining = max(
            0.0,
            deadline - time.monotonic(),
        )

        readable, _, _ = select.select(
            streams,
            [],
            [],
            min(0.20, remaining),
        )

        for stream in readable:
            line = stream.readline()

            if not line:
                continue

            cleaned = line.strip()

            if not cleaned:
                continue

            observed_lines.append(cleaned)

            lowered = cleaned.lower()

            if expected_state in lowered:
                return True, observed_lines

        if process.poll() is not None:
            for stream in streams:
                remainder = stream.read()

                if remainder:
                    observed_lines.extend(
                        line.strip()
                        for line in remainder.splitlines()
                        if line.strip()
                    )

            break

    return False, observed_lines


def command_suction(
    node,
    colour: str,
    action: str,
    settle_seconds: float = 0.10,
) -> None:
    colour = colour.strip().lower()
    action = action.strip().lower()

    if colour not in (
        'red',
        'green',
        'blue',
    ):
        raise RuntimeError(
            f'Invalid suction colour: {colour}'
        )

    if action not in (
        'attach',
        'detach',
    ):
        raise RuntimeError(
            f'Invalid suction action: {action}'
        )

    expected_state = (
        'attached'
        if action == 'attach'
        else 'detached'
    )

    command_topic = (
        f'/suction/{colour}/{action}'
    )

    state_topic = (
        f'/suction/{colour}/state'
    )

    last_error: Optional[str] = None
    last_observed: List[str] = []

    node.get_logger().info(
        f'Suction: {action.upper()}'
    )

    for attempt in range(
        1,
        MAX_ATTEMPTS + 1,
    ):
        listener = subprocess.Popen(
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

        try:
            time.sleep(
                LISTENER_STARTUP_SECONDS
            )

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

            except subprocess.TimeoutExpired:
                last_error = (
                    f'command timed out on attempt '
                    f'{attempt}/{MAX_ATTEMPTS}'
                )

                node.get_logger().warning(
                    last_error
                )

                continue

            if command.returncode != 0:
                last_error = (
                    command.stderr.strip()
                    or (
                        'Gazebo topic command returned '
                        f'{command.returncode}'
                    )
                )

                node.get_logger().warning(
                    f'Suction {action} command failed '
                    f'on attempt {attempt}/'
                    f'{MAX_ATTEMPTS}: {last_error}'
                )

                continue

            confirmed, observed = (
                _wait_for_state(
                    listener,
                    expected_state,
                    STATE_TIMEOUT_SECONDS,
                )
            )

            last_observed = observed

            if confirmed:
                node.get_logger().info(
                    'Gazebo suction state confirmed: '
                    f'{expected_state}.'
                )

                time.sleep(settle_seconds)
                return

            node.get_logger().warning(
                f'No "{expected_state}" acknowledgement '
                f'on suction attempt {attempt}/'
                f'{MAX_ATTEMPTS}; retrying.'
            )

        finally:
            _stop_process(listener)

    details = ''

    if last_observed:
        details = (
            ' Last state output: '
            + ' | '.join(last_observed[-5:])
        )

    elif last_error:
        details = (
            f' Last command error: {last_error}'
        )

    raise RuntimeError(
        f'Gazebo did not confirm suction {action} '
        f'for {colour} after {MAX_ATTEMPTS} attempts.'
        f'{details}'
    )
