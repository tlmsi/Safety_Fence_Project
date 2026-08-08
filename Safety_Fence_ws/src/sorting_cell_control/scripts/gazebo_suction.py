#!/usr/bin/env python3

import time

import rclpy

from std_srvs.srv import Trigger


SERVICE_WAIT_SECONDS = 2.0
SERVICE_CALL_SECONDS = 5.0

_CLIENTS = {}


def _channel(
    colour: str,
    box_index: int,
) -> str:

    if box_index == 1:
        return colour

    return (
        f'{colour}_'
        f'{box_index:02d}'
    )


def _get_client(
    node,
    service_name: str,
):

    key = (
        id(node),
        service_name,
    )

    client = _CLIENTS.get(
        key
    )

    if client is None:

        client = node.create_client(
            Trigger,
            service_name,
        )

        _CLIENTS[key] = client

    return client


def command_suction(
    node,
    colour: str,
    action: str,
    settle_seconds: float = 0.10,
    box_index: int = 1,
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

    box_index = int(
        box_index
    )

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

    if not (
        1 <= box_index <= 16
    ):
        raise RuntimeError(
            f'Invalid physical box index: '
            f'{box_index}'
        )

    channel = _channel(
        colour,
        box_index,
    )

    service_name = (
        f'/sorting/suction_manager/'
        f'{channel}/{action}'
    )

    client = _get_client(
        node,
        service_name,
    )

    if not client.wait_for_service(
        timeout_sec=SERVICE_WAIT_SECONDS,
    ):
        raise RuntimeError(
            'T10 suction service unavailable: '
            f'{service_name}'
        )

    node.get_logger().info(
        f'Suction request -> T10: '
        f'{action.upper()} '
        f'[{channel}]'
    )

    started = time.monotonic()

    future = client.call_async(
        Trigger.Request()
    )

    rclpy.spin_until_future_complete(
        node,
        future,
        timeout_sec=SERVICE_CALL_SECONDS,
    )

    if not future.done():

        raise RuntimeError(
            'Timed out waiting for T10: '
            f'{action} [{channel}]'
        )

    response = future.result()

    if response is None:

        raise RuntimeError(
            'T10 returned no response for '
            f'{action} [{channel}]'
        )

    elapsed = (
        time.monotonic()
        - started
    )

    if not response.success:

        raise RuntimeError(
            'T10 suction failure for '
            f'[{channel}]: '
            f'{response.message}'
        )

    node.get_logger().info(
        f'T10 confirmed '
        f'{action.upper()} '
        f'[{channel}] in '
        f'{elapsed:.3f} s.'
    )

    if settle_seconds > 0:

        time.sleep(
            settle_seconds
        )
