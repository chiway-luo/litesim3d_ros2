#!/usr/bin/env python3
"""Start a stock control process after simulator readiness and advancing ROS time."""
import argparse
import math
import os
from pathlib import Path
import sys
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def positive_timeout(value):
    number = float(value)
    if not math.isfinite(number) or not 0 < number <= 3600:
        raise argparse.ArgumentTypeError('timeout must be finite and in (0, 3600] seconds')
    return number


def wait_for_simulation(timeout, status_topic, clock_topic):
    context = Context()
    rclpy.init(args=[], context=context)
    node = Node(f'litesim_control_startup_{os.getpid()}', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    ready, advancing, initial, fault = False, False, None, False

    def status(message):
        nonlocal ready, advancing, initial, fault
        values = {entry.key: entry.value for item in message.status for entry in item.values}
        fault = values.get('state') == 'fault'
        current = values.get('ready') == 'true'
        if current != ready:
            initial, advancing = None, False
        ready = current

    def clock(message):
        nonlocal initial, advancing
        if not ready:
            return
        stamp = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        if initial is None or stamp < initial:
            initial, advancing = stamp, False
        elif stamp > initial:
            advancing = True

    node.create_subscription(DiagnosticArray, status_topic, status, 10)
    node.create_subscription(Clock, clock_topic, clock, qos_profile_sensor_data)
    deadline = time.monotonic() + timeout
    try:
        while context.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            if fault:
                raise RuntimeError('simulator faulted before controller startup')
            if ready and advancing:
                return
        raise RuntimeError(f'simulator readiness and advancing clock not observed within {timeout}s')
    finally:
        executor.shutdown()
        node.destroy_node()
        if context.ok():
            context.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=positive_timeout, default=60.)
    parser.add_argument('--status-topic', default='/sim/status')
    parser.add_argument('--clock-topic', default='/clock')
    parser.add_argument('--executable', choices=('spawner', 'ros2_control_node'), default='spawner')
    parser.add_argument('spawner_args', nargs=argparse.REMAINDER)
    # The explicit -- boundary preserves every ROS parameter/remap for the
    # target process, while our short-lived readiness node has private arguments.
    args = parser.parse_args(sys.argv[1:])
    forwarded = args.spawner_args[1:] if args.spawner_args[:1] == ['--'] else args.spawner_args
    if not forwarded:
        parser.error('provide stock controller_manager spawner arguments after --')
    try:
        wait_for_simulation(args.timeout, args.status_topic, args.clock_topic)
        spawner = Path(get_package_prefix('controller_manager')) / 'lib/controller_manager' / args.executable
        # Preserve the stock loader, its arguments, signals and exact exit status.
        os.execv(str(spawner), [str(spawner), *forwarded])
    except Exception as error:
        print(f'controller startup: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
