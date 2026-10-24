"""Exercise startup ordering and failure propagation over real DDS."""
import json
import os
from pathlib import Path
import subprocess
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


@pytest.fixture
def startup(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '11')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = Context()
    rclpy.init(context=context)
    node = Node('startup_regression', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    status_pub = node.create_publisher(DiagnosticArray, '/sim/status', 10)
    clock_pub = node.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    prefix = tmp_path / 'overlay'
    marker = prefix / 'share/ament_index/resource_index/packages/controller_manager'
    marker.parent.mkdir(parents=True)
    marker.touch()
    binaries = prefix / 'lib/controller_manager'
    binaries.mkdir(parents=True)
    captured = tmp_path / 'spawned.json'
    processes = []

    def launch(timeout, exit_code=0, target='spawner'):
        binary = binaries / target
        binary.write_text('#!/usr/bin/env python3\nimport json,sys\nfrom pathlib import Path\n'
                          f'Path({str(captured)!r}).write_text(json.dumps(sys.argv[1:]))\n'
                          f'raise SystemExit({exit_code})\n')
        binary.chmod(0o755)
        env = dict(os.environ)
        env['AMENT_PREFIX_PATH'] = str(prefix) + os.pathsep + env.get('AMENT_PREFIX_PATH', '')
        executable = Path(get_package_prefix('litesim3d_ros2_control')) / 'lib/litesim3d_ros2_control/spawn_when_ready.py'
        forwarded = (['body_velocity_controller', '-c', '/controller_manager'] if target == 'spawner'
                     else ['--ros-args', '-p', 'use_sim_time:=true'])
        process = subprocess.Popen([str(executable), '--timeout', str(timeout), '--executable', target,
                                    '--', *forwarded],
                                   env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        processes.append(process)
        return process

    def publish(seconds, *, ready, advancing=False, fault=False):
        end, tick = time.monotonic() + seconds, 1
        while time.monotonic() < end:
            status_pub.publish(DiagnosticArray(status=[DiagnosticStatus(name='litesim3d', values=[
                KeyValue(key='ready', value=str(ready).lower()),
                KeyValue(key='state', value='fault' if fault else 'running')])]))
            message = Clock()
            message.clock.nanosec = tick
            clock_pub.publish(message)
            tick += 1 if advancing else 0
            executor.spin_once(timeout_sec=.02)
    try:
        yield launch, publish, captured
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
            process.communicate(timeout=5)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


@pytest.mark.parametrize('exit_code', [0, 7])
@pytest.mark.parametrize('target', ['spawner', 'ros2_control_node'])
def test_spawner_waits_for_ready_and_advancing_clock_and_preserves_exit(startup, exit_code, target):
    launch, publish, captured = startup
    process = launch(5, exit_code, target)
    publish(.4, ready=False, advancing=True)
    assert process.poll() is None and not captured.exists()
    publish(.4, ready=True)
    assert process.poll() is None and not captured.exists()
    publish(.5, ready=True, advancing=True)
    assert process.wait(timeout=3) == exit_code
    expected = (['body_velocity_controller', '-c', '/controller_manager'] if target == 'spawner'
                else ['--ros-args', '-p', 'use_sim_time:=true'])
    assert json.loads(captured.read_text()) == expected


def test_spawner_fails_with_bounded_deadline_when_clock_stays_frozen(startup):
    launch, publish, captured = startup
    process = launch(.7)
    publish(1., ready=True)
    _, error = process.communicate(timeout=3)
    assert process.returncode == 1 and 'advancing clock not observed' in error
    assert not captured.exists()


def test_spawner_rejects_simulator_fault_before_starting_controller(startup):
    launch, publish, captured = startup
    process = launch(5)
    publish(.6, ready=False, fault=True)
    _, error = process.communicate(timeout=3)
    assert process.returncode == 1 and 'simulator faulted' in error
    assert not captured.exists()
