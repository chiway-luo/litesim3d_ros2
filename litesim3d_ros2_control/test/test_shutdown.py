"""Check real manager exit codes with moving, frozen and wall clocks."""
import json
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from controller_manager_msgs.srv import ListHardwareComponents
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM])
@pytest.mark.parametrize('clock_mode', ['advancing', 'frozen', 'wall'])
def test_manager_shutdown(tmp_path, monkeypatch, stop_signal, clock_mode):
    monkeypatch.setenv('ROS_DOMAIN_ID', '12')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    context = Context()
    rclpy.init(context=context)
    # A unique graph prevents another test's manager/clock from satisfying the
    # readiness check when normal and instrumented suites share a ROS domain.
    namespace = '/shutdown_' + uuid.uuid4().hex
    clock_topic = namespace + '/clock'
    node = Node('shutdown_regression', namespace=namespace, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    publisher = node.create_publisher(Clock, clock_topic, qos_profile_sensor_data)
    client = node.create_client(ListHardwareComponents, namespace + '/controller_manager/list_hardware_components')
    model = '''<robot name="shutdown"><link name="base"/>
      <ros2_control name="TestSystem" type="system">
        <hardware><plugin>mock_components/GenericSystem</plugin></hardware>
        <joint name="wheel"><command_interface name="velocity"/>
          <state_interface name="position"/><state_interface name="velocity"/></joint>
      </ros2_control></robot>'''
    executable = Path(get_package_prefix('litesim3d_ros2_control')) / 'lib/litesim3d_ros2_control/ros2_control_node'
    log = tmp_path / 'manager.log'
    process = None
    clock_stop = threading.Event()
    published, received, clock_errors = [], [], []
    node.create_subscription(Clock, clock_topic,
        lambda msg: received.append(msg.clock.sec * 1_000_000_000 + msg.clock.nanosec),
        qos_profile_sensor_data)

    def publish_clock():
        tick = 1
        try:
            while not clock_stop.is_set():
                stamp = tick * 10_000_000
                message = Clock()
                message.clock.sec, message.clock.nanosec = divmod(stamp, 1_000_000_000)
                publisher.publish(message)
                published.append(stamp)
                tick += 1
                clock_stop.wait(.01)
        except Exception as error:
            clock_errors.append(error)

    clock_thread = threading.Thread(target=publish_clock)
    clock_thread.start()

    def spin():
        executor.spin_once(timeout_sec=.01)
        assert not clock_errors, clock_errors

    try:
        with log.open('w') as output:
            process = subprocess.Popen([
                str(executable), '--ros-args', '-p', 'robot_description:=' + model,
                '-r', '__ns:=' + namespace, '-r', '/clock:=' + clock_topic,
                '-p', 'use_sim_time:=' + str(clock_mode != 'wall').lower(),
                '-p', 'update_rate:=100'], stdout=output, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 10
            while not client.service_is_ready() and time.monotonic() < deadline:
                spin()
                assert process.poll() is None, log.read_text()
            assert client.service_is_ready(), log.read_text()
            future = client.call_async(ListHardwareComponents.Request())
            while not future.done() and time.monotonic() < deadline:
                spin()
            assert future.done(), log.read_text()
            assert future.result().component[0].state.label == 'active', log.read_text()
            for _ in range(30):
                spin()
            assert len(received) >= 2 and received[-1] > received[0]
            if clock_mode == 'frozen':
                # Leave the manager blocked in its simulation-clock sleep.
                clock_stop.set()
                clock_thread.join(timeout=2)
                assert not clock_thread.is_alive()
                time.sleep(.1)
            before_signal = published[-1]
            process.send_signal(stop_signal)
            # Snapshot after send_signal returns: a tick racing with the call
            # cannot masquerade as the required later, post-signal sample.
            at_signal = published[-1]
            deadline = time.monotonic() + 5
            # The independent publisher stays active across signal delivery and
            # process exit. Also receive a post-signal tick even on a fast exit.
            while process.poll() is None or (clock_mode == 'advancing' and received[-1] <= at_signal):
                assert time.monotonic() < deadline, log.read_text()
                spin()
            if clock_mode == 'advancing':
                assert published[-1] > at_signal and received[-1] > at_signal
            elif clock_mode == 'frozen':
                assert published[-1] == at_signal
            (tmp_path / 'clock.json').write_text(json.dumps({
                'mode': clock_mode, 'signal': stop_signal.name,
                'first_received_ns': received[0], 'before_signal_ns': before_signal,
                'at_signal_ns': at_signal,
                'last_published_ns': published[-1], 'last_received_ns': received[-1]}, indent=2))
            assert process.returncode == 0, log.read_text()
            assert 'namespace collision' not in log.read_text(), log.read_text()
            assert 'Shutting down the controller manager' in log.read_text()
    finally:
        clock_stop.set()
        clock_thread.join(timeout=2)
        if process is not None and process.poll() is None:
            process.kill()
            process.wait()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        assert not clock_thread.is_alive() and not clock_errors, clock_errors
