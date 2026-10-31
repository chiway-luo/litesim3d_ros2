"""Check real manager exit codes with moving, frozen and wall clocks."""
import os
from pathlib import Path
import signal
import subprocess
import time

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
    node = Node('shutdown_regression', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    publisher = node.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    client = node.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
    model = '''<robot name="shutdown"><link name="base"/>
      <ros2_control name="TestSystem" type="system">
        <hardware><plugin>mock_components/GenericSystem</plugin></hardware>
        <joint name="wheel"><command_interface name="velocity"/>
          <state_interface name="position"/><state_interface name="velocity"/></joint>
      </ros2_control></robot>'''
    executable = Path(get_package_prefix('litesim3d_ros2_control')) / 'lib/litesim3d_ros2_control/ros2_control_node'
    log = tmp_path / 'manager.log'
    process = None
    tick = 1

    def spin():
        nonlocal tick
        message = Clock()
        message.clock.sec, message.clock.nanosec = divmod(tick * 10_000_000, 1_000_000_000)
        publisher.publish(message)
        tick += 1
        executor.spin_once(timeout_sec=.01)

    try:
        with log.open('w') as output:
            process = subprocess.Popen([
                str(executable), '--ros-args', '-p', 'robot_description:=' + model,
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
            if clock_mode == 'frozen':
                # Leave the manager blocked in its simulation-clock sleep.
                time.sleep(.1)
            process.send_signal(stop_signal)
            assert process.wait(timeout=5) == 0, log.read_text()
            assert 'Shutting down the controller manager' in log.read_text()
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
