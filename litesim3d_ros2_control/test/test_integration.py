"""Real Humble controller_manager, pluginlib hardware, controllers and simulator."""
import os
import json
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from controller_manager_msgs.srv import ListHardwareComponents, SwitchController
from geometry_msgs.msg import Twist
from litesim3d_interfaces.msg import JointFeedback
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
import yaml


def executable(package, name):
    return str(Path(get_package_prefix(package)) / 'lib' / package / name)


def test_controller_manager_joint_modes_and_feedback(tmp_path):
    os.environ['ROS_DOMAIN_ID'] = str(90 + os.getpid() % 10)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = str(tmp_path / 'logs')
    context = Context()
    rclpy.init(context=context)
    node = Node('control_contract', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    processes, files, messages, subscriptions = [], [], {}, []

    def spawn(name, args):
        output = (tmp_path / (name + '.log')).open('w')
        files.append(output)
        process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    def subscribe(key, cls, topic):
        subscriptions.append(node.create_subscription(cls, topic, lambda msg: messages.__setitem__(key, msg), 10))

    def wait(predicate, seconds=15):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)
            if predicate():
                return
        logs = '\n'.join(p.read_text()[-5000:] for p in tmp_path.glob('*.log'))
        raise AssertionError('condition timed out\n' + logs)

    def drive(publisher, message, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            publisher.publish(message)
            executor.spin_once(timeout_sec=.02)

    def switch(start, stop, expect=True):
        client = node.create_client(SwitchController, '/test_manager/switch_controller')
        assert client.wait_for_service(timeout_sec=10)
        request = SwitchController.Request()
        request.activate_controllers, request.deactivate_controllers = start, stop
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 3
        future = client.call_async(request)
        wait(future.done)
        assert future.result().ok == expect
        node.destroy_client(client)

    try:
        # The simulator and hardware use remapped private topics, and a non-default manager name.
        wheels = ''
        controls = ''
        for name, y in [('left_wheel_joint', .25), ('right_wheel_joint', -.25)]:
            wheels += f'''<link name="{name}_link"><collision><origin rpy="1.57079632679 0 0"/>
              <geometry><cylinder radius="0.1" length="0.04"/></geometry></collision></link>
              <joint name="{name}" type="continuous"><parent link="base_link"/><child link="{name}_link"/>
              <origin xyz="0 {y} 0.1"/><axis xyz="0 1 0"/><limit effort="1" velocity="10"/></joint>'''
            controls += f'''<joint name="{name}"><command_interface name="position"/><command_interface name="velocity"/>
              <state_interface name="position"/><state_interface name="velocity"/></joint>'''
        model = f'''<robot name="test"><link name="base_link"><collision><origin xyz="0 0 .25"/>
          <geometry><box size=".4 .3 .2"/></geometry></collision></link>{wheels}
          <litesim3d version="1">
            <chassis name="drive" class="litesim3d_diff_drive/DifferentialDrive">
              <parameters file="package://litesim3d_diff_drive/config/parameters.yaml"/></chassis>
            <odometry name="odom" class="litesim3d_diff_odometry/DifferentialOdometry">
              <parameters file="package://litesim3d_diff_odometry/config/parameters.yaml"/></odometry>
          </litesim3d>
          <ros2_control name="TestSystem" type="system"><hardware>
            <plugin>litesim3d_ros2_control/LiteSimSystem</plugin>
            <param name="command_topic">/renamed/command</param><param name="state_topic">/renamed/feedback</param>
            <param name="feedback_timeout">0.7</param>
          </hardware>{controls}</ros2_control></robot>'''
        robot = tmp_path / 'robot.urdf'
        robot.write_text(model)
        config = tmp_path / 'runtime.yaml'
        config.write_text(yaml.safe_dump({
            'schema_version': 1, 'units': 'm_rad_s',
            'simulation': {'control_mode': 'ros2_control', 'command_timeout': .2},
            'robot': {'urdf': str(robot), 'base_link': 'base_link', 'id': 'test', 'sensor_geometry': 'collision'},
            'terrain': {'enabled': False}, 'entities': [],
        }))
        params = {'/test_manager': {'ros__parameters': {'robot_description': model, 'use_sim_time': True,
            'update_rate': 100, 'state': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
            'drive': {'type': 'diff_drive_controller/DiffDriveController'},
            'velocity': {'type': 'forward_command_controller/ForwardCommandController'},
            'position': {'type': 'forward_command_controller/ForwardCommandController'}}},
            '/drive': {'ros__parameters': {'left_wheel_names': ['left_wheel_joint'],
                'right_wheel_names': ['right_wheel_joint'], 'wheel_radius': .1, 'wheel_separation': .5,
                'use_stamped_vel': False, 'cmd_vel_timeout': .2, 'enable_odom_tf': False,
                'position_feedback': True, 'open_loop': False}},
            '/velocity': {'ros__parameters': {'joints': ['left_wheel_joint', 'right_wheel_joint'], 'interface_name': 'velocity'}},
            '/position': {'ros__parameters': {'joints': ['left_wheel_joint', 'right_wheel_joint'], 'interface_name': 'position'}}}
        manager_params = tmp_path / 'controllers.yaml'
        manager_params.write_text(yaml.safe_dump(params))
        subscribe('feedback', JointFeedback, '/renamed/feedback')
        subscribe('joints', JointState, '/renamed/joints')
        subscribe('truth', Odometry, '/sim/ground_truth/test/odom')
        sim = spawn('sim', [executable('litesim3d_ros2', 'sim_node'), '--ros-args', '-p', 'config:=' + str(config),
            '-p', 'publish_odom_tf:=false', '-r', 'sim/joint_commands:=/renamed/command', '-r', 'sim/joint_feedback:=/renamed/feedback'])
        wait(lambda: 'feedback' in messages)
        manager = spawn('manager', [executable('litesim3d_ros2_control', 'ros2_control_node'), '--ros-args',
            '-r', 'controller_manager:__node:=test_manager', '-r', 'joint_states:=/renamed/joints', '--params-file', str(manager_params)])
        for name in ['state', 'drive', 'velocity', 'position']:
            args = [executable('controller_manager', 'spawner'), name, '-c', '/test_manager']
            if name in ['velocity', 'position']:
                args.append('--inactive')
            process = spawn('spawn_' + name, args)
            wait(lambda: process.poll() is not None, 30)
            assert process.returncode == 0
        wait(lambda: 'joints' in messages and 'truth' in messages)
        assert node.count_publishers('/renamed/joints') == 1
        assert node.count_publishers('/joint_states') == 0
        assert node.count_subscribers('/cmd_vel') == 0  # no simulator Twist input in this mode
        cmd = node.create_publisher(Twist, '/drive/cmd_vel_unstamped', 10)
        twist = Twist(); twist.linear.x = .2
        initial = messages['truth'].pose.pose.position.x
        drive(cmd, twist, .7)
        wait(lambda: messages['truth'].pose.pose.position.x > initial + .07)
        wait(lambda: all(abs(v) < 1e-4 for v in messages['feedback'].state.velocity), 5)  # controller timeout
        # A normal pause still sends feedback; it must not trigger a wall timeout.
        pause = node.create_client(SetBool, '/sim/pause')
        assert pause.wait_for_service(timeout_sec=5)
        future = pause.call_async(SetBool.Request(data=True)); wait(future.done)
        assert future.result().success
        time.sleep(1)
        future = pause.call_async(SetBool.Request(data=False)); wait(future.done)
        assert future.result().success
        drive(cmd, twist, .3)
        switch(['velocity'], ['drive'])
        wait(lambda: all(abs(v) < 1e-4 for v in messages['feedback'].state.velocity))
        velocity = node.create_publisher(Float64MultiArray, '/velocity/commands', 10)
        drive(velocity, Float64MultiArray(data=[2., 2.]), .4)
        wait(lambda: all(abs(v-2) < .01 for v in messages['feedback'].state.velocity))
        # Position and velocity interfaces of the same joints cannot be active together.
        switch(['position'], [], expect=False)
        switch(['position'], ['velocity'])
        target = max(messages['feedback'].state.position) + .3
        position = node.create_publisher(Float64MultiArray, '/position/commands', 10)
        drive(position, Float64MultiArray(data=[target, target]), .4)
        wait(lambda: all(abs(p-target) < 1e-3 for p in messages['feedback'].state.position))
        wait(lambda: all(abs(p-target) < 1e-3 for p in messages['joints'].position))
        switch(['velocity'], ['position'])
        drive(velocity, Float64MultiArray(data=[1., 1.]), .4)
        wait(lambda: all(abs(v-1) < .01 for v in messages['feedback'].state.velocity))
        # Loss of manager must not leave the simulator moving indefinitely.
        manager.send_signal(signal.SIGSTOP)
        wait(lambda: all(abs(v) < 1e-4 for v in messages['feedback'].state.velocity), 5)
        manager.send_signal(signal.SIGCONT)
        # Loss of feedback faults hardware and blocks stale commands on clock recovery.
        sim.send_signal(signal.SIGSTOP)
        client = node.create_client(ListHardwareComponents, '/test_manager/list_hardware_components')
        time.sleep(1)
        # With use_sim_time the manager loop can sleep while /clock is stopped.
        # The transport's wall watchdog must latch the fault, even if fresh data
        # arrives before the next read on resumed simulation time.
        sim.send_signal(signal.SIGCONT)
        def hardware_stopped():
            future = client.call_async(ListHardwareComponents.Request()); wait(future.done)
            hardware = [c for c in future.result().component if c.name == 'TestSystem']
            return len(hardware) == 1 and hardware[0].state.label == 'unconfigured'
        wait(hardware_stopped, 5)
        wait(lambda: all(abs(v) < 1e-4 for v in messages['feedback'].state.velocity), 5)
    finally:
        forced = []
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGCONT)
                process.send_signal(signal.SIGINT)
        for process in reversed(processes):
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                forced.append(process.pid)
                process.kill(); process.wait()
        for output in files:
            output.close()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        exits = [{'executable': p.args[0], 'pid': p.pid, 'exit_code': p.returncode} for p in processes]
        (tmp_path / 'process-exits.json').write_text(json.dumps({'processes': exits, 'forced_kills': forced}, indent=2))
        assert not forced, exits
        assert all(p.returncode == 0 for p in processes), exits
