"""Real ROS publications from runtime-loaded chassis and encoder odometry plugins."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.time import Time
from sensor_msgs.msg import JointState

from test_ros import h
from test_collision_odometry_ros import configure, drive_into_wall
from test_m2_ros import reset, start_session


def add_plugins(robot, drive_package, drive_class, odom_package, odom_class, wheels=None):
    block = ET.SubElement(robot, 'litesim3d', version='1')
    drive = ET.SubElement(block, 'chassis', name='drive', **{'class': f'{drive_package}/{drive_class}'})
    ET.SubElement(drive, 'parameters', file=f'package://{drive_package}/config/parameters.yaml')
    ET.SubElement(drive, 'overrides').text = 'limits: {unlimited_acceleration: true}'
    odom = ET.SubElement(block, 'odometry', name='encoders', **{'class': f'{odom_package}/{odom_class}'})
    ET.SubElement(odom, 'parameters', file=f'package://{odom_package}/config/parameters.yaml')
    if wheels is not None:
        ET.SubElement(odom, 'overrides').text = yaml.safe_dump({'wheels': wheels})


def use_plugin_model(h, root):
    description = h.directory / 'composed.urdf'
    ET.ElementTree(root).write(description, encoding='unicode')
    keep_driving = h.config['robot']['drive']['continue_on_collision']
    h.config['robot'].pop('drive')
    h.config['robot'].pop('motion_model', None)
    h.config.pop('odom', None)
    h.config['robot'].update(urdf=str(description), continue_on_collision=keep_driving)


def omni4_robot(wheels):
    """The test owns the URDF geometry; plugin parameters only bind joints."""
    robot = ET.Element('robot', name='omni4')
    body = ET.SubElement(robot, 'link', name='base_link')
    for role in ['visual', 'collision']:
        shape = ET.SubElement(body, role); ET.SubElement(shape, 'origin', xyz='0 0 .25')
        ET.SubElement(ET.SubElement(shape, 'geometry'), 'box', size='.4 .3 .2')
    # The ordinary omni-wheel layout matches the documented four-wheel example.
    # Radius, position and rolling direction must be resolved from this URDF,
    # not read back from the joint-only parameters.yaml bindings.
    layout = [(.3, .3, 3 * math.pi / 4), (-.3, .3, 5 * math.pi / 4),
              (-.3, -.3, 7 * math.pi / 4), (.3, -.3, 9 * math.pi / 4)]
    radius = .1
    for wheel, (x, y, angle) in zip(wheels, layout, strict=True):
        name = wheel['joint']
        link = ET.SubElement(robot, 'link', name=name + '_link')
        for role in ['visual', 'collision']:
            shape = ET.SubElement(link, role)
            ET.SubElement(shape, 'origin', rpy=f'{-math.pi / 2} 0 {angle}')
            ET.SubElement(ET.SubElement(shape, 'geometry'), 'cylinder', radius=str(radius), length='.04')
        joint = ET.SubElement(robot, 'joint', name=name, type='continuous')
        ET.SubElement(joint, 'parent', link='base_link'); ET.SubElement(joint, 'child', link=name + '_link')
        ET.SubElement(joint, 'origin', xyz=f'{x} {y} {radius}')
        ET.SubElement(joint, 'axis', xyz=f'{-math.sin(angle)} {math.cos(angle)} 0')
        ET.SubElement(joint, 'limit', effort='10', velocity='100')
    add_plugins(robot, 'litesim3d_omni4_drive', 'Omni4Drive', 'litesim3d_omni_odometry', 'OmniOdometry', wheels)
    return robot


@pytest.mark.parametrize('enabled', [False, True])
def test_plugin_diff_ros_collision_encoder_and_tf(h, enabled):
    configure(h, enabled)
    robot = ET.fromstring(Path(h.config['robot']['urdf']).read_text())
    add_plugins(robot, 'litesim3d_diff_drive', 'DifferentialDrive', 'litesim3d_diff_odometry', 'DifferentialOdometry')
    use_plugin_model(h, robot)
    h.rsp(); h.sim(start_paused=True, sensor_reliable=True); h.ready()
    drive_into_wall(h)
    truth, odom = h.messages['truth'][-1], h.messages['odom'][-1]
    expected = 1.6 if enabled else truth.pose.pose.position.x
    assert truth.pose.pose.position.x == pytest.approx(.35, abs=2e-5)
    assert truth.twist.twist.linear.x == 0
    assert odom.pose.pose.position.x == pytest.approx(expected, abs=1e-8)
    assert odom.twist.twist.linear.x == pytest.approx(.8 if enabled else 0)
    timestamp = Time.from_msg(odom.header.stamp)
    h.until(lambda: h.buffer.can_transform('odom', 'base_link', timestamp))
    transform = h.buffer.lookup_transform('odom', 'base_link', timestamp)
    assert transform.transform.translation.x == pytest.approx(expected, abs=1e-8)
    assert h.messages['imu'][-1].angular_velocity.z == 0


@pytest.mark.parametrize('enabled', [False, True])
def test_plugin_omni_ros_lateral_odometry_is_independent_of_truth(h, enabled):
    configure(h, enabled)
    p = yaml.safe_load((Path(get_package_share_directory('litesim3d_omni4_drive')) / 'config/parameters.yaml').read_text())
    robot = omni4_robot(p['wheels'])
    use_plugin_model(h, robot)
    h.config['lidars'] = []
    h.config['entities'] = [{'id': 'wall', 'pose': {'xyz': [0, 1, 1]}, 'geometry': {'type': 'box', 'size': [10, .1, 2]}}]
    h.rsp(); h.sim(start_paused=True, sensor_reliable=True); h.ready()
    publisher = h.node.create_publisher(Twist, '/cmd_vel', 1)
    h.until(lambda: publisher.get_subscription_count() == 1)
    command = Twist(); command.linear.y = .8
    h.publish_paused_command(publisher, command)
    assert h.step(400, 'lateral-collision').success
    h.wait_for_stamp(2_000_000_001, 'odom', 'truth', 'joints')
    truth, odom = h.messages['truth'][-1], h.messages['odom'][-1]
    assert .5 < truth.pose.pose.position.y < .8
    assert truth.twist.twist.linear.y == 0
    assert odom.twist.twist.linear.y == pytest.approx(.8 if enabled else 0, abs=1e-10)
    expected = 1.6 if enabled else truth.pose.pose.position.y
    assert odom.pose.pose.position.y == pytest.approx(expected, abs=1e-8)
    assert odom.pose.pose.position.x == pytest.approx(0, abs=1e-8)
    timestamp = Time.from_msg(odom.header.stamp)
    h.until(lambda: h.buffer.can_transform('odom', 'base_link', timestamp))
    assert h.buffer.lookup_transform('odom', 'base_link', timestamp).transform.translation.y == pytest.approx(expected, abs=1e-8)


def test_plugin_encoder_drift_resets_with_session(h):
    configure(h, True)
    robot = ET.fromstring(Path(h.config['robot']['urdf']).read_text())
    add_plugins(robot, 'litesim3d_diff_drive', 'DifferentialDrive', 'litesim3d_diff_odometry', 'DifferentialOdometry')
    use_plugin_model(h, robot)
    process, _ = start_session(h)
    publisher = drive_into_wall(h)
    h.node.destroy_publisher(publisher)
    h.until(lambda: h.node.count_publishers('/cmd_vel') == 0)
    old = h.status()['run_id']; response = reset(h, old, 'reset-plugins')
    assert response.success and response.run_id != old
    assert h.step(20, 'plugin-new-session').success
    h.wait_for_stamp(100_000_001, 'odom', 'truth', 'joints')
    assert h.messages['odom'][-1].pose.pose.position.x == 0
    assert h.messages['truth'][-1].pose.pose.position.x == 0
    assert all(value == 0 for value in h.messages['joints'][-1].position)
    assert process.poll() is None
