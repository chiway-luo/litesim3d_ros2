"""Collision wheel encoders, true sensor state and reset over real ROS topics."""
import json

import pytest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from rclpy.time import Time

from test_ros import h
from test_m2_ros import reset, start_session


def configure(h, enabled):
    h.config['simulation']['command_timeout'] = 60.0
    h.config['robot']['drive'].update(continue_on_collision=enabled, unlimited_acceleration=True)
    h.config['entities'] = [{'id': 'wall', 'pose': {'xyz': [.8, 0, 1]},
                             'geometry': {'type': 'box', 'size': [.1, 10, 3]}}]
    h.config['imus'] = [{'name': 'imu', 'link': 'base_link', 'topic': 'imu/data_raw', 'rate_hz': 200}]
    h.subscribe('odom', Odometry, '/odom')
    h.subscribe('truth', Odometry, '/sim/ground_truth/robot_0/odom')
    h.subscribe('joints', JointState, '/joint_states')
    h.subscribe('imu', Imu, '/imu/data_raw')


def drive_into_wall(h):
    publisher = h.node.create_publisher(Twist, '/cmd_vel', 1)
    h.until(lambda: publisher.get_subscription_count() == 1)
    command = Twist(); command.linear.x = .8
    h.publish_paused_command(publisher, command)
    result = h.step(400, 'drive-into-wall')
    assert result.success
    h.wait_for_stamp(2_000_000_001, 'odom', 'truth', 'joints', 'imu')
    return publisher


@pytest.mark.parametrize('enabled', [False, True])
def test_collision_encoder_truth_imu_tf_and_reverse(h, enabled):
    configure(h, enabled)
    h.rsp(); h.sim(start_paused=True, sensor_reliable=True); h.ready()
    publisher = drive_into_wall(h)
    odom = h.messages['odom'][-1]
    truth = h.messages['truth'][-1]
    joints = h.messages['joints'][-1]
    imu = h.messages['imu'][-1]
    expected = 1.6 if enabled else truth.pose.pose.position.x
    assert truth.pose.pose.position.x == pytest.approx(.35, abs=2e-5)
    assert truth.twist.twist.linear.x == 0
    assert odom.pose.pose.position.x == pytest.approx(expected, abs=1e-8)
    assert odom.twist.twist.linear.x == pytest.approx(.8 if enabled else 0)
    for name in ['left_wheel_joint', 'right_wheel_joint']:
        i = joints.name.index(name)
        assert joints.position[i] * .1 == pytest.approx(expected, abs=1e-8)
        assert joints.velocity[i] == pytest.approx(8 if enabled else 0)
    assert imu.angular_velocity.z == 0
    assert imu.linear_acceleration.x == 0
    assert imu.linear_acceleration.z == pytest.approx(9.81)
    stamp = Time.from_msg(odom.header.stamp)
    h.until(lambda: h.buffer.can_transform('odom', 'base_link', stamp))
    transform = h.buffer.lookup_transform('odom', 'base_link', stamp)
    assert transform.transform.translation.x == pytest.approx(expected, abs=1e-8)
    command = Twist(); command.linear.x = -.5
    h.publish_paused_command(publisher, command)
    assert h.step(100, 'reverse-from-wall').success
    h.wait_for_stamp(2_500_000_001, 'odom', 'truth')
    assert h.messages['truth'][-1].pose.pose.position.x == pytest.approx(.1, abs=2e-5)
    assert h.messages['odom'][-1].pose.pose.position.x == pytest.approx(expected - .25, abs=1e-8)
    (h.directory / 'collision-result.json').write_text(json.dumps({
        'continue_on_collision': enabled, 'blocked_truth_x': truth.pose.pose.position.x,
        'wheel_odom_x': odom.pose.pose.position.x,
        'reverse_truth_x': h.messages['truth'][-1].pose.pose.position.x,
    }, indent=2))


def test_collision_encoder_drift_cleared_by_session_reset(h):
    configure(h, True)
    process, _ = start_session(h)
    publisher = drive_into_wall(h)
    # Session reset requires control producers to be stopped. This observer is
    # allowed to read measurements, but must not publish into the new epoch.
    h.node.destroy_publisher(publisher)
    h.until(lambda: h.node.count_publishers('/cmd_vel') == 0)
    old = h.status()['run_id']
    response = reset(h, old, 'reset-collision-drift')
    assert response.success and response.run_id != old
    assert h.step(20, 'stationary-new-session').success
    h.wait_for_stamp(100_000_001, 'odom', 'truth', 'joints')
    assert h.messages['odom'][-1].pose.pose.position.x == 0
    assert h.messages['truth'][-1].pose.pose.position.x == 0
    assert all(p == 0 for p in h.messages['joints'][-1].position)
    assert process.poll() is None
