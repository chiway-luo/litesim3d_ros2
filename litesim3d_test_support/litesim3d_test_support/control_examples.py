"""Real controller_manager / DDS integration checks for the remaining example models."""
import json
import math
import subprocess
import time

from controller_manager_msgs.srv import ConfigureController, ListControllers, LoadController, SwitchController
from geometry_msgs.msg import Twist
from litesim3d_interfaces.msg import JointFeedback
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray
from std_srvs.srv import SetBool


def service(rig, kind, name, request):
    client = rig.node.create_client(kind, name)
    assert client.wait_for_service(timeout_sec=15), name
    future = client.call_async(request)
    rig.until(future.done, timeout=20)
    result = future.result()
    rig.node.destroy_client(client)
    return result


def controllers(rig):
    return service(rig, ListControllers, '/controller_manager/list_controllers', ListControllers.Request()).controller


def switch(rig, start, stop=()):
    request = SwitchController.Request()
    request.activate_controllers = list(start)
    request.deactivate_controllers = list(stop)
    request.strictness = SwitchController.Request.STRICT
    return service(rig, SwitchController, '/controller_manager/switch_controller', request).ok


def load(rig, name):
    assert service(rig, LoadController, '/controller_manager/load_controller', LoadController.Request(name=name)).ok
    assert service(rig, ConfigureController, '/controller_manager/configure_controller', ConfigureController.Request(name=name)).ok


def stamp(message):
    return message.sec + message.nanosec * 1e-9


def drive(rig, publisher, message, seconds):
    rig.until(lambda: publisher.get_subscription_count() == 1)
    begin = stamp(rig.messages['clock'][-1].clock)
    end = time.monotonic() + 45
    while stamp(rig.messages['clock'][-1].clock) < begin + seconds:
        assert time.monotonic() < end, 'simulation clock stopped'
        publisher.publish(message)
        rig.spin(.025)


def start(rig, controller, *, drone=False, **arguments):
    rig.subscribe('feedback', JointFeedback, '/sim/joint_feedback', 10)
    robot = 'quadrotor' if drone else 'wheel_car_4w'
    rig.subscribe('truth', Odometry, f'/sim/ground_truth/{robot}/odom', 10)
    rig.start(reliable_sensors=True, external_joint_states=True, joint_states=not drone, **arguments)
    deadline = time.monotonic() + 30
    while not any(c.name == controller and c.state == 'active' for c in controllers(rig)):
        assert time.monotonic() < deadline
        rig.spin(.1)
    rig.until(lambda: rig.messages['feedback'] and rig.messages['truth'] and rig.messages['imu'] and rig.messages['cloud'])
    assert rig.node.count_subscribers('/cmd_vel') == 1
    assert rig.node.count_publishers('/joint_states') == (0 if drone else 1)
    return rig.node.create_publisher(Twist, '/cmd_vel', 10)


def verify_drone(rig):
    publisher = start(rig, 'body_velocity_controller', drone=True)
    beginning = rig.messages['truth'][-1].pose.pose.position
    cmd = Twist(); cmd.linear.x = .2; cmd.linear.y = .15; cmd.linear.z = .3; cmd.angular.z = .2
    drive(rig, publisher, cmd, 1.5)
    rig.until(lambda: rig.messages['feedback'][-1].body_velocity.linear.z > .25)
    feedback = rig.messages['feedback'][-1]
    assert feedback.body_command_supported and not feedback.state.name
    assert feedback.body_pose.position.x - beginning.x > .1
    assert feedback.body_pose.position.y - beginning.y > .1
    assert feedback.body_pose.position.z - beginning.z > .2
    assert feedback.body_pose.orientation.z > .05
    # Stop command streams: controller expires in simulation time, body then decelerates.
    rig.until(lambda: abs(rig.messages['feedback'][-1].body_velocity.linear.z) < 1e-6, timeout=8)
    cmd.linear.z = float('nan')
    drive(rig, publisher, cmd, .3)
    assert abs(rig.messages['feedback'][-1].body_velocity.linear.z) < 1e-6
    # Real collision-accepted feedback, not the requested downward velocity.
    cmd = Twist(); cmd.linear.z = -1.5
    drive(rig, publisher, cmd, 2.0)
    rig.until(lambda: int(rig.status().get('contact_count', '0')) > 0)
    assert .23 < rig.messages['feedback'][-1].body_pose.position.z < .26
    assert rig.messages['feedback'][-1].body_velocity.linear.z == 0
    # Reactivation must not replay the last descent request.
    assert switch(rig, [], ['body_velocity_controller'])
    assert switch(rig, ['body_velocity_controller'])
    rig.spin(.4)
    assert rig.messages['feedback'][-1].body_velocity.linear.z == 0
    assert service(rig, SetBool, '/sim/pause', SetBool.Request(data=True)).success
    rig.spin(.15); t = rig.messages['feedback'][-1].state.header.stamp
    rig.spin(.3); assert rig.messages['feedback'][-1].state.header.stamp == t
    assert service(rig, SetBool, '/sim/pause', SetBool.Request(data=False)).success
    (rig.directory / 'control-evidence.json').write_text(json.dumps({
        'model': 'multicopter', 'actual_height': rig.messages['feedback'][-1].body_pose.position.z,
        'body_interfaces': [c.claimed_interfaces for c in controllers(rig) if c.name == 'body_velocity_controller'],
        'cmd_vel_subscribers': rig.node.count_subscribers('/cmd_vel'), 'joint_state_publishers': rig.node.count_publishers('/joint_states'),
        'xyz_yaw_motion': True, 'timeout_and_invalid_stop': True, 'floor_collision': True, 'pause_resume': True,
    }, indent=2))


def verify_wheel_car(rig, **arguments):
    publisher = start(rig, 'diff_drive_controller', **arguments)
    beginning = rig.messages['truth'][-1].pose.pose.position
    cmd = Twist(); cmd.linear.x = .3
    drive(rig, publisher, cmd, 1.0)
    rig.until(lambda: rig.messages['truth'][-1].pose.pose.position.x - beginning.x > .05)
    rig.until(lambda: abs(rig.messages['feedback'][-1].body_velocity.linear.x) < 1e-6, timeout=8)
    load(rig, 'wheel_velocity_controller')
    assert not switch(rig, ['wheel_velocity_controller'])
    assert switch(rig, ['wheel_velocity_controller'], ['diff_drive_controller'])
    direct = rig.node.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)
    drive(rig, direct, Float64MultiArray(data=[1.0, 1.0]), .5)
    def turning():
        state = rig.messages['feedback'][-1].state
        values = dict(zip(state.name, state.velocity))
        return all(abs(values.get(n, 0)-1) < 1e-5 for n in ['rear_left_wheel_joint', 'front_left_wheel_joint', 'rear_right_wheel_joint', 'front_right_wheel_joint'])
    rig.until(turning)
    assert switch(rig, ['diff_drive_controller'], ['wheel_velocity_controller'])
    rig.until(lambda: abs(rig.messages['feedback'][-1].body_velocity.linear.x) < 1e-6)
    (rig.directory / 'control-evidence.json').write_text(json.dumps({
        'model': 'wheel_car_4w', 'mimic_feedback': True, 'controller_conflict_rejected': True,
        'controller_handoff': True, 'timeout_stop': True,
        'joint_state_publishers': rig.node.count_publishers('/joint_states'),
    }, indent=2))


def verify_external_altitude(rig, parameter_file):
    publisher = start(rig, 'body_velocity_controller', drone=True, controllers=parameter_file)
    load(rig, 'altitude_controller')
    assert not switch(rig, ['altitude_controller'])
    assert switch(rig, ['altitude_controller'], ['body_velocity_controller'])
    before = rig.messages['feedback'][-1].body_pose.position.z
    altitude = rig.node.create_publisher(Float64, '/altitude_target', 10)
    drive(rig, altitude, Float64(data=before + .4), 4.0)
    after = rig.messages['feedback'][-1].body_pose.position.z
    assert abs(after - before - .4) < .03
    # The stopped velocity controller cannot override the third-party owner.
    cmd = Twist(); cmd.linear.z = -1.0
    drive(rig, publisher, cmd, .8)
    rig.until(lambda: abs(rig.messages['feedback'][-1].body_velocity.linear.z) < 1e-6)
    assert switch(rig, ['body_velocity_controller'], ['altitude_controller'])
    rig.spin(.3)
    assert abs(rig.messages['feedback'][-1].body_velocity.linear.z) < 1e-6
    (rig.directory / 'external-controller-evidence.json').write_text(json.dumps({
        'before_height': before, 'target_height': before + .4, 'after_height': after,
        'conflict_rejected': True, 'atomic_handoff': True, 'timeout_stop': True,
        'reactivation_without_replaying_input': True,
    }, indent=2))
