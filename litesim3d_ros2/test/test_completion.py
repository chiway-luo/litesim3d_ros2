"""Document audit regressions: real multi-robot interfaces and launch failures."""
import json
import os
from pathlib import Path
import re
import signal

import psutil
import pytest
import yaml
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster
from test_ros import h, ns
from test_m3_ros import start_world


def statuses(h):
    if not h.messages['status']:
        return {}
    return {s.name.split('/')[0]: {v.key: v.value for v in s.values}
            for s in h.messages['status'][-1].status}


def test_AUD01_stamped_commands_reject_wrong_robot_and_stale_input(h):
    process = start_world(h, 2, cmd_mode='stamped', command_timeout=.1)
    publisher = h.node.create_publisher(TwistStamped, '/r0/cmd_vel', 1)
    h.until(lambda: publisher.get_subscription_count() == 1)
    graph = dict(h.node.get_topic_names_and_types())
    assert graph['/r0/cmd_vel'] == ['geometry_msgs/msg/TwistStamped']
    command = TwistStamped(); command.header.frame_id = 'r0/base_link'
    command.header.stamp.nanosec = 1; command.twist.linear.x = .4
    publisher.publish(command); h.spin(.1)
    assert h.step(20, 'valid').success
    h.until(lambda: bool(h.messages['r0-odom']))
    assert h.messages['r0-odom'][-1].pose.pose.position.x == pytest.approx(.04)
    invalid = [('r1/base_link', 100_000_001, .4), ('', 100_000_001, .4),
               ('r0/base_link', 0, .4), ('r0/base_link', 200_000_001, .4),
               ('r0/base_link', 1_000_000_000, .4), ('r0/base_link', 100_000_001, float('nan'))]
    for i, (frame, stamp, velocity) in enumerate(invalid):
        command.header.frame_id = frame; command.header.stamp.nanosec = stamp
        command.twist.linear.x = velocity; publisher.publish(command)
        h.until(lambda: int(statuses(h)['r0']['rejected_commands']) == i+1)
    assert h.step(40, 'timeout').success
    h.until(lambda: statuses(h)['r0']['command_timeout'] == 'true')
    assert h.messages['r0-odom'][-1].twist.twist.linear.x == 0
    assert statuses(h)['r1']['rejected_commands'] == '0'
    assert h.messages['r1-odom'][-1].pose.pose.position.x == 0
    assert h.messages['status'][-1].status[0].level in (1, b'\x01')
    process.send_signal(signal.SIGINT); h.until(lambda: process.poll() is not None)
    assert process.returncode == 0
    manifest = json.loads((h.directory/'world-manifest.json').read_text())
    assert manifest['ros_distro'] == 'humble' and manifest['rmw']
    assert len(manifest['bridge_source_sha256']) == 64
    assert manifest['ros_interface']['cmd_mode'] == 'stamped'
    assert all(r['exit_reason'] == 'shutdown' for r in manifest['robots'])


@pytest.mark.parametrize('reliable', [False, True])
def test_AUD02_world_qos_matrix_and_per_robot_diagnostics(h, reliable):
    start_world(h, 2, sensor_reliable=reliable)
    h.subscribe('reliable', PointCloud2, '/r0/points_raw', reliable=True)
    if not reliable:
        h.until(lambda: int(statuses(h)['r0'].get('qos_conflicts', '0')) >= 1)
        assert statuses(h)['r1']['qos_conflicts'] == '0'
    else:
        h.until(lambda: h.node.count_publishers('/r0/points_raw') == 1)
        h.spin(.2)
    assert h.step(20, 'qos').success
    h.until(lambda: bool(h.messages['r0-cloud']))
    if reliable:
        h.until(lambda: bool(h.messages['reliable']))
        assert statuses(h)['r0']['qos_conflicts'] == '0'
    else:
        h.spin(.2); assert not h.messages['reliable']
        assert h.messages['status'][-1].status[0].level in (1, b'\x01')
    h.until(lambda: 'sensor_queue_depth' in statuses(h)['r0'])
    assert statuses(h)['r0']['sensor_queue_depth'] == '5'
    assert statuses(h)['r0']['sensor_queue_policy'] == ('keep_last/reliable' if reliable else 'keep_last/best_effort')


def test_AUD03_external_tf_handover_is_per_robot_and_not_truth_input(h):
    start_world(h, 2, external_odom_tf_robots=['r1'])
    assert h.step(20, 'handover').success
    h.until(lambda: bool(h.messages['r0-cloud']))
    tf = TransformBroadcaster(h.node)
    transform = TransformStamped(); transform.header.frame_id = 'r1/odom'
    transform.child_frame_id = 'r1/base_link'; transform.header.stamp.nanosec = 100_000_001
    transform.transform.rotation.w = 1.; transform.transform.translation.x = 7.
    for _ in range(5):
        tf.sendTransform(transform); h.spin(.05)
    edges = [t for m in h.messages['tf'] for t in m.transforms]
    own = [t for t in edges if t.child_frame_id == 'r0/base_link']
    external = [t for t in edges if t.child_frame_id == 'r1/base_link']
    assert own and external
    assert all(t.transform.translation.x == 7. for t in external)
    assert h.messages['r1-odom'][-1].pose.pose.position.x == 0.
    # Changing the estimator TF must not move the peer's physical surface.
    import struct
    x = struct.unpack_from('<f', bytes(h.messages['r0-cloud'][0].data))[0]
    assert x == pytest.approx(1.4, abs=1e-5)
    assert statuses(h)['r1']['publish_odom_tf'] == 'false'
    assert statuses(h)['r0']['publish_odom_tf'] == 'true'


@pytest.mark.parametrize('parent', ['r0/odom', 'map'])
def test_AUD04_duplicate_or_wrong_parent_tf_faults_world(h, parent):
    process = start_world(h, 2)
    assert h.step(20, 'own-tf').success
    h.spin(.2)
    tf = TransformBroadcaster(h.node)
    transform = TransformStamped(); transform.header.frame_id = parent
    transform.child_frame_id = 'r0/base_link'; transform.transform.rotation.w = 1.
    transform.header.stamp.nanosec = 100_000_001
    for _ in range(5):
        tf.sendTransform(transform); h.spin(.05)
    h.until(lambda: process.poll() is not None)
    assert process.returncode != 0
    manifest = json.loads((h.directory/'world-manifest.json').read_text())
    assert manifest['exit_reason'] == 'fault' and 'TF' in manifest['error']
    assert all(r['exit_reason'] == 'fault' for r in manifest['robots'])


@pytest.mark.parametrize('settings, message', [
    ({'cmd_mode': 'invalid'}, 'cmd_mode'),
    ({'startup_timeout': 0.0}, 'startup_timeout'),
    ({'external_odom_tf_robots': ['missing']}, 'unknown'),
    ({'external_odom_tf_robots': ['r1', 'r1']}, 'duplicate'),
])
def test_AUD05_world_rejects_invalid_interface_parameters(h, settings, message):
    process = start_world(h, 2, ready=False, **settings)
    h.until(lambda: process.poll() is not None)
    assert process.returncode != 0
    assert message in (h.directory/'world.log').read_text()


def test_AUD06_world_readiness_timeout_is_bounded_and_recorded(h):
    process = start_world(h, 2, ready=False, startup_timeout=.2,
                          robot_state_publisher_name='missing_rsp')
    h.until(lambda: process.poll() is not None)
    assert process.returncode != 0
    manifest = json.loads((h.directory/'world-manifest.json').read_text())
    assert 'readiness timeout' in manifest['error']


def launch(h, multi, extra=(), invalid=False, file_uri=False):
    h.config['entities'] = []
    if invalid:
        h.config['unknown_field'] = True
    if file_uri:
        h.config['robot']['urdf'] = 'file://' + h.config['robot']['urdf']
    config = h.directory/'base.yaml'; config.write_text(yaml.safe_dump(h.config))
    if multi:
        world = h.directory/'world.yaml'
        world.write_text(yaml.safe_dump({'schema_version': 1, 'config': str(config),
            'robots': [{'id':'r0','xyz':[0,0,0]}, {'id':'r1','xyz':[2,0,0]}]}))
        config = world
    filename = 'world.launch.py' if multi else 'demo.launch.py'
    command = ['ros2', 'launch', 'litesim3d_ros2', filename,
               'config:='+str(config), 'paused:=true', *extra]
    if not multi:
        command.append('rviz:=false')
    return h.process('launch', command)


@pytest.mark.parametrize('multi', [False, True])
@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGKILL])
@pytest.mark.parametrize('target_name', ['sim_', 'robot_state_publisher'])
def test_AUD07_required_child_exit_propagates_failure_and_cleans_up(h, multi, sig, target_name):
    process = launch(h, multi); h.ready()
    log = h.directory/'launch.log'
    def children():
        return [(name, int(pid)) for name,pid in re.findall(
            r'\[([^\]]+)\]: process started with pid \[(\d+)\]', log.read_text())]
    h.until(lambda: len(children()) == (3 if multi else 2))
    pids = children()
    target = next(pid for name,pid in pids if name.startswith(target_name))
    os.kill(target, sig)
    h.until(lambda: process.poll() is not None, timeout=12)
    assert process.returncode != 0, log.read_text()
    h.until(lambda: not any(psutil.pid_exists(pid) for _,pid in pids))
    (h.directory/'launch-exits.json').write_text(json.dumps({'code':process.returncode,'children':pids}))


@pytest.mark.parametrize('multi', [False, True])
def test_AUD08_launch_rejects_bad_config_and_boolean(h, multi):
    process = launch(h, multi, invalid=True)
    h.until(lambda: process.poll() is not None, timeout=12)
    assert process.returncode != 0
    assert 'unknown_field' in (h.directory/'launch.log').read_text()
    # A separate process must fail before starting nodes on an invalid launch boolean.
    del h.config['unknown_field']
    process = launch(h, multi, extra=['paused:=maybe'])
    h.until(lambda: process.poll() is not None)
    assert process.returncode != 0
    assert 'true or false' in (h.directory/'launch.log').read_text()


@pytest.mark.parametrize('multi', [False, True])
def test_AUD09_installed_launch_supports_file_uri_and_normal_exit(h, multi):
    process = launch(h, multi, file_uri=True); h.ready()
    assert h.step(20, 'installed-uri').success
    process.send_signal(signal.SIGINT)
    h.until(lambda: process.poll() is not None)
    assert process.returncode == 0


def test_AUD10_installed_world_launch_passes_interface_options(h):
    process = launch(h, True, extra=['cmd_mode:=stamped', 'sensor_reliable:=true',
                                   'external_odom_tf_robots:=r1', 'startup_timeout:=3.0'])
    h.ready()
    h.until(lambda: statuses(h)['r0'].get('cmd_mode') == 'stamped')
    assert statuses(h)['r0']['sensor_queue_policy'] == 'keep_last/reliable'
    assert statuses(h)['r0']['publish_odom_tf'] == 'true'
    assert statuses(h)['r1']['publish_odom_tf'] == 'false'
    assert dict(h.node.get_topic_names_and_types())['/r0/cmd_vel'] == ['geometry_msgs/msg/TwistStamped']
    process.send_signal(signal.SIGINT); h.until(lambda: process.poll() is not None)
    assert process.returncode == 0
