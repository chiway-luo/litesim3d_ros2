"""Official Livox wire contracts against the same deterministic native scan."""
import copy
import hashlib
import json
import math
from pathlib import Path
import signal
import struct

import pytest
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
try:
    get_package_share_directory('litesim3d_mid360')
except PackageNotFoundError:
    pytest.skip("optional litesim3d_mid360 plugin is not installed", allow_module_level=True)
from livox_ros_driver2.msg import CustomMsg
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from test_ros import h, ns
from test_m2_ros import m2_config
from test_m3_ros import start_world


def mid360(h, rate=10):
    m2_config(h)
    lidar = h.config['lidars'][0]
    lidar.update(scan_pattern='livox_mid360',
                 pattern_file='package://litesim3d_core/lidar_patterns/mid360.csv',
                 columns=200000 // rate, rings=1, vertical_angles_deg=[0.0],
                 rate_hz=rate, min_range_m=.1, max_range_m=40.,
                 range_sigma_m=0., range_bias_m=0., dropout=0., publish_range_boundary=True)


def matched(h, topics):
    h.until(lambda: all(json.loads(h.status().get('subscription_matches', '{}')).get(t, 0) >= 1
                        for t in topics))


def test_official_message_sources_are_unmodified():
    share = Path(get_package_share_directory('livox_ros_driver2'))
    upstream = json.loads((share / 'UPSTREAM.json').read_text())
    for name, digest in upstream['sha256'].items():
        assert hashlib.sha256((share / name).read_bytes()).hexdigest() == digest
    assert list(CustomMsg.get_fields_and_field_types()) == ['header', 'timebase', 'point_num', 'lidar_id', 'rsvd', 'points']


@pytest.mark.parametrize('format,rate,namespace', [(1,10,''), (0,20,'fleet')])
def test_livox_matches_native_geometry_and_leaves_imu_unchanged(h, format, rate, namespace):
    mid360(h, rate)
    prefix = namespace + '/' if namespace else ''
    topic = '/' + prefix
    h.subscribe('native', PointCloud2, topic + 'points_raw')
    h.subscribe('native_boundary', PointCloud2, topic + 'points_raw/range_boundary')
    # Exact single-step comparisons need reliable delivery at both ends.
    # A matched best-effort reader can still lose the final IMU sample.
    h.subscribe('imu', Imu, topic + 'imu/data_raw')
    h.rsp(namespace=namespace, frame_prefix=prefix)
    process = h.sim(namespace=namespace, frame_prefix=prefix, start_paused=True, sensor_reliable=True)
    h.ready()
    matched(h, [topic+'points_raw', topic+'points_raw/range_boundary', topic+'imu/data_raw'])
    step = h.step(200 // rate, 'native-frame'); assert step.success
    h.until(lambda: h.messages['native'] and h.messages['native_boundary'] and h.messages['imu']
            and ns(h.messages['imu'][-1].header.stamp) == step.time_ns)
    native = h.messages['native'][-1]
    rows = list(struct.iter_unpack('<ffffH2xf', bytes(native.data)))
    valid = [(slot, p) for slot,p in enumerate(rows) if math.isfinite(p[0])]
    assert 100 < len(valid) < len(rows)
    native_imu = h.messages['imu'][-1]
    process.send_signal(signal.SIGINT); h.wait(process)
    h.until(lambda: h.node.count_publishers(topic+'points_raw') == 0)
    h.spin(.2); h.messages['status'].clear(); h.messages['imu'].clear()
    main = topic+'livox/lidar'
    pc_topic = main+'/pointcloud' if format else main
    h.subscribe('pc', PointCloud2, pc_topic)
    h.subscribe('boundary', PointCloud2, main+'/range_boundary')
    if format:
        h.subscribe('custom', CustomMsg, main)
    h.sim(namespace=namespace, frame_prefix=prefix, start_paused=True, sensor_reliable=True,
          lidar_interface='auto', xfer_format=format, livox_lidar_id=7)
    h.ready()
    matched(h, [main, pc_topic, main+'/range_boundary', topic+'imu/data_raw'])
    step = h.step(200 // rate, 'official-frame'); assert step.success
    h.until(lambda: h.messages['pc'] and h.messages['boundary'] and h.messages['imu']
            and ns(h.messages['imu'][-1].header.stamp) == step.time_ns
            and (not format or h.messages['custom']))
    pc = h.messages['pc'][-1]
    assert pc.header == native.header
    assert (pc.height, pc.width, pc.point_step, pc.row_step, pc.is_bigendian) == (1,len(valid),26,len(valid)*26,False)
    assert [(f.name,f.offset,f.datatype,f.count) for f in pc.fields] == [
        ('x',0,7,1),('y',4,7,1),('z',8,7,1),('intensity',12,7,1),
        ('tag',16,2,1),('line',17,2,1),('timestamp',18,8,1)]
    for (slot, p), actual in zip(valid, struct.iter_unpack('<ffffBBd', bytes(pc.data))):
        assert actual[:3] == p[:3]
        assert actual[3:] == (math.floor(p[3]*255+.5), 0, slot % 4, float(ns(native.header.stamp)+slot*5000))
    if format:
        msg = h.messages['custom'][-1]
        assert msg.header == pc.header and msg.timebase == ns(pc.header.stamp)
        assert msg.point_num == len(msg.points) == len(valid) and msg.lidar_id == 7
        assert list(msg.rsvd) == [0,0,0]
        for (slot,p), actual in zip(valid, msg.points):
            assert (actual.x,actual.y,actual.z) == p[:3]
            assert (actual.offset_time,actual.line,actual.tag,actual.reflectivity) == (
                slot*5000, slot%4, 0, math.floor(p[3]*255+.5))
    boundary = list(struct.iter_unpack('<ffffBBd', bytes(h.messages['boundary'][-1].data)))
    native_boundary = list(struct.iter_unpack('<ffffH2xf', bytes(h.messages['native_boundary'][-1].data)))
    assert len(boundary) == sum(math.isfinite(p[0]) for p in native_boundary) > 100
    assert all(math.sqrt(sum(x*x for x in p[:3])) == pytest.approx(40., abs=1e-5) for p in boundary)
    assert not {p[-1] for p in boundary} & {ns(native.header.stamp)+s*5000 for s,_ in valid}
    imu = h.messages['imu'][-1]
    assert imu == native_imu  # Entire message: frame, stamp, SI units and covariances.
    assert imu.linear_acceleration.z == pytest.approx(9.80665)
    assert h.node.count_publishers(topic+'livox/imu') == 0
    assert h.node.count_publishers(topic+'points_raw') == 0
    assert h.node.get_publishers_info_by_topic(main)[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
    assert h.node.get_publishers_info_by_topic(topic+'imu/data_raw')[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE


@pytest.mark.parametrize('format', [0, 1])
def test_livox_defaults_keep_best_effort_imu(h, format):
    mid360(h)
    h.subscribe('imu', Imu, '/imu/data_raw', reliable=False)
    # Exercise the default QoS while streaming: best effort does not promise
    # delivery of a particular sample from a single paused step.
    h.sim(wait_for_robot_state_publisher=False, lidar_interface='auto', xfer_format=format)
    h.ready()
    h.until(lambda: h.messages['imu'] and h.node.get_publishers_info_by_topic('/livox/lidar')
            and h.node.get_publishers_info_by_topic('/imu/data_raw'))
    assert h.node.get_publishers_info_by_topic('/livox/lidar')[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
    assert h.node.get_publishers_info_by_topic('/imu/data_raw')[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
    assert h.messages['imu'][-1].linear_acceleration.z == pytest.approx(9.80665)
    assert h.node.count_publishers('/livox/imu') == 0
    assert h.node.count_publishers('/points_raw') == 0


def test_mid360_can_coexist_with_unchanged_spinning_lidar(h):
    other = copy.deepcopy(h.config['lidars'][0])
    other.update(name='other', topic='other_points')
    mid360(h)
    h.config['lidars'].append(other)
    h.subscribe('livox', CustomMsg, '/livox/lidar')
    h.subscribe('other', PointCloud2, '/other_points', reliable=False)
    h.sim(start_paused=True, wait_for_robot_state_publisher=False, lidar_interface='auto')
    h.ready(); matched(h, ['/livox/lidar','/other_points'])
    assert h.step(20).success
    h.until(lambda: h.messages['livox'] and h.messages['other'])
    other_msg = h.messages['other'][-1]
    assert (other_msg.height,other_msg.width,other_msg.point_step) == (4,32,24)
    assert [f.name for f in other_msg.fields] == ['x','y','z','intensity','ring','time']
    assert h.node.get_publishers_info_by_topic('/other_points')[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT


def test_world_mid360_topics_frames_and_imu_stay_per_robot(h):
    for robot in ('r0','r1'):
        h.subscribe(robot+'-livox', CustomMsg, f'/{robot}/livox/lidar')
    start_world(h, 2, ready=False, mid360=True, lidar_interface='auto')
    h.ready()
    for robot in ('r0','r1'):
        h.until(lambda robot=robot: json.loads(h.status(robot).get('subscription_matches','{}')).get(f'/{robot}/livox/lidar',0) >= 1)
    assert h.step(20).success
    h.until(lambda: all(h.messages[r+'-livox'] and h.messages[r+'-imu'] for r in ('r0','r1')))
    for robot in ('r0','r1'):
        msg = h.messages[robot+'-livox'][-1]
        assert msg.header.frame_id == robot+'/lidar_link'
        assert msg.point_num > 0
        assert h.messages[robot+'-imu'][-1].linear_acceleration.z == pytest.approx(9.80665)
        assert h.node.count_publishers(f'/{robot}/points_raw') == 0
    assert h.node.count_publishers('/livox/lidar') == 0


@pytest.mark.parametrize('parameters,error', [({'xfer_format':2},'xfer_format'),
    ({'livox_lidar_id':256},'livox_lidar_id'), ({'lidar_interface':'invalid'},'lidar_interface')])
def test_invalid_livox_options_fail_before_publishing(h, parameters, error):
    mid360(h)
    process = h.sim(wait_for_robot_state_publisher=False, **parameters)
    h.until(lambda: process.poll() is not None)
    assert process.returncode != 0
    assert error in (h.directory/'sim.log').read_text()


@pytest.mark.parametrize('format', [0, 1])
def test_livox_shutdown_releases_callback_groups_before_plugin_unload(h, format):
    mid360(h)
    process = h.sim(start_paused=True, wait_for_robot_state_publisher=False,
                    lidar_interface='auto', xfer_format=format)
    h.ready()
    assert h.step(20, 'shutdown-frame').success
    process.send_signal(signal.SIGINT)
    assert h.wait(process) == 0, (h.directory / 'sim.log').read_text()
