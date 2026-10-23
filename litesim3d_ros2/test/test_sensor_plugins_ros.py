"""Runtime sensor plugins: DDS, RSP mount TF, truth sampling and real Ogre pixels."""
import json
import math
import os
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2, Image, CameraInfo

from test_ros import h, ns
from test_collision_odometry_ros import configure
from test_plugins_ros import add_plugins, use_plugin_model


def sensor(block, name, package, cls, parameters):
    tag = ET.SubElement(block, 'sensor', name=name, **{'class': f'{package}/{cls}'})
    ET.SubElement(tag, 'parameters', file=f'package://{package}/config/parameters.yaml')
    ET.SubElement(tag, 'overrides').text = yaml.safe_dump(parameters)


def prepare(h, cameras=False):
    configure(h, True)
    robot = ET.fromstring(Path(h.config['robot']['urdf']).read_text())
    add_plugins(robot, 'litesim3d_diff_drive', 'DifferentialDrive', 'litesim3d_diff_odometry', 'DifferentialOdometry')
    for link, rpy in [('imu_plugin_link', '0 0 0'), ('camera_optical_frame', f'{-math.pi/2} 0 {-math.pi/2}')]:
        ET.SubElement(robot, 'link', name=link)
        joint = ET.SubElement(robot, 'joint', name=link + '_mount', type='fixed')
        ET.SubElement(joint, 'parent', link='base_link'); ET.SubElement(joint, 'child', link=link)
        ET.SubElement(joint, 'origin', xyz='.2 0 .65', rpy=rpy)
    block = ET.SubElement(robot, 'litesim3d', version='1')
    sensor(block, 'front', 'litesim3d_lidar', 'Lidar', dict(link='lidar_link', topic='points_raw', columns=4,
        vertical_angles_deg=[0.], horizontal_start_deg=0., horizontal_end_deg=.0001, rate_hz=10.))
    sensor(block, 'aux', 'litesim3d_lidar', 'Lidar', dict(link='lidar_link', topic='aux/points', columns=4,
        vertical_angles_deg=[0.], rate_hz=7.))
    sensor(block, 'imu', 'litesim3d_imu', 'Imu', dict(link='imu_plugin_link', topic='imu/data_raw', rate_hz=20.))
    sensor(block, 'imu_aux', 'litesim3d_imu', 'Imu', dict(link='imu_plugin_link', topic='imu/aux', rate_hz=13.))
    camera_configs = []
    if cameras:
        for name, kind, rate, noise in [('rgb', 'rgb', 5., {}), ('rgb_bias','rgb',7.,{'color_bias_dn':[10.,0.,0.]}),
                                       ('depth','depth',5.,{}),('rgbd','rgbd',5.,{'depth_bias_m':.2})]:
            p = dict(link='camera_optical_frame',topic='camera/'+name,enabled=True,width=64,height=48,
                     rate_hz=rate,horizontal_fov_deg=70.,near_clip_m=.1,far_clip_m=20.,**noise)
            cls = dict(rgb='RgbCamera', depth='DepthCamera', rgbd='RgbdCamera')[kind]
            sensor(block,name,'litesim3d_'+kind+'_camera',cls,p)
            camera_configs.append(dict(name=name,type=kind,**p))
    use_plugin_model(h,robot)
    h.config['lidars'] = []; h.config['imus'] = []
    h.subscribe('points',PointCloud2,'/points_raw')
    h.subscribe('aux_points',PointCloud2,'/aux/points')
    h.subscribe('aux_imu',Imu,'/imu/aux')
    return camera_configs


def move_and_drain(h, ticks=400):
    publisher = h.node.create_publisher(Twist,'/cmd_vel',1)
    h.until(lambda: publisher.get_subscription_count() == 1)
    cmd = Twist(); cmd.linear.x = .8; h.publish_paused_command(publisher,cmd)
    for start in range(0,ticks,20):
        assert h.step(min(20,ticks-start),f'block-{start}').success
        h.spin(.05)
    h.wait_for_stamp(ticks*5_000_000+1,'odom','truth','joints')
    return publisher


def test_explicit_sensor_plugins_mount_tf_rates_and_blocked_truth(h):
    prepare(h)
    h.rsp(); h.sim(start_paused=True,sensor_reliable=True); h.ready()
    move_and_drain(h)
    h.until(lambda: len(h.messages['points']) == 20 and len(h.messages['aux_points']) == 14
            and len(h.messages['imu']) == 40 and len(h.messages['aux_imu']) == 26)
    for key, rate in [('points',10),('aux_points',7),('imu',20),('aux_imu',13)]:
        assert [ns(m.header.stamp) for m in h.messages[key]] == [1+round(i*1e9/rate) for i in range(1,2*rate+1)]
    assert h.messages['odom'][-1].pose.pose.position.x == pytest.approx(1.6,abs=1e-8)
    assert h.messages['truth'][-1].pose.pose.position.x == pytest.approx(.35,abs=2e-5)
    assert struct.unpack_from('<f',bytes(h.messages['points'][-1].data))[0] == pytest.approx(.2,abs=2e-5)
    imu = h.messages['imu'][-1]
    assert imu.header.frame_id == 'imu_plugin_link'
    assert imu.linear_acceleration.x == 0 and imu.angular_velocity.z == 0
    assert imu.linear_acceleration.z == pytest.approx(9.81)
    h.until(lambda: h.buffer.can_transform('base_link','imu_plugin_link',Time()))
    mount = h.buffer.lookup_transform('base_link','imu_plugin_link',Time()).transform.translation
    assert (mount.x,mount.y,mount.z) == pytest.approx((.2,0,.65))
    assert h.node.count_publishers('/tf_static') == 1


@pytest.mark.skipif(os.environ.get('LWS_GUI_TESTS') != '1',reason='requires real graphics backend')
def test_explicit_camera_plugins_real_render_noise_and_collision_truth(h):
    cameras = prepare(h,True)
    poses = {name:[0,0,0,0,0,0,1] for name in ['base_link','left_wheel','right_wheel','lidar_link','imu_plugin_link','camera_optical_frame']}
    poses['left_wheel'][:3] = [0,.25,.1]; poses['right_wheel'][:3] = [0,-.25,.1]
    for name in ['lidar_link','imu_plugin_link']: poses[name][:3] = [.2,0,.65]
    poses['camera_optical_frame'] = [.2,0,.65,-.5,.5,-.5,.5]
    scene = {'schema_version':1,'scene_id':'plugin-camera-test','resources':{},'lights':[],
        'ambient':[1,1,1,1],'background':[0,0,0,1],'cameras':cameras,'initial_link_poses':poses,
        'visuals':[{'id':'wall','binding':'','pose':[.8,0,1,0,0,0,1],
            'geometry':{'type':'box','size':[.1,10,3]},
            'material':{'diffuse':[.3,.05,.02,1],'ambient':[.3,.05,.02,1],'transparency':0,'cast_shadows':False}}]}
    scene_file = h.directory/'scene.json'; scene_file.write_text(json.dumps(scene))
    executable = Path(get_package_prefix('litesim3d_gui'))/'lib/litesim3d_gui/camera_node'
    renderer = h.process('renderer',[str(executable),'--ros-args','-p',f'scene_description:={scene_file}'])
    for c in cameras:
        for channel in ['color','depth']:
            if (channel=='color' and c['type']!='depth') or (channel=='depth' and c['type']!='rgb'):
                h.subscribe(c['name']+'_'+channel,Image,f"/{c['topic']}/{channel}/image_raw")
                h.subscribe(c['name']+'_'+channel+'_info',CameraInfo,f"/{c['topic']}/{channel}/camera_info")
    startup_timeout = max(30., float(os.environ.get('LWS_TEST_STARTUP_TIMEOUT', '0')))
    h.rsp(); h.sim(start_paused=True,sensor_reliable=True,scene_description=str(scene_file),startup_timeout=startup_timeout,
                   camera_render_timeout=float(os.environ.get('LWS_TEST_RENDER_TIMEOUT', '30')))
    h.until(lambda: h.status().get('ready')=='true',timeout=startup_timeout + 5)
    move_and_drain(h)
    h.until(lambda: len(h.messages['rgb_color'])==10 and len(h.messages['rgb_bias_color'])==14
        and len(h.messages['depth_depth'])==10 and len(h.messages['rgbd_depth'])==10)
    for c in cameras:
        for channel in ['color','depth']:
            key=c['name']+'_'+channel
            if key not in h.messages: continue
            messages=h.messages[key]; infos=h.messages[key+'_info']
            assert [ns(m.header.stamp) for m in messages] == [1+round(i*1e9/c['rate_hz']) for i in range(1,len(messages)+1)]
            assert len(infos)==len(messages)
            for m,info in zip(messages,infos):
                assert m.header==info.header and m.header.frame_id=='camera_optical_frame'
                assert (m.width,m.height)==(64,48)
                assert m.encoding==('rgb8' if channel=='color' else '32FC1')
    depth=h.messages['depth_depth'][-1]; biased=h.messages['rgbd_depth'][-1]
    center=(24*64+32)*4
    assert struct.unpack_from('<f',bytes(depth.data),center)[0] == pytest.approx(.2,abs=.002)
    assert struct.unpack_from('<f',bytes(biased.data),center)[0] == pytest.approx(.4,abs=.002)
    color=h.messages['rgb_color'][-1]; color_bias=h.messages['rgb_bias_color'][-1]
    pixel=(24*64+32)*3
    assert color_bias.data[pixel]==min(255,color.data[pixel]+10)
    assert h.messages['odom'][-1].pose.pose.position.x == pytest.approx(1.6,abs=1e-8)
    assert renderer.poll() is None
    h.until(lambda: h.buffer.can_transform('base_link','camera_optical_frame',Time()))
    (h.directory/'camera-plugin-evidence.json').write_text(json.dumps({'camera_classes':[c['type'] for c in cameras],
        'depth_m':struct.unpack_from('<f',bytes(depth.data),center)[0],
        'biased_depth_m':struct.unpack_from('<f',bytes(biased.data),center)[0],
        'truth_x_m':h.messages['truth'][-1].pose.pose.position.x,
        'encoder_odom_x_m':h.messages['odom'][-1].pose.pose.position.x,'renderer':'real Ogre'},indent=2))


def test_camera_render_timeout_faults_on_unresponsive_renderer(h):
    from litesim3d_interfaces.srv import RenderCameras
    import time
    prepare(h, True)
    scene_file = h.directory / 'scene.json'
    scene_file.write_text(json.dumps({'scene_id': 'delayed-test-renderer'}))
    received = []
    def delayed(request, response):
        received.append(request)
        time.sleep(.4)
        return response
    service = h.node.create_service(RenderCameras, '/sim/render_cameras', delayed)
    try:
        h.rsp(); h.sim(start_paused=True, scene_description=str(scene_file), camera_render_timeout=.1)
        h.ready()
        result = h.step(40, 'render-deadline')
        assert not result.success
        h.until(lambda: bool(received) and h.status().get('state') == 'fault')
        assert 'camera render timeout' in h.status()['last_error']
        assert h.status()['published_camera_frames'] == '0'
    finally:
        h.node.destroy_service(service)
