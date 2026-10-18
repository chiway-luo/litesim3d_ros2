"""Installed aerial example: real DDS, 3D motion, sensors and desktop rendering."""
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys

import pytest
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rclpy.time import Time

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'wheel_car_4w/wheel_car_4w_sim/test'))
from test_wheel_car_4w import Example, stamp


@pytest.fixture
def drone(tmp_path):
    rig = Example(tmp_path, package='drone_3d_sim', launch_file='drone_3d.launch.py')
    rig.subscribe('truth', Odometry, '/sim/ground_truth/quadrotor/odom', 100)
    yield rig
    rig.close()


def command(rig, x=0., y=0., z=0., yaw=0.):
    publisher = rig.node.create_publisher(Twist, '/cmd_vel', 1)
    rig.until(lambda: publisher.get_subscription_count() == 1)
    msg = Twist(); msg.linear.x=x; msg.linear.y=y; msg.linear.z=z; msg.angular.z=yaw
    publisher.publish(msg);rig.spin(.15)
    return publisher


def at_step(rig, count, identity):
    result=rig.step(count,identity)
    rig.until(lambda: rig.messages['odom'] and stamp(rig.messages['odom'][-1].header.stamp)==result.time_ns)
    rig.until(lambda: rig.messages['truth'] and stamp(rig.messages['truth'][-1].header.stamp)==result.time_ns)
    return rig.messages['odom'][-1]


def test_flight_xyz_yaw_sensors_pause_and_watchdog(drone):
    rig=drone;rig.start(paused='true',reliable_sensors=True)
    at_step(rig,20,'initial')
    start=rig.messages['truth'][-1].pose.pose.position
    assert (start.x,start.y,start.z)==pytest.approx((-8,-8,1.2))
    pub=command(rig,.3,.2,.4,.2)
    odom=at_step(rig,60,'three-dimensional-motion')
    pos=odom.pose.pose.position
    assert min(pos.x,pos.y,pos.z)>.005
    assert odom.pose.pose.orientation.z>0
    assert min(odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z)>0
    rig.until(lambda: rig.messages['imu'] and rig.messages['cloud'])
    assert any(m.linear_acceleration.z>9.9 for m in rig.messages['imu'])
    cloud=rig.messages['cloud'][-1]
    assert (cloud.height,cloud.width,cloud.header.frame_id)==(16,256,'lidar_link')
    assert sum(math.isfinite(v[0]) for v in struct.iter_unpack('<ffffH2xf',bytes(cloud.data)))>100
    for link in ('base_link','imu_link','lidar_link','camera_optical_frame'):
        rig.until(lambda link=link:rig.buffer.can_transform('odom',link,Time()))
    assert rig.node.count_publishers('/camera/front/color/image_raw')==0
    assert rig.node.count_publishers('/camera/front/depth/image_raw')==0
    before=stamp(odom.header.stamp);rig.spin(.2)
    assert stamp(rig.messages['clock'][-1].clock)==before
    stopped=at_step(rig,240,'watchdog')
    assert (stopped.twist.twist.linear.x,stopped.twist.twist.linear.y,stopped.twist.twist.linear.z)==pytest.approx((0,0,0),abs=1e-12)
    rig.until(lambda:rig.status().get('command_timeout')=='true')
    assert rig.status()['contact_count']=='0'
    assert rig.messages['joints'][-1].name==[]
    (rig.directory/'flight-evidence.json').write_text(json.dumps({'displacement':[pos.x,pos.y,pos.z],
        'lidar_slots':cloud.height*cloud.width,'pause':'PASS','watchdog':'PASS','camera_default':'OFF'},indent=2))
    rig.node.destroy_publisher(pub)


def test_descent_stops_at_floor_with_budget_disabled(drone):
    rig=drone;data=yaml.safe_load((rig.share/'config/runtime.yaml').read_text())
    data['robot']['drive'].update(unlimited_acceleration=True,max_v=10)
    config=rig.directory/'descent.yaml';config.write_text(yaml.safe_dump(data))
    rig.start(runtime=str(config),paused='true',collision_budget_enabled='false')
    pub=command(rig,z=-5.)
    odom=at_step(rig,60,'descent')
    truth=rig.messages['truth'][-1].pose.pose.position
    assert .23<truth.z<.26
    assert odom.twist.twist.linear.z==0
    rig.until(lambda:int(rig.status().get('contact_count','0'))>0)
    rig.node.destroy_publisher(pub)


@pytest.mark.skipif(os.environ.get('LWS_GUI_TESTS')!='1',reason='requires a desktop graphics backend')
def test_rgbd_topics_follow_altitude_without_camera_window(drone):
    rig=drone;data=yaml.safe_load((rig.share/'config/runtime.yaml').read_text())
    data['cameras'][0].update(enabled=True,type='rgbd',width=160,height=120,rate_hz=10)
    data['robot']['drive']['unlimited_acceleration']=True
    config=rig.directory/'rgbd.yaml';config.write_text(yaml.safe_dump(data))
    rig.subscribe('color',Image,'/camera/front/color/image_raw',100)
    rig.subscribe('depth',Image,'/camera/front/depth/image_raw',100)
    rig.start(runtime=str(config),paused='true',reliable_sensors=True)
    rig.until(lambda:json.loads(rig.status().get('subscription_matches','{}')).get('/camera/front/depth/image_raw',0)==1)
    at_step(rig,40,'before-ascent');rig.until(lambda:len(rig.messages['depth'])==2)
    before=bytes(rig.messages['depth'][-1].data)
    pub=command(rig,z=1.)
    at_step(rig,40,'after-ascent');rig.until(lambda:len(rig.messages['depth'])==4 and len(rig.messages['color'])==4)
    after=bytes(rig.messages['depth'][-1].data)
    a=struct.unpack('<19200f',before);b=struct.unpack('<19200f',after)
    changed=sum(math.isfinite(x) and math.isfinite(y) and abs(x-y)>.01 for x,y in zip(a,b))
    assert changed>500
    assert [stamp(m.header.stamp) for m in rig.messages['color']]==[100000001,200000001,300000001,400000001]
    assert not any(name.startswith('camera_view') for name in rig.node.get_node_names())
    from PyQt5.QtGui import QImage
    color=rig.messages['color'][-1]
    assert QImage(bytes(color.data),160,120,color.step,QImage.Format_RGB888).save(str(rig.directory/'drone-rgb.png'))
    (rig.directory/'depth-before.f32').write_bytes(before);(rig.directory/'depth-after.f32').write_bytes(after)
    (rig.directory/'rgbd-evidence.json').write_text(json.dumps({'changed_depth_pixels':changed,'camera_window':False}))
    rig.node.destroy_publisher(pub)


@pytest.mark.skipif(os.environ.get('LWS_GUI_TESTS')!='1',reason='requires a desktop graphics backend')
def test_arena_viewer_reads_sdf_overview_and_renders(drone):
    rig=drone;rig.start(paused='true')
    with (rig.directory/'viewer.log').open('w') as log:
        child=subprocess.Popen(['ros2','run','litesim3d_gui','viewer','--ros-args','-p','exit_after_frames:=35',
            '-p',f'capture_path:={rig.directory / "drone-arena.png"}'],env=rig.environment,stdout=log,stderr=subprocess.STDOUT)
        try:
            rig.until(lambda:child.poll() is not None,timeout=25)
            assert child.returncode==0,(rig.directory/'viewer.log').read_text()
            from PyQt5.QtGui import QImage
            image=QImage(str(rig.directory/'drone-arena.png'))
            assert not image.isNull()
            colors={image.pixel(x,y) for y in range(50,image.height()-50,20) for x in range(50,image.width()-50,20)}
            assert len(colors)>50,'missing geometry/materials in rendered arena'
        finally:
            if child.poll() is None:child.terminate();child.wait(timeout=5)
