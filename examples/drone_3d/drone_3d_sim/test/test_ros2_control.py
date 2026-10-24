import os
import pytest
from litesim3d_test_support.harness import SimulationHarness
from litesim3d_test_support.control_examples import verify_drone, verify_external_altitude

@pytest.fixture
def controlled_drone(tmp_path):
    rig = SimulationHarness(tmp_path, package='drone_3d_sim', launch_file='drone_3d.launch.py')
    try:
        yield rig
    finally:
        rig.close(require_clean_children=True)

def test_body_control_motion_collision_lifecycle(controlled_drone):
    verify_drone(controlled_drone)

@pytest.mark.skipif(not os.environ.get('LITESIM_EXTERNAL_CONTROLLER_PARAMS'), reason='requires separately built controller overlay')
def test_external_controller_without_simulator_changes(controlled_drone):
    verify_external_altitude(controlled_drone, os.environ['LITESIM_EXTERNAL_CONTROLLER_PARAMS'])

@pytest.mark.skipif(os.environ.get('LWS_GUI_TESTS') != '1', reason='requires a real graphics backend')
@pytest.mark.parametrize('camera_type', ['rgb', 'depth', 'rgbd'])
def test_camera_plugins_with_ros2_control(controlled_drone, camera_type):
    import json
    import math
    import struct
    import xml.etree.ElementTree as ET
    import xacro
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Image, CameraInfo
    from litesim3d_test_support.control_examples import start, drive
    rig = controlled_drone
    model = ET.fromstring(xacro.process_file(str(rig.share / 'urdf/simulation.urdf.xacro'),
                                            mappings={'camera_type': camera_type}).toxml())
    camera = model.find("litesim3d/sensor[@name='front_camera']")
    ET.SubElement(camera, 'overrides').text = 'enabled: true\nwidth: 64\nheight: 48\nrate_hz: 5.0'
    path = rig.directory / 'camera.urdf'; ET.ElementTree(model).write(path, encoding='unicode')
    channels = ['color'] if camera_type == 'rgb' else ['depth'] if camera_type == 'depth' else ['color', 'depth']
    for channel in channels:
        rig.subscribe(channel, Image, f'/camera/front/{channel}/image_raw', 10)
        rig.subscribe(channel + '_info', CameraInfo, f'/camera/front/{channel}/camera_info', 10)
    publisher = start(rig, 'body_velocity_controller', drone=True, robot_urdf=str(path))
    cmd = Twist(); cmd.linear.z = .2
    drive(rig, publisher, cmd, .8)
    rig.until(lambda: all(rig.messages[channel] and rig.messages[channel + '_info'] for channel in channels))
    evidence = {'camera_type': camera_type, 'renderer': 'real Ogre', 'controller': 'BodyVelocityController'}
    for channel in channels:
        image = rig.messages[channel][-1]
        rig.until(lambda: any(info.header == image.header for info in rig.messages[channel + '_info']))
        assert (image.width, image.height) == (64, 48)
        assert image.encoding == ('rgb8' if channel == 'color' else '32FC1')
        assert image.header.frame_id == 'camera_optical_frame'
        if channel == 'depth':
            values = struct.unpack('<' + 'f' * (64*48), bytes(image.data))
            assert any(math.isfinite(value) and value > 0 for value in values)
        else:
            assert max(image.data) > min(image.data)
        evidence[channel + '_encoding'] = image.encoding
    assert rig.messages['feedback'][-1].body_velocity.linear.z > .15
    (rig.directory / 'camera-control-evidence.json').write_text(json.dumps(evidence, indent=2))
