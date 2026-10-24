import pytest
from litesim3d_test_support.harness import SimulationHarness
from litesim3d_test_support.control_examples import verify_wheel_car

@pytest.mark.parametrize('lidar_profile', ['config', 'mid360'])
def test_controlled_wheels_and_sensor_profiles(tmp_path, lidar_profile):
    rig = SimulationHarness(tmp_path, package='wheel_car_4w_sim', launch_file='wheel_car_4w.launch.py')
    try:
        verify_wheel_car(rig, lidar_profile=lidar_profile, lidar_interface='native')
    finally:
        rig.close()
