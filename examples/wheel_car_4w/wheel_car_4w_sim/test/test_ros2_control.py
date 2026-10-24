import pytest
from controller_manager_msgs.srv import SwitchController
from std_srvs.srv import SetBool
from litesim3d_test_support.control_examples import load, service, start, switch_request
from litesim3d_test_support.harness import SimulationHarness
from litesim3d_test_support.control_examples import verify_wheel_car

@pytest.mark.parametrize('lidar_profile', ['config', 'mid360'])
def test_controlled_wheels_and_sensor_profiles(tmp_path, lidar_profile):
    rig = SimulationHarness(tmp_path, package='wheel_car_4w_sim', launch_file='wheel_car_4w.launch.py')
    try:
        verify_wheel_car(rig, lidar_profile=lidar_profile, lidar_interface='native')
    finally:
        rig.close()


def test_controller_switch_survives_paused_simulation_clock(tmp_path):
    rig = SimulationHarness(tmp_path, package='wheel_car_4w_sim', launch_file='wheel_car_4w.launch.py')
    try:
        start(rig, 'diff_drive_controller')
        load(rig, 'wheel_velocity_controller')
        assert service(rig, SetBool, '/sim/pause', SetBool.Request(data=True)).success
        # Let the manager consume the final clock sample and enter its sleep.
        rig.spin(.2)
        client = rig.node.create_client(SwitchController, '/controller_manager/switch_controller')
        assert client.wait_for_service(timeout_sec=10)
        request = switch_request(rig, ['wheel_velocity_controller'], ['diff_drive_controller'])
        future = client.call_async(request)
        rig.spin(1.3)  # Longer than Humble's implicit one-second switch timeout.
        assert not future.done(), 'switch expired while waiting for the paused update loop'
        assert service(rig, SetBool, '/sim/pause', SetBool.Request(data=False)).success
        rig.until(future.done, timeout=request.timeout.sec + 5)
        assert future.result().ok
        rig.node.destroy_client(client)
    finally:
        rig.close()
