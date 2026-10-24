"""Installed four-wheel example: actual DDS, all wheel states, TF and shutdown."""
import csv
import json
import math
import os
from pathlib import Path
import re
import signal
import struct
import subprocess
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
import yaml


from litesim3d_test_support import SimulationHarness, stamp


class Example(SimulationHarness):
    """Car-specific defaults; behavior lives in the installed support module."""
    def __init__(self, directory, **kwargs):
        super().__init__(directory, package="wheel_car_4w_sim",
                         launch_file="legacy_twist.launch.py", **kwargs)


@pytest.fixture
def car(tmp_path):
    example = Example(tmp_path)
    yield example
    example.close()


def assert_wheels(car, moving):
    car.until(lambda: bool(car.messages["joints"]) and bool(car.messages["odom"]))
    sample = car.messages["joints"][-1]
    positions = dict(zip(sample.name, sample.position))
    velocities = dict(zip(sample.name, sample.velocity))
    for side in ("left", "right"):
        front, rear = f"front_{side}_wheel_joint", f"rear_{side}_wheel_joint"
        assert positions[front] == pytest.approx(positions[rear], abs=1e-12)
        assert velocities[front] == pytest.approx(velocities[rear], abs=1e-12)
        if moving:
            assert abs(velocities[front]) > .05
        else:
            assert velocities[front] == pytest.approx(0.0, abs=1e-10)
    return {"position": positions, "velocity": velocities}


def test_installed_model_and_headless(tmp_path):
    share = Path(get_package_share_directory("wheel_car_4w_sim"))
    description = Path(get_package_share_directory("wheel_car_4w_description"))
    model = ET.parse(description / "urdf/wheel_car_4w.urdf").getroot()
    wheels = [joint for joint in model.findall("joint") if joint.attrib["type"] == "continuous"]
    assert len(wheels) == 4
    assert sum(joint.find("mimic") is not None for joint in wheels) == 2
    config = yaml.safe_load((share / "config/simulation.yaml").read_text())
    assert len(config["robot"]["drive"]["support_links"]) == 4
    executable = Path(get_package_prefix("litesim3d_core")) / "bin/sim_headless"
    result = subprocess.run([str(executable), str(share / "config/simulation.yaml"), "400",
                             str(tmp_path / "headless.json"), ".2", ".1", "0"],
                            cwd=tmp_path, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    manifest = json.loads((tmp_path / "headless.json").read_text())
    assert manifest["frames"] == 20
    assert manifest["point_slots"] == 20 * 16 * 512
    assert manifest["status"]["contact_count"] == 0
    assert Path(manifest["model"]["source"]).is_relative_to(description)


def test_launch_sensors_four_wheels_step_and_watchdog(car):
    car.start(paused="true")
    assert car.status()["sensor_queue_policy"] == "keep_last/best_effort"
    initial = car.step(20, "initial")
    car.until(lambda: car.messages["cloud"] and car.messages["imu"])
    imu = car.messages["imu"][-1]
    assert imu.header.frame_id == "imu_link" and imu.orientation_covariance[0] == -1
    assert imu.linear_acceleration.z == pytest.approx(9.81, abs=1e-8)
    cloud = car.messages["cloud"][-1]
    assert (cloud.height, cloud.width, cloud.point_step, cloud.header.frame_id) == (16, 512, 24, "lidar_link")
    assert sum(math.isfinite(values[0]) for values in struct.iter_unpack("<ffffH2xf", bytes(cloud.data))) > 100
    assert car.node.count_publishers("/clock") == 1
    for link in ("lidar_link", "imu_link", "base_footprint", "front_left_wheel_link",
                 "rear_left_wheel_link", "front_right_wheel_link", "rear_right_wheel_link"):
        car.until(lambda link=link: car.buffer.can_transform("odom", link, Time()))
    lidar = car.buffer.lookup_transform("base_link", "lidar_link", Time()).transform.translation
    assert (lidar.x, lidar.y, lidar.z) == pytest.approx((.12, 0, .49), abs=1e-9)
    publisher = car.node.create_publisher(Twist, "/cmd_vel", 1)
    car.until(lambda: publisher.get_subscription_count() == 1)
    command = Twist(); command.linear.x = .24; command.angular.z = .4
    publisher.publish(command); car.spin(.1)
    moved = car.step(60, "move")
    car.until(lambda: stamp(car.messages["odom"][-1].header.stamp) == moved.time_ns)
    car.until(lambda: stamp(car.messages["joints"][-1].header.stamp) == moved.time_ns)
    wheels = assert_wheels(car, moving=True)
    assert wheels["velocity"]["rear_left_wheel_joint"] < wheels["velocity"]["rear_right_wheel_joint"]
    assert car.messages["odom"][-1].pose.pose.position.x > .01
    assert car.step(60, "move").time_ns == moved.time_ns
    car.spin(.2)
    assert stamp(car.messages["clock"][-1].clock) == moved.time_ns
    assert moved.time_ns - initial.time_ns == 300_000_000
    stopped = car.step(200, "watchdog-stop")
    car.until(lambda: stamp(car.messages["joints"][-1].header.stamp) == stopped.time_ns)
    assert_wheels(car, moving=False)
    car.until(lambda: car.status().get("command_timeout") == "true")
    (car.directory / "wheels-moving.json").write_text(json.dumps(wheels, indent=2))


def test_ros_parameter_file_stamped_reliable_and_prefix(car):
    params = yaml.safe_load((car.share / "config/ros.yaml").read_text())
    params["/litesim3d"]["ros__parameters"].update(
        cmd_mode="stamped", sensor_reliable=True, frame_prefix="car/", start_paused=True)
    path = car.directory / "parameters.yaml"
    path.write_text(yaml.safe_dump(params))
    car.start(params=str(path), reliable_sensors=True)
    car.until(lambda: car.status().get("state") == "paused")
    car.step(20, "initial")
    car.until(lambda: bool(car.messages["cloud"]))
    assert car.messages["cloud"][-1].header.frame_id == "car/lidar_link"
    assert car.node.get_publishers_info_by_topic("/points_raw")[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
    car.until(lambda: car.buffer.can_transform("car/odom", "car/lidar_link", Time()))
    publisher = car.node.create_publisher(TwistStamped, "/cmd_vel", 1)
    car.until(lambda: publisher.get_subscription_count() == 1)
    command = TwistStamped()
    command.header.stamp = car.messages["clock"][-1].clock
    command.header.frame_id = "car/base_link"
    command.twist.linear.x = .24; command.twist.angular.z = -.4
    publisher.publish(command); car.spin(.1)
    stepped = car.step(60, "stamped-move")
    car.until(lambda: stamp(car.messages["joints"][-1].header.stamp) == stepped.time_ns)
    assert_wheels(car, moving=True)
    assert int(car.status().get("rejected_commands", "0")) == 0


def test_required_rsp_exit_propagates_failure(car):
    car.start(paused="true")
    match = re.search(r"\[robot_state_publisher-\d+\]: process started with pid \[(\d+)\]",
                      (car.directory / "launch.log").read_text())
    assert match, (car.directory / "launch.log").read_text()
    os.kill(int(match.group(1)), signal.SIGTERM)  # Only this launch's logged child PID.
    car.until(lambda: car.process.poll() is not None, timeout=12)
    assert car.process.returncode != 0
    car.until(lambda: car.node.count_publishers("/clock") == 0)


def test_invalid_launch_boolean_is_rejected(tmp_path):
    environment = {**os.environ, "ROS_LOG_DIR": str(tmp_path / "ros-logs")}
    result = subprocess.run(["ros2", "launch", "wheel_car_4w_sim", "legacy_twist.launch.py",
                              "paused:=yes"], cwd=tmp_path, env=environment,
                            capture_output=True, text=True, timeout=15)
    assert result.returncode != 0
    assert "paused must be true or false" in result.stdout + result.stderr


@pytest.mark.parametrize("profile,rows,columns,min_range,max_range", [
    ("vlp16", 16, 1800, 1.0, 100.0),
    ("mid360", 1, 20000, .1, 40.0),
    ("mid40", 1, 10000, 1.0, 90.0),
])
def test_lidar_profiles_have_ideal_returns_and_correct_fov(car, profile, rows, columns, min_range, max_range):
    settings = yaml.safe_load((car.share / f"config/lidars/{profile}.yaml").read_text())
    settings["lidar"]["publish_range_boundary"] = True
    path = car.directory / "with-boundary.yaml"
    path.write_text(yaml.safe_dump(settings))
    car.start(lidar_interface="native", lidar_profile=str(path), paused="true", reliable_sensors=True)
    car.step(20, "profile-first")
    car.until(lambda: len(car.messages["cloud"]) >= 1)
    first = car.messages["cloud"][-1]
    car.step(20, "profile-second")
    car.until(lambda: len(car.messages["cloud"]) >= 2)
    second = car.messages["cloud"][-1]
    assert (second.height, second.width, second.point_step) == (rows, columns, 24)
    assert second.header.frame_id == "lidar_link"
    assert stamp(second.header.stamp) - stamp(first.header.stamp) == 100_000_000
    assert settings["lidar"]["range_sigma_m"] == 0
    # A stationary mechanical scan repeats exactly at zero error; the two
    # non-repeating patterns still change their directions without added noise.
    assert (first.data != second.data) == (profile != "vlp16")
    car.until(lambda: any(m.header.stamp == second.header.stamp for m in car.messages["boundary"]))
    boundary = next(m for m in car.messages["boundary"] if m.header.stamp == second.header.stamp)
    assert (boundary.height, boundary.width, boundary.point_step, boundary.header.frame_id) == (rows, columns, 24, "lidar_link")
    boundary_count = 0
    valid = []
    for i in range(rows * columns):
        x, y, z = struct.unpack_from("<fff", second.data, i * 24)
        offset = struct.unpack_from("<f", second.data, i * 24 + 20)[0]
        ring = struct.unpack_from("<H", second.data, i * 24 + 16)[0]
        assert 0 <= offset < .1 and 0 <= ring < rows
        bx, by, bz = struct.unpack_from("<fff", boundary.data, i * 24)
        if math.isfinite(bx):
            assert not math.isfinite(x)
            assert math.sqrt(bx*bx+by*by+bz*bz) == pytest.approx(max_range, abs=2e-5)
            assert boundary.data[i*24+16:i*24+24] == second.data[i*24+16:i*24+24]
            boundary_count += 1
        if not math.isfinite(x):
            continue
        distance = math.sqrt(x*x+y*y+z*z)
        assert min_range - 1e-5 <= distance <= max_range + 1e-5
        elevation = math.degrees(math.asin(z / distance))
        if profile == "vlp16":
            assert elevation == pytest.approx(-15 + 2 * ring, abs=1e-4)
        elif profile == "mid360":
            assert -7.0001 <= elevation <= 52.0001
        else:
            assert math.degrees(math.acos(x / distance)) <= 19.2001
        valid.append((x,y,z))
    assert len(valid) > 100
    if profile != "mid40":
        assert boundary_count > 100
    car.until(lambda: bool(car.messages["odom"]))
    assert abs(car.messages["odom"][-1].pose.pose.position.x) < 1e-8
    (car.directory / "profile-check.json").write_text(json.dumps({
        "profile":profile, "valid_points":len(valid), "slots":rows*columns,
        "boundary_points":boundary_count, "boundary_range_passed":True,
        "frames_different":first.data != second.data, "stationary":True, "fov_and_range_passed":True}, indent=2))


@pytest.mark.parametrize("profile", ["ideal", "algorithm_test"])
def test_measurement_error_profiles_reach_ros_outputs(car, profile):
    filename = f"package://wheel_car_4w_sim/config/noise/{profile}.yaml"
    car.start(paused="true", noise_config=filename, reliable_sensors=True)
    stepped = car.step(40, "measurement-errors")
    car.until(lambda: car.messages["imu"] and car.messages["odom"]
              and stamp(car.messages["odom"][-1].header.stamp) == stepped.time_ns)
    imu = car.messages["imu"][-1]; odom = car.messages["odom"][-1]
    if profile == "ideal":
        assert imu.linear_acceleration.x == 0
        assert imu.linear_acceleration.z == pytest.approx(9.81, abs=1e-9)
        assert imu.angular_velocity.z == 0
        assert imu.linear_acceleration_covariance[0] == 0
        assert odom.twist.twist.linear.x == 0 and odom.twist.twist.angular.z == 0
    else:
        assert imu.linear_acceleration_covariance[0] == pytest.approx(0.02 ** 2)
        assert imu.angular_velocity_covariance[0] == pytest.approx(0.001 ** 2)
        assert abs(odom.twist.twist.linear.x) > 1e-8
        assert imu.linear_acceleration.z != pytest.approx(9.81, abs=1e-8)
    assert not any("rviz" in name.lower() for name in car.node.get_node_names())


@pytest.mark.parametrize("package,filename", [
    ("litesim3d_ros2", "demo.rviz"),
    ("wheel_car_4w_sim", "wheel_car_4w.rviz"),
    ("wheel_car_4w_sim", "mid360.rviz"),
])
def test_installed_rviz_views_keep_robot_and_lidar_without_scene_map(package, filename):
    path = Path(get_package_share_directory(package)) / "rviz" / filename
    displays = yaml.safe_load(path.read_text())["Visualization Manager"]["Displays"]
    assert any(d["Class"] == "rviz_default_plugins/RobotModel" and d["Enabled"] for d in displays)
    assert any(d["Class"] == "rviz_default_plugins/PointCloud2" and d["Enabled"] for d in displays)
    assert not any(d["Class"] == "rviz_default_plugins/MarkerArray" for d in displays)
    assert all(d.get("Topic", {}).get("Value") != "/scene" for d in displays)


def test_mid360_range_boundary_is_disabled_by_default(car):
    profile = yaml.safe_load((car.share / "config/lidars/mid360.yaml").read_text())
    assert profile["lidar"]["publish_range_boundary"] is False
    car.start(lidar_interface="native", lidar_profile="mid360", paused="true", reliable_sensors=True)
    car.step(20, "without-boundary")
    car.until(lambda: bool(car.messages["cloud"]))
    assert car.node.count_publishers("/points_raw/range_boundary") == 0
    assert not car.messages["boundary"]


@pytest.mark.parametrize("rate,columns", [(10, 20000), (20, 10000)])
def test_mid360_directions_match_official_asset_at_both_frame_rates(car, rate, columns):
    profile = yaml.safe_load((car.share / "config/lidars/mid360.yaml").read_text())
    assert profile["lidar"]["scan_pattern"] == "livox_mid360"
    profile["lidar"].update(rate_hz=float(rate), columns=columns, publish_range_boundary=True)
    path = car.directory / "reference-profile.yaml"
    path.write_text(yaml.safe_dump(profile))
    reference = Path(get_package_share_directory("litesim3d_core")) / "lidar_patterns/mid360.csv"
    with reference.open() as stream:
        reader = csv.reader(stream)
        assert next(reader) == ["Time/s", "Azimuth/deg", "Zenith/deg"]
        expected = [next(reader) for _ in range(columns * 2)]
    car.start(lidar_interface="native", lidar_profile=str(path), paused="true", reliable_sensors=True)
    captured = []
    checked = cropped = 0
    for frame in range(2):
        car.step(200 // rate, f"reference-frame-{frame}")
        car.until(lambda: len(car.messages["cloud"]) >= frame + 1)
        cloud = car.messages["cloud"][-1]
        car.until(lambda: any(m.header.stamp == cloud.header.stamp for m in car.messages["boundary"]))
        boundary = next(m for m in car.messages["boundary"] if m.header.stamp == cloud.header.stamp)
        assert (cloud.height, cloud.width, cloud.point_step) == (1, columns, 24)
        captured.append(stamp(cloud.header.stamp))
        for i in range(columns):
            ordinal, az, zen = map(float, expected[frame*columns+i])
            assert ordinal == frame*columns+i+1
            raw = struct.unpack_from("<fff", cloud.data, i*24)
            helper = struct.unpack_from("<fff", boundary.data, i*24)
            elevation = 90-zen
            if not -7 <= elevation <= 52:
                assert all(math.isnan(x) for x in (*raw, *helper))
                cropped += 1
                continue
            p = raw if math.isfinite(raw[0]) else helper
            assert all(math.isfinite(v) for v in p)
            length = math.sqrt(sum(v*v for v in p))
            az, el = math.radians(az), math.radians(elevation)
            wanted = (math.cos(el)*math.cos(az), math.cos(el)*math.sin(az), math.sin(el))
            assert max(abs(v/length-w) for v,w in zip(p,wanted)) < 2e-7
            assert struct.unpack_from("<f",cloud.data,i*24+20)[0] == pytest.approx(i/200000, abs=1e-8)
            checked += 1
    assert captured[1]-captured[0] == 1_000_000_000 // rate
    assert checked > columns * 1.98 and cropped > 0
    (car.directory / "official-reference-check.json").write_text(json.dumps({
        "rate_hz":rate,"columns":columns,"sample_rate_hz":rate*columns,
        "compared_directions":checked,"masked_slots":cropped,"max_unit_vector_error_limit":2e-7},indent=2))


def test_custom_profile_crops_fov_and_resolves_relative_config_paths(car):
    config = yaml.safe_load((car.share / "config/simulation.yaml").read_text())
    model = Path(get_package_share_directory("wheel_car_4w_description")) / "urdf/wheel_car_4w.urdf"
    (car.directory / "robot.urdf").write_text(model.read_text())
    config["robot"]["urdf"] = "robot.urdf"
    # Fill the cropped sector with a known plane inside the 4 m range. Cropping
    # masks source rays; it must not repopulate a narrow sector with 20,000 rays.
    config["entities"] = [next(e for e in config["entities"] if e["id"] == "ground"), {
        "id": "crop_target", "pose": {"xyz": [3.05, 0, 1], "rpy": [0, 0, 0]},
        "geometry": {"type": "box", "size": [.1, 5, 5]}, "support": False}]
    config_file = car.directory / "simulation.yaml"
    config_file.write_text(yaml.safe_dump(config))
    profile = yaml.safe_load((car.share / "config/lidars/mid360.yaml").read_text())
    # A scan table relative to the custom profile must survive the launch's temporary YAML.
    reference = Path(get_package_share_directory("litesim3d_core")) / "lidar_patterns/mid360.csv"
    (car.directory / "scan.csv").write_bytes(reference.read_bytes())
    profile["lidar"].update(horizontal_start_deg=-20, horizontal_end_deg=20,
                            vertical_min_deg=-5, vertical_max_deg=15, max_range_m=4.0,
                            pattern_file="scan.csv", range_sigma_m=0.0)
    profile_file = car.directory / "custom-lidar.yaml"
    profile_file.write_text(yaml.safe_dump(profile))
    car.start(config=str(config_file), lidar_interface="native", lidar_profile=str(profile_file), paused="true", reliable_sensors=True)
    car.step(20, "custom-fov")
    car.until(lambda: bool(car.messages["cloud"]))
    cloud=car.messages["cloud"][-1]
    with reference.open() as stream:
        reader = csv.reader(stream)
        next(reader)
        expected = [next(reader) for _ in range(cloud.width)]
    valid=0
    for i in range(cloud.width):
        x,y,z=struct.unpack_from("<fff",cloud.data,i*24)
        _, az, zen = map(float, expected[i])
        az = (az + 180) % 360 - 180
        inside = -20 <= az < 20 and -5 <= 90-zen <= 15
        assert math.isfinite(x) == inside
        if not math.isfinite(x):
            continue
        assert x == pytest.approx(3.0-.12, abs=1e-6)
        distance=math.sqrt(x*x+y*y+z*z)
        assert distance <= 4.00001
        assert -20.0001 <= math.degrees(math.atan2(y,x)) <= 20.0001
        assert -5.0001 <= math.degrees(math.asin(z/distance)) <= 15.0001
        valid+=1
    assert valid > 100
    assert cloud.width == 20000
    (car.directory / "crop-check.json").write_text(json.dumps({
        "slots": cloud.width, "hits_on_target_plane": valid,
        "masked_slots": cloud.width-valid, "plane_x_in_lidar_m": 2.88}, indent=2))


def test_unknown_profile_fails_without_starting_nodes(tmp_path):
    environment={**os.environ,"ROS_LOG_DIR":str(tmp_path/"ros-logs")}
    result=subprocess.run(["ros2","launch","wheel_car_4w_sim","legacy_twist.launch.py",
                           "lidar_profile:=missing-model"],
                          cwd=tmp_path,env=environment,capture_output=True,text=True,timeout=15)
    assert result.returncode != 0
    assert "lidar_profile must be" in result.stdout+result.stderr
    assert "process started with pid" not in result.stdout+result.stderr



def test_mid360_launch_defaults_to_official_messages_and_keeps_other_sensors(tmp_path):
    from livox_ros_driver2.msg import CustomMsg
    from sensor_msgs.msg import PointCloud2, Imu, JointState
    from nav_msgs.msg import Odometry
    car = Example(tmp_path, monitor_only=True)
    try:
        car.subscribe('livox', CustomMsg, '/livox/lidar', 10)
        car.subscribe('preview', PointCloud2, '/livox/lidar/pointcloud', 10)
        car.subscribe('boundary', PointCloud2, '/livox/lidar/range_boundary', 10)
        car.subscribe('imu', Imu, '/imu/data_raw')
        car.subscribe('odom', Odometry, '/odom', 10)
        car.subscribe('joints', JointState, '/joint_states', 10)
        car.start(lidar_profile='mid360', paused='true')
        car.until(lambda: all(json.loads(car.status().get('subscription_matches','{}')).get(t,0) >= 1
                  for t in ['/livox/lidar','/livox/lidar/pointcloud']))
        step = car.step(20, 'official-launch')
        car.until(lambda: all(car.messages[k] for k in ['livox','preview','imu','odom','joints']))
        msg = car.messages['livox'][-1]
        assert msg.header.frame_id == 'lidar_link' and msg.point_num > 100
        assert car.messages['preview'][-1].width == msg.point_num
        assert car.node.count_publishers('/livox/lidar/range_boundary') == 0
        assert not car.messages['boundary']
        imu = car.messages['imu'][-1]
        assert imu.header.frame_id == 'imu_link'
        assert imu.linear_acceleration.z == pytest.approx(9.81)
        assert car.messages['odom'][-1].child_frame_id == 'base_link'
        assert len(car.messages['joints'][-1].name) == 4
        assert car.node.count_publishers('/points_raw') == car.node.count_publishers('/livox/imu') == 0
        assert car.node.count_publishers('/clock') == 1
        manifest = json.loads((tmp_path/'manifest.json').read_text())
        assert manifest['livox_output']['lidars'][0]['message_type'] == 'livox_ros_driver2/msg/CustomMsg'
        assert any(plugin['class'] == 'litesim3d_mid360/Mid360'
                   for plugin in manifest['config']['plugins'])
    finally:
        car.close()


@pytest.mark.skipif(os.environ.get("LWS_4W_GUI") != "1", reason="Set LWS_4W_GUI=1 in an accessible desktop session")
@pytest.mark.parametrize("profile,interface,cloud_topic", [
    ("vlp16", "native", "/points_raw"),
    ("mid360", "native", "/points_raw"),
    ("mid360", "auto", "/livox/lidar/pointcloud"),
])
def test_rviz_starts_and_consumes_installed_model(car, profile, interface, cloud_topic):
    if interface == "auto":
        # Official Livox output replaces the native cloud topics.
        for subscription in list(car.subscriptions):
            if subscription.topic_name in ("/points_raw", "/points_raw/range_boundary"):
                car.node.destroy_subscription(subscription)
                car.subscriptions.remove(subscription)
    car.start(rviz="true", paused="true", lidar_interface=interface, lidar_profile=profile)
    car.until(lambda: "OpenGl version" in (car.directory / "launch.log").read_text(), timeout=15)
    car.step(40, "rviz-sensors")
    for topic in ("/robot_description", cloud_topic):
        car.until(lambda topic=topic: any(item.node_name == "rviz2"
                  for item in car.node.get_subscriptions_info_by_topic(topic)))
    hidden_topics = ["/scene"]
    if profile == "mid360":
        hidden_topics += ["/points_raw/range_boundary", "/livox/lidar/range_boundary"]
    for topic in hidden_topics:
        assert not any(item.node_name == "rviz2" for item in car.node.get_subscriptions_info_by_topic(topic))
    car.spin(1)
    assert car.process.poll() is None
    log = (car.directory / "launch.log").read_text()
    assert "process has died" not in log
