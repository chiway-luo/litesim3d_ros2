"""Installed four-wheel example: actual DDS, all wheel states, TF and shutdown."""
from collections import deque
import csv
import json
import math
import os
from pathlib import Path
import re
import signal
import struct
import subprocess
import time
import xml.etree.ElementTree as ET

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist, TwistStamped
from litesim3d_interfaces.srv import Step
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, PointCloud2
from tf2_ros import Buffer, TransformListener
import yaml


def stamp(message):
    return message.sec * 1_000_000_000 + message.nanosec


class Example:
    serial = 0

    def __init__(self, directory, *, monitor_only=False, package="wheel_car_4w_sim",
                 launch_file="wheel_car_4w.launch.py"):
        self.directory = directory
        directory.mkdir(parents=True, exist_ok=True)
        Example.serial += 1
        os.environ["ROS_DOMAIN_ID"] = str(int(os.environ.get("LWS_4W_TEST_DOMAIN_BASE", "40")) + Example.serial)
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        os.environ["ROS_LOG_DIR"] = str(directory / "ros-logs")
        self.environment = os.environ.copy()
        self.package, self.launch_file = package, launch_file
        self.share = Path(get_package_share_directory(package))
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node("wheel_car_example_test", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.buffer = Buffer()
        self.listener = None if monitor_only else TransformListener(self.buffer, self.node, spin_thread=False)
        self.messages = {}
        self.subscriptions = []
        self.process = None
        self.log = None
        self.subscribe("status", DiagnosticArray, "/sim/status", 10)
        self.subscribe("clock", Clock, "/clock")
        if not monitor_only:
            self.subscribe("cloud", PointCloud2, "/points_raw")
            self.subscribe("boundary", PointCloud2, "/points_raw/range_boundary")
            self.subscribe("imu", Imu, "/imu/data_raw")
            self.subscribe("odom", Odometry, "/odom", 10)
            self.subscribe("joints", JointState, "/joint_states", 10)

    def subscribe(self, key, kind, topic, qos=qos_profile_sensor_data):
        self.messages[key] = deque(maxlen=200 if key in {"joints", "odom"} else 20)
        self.subscriptions.append(self.node.create_subscription(kind, topic, self.messages[key].append, qos))

    def start(self, *, reliable_sensors=False, **arguments):
        if reliable_sensors:
            # Exact single-frame payload checks need retransmission for large
            # fragmented clouds as well as endpoint discovery. Defaults remain
            # covered by the ordinary launch and best-effort streaming tests.
            sensors = {"/points_raw": ("cloud", PointCloud2),
                       "/points_raw/range_boundary": ("boundary", PointCloud2),
                       "/imu/data_raw": ("imu", Imu)}
            for subscription in list(self.subscriptions):
                if subscription.topic_name in sensors:
                    self.subscriptions.remove(subscription)
                    self.node.destroy_subscription(subscription)
            for topic, (key, kind) in sensors.items():
                self.subscribe(key, kind, topic, 100)
            params = yaml.safe_load(Path(arguments.get("params", self.share / "config/ros.yaml")).read_text())
            params["/litesim3d"]["ros__parameters"]["sensor_reliable"] = True
            path = self.directory / "reliable-ros.yaml"
            path.write_text(yaml.safe_dump(params))
            arguments["params"] = str(path)
        self.log = (self.directory / "launch.log").open("w")
        command = ["ros2", "launch", self.package, self.launch_file,
                   "manifest:=" + str(self.directory / "manifest.json")]
        command.extend(f"{name}:={value}" for name, value in {"rviz": "false", "gui": "false", **arguments}.items())
        self.process = subprocess.Popen(command, env=self.environment, cwd=self.directory,
                                        stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        self.until(lambda: self.status().get("ready") == "true", timeout=15)
        # Kernel/RSP readiness does not imply this test's sensor readers have
        # discovered the writers. Humble rclpy has no subscription match-count
        # API: check the reader's graph and the simulator's actual DDS matches.
        def matched():
            counts = json.loads(self.status().get("subscription_matches", "{}"))
            required = [s for s in self.subscriptions
                        if s.topic_name in {"/clock", "/points_raw", "/imu/data_raw", "/odom", "/joint_states"}]
            if "/points_raw/range_boundary" in counts:
                required += [s for s in self.subscriptions if s.topic_name == "/points_raw/range_boundary"]
            return all(self.node.count_publishers(s.topic_name) == 1
                       and counts.get(s.topic_name, 0) >= self.node.count_subscribers(s.topic_name)
                       for s in required)
        self.until(matched, timeout=15)

    def status(self):
        if not self.messages["status"]:
            return {}
        return {entry.key: entry.value for entry in self.messages["status"][-1].status[0].values}

    def spin(self, seconds=.1):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            # Humble recreates its ready-callback iterator when timeout_sec
            # changes. Keep it stable so busy early subscriptions cannot starve
            # later status/scene callbacks. Wall waits may overrun by <=20 ms.
            self.executor.spin_once(timeout_sec=.02)

    def until(self, predicate, timeout=6):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return
            self.spin(.02)
        log = (self.directory / "launch.log").read_text() if self.log else ""
        assert predicate(), log[-6000:]

    def step(self, steps, identity):
        client = self.node.create_client(Step, "/sim/step")
        assert client.wait_for_service(timeout_sec=5)
        request = Step.Request(request_id=identity, steps=steps)
        future = client.call_async(request)
        self.until(future.done)
        result = future.result()
        self.node.destroy_client(client)
        assert result.success, result
        self.until(lambda: self.messages["clock"] and stamp(self.messages["clock"][-1].clock) >= result.time_ns)
        return result

    def close(self):
        try:
            if self.process:
                if self.process.poll() is None:
                    os.killpg(self.process.pid, signal.SIGINT)
                    try:
                        self.process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(self.process.pid, signal.SIGKILL)
                        self.process.wait(timeout=3)
                        pytest.fail("example launch failed to stop its process group")
                (self.directory / "exit.json").write_text(json.dumps({"exit_code": self.process.returncode}))
            (self.directory / "status.json").write_text(json.dumps(self.status(), indent=2))
        finally:
            if self.log:
                self.log.close()
            if self.listener:
                self.listener.unregister()
            self.executor.shutdown()
            self.node.destroy_node()
            self.context.shutdown()


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
    assert imu.linear_acceleration.z == pytest.approx(9.80665, abs=1e-8)
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
    result = subprocess.run(["ros2", "launch", "wheel_car_4w_sim", "wheel_car_4w.launch.py",
                             "rviz:=false", "paused:=yes"], cwd=tmp_path, env=environment,
                            capture_output=True, text=True, timeout=15)
    assert result.returncode != 0
    assert "paused must be true or false" in result.stdout + result.stderr


@pytest.mark.parametrize("profile,rows,columns,min_range,max_range", [
    ("vlp16", 16, 1800, 1.0, 100.0),
    ("mid360", 1, 20000, .1, 40.0),
    ("mid40", 1, 10000, 1.0, 90.0),
])
def test_lidar_profiles_have_changing_returns_and_correct_fov(car, profile, rows, columns, min_range, max_range):
    car.start(lidar_profile=profile, paused="true", reliable_sensors=True)
    car.step(20, "profile-first")
    car.until(lambda: len(car.messages["cloud"]) >= 1)
    first = car.messages["cloud"][-1]
    car.step(20, "profile-second")
    car.until(lambda: len(car.messages["cloud"]) >= 2)
    second = car.messages["cloud"][-1]
    assert (second.height, second.width, second.point_step) == (rows, columns, 24)
    assert second.header.frame_id == "lidar_link"
    assert stamp(second.header.stamp) - stamp(first.header.stamp) == 100_000_000
    assert first.data != second.data  # Stationary robot; real frame content changes.
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
        "frames_different":True, "stationary":True, "fov_and_range_passed":True}, indent=2))


def test_range_boundary_can_be_disabled(car):
    profile = yaml.safe_load((car.share / "config/lidars/mid360.yaml").read_text())
    profile["lidar"]["publish_range_boundary"] = False
    path = car.directory / "without-boundary.yaml"
    path.write_text(yaml.safe_dump(profile))
    car.start(lidar_profile=str(path), paused="true", reliable_sensors=True)
    car.step(20, "without-boundary")
    car.until(lambda: bool(car.messages["cloud"]))
    assert car.node.count_publishers("/points_raw/range_boundary") == 0
    assert not car.messages["boundary"]


@pytest.mark.parametrize("rate,columns", [(10, 20000), (20, 10000)])
def test_mid360_directions_match_official_asset_at_both_frame_rates(car, rate, columns):
    profile = yaml.safe_load((car.share / "config/lidars/mid360.yaml").read_text())
    assert profile["lidar"]["scan_pattern"] == "livox_mid360"
    profile["lidar"].update(rate_hz=float(rate), columns=columns)
    path = car.directory / "reference-profile.yaml"
    path.write_text(yaml.safe_dump(profile))
    reference = Path(get_package_share_directory("litesim3d_core")) / "lidar_patterns/mid360.csv"
    with reference.open() as stream:
        reader = csv.reader(stream)
        assert next(reader) == ["Time/s", "Azimuth/deg", "Zenith/deg"]
        expected = [next(reader) for _ in range(columns * 2)]
    car.start(lidar_profile=str(path), paused="true", reliable_sensors=True)
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
    car.start(config=str(config_file), lidar_profile=str(profile_file), paused="true", reliable_sensors=True)
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
    result=subprocess.run(["ros2","launch","wheel_car_4w_sim","wheel_car_4w.launch.py",
                           "rviz:=false","lidar_profile:=missing-model"],
                          cwd=tmp_path,env=environment,capture_output=True,text=True,timeout=15)
    assert result.returncode != 0
    assert "lidar_profile must be" in result.stdout+result.stderr
    assert "process started with pid" not in result.stdout+result.stderr


@pytest.mark.skipif(os.environ.get("LWS_4W_GUI") != "1", reason="Set LWS_4W_GUI=1 in an accessible desktop session")
def test_rviz_starts_and_consumes_installed_model(car):
    car.start(rviz="true", paused="true", lidar_profile="mid360")
    car.until(lambda: "OpenGl version" in (car.directory / "launch.log").read_text(), timeout=15)
    car.step(40, "rviz-sensors")
    for topic in ("/robot_description", "/points_raw", "/points_raw/range_boundary", "/scene"):
        car.until(lambda topic=topic: any(item.node_name == "rviz2"
                  for item in car.node.get_subscriptions_info_by_topic(topic)))
    car.spin(1)
    assert car.process.poll() is None
    log = (car.directory / "launch.log").read_text()
    assert "process has died" not in log
