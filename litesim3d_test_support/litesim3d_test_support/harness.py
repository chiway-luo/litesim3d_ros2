"""Launch a configured simulation and observe its real DDS contracts."""
from collections import deque
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from litesim3d_interfaces.srv import Step
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, PointCloud2
from tf2_ros import Buffer, TransformListener


def stamp(message):
    return message.sec * 1_000_000_000 + message.nanosec


class SimulationHarness:
    serial = 0

    def __init__(self, directory, *, package, launch_file, monitor_only=False):
        self.directory = directory
        directory.mkdir(parents=True, exist_ok=True)
        SimulationHarness.serial += 1
        # Avoid the Linux ephemeral-port range and the bridge tests' 10..59 pool.
        os.environ["ROS_DOMAIN_ID"] = str(int(os.environ.get("LWS_TEST_DOMAIN_BASE", os.environ.get("LWS_4W_TEST_DOMAIN_BASE", "60"))) + SimulationHarness.serial % 40)
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        os.environ["ROS_LOG_DIR"] = str(directory / "ros-logs")
        self.environment = os.environ.copy()
        self.package, self.launch_file = package, launch_file
        self.share = Path(get_package_share_directory(package))
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node("litesim_example_test", context=self.context)
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
                        raise AssertionError("example launch failed to stop its process group")
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


