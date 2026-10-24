"""Launch a configured simulation and observe its real DDS contracts."""
from collections import deque
import json
import os
from pathlib import Path
import re
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

    def start(self, *, reliable_sensors=False, external_joint_states=False, joint_states=True, **arguments):
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
        startup_timeout = self.environment.get("LWS_TEST_STARTUP_TIMEOUT")
        if external_joint_states:
            arguments.setdefault("controller_startup_timeout", str(self.wall_timeout(60)))
        if reliable_sensors or startup_timeout:
            params = yaml.safe_load(Path(arguments.get("params", self.share / "config/ros.yaml")).read_text())
            settings = params["/litesim3d"]["ros__parameters"]
            if reliable_sensors:
                settings["sensor_reliable"] = True
            if startup_timeout:
                settings["startup_timeout"] = max(float(settings.get("startup_timeout", 0)), float(startup_timeout))
                settings["camera_render_timeout"] = float(self.environment.get("LWS_TEST_RENDER_TIMEOUT", "30"))
            path = self.directory / "test-ros.yaml"
            path.write_text(yaml.safe_dump(params))
            arguments["params"] = str(path)
        shutdown_timeout = self.environment.get("LWS_TEST_SHUTDOWN_TIMEOUT")
        if shutdown_timeout:
            arguments.update(sigterm_timeout=shutdown_timeout, sigkill_timeout=shutdown_timeout)
        self.log = (self.directory / "launch.log").open("w")
        command = ["ros2", "launch", self.package, self.launch_file,
                   "manifest:=" + str(self.directory / "manifest.json")]
        command.extend(f"{name}:={value}" for name, value in {"rviz": "false", "gui": "false", **arguments}.items())
        self.process = subprocess.Popen(command, env=self.environment, cwd=self.directory,
                                        stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        (self.directory / "launch-process.json").write_text(json.dumps({
            "command": command, "pid": self.process.pid,
            "observer_pid": os.getpid(), "wall_time_ns": time.time_ns(),
            "domain_id": int(self.environment["ROS_DOMAIN_ID"]),
            "localhost_only": self.environment["ROS_LOCALHOST_ONLY"],
        }, indent=2))
        self.until(lambda: self.status().get("ready") == "true", timeout=max(15, float(startup_timeout or 0) + 5))
        # Kernel/RSP readiness does not imply this test's sensor readers have
        # discovered the writers. Humble rclpy has no subscription match-count
        # API: check the reader's graph and the simulator's actual DDS matches.
        def matched():
            counts = json.loads(self.status().get("subscription_matches", "{}"))
            required = [s for s in self.subscriptions
                        if s.topic_name in {"/clock", "/points_raw", "/imu/data_raw", "/odom", "/joint_states"}
                        and (joint_states or s.topic_name != "/joint_states")]
            if "/points_raw/range_boundary" in counts:
                required += [s for s in self.subscriptions if s.topic_name == "/points_raw/range_boundary"]
            return all(self.node.count_publishers(s.topic_name) == 1
                       and ((external_joint_states and s.topic_name == "/joint_states")
                            or counts.get(s.topic_name, 0) >= self.node.count_subscribers(s.topic_name))
                       for s in required)
        self.until(matched, timeout=15)

    def status(self):
        if not self.messages["status"]:
            return {}
        return {entry.key: entry.value for entry in self.messages["status"][-1].status[0].values}

    def graph_snapshot(self, topics):
        """Record endpoint identities as well as counts for graph-contract failures."""
        return {
            "wall_time_ns": time.time_ns(),
            "domain_id": int(self.environment["ROS_DOMAIN_ID"]),
            "observer_pid": os.getpid(),
            "launch_pid": self.process.pid if self.process else None,
            "nodes": self.node.get_node_names_and_namespaces(),
            "topics": {topic: {
                "publisher_count": self.node.count_publishers(topic),
                "publishers": [{
                    "node_name": endpoint.node_name,
                    "node_namespace": endpoint.node_namespace,
                    "topic_type": endpoint.topic_type,
                    "gid": bytes(endpoint.endpoint_gid).hex(),
                } for endpoint in self.node.get_publishers_info_by_topic(topic)],
            } for topic in topics},
        }

    def spin(self, seconds=.1):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            # Humble recreates its ready-callback iterator when timeout_sec
            # changes. Keep it stable so busy early subscriptions cannot starve
            # later status/scene callbacks. Wall waits may overrun by <=20 ms.
            self.executor.spin_once(timeout_sec=.02)

    @staticmethod
    def wall_timeout(seconds):
        return max(seconds, float(os.environ.get("LWS_TEST_WAIT_TIMEOUT", "0")))

    def until(self, predicate, timeout=6):
        end = time.monotonic() + self.wall_timeout(timeout)
        while time.monotonic() < end:
            if predicate():
                return
            self.spin(.02)
        log = (self.directory / "launch.log").read_text() if self.log else ""
        assert predicate(), log[-6000:]

    def step(self, steps, identity):
        client = self.node.create_client(Step, "/sim/step")
        assert client.wait_for_service(timeout_sec=self.wall_timeout(5))
        request = Step.Request(request_id=identity, steps=steps)
        future = client.call_async(request)
        # Instrumented builds can spend longer computing a large synchronous
        # step. Keep the normal six-second contract; only the quality runner
        # opts into a larger wall budget, with identical simulated-time checks.
        self.until(future.done, timeout=float(os.environ.get("LWS_TEST_STEP_TIMEOUT", "6")))
        result = future.result()
        self.node.destroy_client(client)
        assert result.success, result
        self.until(lambda: self.messages["clock"] and stamp(self.messages["clock"][-1].clock) >= result.time_ns)
        return result

    def publish_paused_command(self, publisher, message):
        self.until(lambda: self.status().get("state") == "paused"
                   and self.status().get("pending_command") == "false")
        run_id = self.status()["run_id"]
        publisher.publish(message)
        # Topic delivery and the step service are independently scheduled.
        # Wait for actual acceptance before advancing the simulation.
        self.until(lambda: self.status().get("run_id") == run_id
                   and self.status().get("pending_command") == "true")

    def close(self, *, require_clean_children=False, expected_child_exits=None):
        try:
            # Capture before SIGINT removes the writers. Keep failures in their
            # original run; do not retry graph assertions or hide extra writers.
            try:
                graph = self.graph_snapshot(["/tf_static", "/tf", "/clock", "/joint_states"])
            except Exception as error:
                graph = {"capture_error": str(error)}
            (self.directory / "graph-before-shutdown.json").write_text(json.dumps(graph, indent=2))
            if self.process:
                # start_new_session gives this launch an exclusively owned
                # process group. A failed launch may already have exited while
                # a just-spawned RSP survives: poll() only describes the leader.
                leader_exited = self.process.poll() is not None

                def group_exists():
                    self.process.poll()  # Reap our child before checking its group.
                    try:
                        os.killpg(self.process.pid, 0)
                        return True
                    except ProcessLookupError:
                        return False

                try:
                    # A live launch forwards SIGINT to its children. Signalling
                    # the whole group as well sends it twice and can interrupt
                    # native destructors or LeakSanitizer during shutdown.
                    if leader_exited:
                        os.killpg(self.process.pid, signal.SIGINT)
                    else:
                        self.process.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass
                deadline = time.monotonic() + self.wall_timeout(10)
                while group_exists() and time.monotonic() < deadline:
                    time.sleep(.02)
                if group_exists() and self.process.poll() is not None:
                    # Retain cleanup for an exited launch that left a writer
                    # behind without relaying its shutdown signal.
                    try:
                        os.killpg(self.process.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
                    deadline = time.monotonic() + self.wall_timeout(3)
                    while group_exists() and time.monotonic() < deadline:
                        time.sleep(.02)
                forced_kill = group_exists()
                if forced_kill:
                    try:
                        os.killpg(self.process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                self.process.wait(timeout=3)
                (self.directory / "exit.json").write_text(json.dumps({
                    "exit_code": self.process.returncode,
                    "leader_exited_before_cleanup": leader_exited,
                    "forced_kill": forced_kill,
                    "group_exists_after_cleanup": group_exists(),
                }, indent=2))
                assert not forced_kill, "example launch failed to stop its process group"
                if require_clean_children:
                    output = (self.directory / "launch.log").read_text()
                    started = {pid: name for name, pid in re.findall(
                        r"\[([^\]]+)\]: process started with pid \[(\d+)\]", output)}
                    exited = {pid: 0 for _, pid in re.findall(
                        r"\[([^\]]+)\]: process has finished cleanly \[pid (\d+)\]", output)}
                    exited.update({pid: int(code) for _, pid, code in re.findall(
                        r"\[([^\]]+)\]: process has died \[pid (\d+), exit code (-?\d+)", output)})
                    expected = expected_child_exits or {}
                    assert set(expected) <= set(started), "expected exit names an unobserved child PID"
                    children = {pid: {"name": name, "exit_code": exited.get(pid),
                                      "expected_exit_code": expected.get(pid, 0)}
                                for pid, name in started.items()}
                    (self.directory / "child-exits.json").write_text(json.dumps(children, indent=2))
                    assert children and all(item["exit_code"] == item["expected_exit_code"]
                                            for item in children.values()), children
            (self.directory / "status.json").write_text(json.dumps(self.status(), indent=2))
        finally:
            if self.log:
                self.log.close()
            if self.listener:
                self.listener.unregister()
            self.executor.shutdown()
            self.node.destroy_node()
            self.context.shutdown()


