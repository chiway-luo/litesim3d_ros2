"""Acceptance helpers shared by independently installed example packages."""
import hashlib
import json
import math
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
import yaml

from .harness import SimulationHarness, stamp


def setup(directory, kind, enabled=True):
    h = SimulationHarness(directory, package=kind + "_sim", launch_file=kind + ".launch.py")
    h.subscribe("truth", Odometry, f"/sim/ground_truth/{kind}/odom", 10)
    h.subscribe("description", String, "/sim/scene_description", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    data = yaml.safe_load((h.share / "config/runtime.yaml").read_text())
    data["simulation"]["command_timeout"] = 100.0
    data["robot"]["continue_on_collision"] = enabled
    runtime = directory / "runtime.yaml"; runtime.write_text(yaml.safe_dump(data))
    try:
        h.start(runtime=str(runtime), paused="true")
    except BaseException:
        h.close(); raise
    return h


def move(h, x, y, w, steps, identity):
    publisher = h.node.create_publisher(Twist, "/cmd_vel", 1)
    h.until(lambda: publisher.get_subscription_count() == 1)
    command = Twist(); command.linear.x = float(x); command.linear.y = float(y); command.angular.z = float(w)
    h.publish_paused_command(publisher, command)
    result = h.step(steps, identity)
    h.until(lambda: all(h.messages[key] and stamp(h.messages[key][-1].header.stamp) == result.time_ns
                        for key in ("odom", "truth", "joints")))
    h.node.destroy_publisher(publisher)
    return h.messages["truth"][-1], h.messages["odom"][-1]


def verify_motion_and_model(directory, kind):
    h = setup(directory, kind)
    try:
        h.until(lambda: h.messages["description"])
        scene = json.loads(h.messages["description"][-1].data)
        client = h.node.create_client(GetParameters, "/robot_state_publisher/get_parameters")
        assert client.wait_for_service(timeout_sec=5)
        future = client.call_async(GetParameters.Request(names=["robot_description"]))
        h.until(future.done)
        xml = future.result().values[0].string_value
        assert hashlib.sha256(xml.encode()).hexdigest() == scene["robot_model"]["sha256"]
        robot = ET.fromstring(xml)
        assert not any("xacro" in el.tag for el in robot.iter())
        assert len(robot.findall("./litesim3d/chassis")) == len(robot.findall("./litesim3d/odometry")) == 1
        links = {link.get("name") for link in robot.findall("link")}
        assert set(scene["initial_link_poses"]) == links
        assert all(v["binding"] in links for v in scene["visuals"] if v["binding"])
        h.node.destroy_client(client)

        truth, odom = move(h, 0, 0, .4, 100, "pivot")
        if kind == "ackermann":
            assert abs(truth.pose.pose.orientation.z) < 1e-10  # Ackermann has no stationary pivot.
        else:
            assert truth.pose.pose.orientation.z > .1
        move(h, 0, 0, 0, 60, "settle")
        truth, odom = move(h, .4, 0, .2, 100, "turn")
        assert math.hypot(truth.pose.pose.position.x, truth.pose.pose.position.y) > .15
        for axis in ("x", "y", "z"):
            assert abs(getattr(truth.pose.pose.position, axis) - getattr(odom.pose.pose.position, axis)) < 1e-8
        if kind == "ackermann":
            positions = dict(zip(h.messages["joints"][-1].name, h.messages["joints"][-1].position))
            assert 0 < positions["right_steering_joint"] < positions["left_steering_joint"] <= .6
        else:
            # The planar acceleration bound applies to the velocity vector;
            # changing (.4,0) into (0,.4) takes sqrt(.4²+.4²)/.5 > 1 second.
            _, odom = move(h, 0, .4, 0, 150, "lateral")
            assert abs(odom.twist.twist.linear.y - .4) < 1e-9
        for frame in ("lidar_link", "imu_link", "camera_optical_frame"):
            h.until(lambda: h.buffer.can_transform("base_link", frame, Time()))
            actual = h.buffer.lookup_transform("base_link", frame, Time()).transform.translation
            expected = scene["initial_link_poses"][frame][:3]
            assert max(abs(a-b) for a,b in zip((actual.x,actual.y,actual.z),expected)) < 1e-9
        assert h.node.count_publishers("/tf_static") == 1
        assert {p.node_name for p in h.node.get_publishers_info_by_topic("/tf")} == {"litesim3d", "robot_state_publisher"}
        assert h.messages["cloud"][-1].header.frame_id == "lidar_link"
        assert h.messages["imu"][-1].header.frame_id == "imu_link"
    finally:
        h.close()


def verify_collision(directory, kind, enabled):
    h = setup(directory, kind, enabled)
    try:
        lateral = kind.startswith("omni")
        truth, odom = move(h, 0 if lateral else 1, 1 if lateral else 0, 0, 500, "hit-wall")
        axis = "y" if lateral else "x"
        t, o = (getattr(m.pose.pose.position, axis) for m in (truth, odom))
        assert .8 < t < 2.4 and abs(getattr(truth.twist.twist.linear, axis)) < 1e-8
        assert o > t + 1 if enabled else abs(o-t) < 1e-8
        assert abs(getattr(odom.twist.twist.linear, axis) - (1 if enabled else 0)) < 1e-8
        h.until(lambda: int(h.status().get("contact_count", "0")) > 0)
        wheels = h.messages["joints"][-1]
        assert any(abs(v) > 1 for v in wheels.velocity) if enabled else all(abs(v) < 1e-8 for v in wheels.velocity)
        (directory / "collision-evidence.json").write_text(json.dumps({
            "kind":kind,"continue_on_collision":enabled,"truth_m":t,"odometry_m":o,"axis":axis}, indent=2))
    finally:
        h.close()
