"""Acceptance helpers shared by independently installed example packages."""
import hashlib
import json
import math
import time
import subprocess
import xml.etree.ElementTree as ET
from urllib.parse import unquote, urlsplit

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String, Float64MultiArray
from controller_manager_msgs.srv import ListControllers, SwitchController
from litesim3d_interfaces.msg import JointCommand
from rosidl_runtime_py.convert import message_to_ordereddict
import yaml

from .harness import SimulationHarness, stamp


def setup(directory, kind, enabled=True, wheel_limit=None):
    h = SimulationHarness(directory, package=kind + "_sim", launch_file=kind + ".launch.py")
    h.subscribe("truth", Odometry, f"/sim/ground_truth/{kind}/odom", 10)
    h.subscribe("description", String, "/sim/scene_description", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    h.subscribe("sim_model", String, "/sim/robot_description", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    data = yaml.safe_load((h.share / "config/runtime.yaml").read_text())
    data["simulation"]["command_timeout"] = 100.0
    data["robot"]["continue_on_collision"] = enabled
    runtime = directory / "runtime.yaml"; runtime.write_text(yaml.safe_dump(data))
    arguments = {}
    if wheel_limit is not None:
        parameters = yaml.safe_load((h.share / "config/controllers.yaml").read_text())
        settings = parameters["/omni_controller"]["ros__parameters"]
        settings["max_wheel_velocity"] = [float(wheel_limit)] * len(settings["wheel_joints"])
        path = directory / "controllers.yaml"; path.write_text(yaml.safe_dump(parameters))
        arguments["controllers"] = str(path)
    try:
        h.start(runtime=str(runtime), external_joint_states=True, **arguments)
        controller = "ackermann_controller" if kind == "ackermann" else "omni_controller"
        client = h.node.create_client(ListControllers, "/controller_manager/list_controllers")
        assert client.wait_for_service(timeout_sec=30)
        def active():
            future = client.call_async(ListControllers.Request()); h.until(future.done)
            return {c.name for c in future.result().controller if c.state == "active"} == {controller, "joint_state_broadcaster"}
        h.until(active, timeout=40)
        h.node.destroy_client(client)
        h.until(lambda: h.messages["joints"] and h.messages["odom"] and h.messages["truth"])
    except BaseException:
        h.close(); raise
    return h


def move(h, x, y, w, steps, identity):
    publisher = h.node.create_publisher(Twist, "/cmd_vel", 1)
    h.until(lambda: publisher.get_subscription_count() == 1)
    command = Twist(); command.linear.x = float(x); command.linear.y = float(y); command.angular.z = float(w)
    start = stamp(h.messages["clock"][-1].clock)
    deadline = time.monotonic() + 30
    while stamp(h.messages["clock"][-1].clock) - start < steps * 10_000_000:
        assert time.monotonic() < deadline, "simulation stopped progressing"
        publisher.publish(command); h.spin(.04)
    h.node.destroy_publisher(publisher)
    # Truth and encoder odometry are independently published at the same sim stamps.
    def paired():
        truth = {stamp(m.header.stamp): m for m in h.messages["truth"]}
        odom = {stamp(m.header.stamp): m for m in h.messages["odom"]}
        common = truth.keys() & odom.keys()
        return (truth[max(common)], odom[max(common)]) if common else None
    h.until(lambda: paired() is not None)
    return paired()


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
        h.until(lambda: h.messages["sim_model"])
        raw_xml = h.messages["sim_model"][-1].data
        assert hashlib.sha256(raw_xml.encode()).hexdigest() == scene["robot_model"]["sha256"]
        # RSP's XML may be serialized differently and uses file:// URLs for
        # RViz. Verify the full model contents after normalizing only those URLs.
        normalized = ET.fromstring(xml)
        for element in normalized.findall(".//mesh") + normalized.findall(".//texture"):
            filename = element.get("filename", "")
            if filename.startswith("file://"):
                element.set("filename", unquote(urlsplit(filename).path))
        expected_model = ET.fromstring(raw_xml)
        # Simulation plugin declarations and root metadata are intentionally absent from RSP.
        for plugin in expected_model.findall("litesim3d"):
            expected_model.remove(plugin)
        expected_model.attrib = normalized.attrib
        assert ET.canonicalize(ET.tostring(normalized, encoding="unicode"), strip_text=True) == \
            ET.canonicalize(ET.tostring(expected_model, encoding="unicode"), strip_text=True)
        robot = ET.fromstring(xml)
        assert not any("xacro" in el.tag for el in robot.iter())
        assert not robot.findall("litesim3d") and not robot.findall("ros2_control")
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
            _, odom = move(h, 0, .4, 0, 150, "lateral")
            assert abs(odom.twist.twist.linear.y - .4) < 1e-9
        for frame in ("lidar_link", "imu_link", "camera_optical_frame"):
            h.until(lambda: h.buffer.can_transform("base_link", frame, Time()))
            actual = h.buffer.lookup_transform("base_link", frame, Time()).transform.translation
            expected = scene["initial_link_poses"][frame][:3]
            assert max(abs(a-b) for a,b in zip((actual.x,actual.y,actual.z),expected)) < 1e-9
        graph = h.graph_snapshot(["/tf_static", "/tf", "/clock", "/joint_states"])
        (directory / "tf-graph.json").write_text(json.dumps(graph, indent=2))
        assert graph["topics"]["/tf_static"]["publisher_count"] == 1, json.dumps(graph, indent=2)
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


def verify_control_handoff(directory, kind):
    h = setup(directory, kind, wheel_limit=2 if kind.startswith("omni") else None)
    h.subscribe("commands", JointCommand, "/sim/joint_commands", 10)
    try:
        controller = "ackermann_controller" if kind == "ackermann" else "omni_controller"
        manager = "/controller_manager"
        assert {info.node_name for info in h.node.get_subscriptions_info_by_topic("/cmd_vel")} == {controller}
        assert {info.node_name for info in h.node.get_publishers_info_by_topic("/joint_states")} == {"joint_state_broadcaster"}
        move(h, .2, 0, 0, 60, "command")
        h.until(lambda: all(abs(v) < 1e-6 for v in h.messages["joints"][-1].velocity), timeout=5)
        # Stop publishing Twist: controller's timeout stops motion despite a healthy hardware heartbeat.
        h.until(lambda: abs(h.messages["truth"][-1].twist.twist.linear.x) < 1e-6)
        if kind.startswith("omni"):
            # With a deliberately low controller wheel limit, every wheel must
            # stay bounded while saturated body commands keep their direction.
            move(h, 100, 100, 100, 20, "saturated")
            velocities = h.messages["joints"][-1].velocity
            assert max(abs(v) for v in velocities) <= 2.000001
            assert max(abs(v) for v in velocities) > 1.9
            invalid = h.node.create_publisher(Twist, "/cmd_vel", 1)
            h.until(lambda: invalid.get_subscription_count() == 1)
            message = Twist(); message.linear.x = float("nan")
            invalid.publish(message)
            h.until(lambda: all(abs(v) < 1e-6 for v in h.messages["joints"][-1].velocity), timeout=3)
            h.node.destroy_publisher(invalid)
        names = ["wheel_velocity_controller"]
        if kind == "ackermann": names.append("steering_position_controller")
        for name in names:
            process = subprocess.Popen(["ros2", "run", "controller_manager", "spawner", name,
                                        "-c", manager, "--inactive"], env=h.environment,
                                       stdout=h.log, stderr=subprocess.STDOUT)
            try:
                h.until(lambda: process.poll() is not None, timeout=30)
                assert process.returncode == 0
            finally:
                if process.poll() is None: process.terminate(); process.wait(timeout=5)
        client = h.node.create_client(SwitchController, manager + "/switch_controller")
        assert client.wait_for_service(timeout_sec=5)
        def switch(start, stop, expected):
            req = SwitchController.Request(activate_controllers=start, deactivate_controllers=stop,
                                           strictness=SwitchController.Request.STRICT)
            req.timeout.sec = 3
            future = client.call_async(req); h.until(future.done)
            assert future.result().ok == expected
        switch(names, [], False)  # Controller manager rejects conflicting wheel ownership.
        switch(names, [controller], True)
        velocity = h.node.create_publisher(Float64MultiArray, "/wheel_velocity_controller/commands", 1)
        h.until(lambda: velocity.get_subscription_count() == 1)
        if kind == "ackermann":
            steering = h.node.create_publisher(Float64MultiArray, "/steering_position_controller/commands", 1)
            h.until(lambda: steering.get_subscription_count() == 1)
            steering.publish(Float64MultiArray(data=[.3, .25]))
            rates = [2.0, 2.0]
        else:
            wheels = yaml.safe_load((h.share / "config/geometry.yaml").read_text())["wheels"]
            rates = [.2 * math.sin(w["rolling_angle_rad"]) / w["radius_m"] for w in wheels]
        velocity.publish(Float64MultiArray(data=rates))
        h.until(lambda: (h.messages["truth"][-1].twist.twist.angular.z > .05 if kind == "ackermann"
                        else h.messages["truth"][-1].twist.twist.linear.y > .15))
        if kind == "ackermann":
            positions = dict(zip(h.messages["joints"][-1].name, h.messages["joints"][-1].position))
            h.until(lambda: abs(dict(zip(h.messages["joints"][-1].name, h.messages["joints"][-1].position))["left_steering_joint"]-.3) < 1e-5)
            assert positions["front_left_wheel_joint"] != 0  # State-only passive wheel also rolls.
        switch([controller], names, True)
        h.until(lambda: all(abs(v) < 1e-6 for v in h.messages["joints"][-1].velocity))
        # Same-interface handoff must not replay the previous wheel speed.
        h.spin(.2)
        assert all(abs(v) < 1e-6 for v in h.messages["joints"][-1].velocity)
        h.node.destroy_client(client)
    finally:
        (directory / "control-evidence.json").write_text(json.dumps({
            key: message_to_ordereddict(h.messages[key][-1]) if h.messages[key] else None
            for key in ("truth", "odom", "joints", "commands", "status")}, indent=2))
        h.close()


def verify_controller_geometry(kind):
    from ament_index_python.packages import get_package_share_directory
    from pathlib import Path
    import xacro
    share = Path(get_package_share_directory(kind + "_sim"))
    geometry = yaml.safe_load((share / "config/geometry.yaml").read_text())
    control = yaml.safe_load((share / "config/controllers.yaml").read_text())
    xml = ET.fromstring(xacro.process_file(str(share / "urdf/control.urdf.xacro")).toxml())
    interfaces = {j.get("name"): {i.get("name") for i in j.findall("command_interface")}
                  for j in xml.findall("./ros2_control/joint")}
    if kind == "ackermann":
        g = geometry["common"]; c = control["/ackermann_controller"]["ros__parameters"]
        assert c["rear_wheels_names"] == [g["rear_right_joint"], g["rear_left_joint"]]
        assert c["front_wheels_names"] == [g["right_steering_joint"], g["left_steering_joint"]]
        assert c["wheelbase"] == g["wheelbase_m"]
        assert c["front_wheel_track"] == c["rear_wheel_track"] == g["track_width_m"]
        assert c["front_wheels_radius"] == c["rear_wheels_radius"] == g["wheel_radius_m"]
        for name in geometry["drive"].values(): assert not interfaces[name]
    else:
        c = control["/omni_controller"]["ros__parameters"]; wheels = geometry["wheels"]
        for key, source in [("wheel_joints", "joint"), ("wheel_x", "x_m"), ("wheel_y", "y_m"),
                            ("wheel_radius", "radius_m"), ("wheel_angle", "rolling_angle_rad")]:
            assert c[key] == [w[source] for w in wheels]
        for name, limit in zip(c["wheel_joints"], c["max_wheel_velocity"]):
            joint = xml.find("./joint[@name='" + name + "']")
            assert 0 < limit <= float(joint.find("limit").get("velocity"))
            assert "velocity" in interfaces[name]
