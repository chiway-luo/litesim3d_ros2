"""M1 integration tests. Every assertion uses real ROS 2 messages/processes."""
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import xml.etree.ElementTree as ET

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, JointState
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_msgs.msg import TFMessage
from litesim3d_interfaces.srv import Step, SetPose
import yaml


def ns(msg):
    return msg.sec * 1_000_000_000 + msg.nanosec


class Harness:
    serial = 0

    def __init__(self, directory):
        self.directory = directory
        directory.mkdir(parents=True, exist_ok=True)
        Harness.serial += 1
        # Keep the default pool below Linux's ephemeral-port overlap, and
        # separate it from the example harness's 60..99 pool.
        os.environ["ROS_DOMAIN_ID"] = str(int(os.environ.get("LWS_TEST_DOMAIN_BASE", "10")) + Harness.serial % 50)
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        os.environ["ROS_LOG_DIR"] = str(directory / "ros_logs")
        self.env = os.environ.copy()
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node("m1_test", context=self.context, parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.processes = []
        self.files = []
        self.messages = {}
        self.subs = []
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node, spin_thread=False)
        self.share = Path(get_package_share_directory("litesim3d_core"))
        self.config = yaml.safe_load((self.share / "config/demo.yaml").read_text())
        self.config["robot"]["urdf"] = str(self.share / "urdf/r01.urdf")
        self.config["lidars"][0]["columns"] = 32
        self.config["lidars"][0]["vertical_angles_deg"] = [-15., -5., 5., 15.]
        self.config["lidars"][0]["rings"] = 4
        self.config["simulation"]["real_time_factor"] = 1.0
        self.subscribe("clock", Clock, "/clock", reliable=False)
        self.subscribe("status", DiagnosticArray, "/sim/status")
        self.subscribe("tf", TFMessage, "/tf")

    def subscribe(self, key, msg_type, topic, reliable=True, durable=False):
        self.messages[key] = []
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL if durable else DurabilityPolicy.VOLATILE)
        self.subs.append(self.node.create_subscription(msg_type, topic, lambda m: self.messages[key].append(m), qos))

    def process(self, name, command):
        f = (self.directory / f"{name}.log").open("w")
        self.files.append(f)
        p = subprocess.Popen(command, env=self.env, stdout=f, stderr=subprocess.STDOUT, cwd=self.directory)
        self.processes.append((name, p))
        return p

    def rsp(self, namespace="", frame_prefix="", use_sim_time=True):
        robot = Path(self.config["robot"]["urdf"]).read_text()
        params = self.directory / "rsp.yaml"
        params.write_text(yaml.safe_dump({"/**": {"ros__parameters": {"robot_description": robot,
                          "use_sim_time": use_sim_time, "frame_prefix": frame_prefix, "publish_frequency": 200.0}}}))
        return self.process("rsp", [str(Path(get_package_prefix("robot_state_publisher")) / "lib/robot_state_publisher/robot_state_publisher"),
                                    "--ros-args", "--params-file", str(params), "-r", f"__ns:=/{namespace}"])

    def sim(self, namespace="", **parameters):
        config = self.directory / "config.yaml"
        config.write_text(yaml.safe_dump(self.config))
        settings = {"config": str(config), "use_sim_time": True, "manifest_path": str(self.directory / "manifest.json"), **parameters}
        params = self.directory / "sim.yaml"
        params.write_text(yaml.safe_dump({"/**": {"ros__parameters": settings}}))
        return self.process("sim", [str(Path(get_package_prefix("litesim3d_ros2")) / "lib/litesim3d_ros2/sim_node"),
                                    "--ros-args", "--params-file", str(params), "-r", f"__ns:=/{namespace}"])

    def spin(self, duration=.1):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            # A changing timeout restarts Humble's ready-callback iterator and
            # can starve later subscriptions behind high-rate clock/TF data.
            self.executor.spin_once(timeout_sec=.02)

    def until(self, predicate, timeout=6):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return
            self.spin(.02)
        logs = "\n".join(p.read_text()[-3000:] for p in self.directory.glob("*.log"))
        assert predicate(), f"condition timeout\n{logs}"

    def status(self, robot_id=None):
        if not self.messages["status"]:
            return {}
        for status in self.messages["status"][-1].status:
            values = {v.key: v.value for v in status.values}
            if robot_id is None or values.get("robot_id") == robot_id:
                return values
        return {}

    def publish_paused_command(self, publisher, message, robot_id=None):
        """One control source: observe acceptance before requesting a step."""
        def idle():
            status = self.status(robot_id)
            return status.get("state") == "paused" and status.get("pending_command") == "false"
        self.until(idle)
        run_id = self.status(robot_id)["run_id"]
        publisher.publish(message)
        self.until(lambda: self.status(robot_id).get("run_id") == run_id
                   and self.status(robot_id).get("pending_command") == "true")

    def wait_for_stamp(self, stamp, *keys):
        """Topics and service responses have independent delivery order."""
        def observed(key):
            if not self.messages[key]:
                return False
            message = self.messages[key][-1]
            return ns(message.clock if isinstance(message, Clock) else message.header.stamp) == stamp
        self.until(lambda: all(observed(key) for key in keys))

    def ready(self):
        self.until(lambda: self.status().get("ready") == "true")

    def call(self, kind, service, request, timeout=6):
        client = self.node.create_client(kind, service)
        assert client.wait_for_service(timeout_sec=4), service
        future = client.call_async(request)
        self.until(future.done, timeout=timeout)
        result = future.result()
        self.node.destroy_client(client)
        return result

    def pause(self, value=True):
        request = SetBool.Request(); request.data = value
        assert self.call(SetBool, "/sim/pause", request).success

    def step(self, count, request_id="test"):
        request = Step.Request(); request.steps=count; request.request_id=request_id
        return self.call(Step, "/sim/step", request)

    def close(self):
        records = {}
        for name,p in reversed(self.processes):
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    p.kill(); p.wait(); records[name+"_forced_kill"] = True
            records[name] = p.returncode
        for f in self.files:
            f.close()
        graph = self.node.get_topic_names_and_types()
        (self.directory / "graph.json").write_text(json.dumps(graph, indent=2))
        (self.directory / "processes.json").write_text(json.dumps(records, indent=2))
        (self.directory / "observations.json").write_text(json.dumps({"counts": {k:len(v) for k,v in self.messages.items()}, "status": self.status()}, indent=2))
        self.listener.unregister()
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()
        assert not any(k.endswith("forced_kill") for k in records), records


@pytest.fixture
def h(tmp_path):
    instance = Harness(tmp_path)
    yield instance
    instance.close()


@pytest.mark.parametrize("mode", ["twist", "stamped"])
def test_T_ROS_01_topics_and_commands(h, mode):
    h.subscribe("odom", Odometry, "/robot/odom")
    h.subscribe("points", PointCloud2, "/robot/points_raw", reliable=False)
    h.rsp("robot", "robot/"); h.sim("robot", frame_prefix="robot/", cmd_mode=mode); h.ready()
    kind = Twist if mode == "twist" else TwistStamped
    publisher = h.node.create_publisher(kind, "/robot/cmd_vel", 1)
    h.until(lambda: publisher.get_subscription_count() == 1)
    if mode == "stamped":
        # Status and clock callbacks have independent delivery order.
        h.until(lambda: bool(h.messages["clock"]))
    for _ in range(25):
        command = kind()
        if mode == "twist":
            command.linear.x = .5
        else:
            command.header.frame_id = "robot/base_link"
            command.header.stamp = h.messages["clock"][-1].clock
            command.twist.linear.x = .5
        publisher.publish(command); h.spin(.03)
    h.until(lambda: len(h.messages["odom"]) > 10 and len(h.messages["points"]) > 2)
    assert h.messages["odom"][-1].pose.pose.position.x > .02
    assert h.messages["points"][-1].header.frame_id == "robot/lidar_link"
    graph = dict(h.node.get_topic_names_and_types())
    assert graph["/robot/cmd_vel"] == ["geometry_msgs/msg/" + ("Twist" if mode == "twist" else "TwistStamped")]
    assert graph["/clock"] == ["rosgraph_msgs/msg/Clock"]


def test_T_ROS_02_03_time_tf_and_point_bytes(h):
    h.config["lidars"][0]["columns"] = 1024
    h.config["lidars"][0]["vertical_angles_deg"] = [-15+30*i/31 for i in range(32)]
    h.config["lidars"][0]["rings"] = 32
    h.subscribe("points", PointCloud2, "/points_raw", reliable=False)
    h.rsp();h.sim();h.ready();h.until(lambda:len(h.messages["points"]) >= 5)
    assert h.node.count_publishers("/clock") == 1
    clouds = list(h.messages["points"])
    clocks = [ns(c.clock) for c in h.messages["clock"]]
    assert all(a <= b for a,b in zip(clocks,clocks[1:]))
    for c in clouds:
        h.until(lambda: h.buffer.can_transform("odom", c.header.frame_id, Time.from_msg(c.header.stamp)), timeout=.5)
        tf = h.buffer.lookup_transform("odom",c.header.frame_id,Time.from_msg(c.header.stamp))
        assert tf.transform.translation.x == pytest.approx(.2, abs=1e-6)
        assert tf.transform.translation.z == pytest.approx(.65, abs=1e-6)
        assert ns(c.header.stamp) <= max(clocks)
        assert c.point_step == 24 and c.row_step == c.width*24
        assert c.height == 32 and c.width == 1024 and len(c.data) == 32768*24
        assert [(f.name,f.offset,f.datatype,f.count) for f in c.fields] == [("x",0,7,1),("y",4,7,1),("z",8,7,1),("intensity",12,7,1),("ring",16,4,1),("time",20,7,1)]
        assert not c.is_bigendian
        valid=0
        for index,(x,y,z,intensity,ring,t) in enumerate(struct.iter_unpack("<ffffH2xf",bytes(c.data))):
            assert ring == index//c.width and t == 0
            assert bytes(c.data[index*24+18:index*24+20]) == bytes(2)
            if math.isfinite(x):
                valid += 1
                assert 0 <= intensity <= 1 and math.isfinite(y) and math.isfinite(z)
            else:
                assert math.isnan(y) and math.isnan(z) and intensity == 0
        assert valid > 100 and c.is_dense == (valid == 32768)
    (h.directory/"cloud.bin").write_bytes(bytes(clouds[-1].data))
    assert h.node.get_parameter("use_sim_time").value is True
    h.node.set_parameters([Parameter("use_sim_time", value=False)])
    assert abs(h.node.get_clock().now().nanoseconds-max(clocks)) > 1_000_000_000


def test_T_URD_19_joint_tf(h):
    h.subscribe("joints",JointState,"/joint_states")
    h.rsp();h.sim();h.ready()
    publisher=h.node.create_publisher(Twist,"/cmd_vel",1)
    h.until(lambda:publisher.get_subscription_count()==1)
    c=Twist();c.linear.x=.4;c.angular.z=.2
    for _ in range(15): publisher.publish(c);h.spin(.03)
    states=[s for s in h.messages["joints"] if s.position and max(s.position)>.1]
    assert states
    for s in states[-10:]:
        for joint,link in [("left_wheel_joint","left_wheel"),("right_wheel_joint","right_wheel")]:
            angle=s.position[s.name.index(joint)]
            h.until(lambda:h.buffer.can_transform("base_link",link,Time.from_msg(s.header.stamp)),timeout=.5)
            t=h.buffer.lookup_transform("base_link",link,Time.from_msg(s.header.stamp))
            assert abs(t.transform.rotation.y) == pytest.approx(abs(math.sin(angle/2)),abs=1e-6)
            assert t.transform.translation.y == pytest.approx(.25 if link.startswith("left") else -.25,abs=1e-6)


@pytest.mark.parametrize("footprint",[False,True])
def test_T_URD_21_root_frames(h,footprint):
    if footprint:h.config["robot"]["urdf"]=str(h.share/"urdf/r01_footprint.urdf")
    h.rsp();h.sim();h.ready();h.until(lambda:len(h.messages["tf"])>10)
    root="base_footprint" if footprint else "base_link"
    edges={(t.header.frame_id,t.child_frame_id) for m in h.messages["tf"] for t in m.transforms}
    assert ("odom",root) in edges
    if footprint:assert ("odom","base_link") not in edges
    sample=h.messages["clock"][-1].clock
    h.until(lambda:h.buffer.can_transform("odom","lidar_link",Time.from_msg(sample)),timeout=.5)
    t=h.buffer.lookup_transform("odom","lidar_link",Time.from_msg(sample))
    assert t.transform.translation.z == pytest.approx(.65,abs=1e-6)


def test_T_ROS_04_duplicate_tf_diagnostic(h):
    h.rsp();h.sim();h.ready();h.until(lambda:len(h.messages["tf"])>5)
    broadcaster=TransformBroadcaster(h.node)
    for _ in range(20):
        t=TransformStamped();t.header.frame_id="odom";t.child_frame_id="base_link";t.header.stamp=h.messages["clock"][-1].clock;t.transform.rotation.w=1.
        broadcaster.sendTransform(t);h.spin(.03)
    h.until(lambda:h.status().get("state")=="fault")
    assert "duplicate" in h.status()["last_error"]


@pytest.mark.parametrize("reliable,subscriber,compatible",[(False,False,True),(True,True,True),(False,True,False)])
def test_T_ROS_05_qos(h,reliable,subscriber,compatible):
    h.subscribe("points",PointCloud2,"/points_raw",reliable=subscriber)
    h.rsp();h.sim(sensor_reliable=reliable);h.ready();h.spin(.8)
    if compatible:
        assert len(h.messages["points"]) >= 3
        times=[ns(c.header.stamp) for c in h.messages["points"]]
        assert all(b-a==100_000_000 for a,b in zip(times,times[1:]))
    else:
        assert not h.messages["points"]
        h.until(lambda:int(h.status().get("qos_conflicts","0"))>0)


def test_T_ROS_06_late_subscriber(h):
    h.rsp();h.sim();h.ready();h.spin(.5)
    h.subscribe("late_static",TFMessage,"/tf_static",durable=True)
    h.subscribe("description",String,"/robot_description",durable=True)
    h.until(lambda:bool(h.messages["late_static"]) and bool(h.messages["description"]))
    assert any(t.child_frame_id=="lidar_link" for m in h.messages["late_static"] for t in m.transforms)
    assert "left_wheel_joint" in h.messages["description"][-1].data


def test_T_ROS_08_pause_step_service(h):
    h.subscribe("points",PointCloud2,"/points_raw",reliable=False)
    h.rsp();h.sim();h.ready();h.until(lambda:len(h.messages["points"])>2);h.pause();h.spin(.2)
    count=len(h.messages["points"]);before=ns(h.messages["clock"][-1].clock)
    h.spin(.3)
    assert len(h.messages["points"])==count and ns(h.messages["clock"][-1].clock)==before
    result=h.step(10,"idempotency");assert result.success and result.time_ns==before+50_000_000
    again=h.step(10,"idempotency");assert again.success and again.time_ns==result.time_ns
    assert not h.step(11,"idempotency").success
    h.pause(False);h.until(lambda:ns(h.messages["clock"][-1].clock)>result.time_ns)
    assert not h.step(1,"running").success


@pytest.mark.parametrize("delayed",[False,True])
def test_T_ROS_10_startup_order(h,delayed):
    if delayed:
        # Observe the initialized, waiting node before starting its dependency.
        # Use the normal startup budget; the separate timeout test exercises
        # deadline failure without relying on a short DDS discovery window.
        process=h.sim()
        h.until(lambda:bool(h.status()))
        assert process.poll() is None and h.status()["ready"]=="false"
        assert h.status()["step"]=="0" and h.status()["time_ns"]=="1"
        h.rsp()
    else:
        h.rsp();h.spin(.3);h.sim()
    h.ready();h.until(lambda:len(h.messages["clock"])>10)


def test_T_ROS_10_timeout(h):
    process=h.sim(startup_timeout=.3);h.until(lambda:h.status().get("state")=="fault")
    assert "readiness timeout" in h.status()["last_error"]
    # The one initial best-effort clock can precede DDS discovery. Reliable
    # diagnostics and the exit manifest independently prove no step occurred.
    assert h.status()["step"] == "0" and h.status()["time_ns"] == "1"
    assert ns(h.messages["status"][-1].header.stamp) == 1
    assert all(ns(c.clock) == 1 for c in h.messages["clock"])
    h.until(lambda: process.poll() is not None, timeout=3)
    assert process.returncode == 1
    final=json.loads((h.directory/"manifest.json").read_text())
    assert final["status"]["state"] == "fault"
    assert final["status"]["step"] == 0 and final["status"]["time_ns"] == 1


def test_T_ROS_11_diagnostics(h):
    h.rsp();h.sim(test_drop_frames=1);h.ready();p=h.node.create_publisher(Twist,"/cmd_vel",1)
    h.until(lambda:p.get_subscription_count()==1)
    command=Twist();command.linear.y=1.;p.publish(command)
    h.until(lambda:int(h.status().get("rejected_commands","0"))>0)
    h.until(lambda:h.status().get("command_timeout")=="true")
    status=h.messages["status"][-1].status[0]
    level = status.level[0] if isinstance(status.level,bytes) else status.level
    assert status.hardware_id == h.status()["run_id"] and status.hardware_id.startswith("m2-") and level>=1
    assert h.status()["robot_id"]=="robot_0"
    assert h.status()["dropped_frames"]=="1"


def test_T_ODO_06_external_tf(h):
    h.subscribe("points",PointCloud2,"/points_raw",reliable=False)
    h.subscribe("odom",Odometry,"/odom")
    h.rsp();h.sim(publish_odom_tf=False,external_odom_tf=True);h.ready()
    h.until(lambda:len(h.messages["odom"])>10)
    assert not any(t.header.frame_id=="odom" for m in h.messages["tf"] for t in m.transforms)
    broadcaster=TransformBroadcaster(h.node)
    for _ in range(10):
        t=TransformStamped();t.header.frame_id="odom";t.child_frame_id="base_link";t.header.stamp=h.messages["clock"][-1].clock
        t.transform.translation.x=9.;t.transform.rotation.w=1.;broadcaster.sendTransform(t);h.spin(.03)
    assert h.messages["points"] and abs(h.messages["odom"][-1].pose.pose.position.x)<1e-9
    assert h.status()["state"]!="fault"


def test_T_DEL_02_03_installed_headless(h):
    h.env.pop("DISPLAY",None)
    h.rsp();h.sim();h.ready()
    h.until(lambda:bool(h.messages["clock"]) and ns(h.messages["clock"][-1].clock)>300_000_000)
    manifest=json.loads((h.directory/"manifest.json").read_text())
    assert manifest["model"]["source"].startswith(str(h.share))
    assert manifest["model"]["resources"]=={}  # R01 has no external mesh files.
    assert h.status()["state"]=="running"


@pytest.mark.parametrize("angle",[-.5,0.,.3])
def test_T_URD_19_articulated_tf(h,angle):
    root=ET.parse(h.share/"urdf/r01.urdf").getroot()
    joints=ET.parse(h.share/"urdf/r03.urdf").getroot()
    for element in joints:
        if element.tag=="link" and element.get("name")=="base_link":continue
        root.append(element)
    urdf=h.directory/"combined.urdf";urdf.write_text(ET.tostring(root,encoding="unicode"))
    h.config["robot"]["urdf"]=str(urdf)
    h.config["robot"]["initial_joints"]={"main":angle,"slide":.25}
    h.config["entities"]=[]  # R03 is a free kinematic fixture, not a supported ground robot.
    h.rsp();h.sim();h.ready();h.until(lambda:len(h.messages["clock"])>15)
    sample=h.messages["clock"][-1].clock
    for child in ("arm","slider","follower","chain","tip"):
        h.until(lambda:h.buffer.can_transform("base_link",child,Time.from_msg(sample)),timeout=.5)
    t=h.buffer.lookup_transform("base_link","tip",Time.from_msg(sample)).transform
    a=1/math.sqrt(3);v=[.25,0,.2];cross=[a*(v[2]-v[1]),a*(v[0]-v[2]),a*(v[1]-v[0])]
    expected=[v[i]*math.cos(angle)+cross[i]*math.sin(angle)+a*a*sum(v)*(1-math.cos(angle))+[.2,0,.5][i] for i in range(3)]
    assert [t.translation.x,t.translation.y,t.translation.z]==pytest.approx(expected,abs=1e-6)
    follower=h.buffer.lookup_transform("base_link","follower",Time.from_msg(sample)).transform
    assert abs(follower.rotation.y)==pytest.approx(abs(math.sin((2*angle+.1)/2)),abs=1e-6)


def test_T_ODO_04_ros_covariance(h):
    covariance=[.1 if i%7==0 else 0. for i in range(36)];covariance[1]=covariance[6]=.01
    h.config["odom"]={"mode":"ideal","pose_covariance":covariance,"twist_covariance":covariance}
    h.subscribe("odom",Odometry,"/odom");h.rsp();h.sim();h.ready();h.until(lambda:len(h.messages["odom"])>5)
    assert list(h.messages["odom"][-1].pose.covariance)==covariance
    assert list(h.messages["odom"][-1].twist.covariance)==covariance


def test_T_ROS_04_invalid_responsibility_config(h):
    p=h.sim(publish_odom_tf=True,external_odom_tf=True)
    h.until(lambda:p.poll() is not None)
    assert p.returncode!=0
    assert "conflicting odom TF" in (h.directory/"sim.log").read_text()
