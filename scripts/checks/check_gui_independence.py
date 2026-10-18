#!/usr/bin/env python3
"""T-COR-01: compare real ROS output with RViz off/on over 60 simulated seconds.

Requires an accessible desktop session. Uses installed binaries and saves real
messages, hashes, process exits and JUnit; never treats a GUI startup failure as PASS.
"""
import hashlib
import json
import os
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paths import REPO_ROOT, EVIDENCE_DIR
import struct
import subprocess
import xml.etree.ElementTree as ET

ROOT = REPO_ROOT
sys.path.insert(0, str(ROOT / "litesim3d_ros2/test"))
from test_ros import Harness, Twist, Odometry, PointCloud2, ns
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.serialization import serialize_message
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict


def cdr_padding(data, label):
    """Locate CDR1 alignment gaps, never message fields or point data bytes."""
    assert data[:2] in (b"\x00\x00", b"\x00\x01"), "Expected CDR1 encapsulation"
    endian = "<" if data[1] == 1 else ">"
    position, padding = 4, []

    def align(width):
        nonlocal position
        aligned = 4 + ((position - 4 + width - 1) // width) * width
        padding.extend(range(position, aligned))
        position = aligned

    def u32():
        nonlocal position
        align(4)
        value = struct.unpack_from(endian + "I", data, position)[0]
        position += 4
        return value

    def string():
        nonlocal position
        size = u32()
        assert size > 0 and data[position + size - 1] == 0
        position += size

    u32(); u32()  # Header stamp.
    string()  # Header frame_id.
    if label == "points":
        u32(); u32()  # height, width.
        for _ in range(u32()):
            string(); u32()  # PointField name, offset.
            position += 1  # datatype.
            u32()  # count.
        position += 1  # is_bigendian.
        u32(); u32()  # point_step, row_step.
        size = u32()
        position += size + 1  # Opaque data bytes and is_dense.
    else:
        assert label == "odom"
        string()  # child_frame_id.
        align(8)
        # Pose (7), covariance (36), twist (6), covariance (36): all float64.
        position += (7 + 36 + 6 + 36) * 8
    assert position == len(data), (position, len(data))
    return padding


def message_records(directory, label, messages):
    records = {"fields": [], "cdr": [], "cdr_without_alignment_gaps": []}
    if label == "points":
        records["payload_bytes"] = []
    for index, message in enumerate(messages):
        fields = json.dumps(message_to_ordereddict(message), sort_keys=True,
                            separators=(",", ":"), allow_nan=False).encode()
        wire = serialize_message(message)
        normalized = bytearray(wire)
        for offset in cdr_padding(wire, label):
            normalized[offset] = 0
        if label == "points":
            records["payload_bytes"].append(hashlib.sha256(bytes(message.data)).hexdigest())
        records["fields"].append(hashlib.sha256(fields).hexdigest())
        records["cdr"].append(hashlib.sha256(wire).hexdigest())
        records["cdr_without_alignment_gaps"].append(hashlib.sha256(normalized).hexdigest())
        if index == 0:
            (directory / f"first-{label}.cdr").write_bytes(wire)
    return records


def close_test_window(process):
    """Use Qt's normal close path on the explicitly isolated Xvfb display.

    Humble RViz can segfault when SIGINT shuts down its ROS context while Qt
    is still processing display callbacks. A WM_DELETE_WINDOW event lets Qt
    finish first. Never manipulate windows on a user's desktop here.
    """
    assert sys.platform == "linux" and os.environ.get("LWS_PRIVATE_TEST_DISPLAY") == "1"
    from Xlib import X, display, protocol

    connection = display.Display()
    try:
        pid_atom = connection.intern_atom("_NET_WM_PID")
        protocols = connection.intern_atom("WM_PROTOCOLS")
        delete = connection.intern_atom("WM_DELETE_WINDOW")
        windows = []
        for window in connection.screen().root.query_tree().children:
            pid = window.get_full_property(pid_atom, X.AnyPropertyType)
            supported = window.get_wm_protocols() or []
            if pid is not None and list(pid.value) == [process.pid] and delete in supported:
                windows.append(window)
        assert len(windows) == 1, f"Expected one RViz window for PID {process.pid}, found {len(windows)}"
        windows[0].send_event(protocol.event.ClientMessage(
            window=windows[0], client_type=protocols,
            data=(32, [delete, X.CurrentTime, 0, 0, 0])), event_mask=X.NoEventMask)
        connection.flush()
        assert process.wait(timeout=10) == 0, f"RViz window close exited {process.returncode}"
    finally:
        connection.close()


def run_case(directory, gui):
    h = Harness(directory)
    try:
        h.config["simulation"]["command_timeout"] = 120.
        m2 = os.environ.get("LWS_GUI_M2") == "1"
        if m2:
            h.config["lidars"][0]["scan_mode"] = "rolling"
            h.config["lidars"][0]["range_sigma_m"] = .01
            h.config["lidars"][0]["dropout"] = .01
            h.config["simulation"]["threads"] = 4
            h.config["imus"] = [{"name": "imu", "link": "base_link", "rate_hz": 200}]

        h.subscribe("points", PointCloud2, "/points_raw")
        h.subscribe("odom", Odometry, "/odom")
        h.rsp()
        h.sim(start_paused=True, sensor_reliable=True)
        h.ready()
        if gui:
            exe = Path(get_package_prefix("rviz2")) / "lib/rviz2/rviz2"
            config = Path(get_package_share_directory("litesim3d_ros2")) / "rviz/demo.rviz"
            rviz = h.process("rviz", [str(exe), "-d", str(config), "--ros-args", "-p", "use_sim_time:=true"])
            h.until(lambda: "OpenGl version" in (directory / "rviz.log").read_text(), timeout=15)
            assert rviz.poll() is None
            # A live renderer with a wrong scene topic can still look healthy.
            # Require RViz itself to subscribe to the one-shot scene with durable QoS.
            def scene_subscriptions():
                return [s for s in h.node.get_subscriptions_info_by_topic("/scene")
                        if s.node_name == "rviz"]
            h.until(lambda: bool(scene_subscriptions()))
            assert all(s.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL and
                       s.qos_profile.reliability == ReliabilityPolicy.RELIABLE
                       for s in scene_subscriptions())
            (directory / "rviz-scene-subscription.json").write_text(json.dumps([
                {"node": s.node_name, "topic": "/scene", "type": s.topic_type,
                 "durability": s.qos_profile.durability.name,
                 "reliability": s.qos_profile.reliability.name}
                for s in scene_subscriptions()], indent=2) + "\n")
        publisher = h.node.create_publisher(Twist, "/cmd_vel", 1)
        h.until(lambda: publisher.get_subscription_count() == 1)
        command = Twist()
        command.linear.x = .2
        command.angular.z = .15
        publisher.publish(command)
        h.spin(.2)
        for i in range(600):
            result = h.step(20, f"step-{i}")
            assert result.success and result.time_ns == 1 + (i + 1) * 100_000_000
            h.until(lambda: len(h.messages["points"]) >= i + 1)
        h.until(lambda: len(h.messages["odom"]) >= 3000)
        if gui:
            assert rviz.poll() is None
            # Only capture the disposable Xvfb display explicitly requested by the
            # Linux test runner; never inspect the user's desktop this way.
            if sys.platform == "linux" and os.environ.get("LWS_CAPTURE_TEST_DISPLAY") == "1":
                h.spin(.5)
                subprocess.run([sys.executable, "-c",
                    "from PyQt5.QtWidgets import QApplication; import sys; "
                    "app=QApplication([]); assert app.primaryScreen().grabWindow(0).save(sys.argv[1])",
                    str(directory / "rviz-xvfb.png")], check=True, timeout=10, env=h.env)
        frames = h.messages["points"]
        assert len(frames) == 600
        assert [ns(c.header.stamp) for c in frames] == [1 + i * 100_000_000 for i in (range(600) if m2 else range(1, 601))]
        odom = [o for o in h.messages["odom"] if ns(o.header.stamp) > 1]
        assert len(odom) == 3000
        records = {}
        for label, messages in (("points", frames), ("odom", odom)):
            records[label] = message_records(directory, label, messages)
        (directory / "message-hashes.json").write_text(json.dumps(records, indent=2) + "\n")
        (directory / "last-cloud.bin").write_bytes(bytes(frames[-1].data))
        if gui and sys.platform == "linux" and os.environ.get("LWS_PRIVATE_TEST_DISPLAY") == "1":
            close_test_window(rviz)
    finally:
        h.close()
    exits = json.loads((directory / "processes.json").read_text())
    assert all(code == 0 for code in exits.values()), exits
    return records, json.loads((directory / "manifest.json").read_text())


def main():
    out = Path(os.environ.get("LWS_GUI_EVIDENCE", str(EVIDENCE_DIR / "final/gui-independence"))).resolve()
    out.mkdir(parents=True, exist_ok=True)
    suite = ET.Element("testsuite", name="GUIIndependence", tests="1", failures="0")
    case = ET.SubElement(suite, "testcase", classname="T-COR-01", name="real_rviz_off_on_60s")
    result = {"status": "FAIL", "simulated_seconds_per_run": 60, "gui_visual_inspection": "NOT_RUN"}
    try:
        off, off_manifest = run_case(out / "off", False)
        on, on_manifest = run_case(out / "on", True)
        for label in ("points", "odom"):
            assert off[label]["fields"] == on[label]["fields"], f"ROS {label} fields differ"
            assert off[label]["cdr_without_alignment_gaps"] == on[label]["cdr_without_alignment_gaps"], label
        assert off["points"]["payload_bytes"] == on["points"]["payload_bytes"], "PointCloud2 data bytes differ"
        # Preserve wire-level evidence: Fast-CDR need not initialize alignment gaps.
        samples = {}
        for label in ("points", "odom"):
            first_off = (out / f"off/first-{label}.cdr").read_bytes()
            first_on = (out / f"on/first-{label}.cdr").read_bytes()
            assert len(first_off) == len(first_on)
            padding = cdr_padding(first_off, label)
            differences = [i for i, (a, b) in enumerate(zip(first_off, first_on)) if a != b]
            assert set(differences).issubset(padding), (label, differences)
            samples[label] = {"differing_offsets": differences, "cdr_alignment_offsets": padding}
        (out / "cdr-comparison.json").write_text(json.dumps({
            "first_message_samples": samples,
            "all_message_cdr_equal_except_alignment_gaps": True,
            "point_data_bytes_including_reserved_bytes_exactly_equal": True,
            "wire_hash_difference_counts": {label: sum(a != b for a, b in zip(off[label]["cdr"], on[label]["cdr"]))
                                            for label in ("points", "odom")}}, indent=2) + "\n")
        assert off_manifest["final_pose"] == on_manifest["final_pose"]
        assert off_manifest["inputs"] == on_manifest["inputs"]
        result.update(status="PASS", clouds_per_run=600, odom_per_run=3000,
                      equal_message_fields=True, equal_point_data_bytes=True,
                      equal_cdr_except_alignment_gaps=True, equal_final_pose=True,
                      equal_command_application=True)
    except Exception as exc:
        result["error"] = repr(exc)
        suite.set("failures", "1")
        ET.SubElement(case, "failure", message=repr(exc))
        raise
    finally:
        (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        ET.ElementTree(suite).write(out / "result.xml", encoding="utf-8", xml_declaration=True)
        print(json.dumps(result))


if __name__ == "__main__":
    main()
