#!/usr/bin/env python3
"""Maintainer check of the installed dual MID-360 example; motion requires the explicit --drive flag."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def rotate(quaternion, vector):
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return [vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drive", action="store_true", help="send 0.12 m/s and 0.08 rad/s for 8 wall seconds")
    parser.add_argument("--interface", choices=["livox", "native"], default="livox")
    parser.add_argument("--timeout", type=float, default=30)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    rclpy.init()
    node = rclpy.create_node("dual_mid360_check",
        parameter_overrides=[Parameter("use_sim_time", value=True)])
    counts, latest, transforms, first = Counter(), {}, {}, {}
    wheel_bottoms = []
    truth_path = []
    report = {"success": False, "drive": args.drive, "interface": args.interface}
    command = node.create_publisher(Twist, "/cmd_vel", 10) if args.drive else None

    def receive(key, message):
        counts[key] += 1
        latest[key] = message

    def scene(message):
        data = json.loads(message.data)
        poses = data["link_poses"]
        latest["scene"] = data
        counts["scene"] += 1
        for name in ("front_left", "front_right", "rear_left", "rear_right"):
            wheel_bottoms.append(poses[name + "_wheel_link"][2] - 0.15)
        if "start" in first:
            truth_path.append(poses["base_link"][:3])

    def tf(message):
        for item in message.transforms:
            transforms[item.child_frame_id] = item

    def cloud(side, msg):
        receive(side, msg)
        fields = {f.name: f.offset for f in msg.fields}
        finite = 0
        if all(key in fields for key in ("x", "y", "z")):
            endian = ">" if msg.is_bigendian else "<"
            # Check actual returns, including organized native clouds with NaN slots.
            for i in range(0, msg.width * msg.height, 17):
                offset = (i // msg.width) * msg.row_step + (i % msg.width) * msg.point_step
                values = [struct.unpack_from(endian + "f", msg.data, offset + fields[k])[0]
                          for k in ("x", "y", "z")]
                finite += all(math.isfinite(value) for value in values)
        latest[side + "_finite"] = finite

    node.create_subscription(Clock, "/clock", lambda m: receive("clock", m), qos_profile_sensor_data)
    node.create_subscription(Odometry, "/odom", lambda m: receive("odom", m), qos_profile_sensor_data)
    node.create_subscription(Imu, "/imu/data_raw", lambda m: receive("imu", m), qos_profile_sensor_data)
    node.create_subscription(String, "/sim/scene_state", scene, qos_profile_sensor_data)
    node.create_subscription(TFMessage, "/tf", tf, qos_profile_sensor_data)
    node.create_subscription(TFMessage, "/tf_static", tf,
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for side in ("front", "rear"):
        if args.interface == "livox":
            from livox_ros_driver2.msg import CustomMsg
            stem = "/livox/" + side + "_mid360/lidar"
            node.create_subscription(CustomMsg, stem,
                lambda m, key=side + "_custom": receive(key, m), qos_profile_sensor_data)
            topic = stem + "/pointcloud"
        else:
            topic = "/mid360/" + side + "/points"
        node.create_subscription(PointCloud2, topic, lambda m, key=side: cloud(key, m), qos_profile_sensor_data)

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        required = ["clock", "odom", "imu", "scene", "front", "rear"]
        if args.interface == "livox":
            required += ["front_custom", "rear_custom"]
        deadline = time.monotonic() + args.timeout
        while not all(counts[k] >= 2 for k in required) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not all(counts[k] >= 2 for k in required):
            raise RuntimeError("missing streams: " + ", ".join(k for k in required if counts[k] < 2))
        first["start"] = latest["scene"]["link_poses"]["base_link"][:3]
        clock_start = latest["clock"].clock
        if command:
            end = time.monotonic() + 8
            while time.monotonic() < end:
                twist = Twist()
                twist.linear.x, twist.angular.z = 0.12, 0.08
                command.publish(twist)
                counts["commands"] += 1
                spin(0.1)
        else:
            spin(2)
        if command:
            for _ in range(6):
                command.publish(Twist())
                spin(0.1)
        clock_end = latest["clock"].clock
        last = latest["scene"]["link_poses"]["base_link"][:3]
        displacement = math.dist(first["start"], last)
        chain = {}
        for side in ("front", "rear"):
            current = side + "_mid360_link"
            seen = set()
            while current != "base_link" and current in transforms and current not in seen:
                seen.add(current)
                current = transforms[current].header.frame_id
            chain[side] = current == "base_link"
        poses = latest["scene"]["link_poses"]
        base = poses["base_link"]
        inverse = [-v for v in base[3:6]] + [base[6]]
        axes = {s: rotate(inverse, rotate(poses[s + "_mid360_link"][3:], [1, 0, 0]))
                for s in ("front", "rear")}
        # This acceptance tool checks the supplied 15-degree configuration.
        pitch = math.radians(15)
        tilt_ok = all(math.dist(axes[s], [sign*math.cos(pitch), 0, -math.sin(pitch)]) < 1e-6
                      for s, sign in (("front", 1), ("rear", -1)))
        checks = {
            "clock_advances": (clock_end.sec, clock_end.nanosec) > (clock_start.sec, clock_start.nanosec),
            "two_clouds": all(latest[s + "_finite"] > 5 and
                latest[s].header.frame_id == s + "_mid360_link" for s in ("front", "rear")),
            "mount_tf": all(chain.values()),
            "front_rear_tilt": tilt_ok,
            "odom_tf": "base_link" in transforms and transforms["base_link"].header.frame_id == "odom",
            "imu_frame": latest["imu"].header.frame_id == "imu_link",
            "ground_contact": bool(wheel_bottoms) and max(abs(v) for v in wheel_bottoms) < 1e-5,
        }
        if args.interface == "livox":
            checks["custom_streams"] = all(
                latest[s + "_custom"].point_num > 100 and
                latest[s + "_custom"].point_num == len(latest[s + "_custom"].points) and
                latest[s + "_custom"].header.frame_id == s + "_mid360_link" and
                latest[s + "_custom"].lidar_id == i
                for i, s in enumerate(("front", "rear")))
        if args.drive:
            # Truth, not encoder odometry: spinning against an obstacle cannot pass.
            checks["motion"] = len(truth_path) > 2 and displacement > 0.03
        report.update(checks=checks, success=all(checks.values()), truth_displacement_m=displacement,
                      wheel_bottom_z_range_m=[min(wheel_bottoms), max(wheel_bottoms)],
                      lidar_forward_axes_in_base=axes)
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        if command:
            for _ in range(3):
                command.publish(Twist())
                spin(0.05)
        report["counts"] = dict(counts)
        node.destroy_node()
        rclpy.shutdown()
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(text + "\n")
    print(text)
    return int(not report["success"])


if __name__ == "__main__":
    raise SystemExit(main())
