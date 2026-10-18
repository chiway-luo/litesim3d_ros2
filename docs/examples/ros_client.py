#!/usr/bin/env python3
"""仿真接入示例：订阅 clock/点云/IMU/odom；仅指定 --drive 时发布速度。"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="", help="例如 r0；默认根命名空间")
    parser.add_argument("--frame", help="Stamped 控制 frame；默认 [namespace/]base_link")
    parser.add_argument("--stamped", action="store_true")
    parser.add_argument("--drive", action="store_true", help="显式启用 10 Hz 墙钟控制发布")
    parser.add_argument("--linear", type=float, default=0.2, help="m/s")
    parser.add_argument("--angular", type=float, default=0.0, help="rad/s")
    parser.add_argument("--duration", type=float, default=5.0, help="运行墙钟秒数，必须为正")
    parser.add_argument("--output", type=Path, help="可选 JSON 结果文件")
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.linear, args.angular, args.duration)) or args.duration <= 0:
        parser.error("速度和时长必须有限，时长必须为正")
    namespace = args.namespace.strip("/")
    frame = args.frame or ((namespace + "/") if namespace else "") + "base_link"
    counts = {"clock": 0, "points": 0, "imu": 0, "odom": 0, "commands": 0}
    report = {"namespace": namespace, "cmd_mode": "stamped" if args.stamped else "twist",
              "drive": args.drive, "counts": counts, "frames": {}}
    rclpy.init(args=[])
    node = Node("manual_client", namespace=namespace,
                parameter_overrides=[Parameter("use_sim_time", value=True)])

    def on_clock(message):
        counts["clock"] += 1
        report["clock_ns"] = message.clock.sec * 1_000_000_000 + message.clock.nanosec

    def on_cloud(message):
        counts["points"] += 1
        report["frames"]["points"] = message.header.frame_id
        report["cloud"] = {"height": message.height, "width": message.width,
                           "point_step": message.point_step,
                           "fields": [field.name for field in message.fields],
                           "data_bytes": len(message.data)}

    def on_imu(message):
        counts["imu"] += 1
        report["frames"]["imu"] = message.header.frame_id
        report["imu_acceleration"] = [message.linear_acceleration.x,
                                      message.linear_acceleration.y, message.linear_acceleration.z]

    def on_odom(message):
        counts["odom"] += 1
        report["frames"]["odom"] = message.header.frame_id
        report["frames"]["base"] = message.child_frame_id
        position = message.pose.pose.position
        xyz = [position.x, position.y, position.z]
        report.setdefault("first_odom_xyz", xyz)
        report["last_odom_xyz"] = xyz

    node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, "points_raw", on_cloud, qos_profile_sensor_data)
    node.create_subscription(Imu, "imu/data_raw", on_imu, qos_profile_sensor_data)
    node.create_subscription(Odometry, "odom", on_odom, 10)
    publisher = node.create_publisher(TwistStamped if args.stamped else Twist, "cmd_vel", 1) if args.drive else None

    def publish(linear, angular):
        if args.stamped:
            message = TwistStamped()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = frame
            twist = message.twist
        else:
            message = twist = Twist()
        twist.linear.x, twist.angular.z = linear, angular
        publisher.publish(message)
        counts["commands"] += 1

    start = time.monotonic()
    next_send = start
    try:
        while rclpy.ok() and time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if (publisher is not None and now >= next_send and node.get_clock().now().nanoseconds > 0
                    and publisher.get_subscription_count() > 0):
                publish(args.linear, args.angular)
                next_send = now + 0.1
    except KeyboardInterrupt:
        report["interrupted"] = True
    finally:
        if publisher is not None and rclpy.ok():
            for _ in range(3):
                publish(0.0, 0.0)
                rclpy.spin_once(node, timeout_sec=0.05)
        report["wall_seconds"] = time.monotonic() - start
        # IMU 是可选传感器；该示例验证连通性，不声称算法或导航通过。
        report["connected"] = all(counts[key] > 0 for key in ("clock", "points", "odom"))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    text = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
    print(text, end="")
    return 0 if report["connected"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
