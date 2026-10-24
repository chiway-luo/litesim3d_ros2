#!/usr/bin/env python3
"""Execute the marked tutorial commands in one unused, localhost-only ROS domain."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
PAGES = [
    "docs/tutorials/启动与控制.md",
    "docs/tutorials/添加IMU.md",
    "docs/tutorials/ROS2接入.md",
    "docs/tutorials/双MID360从零搭建.md",
]


def commands(root=ROOT):
    result = {}
    for page in PAGES:
        for name, code in re.findall(r"<!-- tutorial-command: ([\w-]+) -->\s*```bash\n(.*?)\n```",
                                     (root / page).read_text(), re.S):
            if name in result:
                raise ValueError(f"duplicate tutorial command: {name}")
            argv = shlex.split(code.replace("$PWD", str(root)))
            allowed = (argv[:4] == ["ros2", "launch", "wheel_car_4w_sim", "wheel_car_4w.launch.py"]
                       or argv[:4] == ["ros2", "launch", "dual_mid360_bringup", "bringup.launch.py"]
                       or argv[:2] == ["python3", "docs/examples/ros_client.py"])
            if not allowed or any(token in {";", "|", "&&", ">", "<"} for token in argv):
                raise ValueError(f"{page}: unexpected executable in {name}")
            result[name] = argv
    return result


def stop(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="new evidence directory")
    parser.add_argument("--domain-id", type=int, default=91, help="unused domain, 0–101")
    parser.add_argument("--only-dual-mid360", action="store_true", help="run only the standalone dual lidar tutorial")
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 101:
        parser.error("domain must be in 0–101")
    out = args.output.resolve() if args.output else Path(tempfile.mkdtemp(prefix="litesim-tutorials-"))
    if args.output:
        out.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY="1",
                      ROS_LOG_DIR=str(out / "ros-logs"), PYTHONDONTWRITEBYTECODE="1")
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage

    selected = commands()
    # Keep maintainer assertions out of the user-facing configuration tutorial.
    dual_check = [sys.executable, str(ROOT / "scripts/checks/check_dual_mid360.py")]
    selected.update({
        "dual-observe": dual_check,
        "dual-drive": [*dual_check, "--drive"],
        "dual-native-observe": [*dual_check, "--interface", "native"],
        "dual-minimal-observe": dual_check,
    })
    selected["dual-minimal-launch"] += ["gui:=false", "rviz:=false"]
    report = {"status": "FAIL", "platform": platform.platform(), "python": sys.executable,
              "ros_distro": os.environ.get("ROS_DISTRO"), "domain_id": args.domain_id,
              "cases": [], "source_sha256": {},
              "scope": "Documented headless tutorials, including dual MID-360 streams, ground contact and truth motion; no GUI, SLAM/Nav2 or device acceptance."}
    sources = [*PAGES, "docs/examples/ros_client.py", "scripts/checks/check_dual_mid360.py",
               "docs/examples/tutorial_robot/config/runtime.yaml",
               "docs/examples/tutorial_robot/urdf/simulation.urdf.xacro",
               "docs/examples/tutorial_robot/urdf/robot.urdf.xacro",
               "docs/examples/tutorial_robot/urdf/sensor_mounts.xacro"]
    sources += [p.relative_to(ROOT).as_posix() for p in sorted((ROOT / "examples/dual_mid360").rglob("*"))
                if p.is_file() and p.suffix in {".py", ".xacro", ".yaml", ".sdf", ".rviz"}
                and "__pycache__" not in p.parts]
    for relative in sources:
        report["source_sha256"][relative] = hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()
    git = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, capture_output=True)
    report["base_commit"] = git.stdout.strip() if git.returncode == 0 else None
    report["working_tree_dirty"] = (bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT))
                                    if git.returncode == 0 else None)
    rclpy.init()
    node = rclpy.create_node("documentation_probe")
    clock_samples, transforms = [], {}
    node.create_subscription(Clock, "/clock", lambda m: clock_samples.append(m.clock), qos_profile_sensor_data)

    def on_tf(message):
        for transform in message.transforms:
            transforms[transform.child_frame_id] = transform

    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(TFMessage, "/tf_static", on_tf, qos)
    processes = []

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

    def client(name, expected=0):
        completed = subprocess.run(selected[name], cwd=ROOT, text=True, capture_output=True, timeout=60)
        (out / f"{name}.stdout").write_text(completed.stdout)
        (out / f"{name}.stderr").write_text(completed.stderr)
        data = json.loads(completed.stdout)
        passed = completed.returncode == expected and data["success"] == (expected == 0)
        if name == "client-missing-imu":
            passed = passed and data["checks"]["imu"] is False and data["counts"]["commands"] == 0
        if name in ("car-observe", "client-observe"):
            passed = passed and data["counts"]["commands"] == 0
        item = {"name": name, "passed": passed, "command": selected[name],
                "returncode": completed.returncode, "result": data}
        report["cases"].append(item)
        if not passed:
            raise RuntimeError(f"tutorial failed: {name}")
        return data

    try:
        spin_for(2)
        if node.count_publishers("/clock") or any(name == "litesim3d" for name, _ in node.get_node_names_and_namespaces()):
            raise RuntimeError("selected domain is already in use; choose another --domain-id")
        groups = [
            ("car-launch", ["car-observe", "car-drive", "client-observe", "client-drive", "client-missing-imu"]),
            ("imu-launch", ["imu-observe"]),
            ("dual-launch", ["dual-observe", "dual-drive"]),
            ("dual-native-launch", ["dual-native-observe"]),
            ("dual-minimal-launch", ["dual-minimal-observe"]),
        ]
        if args.only_dual_mid360:
            groups = [item for item in groups if item[0].startswith("dual-")]
        for launch_name, tests in groups:
            clock_samples.clear()
            transforms.clear()
            with (out / f"{launch_name}.log").open("w") as log:
                process = subprocess.Popen(selected[launch_name], cwd=ROOT, stdout=log,
                                           stderr=subprocess.STDOUT, start_new_session=True)
                processes.append(process)
                try:
                    deadline = time.monotonic() + 45
                    while not clock_samples and process.poll() is None and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.1)
                    if not clock_samples or process.poll() is not None:
                        raise RuntimeError(f"{launch_name}: no active clock; inspect launch log")
                    for name in tests:
                        data = client(name, 1 if name == "client-missing-imu" else 0)
                        if name == "imu-observe":
                            spin_for(2)
                            transform = transforms.get("tutorial_imu_link")
                            actual = None if transform is None else [
                                transform.transform.translation.x, transform.transform.translation.y,
                                transform.transform.translation.z]
                            rotation = None if transform is None else [
                                transform.transform.rotation.x, transform.transform.rotation.y,
                                transform.transform.rotation.z, transform.transform.rotation.w]
                            acceleration = data.get("imu_acceleration", [])
                            passed = (transform is not None and transform.header.frame_id == "base_link"
                                      and all(abs(a-b) < 1e-6 for a, b in zip(actual, [0.1, 0, 0.15]))
                                      and all(abs(a-b) < 1e-6 for a, b in zip(rotation, [0, 0, 0, 1]))
                                      and len(acceleration) == 3
                                      and all(abs(a-b) < 0.05 for a, b in zip(acceleration, [0, 0, 9.81])))
                            report["cases"].append({"name": "imu-transform-and-gravity", "passed": passed,
                                                    "translation": actual, "rotation": rotation,
                                                    "acceleration": acceleration})
                            if not passed:
                                raise RuntimeError("tutorial IMU TF or stationary specific force mismatch")
                finally:
                    stop(process)
            spin_for(2)
        report["status"] = "PASS"
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        for process in processes:
            stop(process)
        node.destroy_node()
        rclpy.shutdown()
        (out / "result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "cases": len(report["cases"]),
                      "error": report.get("error"), "evidence": str(out)}, ensure_ascii=False))
    return int(report["status"] != "PASS")


if __name__ == "__main__":
    raise SystemExit(main())
