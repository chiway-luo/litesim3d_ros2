#!/usr/bin/env python3
"""Build and test the generic bridge without the optional vendor packages."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from checks.run_checked import run


def verify(prefix, evidence):
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError
    for name in ("litesim3d_mid360", "livox_ros_driver2"):
        try:
            location = get_package_prefix(name)
        except PackageNotFoundError:
            continue
        raise RuntimeError(f"Absence test needs a clean ROS underlay: {name} is available in {location}")
    assert Path(get_package_prefix("litesim3d_ros2")).resolve() == prefix.resolve()
    linked = subprocess.check_output(["ldd", str(prefix / "lib/litesim3d_ros2/sim_node")], text=True)
    assert "not found" not in linked and "livox" not in linked and "mid360" not in linked, linked
    (evidence / "dependencies.json").write_text(json.dumps({"status": "PASS", "ldd": linked,
        "absent_packages": ["litesim3d_mid360", "livox_ros_driver2"]}, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--verify-prefix", type=Path)
    parser.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args()
    if args.verify_prefix:
        verify(args.verify_prefix, args.output)
        return 0
    work = args.output.resolve()
    if work.is_relative_to(ROOT) or ROOT.is_relative_to(work) or (work.exists() and any(work.iterdir())):
        parser.error("Choose a new empty directory outside the source tree")
    evidence = work / "evidence"
    evidence.mkdir(parents=True)
    prefix = work / "install"
    os.environ.update(CMAKE_BUILD_PARALLEL_LEVEL=str(args.jobs), MAKEFLAGS=f"-j{args.jobs}")
    result = {"status": "FAIL", "source": str(ROOT), "prefix": str(prefix)}

    def stage(name, command, timeout):
        status = run(list(map(str, command)), timeout, evidence / f"{name}.xml", evidence / f"{name}.log")
        result[name + "_exit"] = status
        (work / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
        return status == 0

    if not stage("build", ["colcon", "--log-base", work / "log", "build", "--base-paths", ROOT,
        "--build-base", work / "build", "--install-base", prefix, "--merge-install", "--parallel-workers", "2",
        "--packages-ignore", "litesim3d_gui", "litesim3d_mid360", "livox_ros_driver2",
        "--packages-up-to", "litesim3d_bringup", "litesim3d_test_support",
        "--cmake-args", "-DBUILD_TESTING=ON", "-DCMAKE_BUILD_TYPE=RelWithDebInfo"], 1800):
        return 1
    environment = ["bash", "-c", 'set -eo pipefail; source "$1/setup.bash"; shift; exec "$@"',
                   "optional-plugin", prefix]
    if not stage("absence", [*environment, sys.executable, Path(__file__),
                              "--verify-prefix", prefix, "--output", evidence], 30):
        return 1
    os.environ["LWS_INSTALL_PREFIX"] = str(prefix)
    if not stage("tests", [*environment, sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider",
                           ROOT / "litesim3d_ros2/test/test_lidar_plugin_loading.py",
                           "--basetemp", evidence / "cases", f"--junitxml={evidence / 'cases.xml'}"], 120):
        return 1
    cases = list(ET.parse(evidence / "cases.xml").iter("testcase"))
    if len(cases) != 2 or any(case.find("skipped") is not None for case in cases):
        raise RuntimeError("Both optional-plugin cases must execute without skips")
    result["status"] = "PASS"
    (work / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
