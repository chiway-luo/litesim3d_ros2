#!/usr/bin/env python3
"""Run T-DEL-01 on a real Ubuntu 22.04 / Humble host and save actual evidence.

Dependencies must already be installed. No system changes are made by this runner.
It always creates a new build directory and propagates failures, skips and timeouts.
"""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paths import REPO_ROOT, EVIDENCE_DIR, SOURCE_FOLDERS, reference, source_files
import platform
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = REPO_ROOT


def main():
    out = EVIDENCE_DIR / "linux-result"
    out.mkdir(parents=True, exist_ok=True)
    result = {"status": "ERROR", "time": datetime.now(timezone.utc).isoformat(),
              "commands": [], "test_reports": {}, "errors": []}
    environment = {"system": platform.system(), "machine": platform.machine(),
                   "kernel": platform.release(), "python": platform.python_version(),
                   "ROS_DISTRO": os.environ.get("ROS_DISTRO")}
    env = os.environ.copy()
    env.update(ROS_LOCALHOST_ONLY="1", PYTEST_DISABLE_PLUGIN_AUTOLOAD="1", LIBGL_ALWAYS_SOFTWARE="1",
               LWS_CAPTURE_TEST_DISPLAY="1")
    env.pop("LWS_INSTALL_PREFIX", None)

    def run(label, command, timeout):
        log = out / f"{label}.log"
        with log.open("w") as stream:
            process = subprocess.run(command, cwd=ROOT, env=env, stdout=stream,
                                     stderr=subprocess.STDOUT, timeout=timeout)
        result["commands"].append({"label": label, "command": command,
                                   "exit_code": process.returncode, "log": reference(log)})
        if process.returncode:
            raise RuntimeError(f"{label} exited {process.returncode}; see {log}")
        return log

    def verify_xml(path, minimum):
        root = ET.parse(path).getroot()
        cases = list(root.iter("testcase"))
        bad = [c.get("name") for c in cases if any(c.find(tag) is not None
               for tag in ("failure", "error", "skipped")) or c.get("status") == "notrun"]
        result["test_reports"][reference(path)] = {"cases": len(cases), "bad": bad}
        if len(cases) < minimum or bad:
            raise RuntimeError(f"Incomplete/failing test report: {path}; cases={len(cases)}, bad={bad}")

    try:
        if environment["system"] != "Linux":
            raise RuntimeError("This gate must run on Linux, not a compatibility mock")
        os_release = dict(line.split("=", 1) for line in Path("/etc/os-release").read_text().splitlines()
                          if "=" in line)
        environment["os_release"] = {k: v.strip('"') for k, v in os_release.items()}
        environment["cpu_count"] = os.cpu_count()
        environment["memory_kib"] = int(next(line.split()[1] for line in Path("/proc/meminfo").read_text().splitlines()
                                               if line.startswith("MemTotal:")))
        environment["compiler"] = subprocess.check_output(["g++", "--version"], text=True).splitlines()[0]
        environment["display"] = "private Xvfb / software OpenGL"
        if environment["os_release"].get("ID") != "ubuntu" or environment["os_release"].get("VERSION_ID") != "22.04":
            raise RuntimeError("Expected the frozen Ubuntu 22.04 baseline")
        if environment["ROS_DISTRO"] != "humble":
            raise RuntimeError("Source /opt/ros/humble/setup.bash before running")
        from rclpy.utilities import get_rmw_implementation_identifier
        environment["rmw"] = get_rmw_implementation_identifier()
        environment["packages"] = subprocess.check_output([
            "dpkg-query", "-W", "-f=${Package} ${Version}\n", "g++", "cmake", "liburdfdom-dev",
            "libfcl-dev", "libassimp-dev", "libyaml-cpp-dev", "ros-humble-rclcpp",
            "ros-humble-rmw-fastrtps-cpp", "ros-humble-robot-state-publisher", "ros-humble-rviz2"
        ], text=True).splitlines()
        fresh = Path(tempfile.mkdtemp(prefix="lws-linux-clean-"))
        assert not any(fresh.iterdir())
        env["LWS_BUILD_ROOT"] = str(fresh)
        result["fresh_build_root"] = str(fresh)
        result["fresh_build_root_initially_empty"] = True
        source_files = [ROOT / "standalone/CMakeLists.txt"]
        for directory in (*SOURCE_FOLDERS, ".github"):
            source_files.extend(p for p in (ROOT / directory).rglob("*")
                                if p.is_file() and not {"__pycache__", ".pytest_cache"}.intersection(p.parts))
        result["source_hashes"] = {reference(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                   for p in sorted(source_files)}
        run("build", ["bash", "scripts/build.sh"], 600)
        log = run("tests", ["bash", "scripts/test_m1.sh"], 180)
        run_dir = next(Path(line[len("Evidence: "):]) for line in log.read_text().splitlines()
                       if line.startswith("Evidence: "))
        result["test_run_directory"] = reference(run_dir)
        verify_xml(run_dir / "core.xml", 67)
        verify_xml(run_dir / "ros.xml", 26)
        verify_xml(run_dir / "launch.xml", 2)
        run("install", [sys.executable, "scripts/checks/check_install.py"], 600)
        installed = json.loads((EVIDENCE_DIR / "final/install-check.json").read_text())
        if installed["status"] != "PASS" or not installed["source_hidden"]:
            raise RuntimeError("Source-hidden installation check failed")
        verify_xml(EVIDENCE_DIR / "final/install-launch.xml", 2)
        verify_xml(EVIDENCE_DIR / "final/install-mesh.xml", 1)
        prefix = (EVIDENCE_DIR / "install-prefix.txt").read_text().strip()
        run("gui", ["bash", "-c", 'source "$1/setup.bash"; exec xvfb-run -a "$2" "$3"',
                    "linux-gui", prefix, sys.executable, str(ROOT / "scripts/checks/check_gui_independence.py")], 180)
        verify_xml(EVIDENCE_DIR / "final/gui-independence/result.xml", 1)
        result["status"] = "PASS"
    except Exception as exc:
        result["errors"].append(repr(exc))
    finally:
        result["environment"] = environment
        (out / "acceptance.json").write_text(json.dumps(result, indent=2) + "\n")
        suite = ET.Element("testsuite", name="UbuntuHumbleDelivery", tests="1",
                           errors=str(int(result["status"] != "PASS")))
        case = ET.SubElement(suite, "testcase", classname="T-DEL-01", name="ubuntu_humble_clean_build_and_runtime")
        if result["status"] != "PASS":
            ET.SubElement(case, "error", message="; ".join(result["errors"]))
        ET.ElementTree(suite).write(out / "gate.xml", encoding="utf-8", xml_declaration=True)
        print(json.dumps({"status": result["status"], "errors": result["errors"]}))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
