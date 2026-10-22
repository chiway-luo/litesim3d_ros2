#!/usr/bin/env python3
"""The same Ubuntu/Humble quality profiles run locally and in GitHub Actions."""
import argparse
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import shutil
import sys
import time
import xml.etree.ElementTree as ET

from checks.run_checked import run
from quality.inventory import inventory, coverage_config
from quality.sanitizer_control import asan_options

ROOT = Path(__file__).resolve().parents[1]
PROFILES = ("functional", "coverage", "asan", "tsan", "release")


def source_snapshot(root):
    files = {}
    for path in sorted(root.rglob("*")):
        if (path.is_file() and not {".git", "__pycache__", ".pytest_cache"}.intersection(path.parts)
                and path.name != ".DS_Store" and not path.name.startswith("._")):
            files[path.relative_to(root).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    return {"files": files, "sha256": hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()}


def tool(name, versioned=None):
    found = shutil.which(versioned) if versioned else None
    return found or shutil.which(name)


class Profile:
    def __init__(self, args, name):
        self.args, self.name = args, name
        self.work = args.output / name
        self.runtime = self.work / "runtime"
        self.prefix = self.runtime / "install"
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        self.evidence = self.work / "runs" / f"{stamp}-{os.getpid()}"
        self.evidence.mkdir(parents=True)
        self.env = dict(os.environ)
        for key in ("LWS_INSTALL_PREFIX", "LWS_BUILD_ROOT", "LWS_BUILD_GUI", "LLVM_PROFILE_FILE",
                    "COVERAGE_PROCESS_START", "COVERAGE_FILE", "ASAN_OPTIONS", "UBSAN_OPTIONS", "TSAN_OPTIONS",
                    "LSAN_OPTIONS", "LWS_TEST_STEP_TIMEOUT", "LWS_INSTRUMENTED_MEMORY",
                    "LWS_TEST_LAUNCH_TIMEOUT", "LWS_TEST_CORE_TIMEOUT", "LWS_TEST_ROS_TIMEOUT",
                    "LWS_TEST_WAIT_TIMEOUT", "LWS_TEST_SUITE_TIMEOUT", "LWS_TEST_RENDER_TIMEOUT",
                    "LWS_TEST_STARTUP_TIMEOUT", "LWS_TEST_SHUTDOWN_TIMEOUT"):
            self.env.pop(key, None)
        self.env.update(LITESIM3D_ARTIFACTS_DIR=str(self.evidence / "artifacts"),
                        LWS_BUILD_ROOT=str(self.runtime), LWS_BUILD_GUI="1",
                        PYTHONDONTWRITEBYTECODE="1", PYTEST_DISABLE_PLUGIN_AUTOLOAD="1",
                        ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID="4",
                        ROS_LOG_DIR=str(self.evidence / "ros_logs"),
                        RMW_IMPLEMENTATION=args.rmw,
                        CMAKE_BUILD_PARALLEL_LEVEL=str(args.jobs), MAKEFLAGS=f"-j{args.jobs}")
        if name in ("coverage", "asan"):
            self.env["LWS_TEST_STEP_TIMEOUT"] = "60"
            self.env["LWS_TEST_LAUNCH_TIMEOUT"] = "60"
        if name in ("asan", "tsan"):
            self.env["LWS_INSTRUMENTED_MEMORY"] = name
        if name == "asan":
            self.env["LWS_TEST_CORE_TIMEOUT"] = "1200"
            self.env["LWS_TEST_ROS_TIMEOUT"] = "2400"
            self.env["LWS_TEST_WAIT_TIMEOUT"] = "120"
            self.env["LWS_TEST_SUITE_TIMEOUT"] = "900"
            self.env["LWS_TEST_RENDER_TIMEOUT"] = "900"
        self.record = {"profile": name, "status": "RUNNING", "source": str(ROOT),
                       "source_snapshot": source_snapshot(ROOT), "stages": [],
                       "platform": platform.platform(), "architecture": platform.machine(),
                       "ros_distro": os.environ.get("ROS_DISTRO"), "rmw": args.rmw,
                       "evidence": str(self.evidence),
                       "step_wall_timeout_seconds": int(self.env.get("LWS_TEST_STEP_TIMEOUT", "6")),
                       "core_wall_timeout_seconds": int(self.env.get("LWS_TEST_CORE_TIMEOUT", "600")),
                       "ros_wall_timeout_seconds": int(self.env.get("LWS_TEST_ROS_TIMEOUT", "1200")),
                       "minimum_wait_timeout_seconds": int(self.env.get("LWS_TEST_WAIT_TIMEOUT", "0")),
                       "render_wall_timeout_seconds": int(self.env.get("LWS_TEST_RENDER_TIMEOUT", "60")),
                       "world_launch_wall_timeout_seconds": int(self.env.get("LWS_TEST_LAUNCH_TIMEOUT", "15"))}
        self.save()

    def save(self):
        (self.evidence / "result.json").write_text(json.dumps(self.record, indent=2) + "\n")
        (self.work / "latest.json").write_text(json.dumps(self.record, indent=2) + "\n")

    def stage(self, name, command, timeout=300, installed=True, unit=False, graphical=False):
        env = dict(self.env)
        if self.name == "coverage" and installed:
            kind = "unit" if unit else "integration"
            profiles = self.evidence / "profiles" / kind
            profiles.mkdir(parents=True, exist_ok=True)
            (profiles / "python").mkdir(exist_ok=True)
            env["LLVM_PROFILE_FILE"] = str(profiles / "%m-%p.profraw")
            env["COVERAGE_FILE"] = str(profiles / "python/.coverage")
        if graphical:
            if self.name == "asan":
                # LLVM's complete allocation stacks make Mesa initialization
                # much slower. Scope these budgets to graphical tests; fault
                # tests and normal simulator defaults retain their deadlines.
                env.update(LWS_TEST_STARTUP_TIMEOUT="600", LWS_TEST_WAIT_TIMEOUT="600",
                           LWS_TEST_SHUTDOWN_TIMEOUT="120", LWS_TEST_SUITE_TIMEOUT="6000")
            command = ["xvfb-run", "-a", "env", "LIBGL_ALWAYS_SOFTWARE=1", "QT_X11_NO_MITSHM=1",
                       "LWS_PRIVATE_TEST_DISPLAY=1", *map(str, command)]
        setup = self.prefix / "setup.bash" if installed else Path(self.args.ros_prefix) / "setup.bash"
        if installed:
            env["LWS_INSTALL_PREFIX"] = str(self.prefix)
        command = ["bash", "-c", 'set -eo pipefail; source "$1"; shift; exec "$@"',
                   "quality-stage", str(setup), *map(str, command)]
        entry = {"name": name, "status": "RUNNING", "command": command,
                 "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
        if graphical:
            entry["graphical_test_budgets"] = {key: int(env[key]) for key in (
                "LWS_TEST_STARTUP_TIMEOUT", "LWS_TEST_WAIT_TIMEOUT",
                "LWS_TEST_SHUTDOWN_TIMEOUT", "LWS_TEST_SUITE_TIMEOUT") if key in env}
        self.record["stages"].append(entry)
        self.save()
        print(f"[{self.name}] START {name} ({self.evidence / (name + '.log')})", flush=True)
        started = time.monotonic()
        original = os.environ.copy()
        try:
            os.environ.clear()
            os.environ.update(env)
            entry["exit_code"] = run(command, timeout, self.evidence / f"{name}.xml",
                                     self.evidence / f"{name}.log")
        finally:
            os.environ.clear()
            os.environ.update(original)
        entry["seconds"] = round(time.monotonic() - started, 2)
        entry["status"] = "PASS" if entry["exit_code"] == 0 else "FAIL"
        self.save()
        print(f"[{self.name}] {entry['status']} {name} ({entry['seconds']}s)", flush=True)
        return entry["status"] == "PASS"

    def configure_coverage(self):
        item = inventory(ROOT)
        (self.evidence / "inventory.json").write_text(json.dumps(item, indent=2) + "\n")
        config = self.evidence / "coveragerc"
        config.write_text(coverage_config(ROOT, self.prefix, item))
        self.env["COVERAGE_PROCESS_START"] = str(config)
        self.env["PYTHONPATH"] = os.pathsep.join(filter(None, [
            str(ROOT / "scripts/quality/python_startup"), self.env.get("PYTHONPATH")]))

    def build(self):
        cmake = ["-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"]
        if self.name == "asan":
            from ament_index_python.packages import get_package_share_directory
            share = Path(get_package_share_directory("rclcpp"))
            version = ET.parse(share / "package.xml").getroot().findtext("version")
            self.record["rclcpp"] = {"version": version, "share": str(share)}
            self.save()
            if tuple(map(int, version.split("."))) < (16, 0, 19):
                raise RuntimeError(f"ASan requires rclcpp >= 16.0.19; found {version}. "
                                   "Older Humble allocators trigger new-delete-type-mismatch. "
                                   "Use an updated ROS underlay; do not disable this detector. "
                                   "Upstream fix: https://github.com/ros2/rclcpp/pull/3072")
        if self.name != "tsan" and not tool("xvfb-run"):
            raise RuntimeError("Install xvfb and xauth before profiles that run desktop tests")
        if self.name == "coverage" and (not importlib.util.find_spec("coverage") or
                not tool("llvm-cov", "llvm-cov-14") or not tool("llvm-profdata", "llvm-profdata-14")):
            raise RuntimeError("Install python3-coverage and llvm-14 before the coverage profile")
        if self.name in ("coverage", "asan", "tsan"):
            clang = tool("clang", "clang-14")
            clangxx = tool("clang++", "clang++-14")
            if not clang or not clangxx:
                raise RuntimeError("Install clang-14 and llvm-14 before instrumented profiles")
            cmake += [f"-DCMAKE_C_COMPILER={clang}", f"-DCMAKE_CXX_COMPILER={clangxx}",
                      f"-DCMAKE_PROJECT_INCLUDE={ROOT / 'scripts/quality/instrument.cmake'}",
                      f"-DLITESIM_QUALITY_MODE={self.name}", "-DCMAKE_CXX_FLAGS_RELWITHDEBINFO=-O1 -g -DNDEBUG"]
        if self.name in ("asan", "tsan"):
            symbolizer = tool("llvm-symbolizer", "llvm-symbolizer-14")
            if not symbolizer:
                raise RuntimeError("Install llvm-14 for readable sanitizer stacks")
            self.env["ASAN_SYMBOLIZER_PATH"] = symbolizer
            self.env["TSAN_SYMBOLIZER_PATH"] = symbolizer
        if self.name == "coverage":
            profiles = self.evidence / "profiles/build"
            profiles.mkdir(parents=True)
            self.env["LLVM_PROFILE_FILE"] = str(profiles / "%m-%p.profraw")
        if self.name == "asan":
            suppression = ROOT / "scripts/quality/lsan.supp"
            self.env.update(ASAN_OPTIONS=asan_options(symbolizer, self.evidence / "asan"),
                            UBSAN_OPTIONS=f"halt_on_error=1:print_stacktrace=1:external_symbolizer_path={symbolizer}:log_path={self.evidence}/ubsan",
                            LSAN_OPTIONS=f"suppressions={suppression}:print_suppressions=1")
        if self.name == "tsan":
            self.env["TSAN_OPTIONS"] = f"halt_on_error=1:external_symbolizer_path={symbolizer}:log_path={self.evidence}/tsan"
        if self.name in ("asan", "tsan") and not self.stage(
                "sanitizer-control", [sys.executable, ROOT / "scripts/quality/sanitizer_control.py",
                "--compiler", clangxx, "--mode", self.name,
                "--output", self.evidence / "positive-control"], installed=False):
            return False
        if self.name == "asan" and not self.stage(
                "sanitizer-render-control", [sys.executable, ROOT / "scripts/quality/sanitizer_control.py",
                "--compiler", clangxx, "--mode", "asan", "--llvm-unwind",
                "--output", self.evidence / "positive-control/render"], installed=False):
            return False
        identity = {"source": str(ROOT), "profile": self.name, "cmake": cmake,
                    "ros_prefix": str(self.args.ros_prefix)}
        identity_path = self.work / "build-identity.json"
        if identity_path.exists() and json.loads(identity_path.read_text()) != identity:
            raise RuntimeError("Build identity changed; choose a new output directory")
        identity_path.write_text(json.dumps(identity, indent=2) + "\n")
        if not self.stage("build", ["bash", ROOT / "scripts/build.sh", *cmake], 2400, installed=False):
            return False
        if self.name == "coverage":
            self.configure_coverage()
        if self.name == "asan" and not self.stage(
                "plugin-leak-control", [sys.executable, ROOT / "scripts/quality/sanitizer_control.py",
                "--compiler", clangxx, "--mode", "asan", "--output", self.evidence / "positive-control",
                "--plugin-probe", self.runtime / "build/litesim3d_plugin_tests/plugin_leak_probe"]):
            return False
        return True

    def native(self):
        for package, binary in (("litesim3d_core", "sim_core_tests"),
                                ("litesim3d_plugin_tests", "plugin_contract_tests"),
                                ("litesim3d_plugin_tests", "core_plugin_contract_tests"),
                                ("litesim3d_mid360", "mid360_plugin_tests")):
            self.stage(binary, [self.runtime / "build" / package / binary,
                       f"--gtest_output=xml:{self.evidence / (binary + '-cases.xml')}"],
                       int(self.env.get("LWS_TEST_CORE_TIMEOUT", "600")), unit=True)
        self.stage("lifecycle", [sys.executable, ROOT / "scripts/quality/check_lifecycle.py",
                   "--binary", self.runtime / "build/litesim3d_core/lifecycle_probe",
                   "--config", self.prefix / "share/litesim3d_core/config/demo.yaml",
                   "--output", self.evidence / "lifecycle.json",
                   *(["--instrumented"] if self.name in ("asan", "tsan") else [])], 600, unit=True)

    def regression(self):
        self.stage("regression", ["bash", ROOT / "scripts/test_m3.sh"], 3600 if self.name == "asan" else 1800)
        self.stage("bringup-unit", [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider",
                   ROOT / "litesim3d_bringup/test",
                   f"--junitxml={self.evidence / 'bringup-cases.xml'}"], 300, unit=True)
        # These packages intentionally have identically named test modules.
        for example in ("ackermann", "omni3", "omni4"):
            self.stage(example, [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider",
                       ROOT / "examples" / example / f"{example}_sim/test",
                       f"--junitxml={self.evidence / (example + '-cases.xml')}"], 300)
        if self.name == "functional":
            # Tutorials assert real-time motion over a fixed wall interval.
            self.stage("tutorials", [sys.executable, ROOT / "scripts/checks/check_documented_tutorials.py",
                       "--output", self.evidence / "tutorials"], 300)
        self.stage("desktop", ["bash", ROOT / "scripts/test_desktop.sh"],
                   9000 if self.name == "asan" else 900, graphical=True)

    def execute(self):
        try:
            if not self.stage("quality-tests", [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider",
                       ROOT / "scripts/quality/tests", f"--junitxml={self.evidence / 'quality-cases.xml'}"], installed=False):
                raise RuntimeError("Quality tooling tests failed")
            if self.name == "release":
                self.stage("versions", [sys.executable, ROOT / "scripts/checks/check_versions.py"], installed=False)
                self.stage("packages", [sys.executable, ROOT / "scripts/checks/check_release_packages.py",
                           "--work-dir", self.evidence / "packages", "--jobs", str(self.args.jobs),
                           "--continue-on-test-failure"], 4200, installed=False)
            elif self.build():
                self.native()
                if self.name != "tsan":
                    self.regression()
                if self.name == "functional":
                    for name in ("check_repository_docs.py", "check_api_docs.py"):
                        self.stage(name[:-3], [sys.executable, ROOT / "scripts/checks" / name])
                    self.stage("documentation-tests", [sys.executable, "-m", "unittest", "discover",
                               "-s", ROOT / "scripts/checks", "-p", "test_documentation_checks.py"])
                    self.stage("rviz-independence", [sys.executable, ROOT / "scripts/checks/check_gui_independence.py"],
                               300, graphical=True)
                    self.stage("source-hidden-install", [sys.executable, ROOT / "scripts/checks/check_m3_install.py"],
                               1800, installed=False)
                    self.stage("optional-plugin-absent", [sys.executable, ROOT / "scripts/quality/check_optional_plugin.py",
                               "--output", self.evidence / "bridge-only", "--jobs", str(self.args.jobs)],
                               2100, installed=False)
                if self.name == "coverage":
                    self.stage("coverage-report", [sys.executable, ROOT / "scripts/quality/collect_coverage.py",
                               "--root", ROOT, "--prefix", self.prefix, "--build", self.runtime / "build",
                               "--evidence", self.evidence], 300)
                if self.name in ("asan", "tsan"):
                    self.stage("sanitizer-report", [sys.executable, ROOT / "scripts/quality/check_sanitizers.py",
                               "--root", ROOT, "--build", self.runtime / "build", "--evidence", self.evidence,
                               "--mode", self.name])
        except Exception as error:
            self.record["error"] = repr(error)
            print(f"[{self.name}] {error}", file=sys.stderr, flush=True)
        changed = source_snapshot(ROOT)["sha256"] != self.record["source_snapshot"]["sha256"]
        if changed:
            self.record["error"] = "Source changed during validation; rerun this profile"
        self.record["status"] = "PASS" if (not self.record.get("error") and self.record["stages"]
            and all(s["status"] == "PASS" for s in self.record["stages"])) else "FAIL"
        self.save()
        print(f"[{self.name}] {self.record['status']}: {self.evidence / 'result.json'}", flush=True)
        return self.record["status"] == "PASS"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("all", *PROFILES), default="all")
    parser.add_argument("--output", type=Path, required=True, help="Outside the checkout; each profile has an isolated build")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--ros-prefix", type=Path, default=Path("/opt/ros/humble"))
    parser.add_argument("--rmw", default="rmw_fastrtps_cpp")
    args = parser.parse_args()
    args.output = args.output.expanduser().resolve()
    if args.jobs < 1 or args.output.is_relative_to(ROOT) or ROOT.is_relative_to(args.output):
        parser.error("Use a positive job count and an output directory outside the checkout")
    if platform.system() != "Linux" or os.environ.get("ROS_DISTRO") != "humble":
        parser.error("Run on Ubuntu after sourcing ROS 2 Humble")
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / ".lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("Another validation is using this output directory")
        os.chdir(ROOT)
        initial = source_snapshot(ROOT)
        results = {}
        snapshots = []
        for profile in PROFILES if args.profile == "all" else (args.profile,):
            current = Profile(args, profile)
            results[profile] = current.execute()
            snapshots.append(current.record["source_snapshot"]["sha256"])
        final = source_snapshot(ROOT)
        same_source = set(snapshots + [final["sha256"]]) == {initial["sha256"]}
        passed = all(results.values()) and same_source
        summary = {"status": "PASS" if passed else "FAIL", "profiles": results,
                   "source_snapshot": initial, "same_source_across_profiles": same_source}
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        return int(not passed)


if __name__ == "__main__":
    raise SystemExit(main())
