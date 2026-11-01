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
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from checks.run_checked import Interrupted, run
from quality.inventory import inventory, coverage_config
from quality.sanitizer_control import asan_options

ROOT = Path(__file__).resolve().parents[1]
PROFILES = ("functional", "coverage", "asan", "tsan", "release")


def scope_details(unit_only, status="RUNNING"):
    if not unit_only:
        return {"scope": "full-profile"}
    return {"scope": "unit-only", "categories": {
        "unit_coverage": status, "integration": "NOT_RUN",
        "graphical_integration": "NOT_RUN", "sanitizer_controls": "NOT_RUN",
        "release_packaging": "NOT_RUN", "full_quality_matrix": "NOT_RUN"},
        "scope_note": "Unit coverage only. Full runtime/GUI compilation and source denominator retained. "
                      "No integration, physical-renderer, sanitizer-control or full-quality-matrix pass is claimed."}


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
        self.unit_only = bool(getattr(args, "unit_only", False))
        if self.unit_only and name != "coverage":
            raise ValueError("unit-only mode is valid only for the coverage profile")
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
                    "LWS_TEST_STARTUP_TIMEOUT", "LWS_TEST_SHUTDOWN_TIMEOUT",
                    "LWS_QUALITY_HOST_PREFIX", "LWS_TEST_LAUNCH_SUITE_TIMEOUT",
                    "LITESIM_FUNCTION_COVERAGE_ROOTS"):
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
            # Whole example suites include several instrumented simulator
            # processes. Keep their outer wall budget separate from each
            # assertion's simulation-time deadline and controller watchdog.
            self.env["LWS_TEST_SUITE_TIMEOUT"] = "900"
        if name in ("asan", "tsan"):
            self.env["LWS_INSTRUMENTED_MEMORY"] = name
        if name == "asan":
            self.env["LWS_TEST_CORE_TIMEOUT"] = "1200"
            self.env["LWS_TEST_ROS_TIMEOUT"] = "2400"
            self.env["LWS_TEST_WAIT_TIMEOUT"] = "120"
            self.env["LWS_TEST_RENDER_TIMEOUT"] = "900"
        self.record = {"profile": name, "status": "RUNNING", "source": str(ROOT),
                       **scope_details(self.unit_only),
                       "source_snapshot": source_snapshot(ROOT), "stages": [],
                       "platform": platform.platform(), "architecture": platform.machine(),
                       "ros_distro": os.environ.get("ROS_DISTRO"), "rmw": args.rmw,
                       "evidence": str(self.evidence),
                       "step_wall_timeout_seconds": int(self.env.get("LWS_TEST_STEP_TIMEOUT", "6")),
                       "core_wall_timeout_seconds": int(self.env.get("LWS_TEST_CORE_TIMEOUT", "600")),
                       "ros_wall_timeout_seconds": int(self.env.get("LWS_TEST_ROS_TIMEOUT", "1200")),
                       "suite_wall_timeout_seconds": int(self.env.get("LWS_TEST_SUITE_TIMEOUT", "180")),
                       "minimum_wait_timeout_seconds": int(self.env.get("LWS_TEST_WAIT_TIMEOUT", "0")),
                       "render_wall_timeout_seconds": int(self.env.get("LWS_TEST_RENDER_TIMEOUT", "60")),
                       "world_launch_wall_timeout_seconds": int(self.env.get("LWS_TEST_LAUNCH_TIMEOUT", "15"))}
        self.save()

    def save(self):
        (self.evidence / "result.json").write_text(json.dumps(self.record, indent=2) + "\n")
        (self.work / "latest.json").write_text(json.dumps(self.record, indent=2) + "\n")

    def stage(self, name, command, timeout=300, installed=True, unit=False, graphical=False):
        if self.unit_only and graphical:
            raise RuntimeError("Graphical integration is outside unit-only coverage scope")
        if self.unit_only and installed and not unit:
            raise RuntimeError("Integration is outside unit-only coverage scope")
        env = dict(self.env)
        if self.unit_only and not installed:
            # Tooling/reporting is not runtime execution. In particular, the
            # collector must not create an integration bucket or need an unset
            # COVERAGE_FILE after configure_coverage() enabled subprocess hooks.
            for key in ("COVERAGE_PROCESS_START", "COVERAGE_FILE", "LITESIM_FUNCTION_COVERAGE_ROOTS"):
                env.pop(key, None)
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
            # Qt must not depend on an SSH user's bus or attempt D-Bus
            # autolaunch in a container with no login session. The private bus
            # exits with this stage, including when the tested command fails.
            command = ["dbus-run-session", "--", "xvfb-run", "-a", "env", "LIBGL_ALWAYS_SOFTWARE=1", "QT_X11_NO_MITSHM=1",
                       "LWS_PRIVATE_TEST_DISPLAY=1", *map(str, command)]
        setup = self.prefix / "setup.bash" if installed else Path(self.args.ros_prefix) / "setup.bash"
        if installed:
            env["LWS_INSTALL_PREFIX"] = str(self.prefix)
        command = ["bash", "-c", 'set -eo pipefail; source "$1"; shift; exec "$@"',
                   "quality-stage", str(setup), *map(str, command)]
        if installed and self.name == "asan":
            # Setup scripts prepend their underlay. Put the quality-only host
            # overlay first afterwards, without preloading Python or the shell.
            command[2] = ('set -eo pipefail; source "$1"; shift; '
                          'export AMENT_PREFIX_PATH="$LWS_QUALITY_HOST_PREFIX:$AMENT_PREFIX_PATH"; exec "$@"')
        entry = {"name": name, "status": "RUNNING", "command": command,
                 "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
        entry["test_level"] = ("unit" if unit else "integration") if installed else "tooling"
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
        except Interrupted as error:
            entry.update(exit_code=128 + error.signum, status="CANCELLED",
                         seconds=round(time.monotonic() - started, 2))
            self.save()
            raise
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
        self.env["LITESIM_FUNCTION_COVERAGE_ROOTS"] = json.dumps([str(ROOT), str(self.prefix)])
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
        if not self.unit_only and self.name != "tsan" and not tool("xvfb-run"):
            raise RuntimeError("Install xvfb and xauth before profiles that run desktop tests")
        if not self.unit_only and self.name != "tsan" and not tool("dbus-run-session"):
            raise RuntimeError("Install dbus before profiles that run desktop tests")
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
        if self.name == "asan" and not self.stage(
                "controller-leak-control", [sys.executable, ROOT / "scripts/quality/control_plugin_control.py",
                "--compiler", clangxx, "--output", self.evidence / "positive-control/control"], installed=False):
            return False
        identity = {"source": str(ROOT), "profile": self.name, "cmake": cmake,
                    "ros_prefix": str(self.args.ros_prefix)}
        identity_path = self.work / "build-identity.json"
        if identity_path.exists() and json.loads(identity_path.read_text()) != identity:
            raise RuntimeError("Build identity changed; choose a new output directory")
        identity_path.write_text(json.dumps(identity, indent=2) + "\n")
        if not self.stage("build", ["bash", ROOT / "scripts/build.sh", *cmake], 2400, installed=False):
            return False
        if self.name == "asan":
            self.env["LWS_QUALITY_HOST_PREFIX"] = str(self.runtime / "controller-host")
            if not self.stage("controller-runtime", [sys.executable, ROOT / "scripts/quality/asan_host.py",
                    "--compiler", clangxx, "--underlay", self.args.ros_prefix,
                    "--project-prefix", self.prefix,
                    "--output", self.env["LWS_QUALITY_HOST_PREFIX"]], installed=False):
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
                                ("litesim3d_core", "core_cli_unit_tests"),
                                ("litesim3d_plugin_tests", "plugin_contract_tests"),
                                ("litesim3d_plugin_tests", "core_plugin_contract_tests"),
                                ("litesim3d_mid360", "mid360_plugin_tests"),
                                ("litesim3d_mid360", "livox_outputs_unit"),
                                ("litesim3d_assets", "test_sdf_import_units"),
                                ("litesim3d_gui", "ros_event_loop_test"),
                                ("litesim3d_gui", "render_helpers_test"),
                                ("litesim3d_gui", "camera_view_unit"),
                                ("litesim3d_ros2", "point_adapter_unit"),
                                ("litesim3d_ros2", "service_responses_unit"),
                                ("litesim3d_ros2", "joint_control_unit"),
                                ("litesim3d_ros2", "lidar_outputs_unit"),
                                ("litesim3d_ros2", "cloud_tools_unit"),
                                ("litesim3d_ros2", "world_node_unit"),
                                ("litesim3d_ros2", "sim_node_unit")):
            self.stage(binary, [self.runtime / "build" / package / binary,
                       f"--gtest_output=xml:{self.evidence / (binary + '-cases.xml')}"],
                       int(self.env.get("LWS_TEST_CORE_TIMEOUT", "600")), unit=True)
        # These in-process configuration/sleep tests do not launch a simulator,
        # controller manager or external ROS peer. Keep real hardware, spawner
        # and shutdown process tests in the integration stage below.
        self.stage("control-unit", ["ctest", "--test-dir",
                   self.runtime / "build/litesim3d_ros2_control", "-R",
                   "^(body_configuration|omni_configuration|control_loop_sleep|controller_units|system_units|spawner_unit)$",
                   "--output-on-failure"], 60, unit=True)

    def lifecycle(self):
        # The real Config/Model/Simulator/sensor-worker stress subprocess is a
        # component/integration check, not an isolated API unit. Keep it in all
        # full non-release profiles, including TSAN, but never borrow its hits.
        self.stage("lifecycle", [sys.executable, ROOT / "scripts/quality/check_lifecycle.py",
                   "--binary", self.runtime / "build/litesim3d_core/lifecycle_probe",
                   "--config", self.prefix / "share/litesim3d_core/config/demo.yaml",
                   "--output", self.evidence / "lifecycle.json",
                   *(["--instrumented"] if self.name in ("asan", "tsan") else [])], 600)

    def python_units(self):
        # Exact in-process cases only. The existing controller-parameter suite
        # executes the real model_geometry subprocess and stays integration.
        self.stage("bringup-unit", [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider",
                   ROOT / "litesim3d_bringup/test/test_model.py",
                   ROOT / "litesim3d_bringup/test/test_launch_validation.py",
                   ROOT / "litesim3d_bringup/test/test_simulation_launch_unit.py",
                   ROOT / "litesim3d_bringup/test/test_controller_parameters_unit.py",
                   ROOT / "litesim3d_bringup/test/test_model_edges_unit.py",
                   f"--junitxml={self.evidence / 'bringup-cases.xml'}"], 300, unit=True)
        self.stage("python-runtime-unit", [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider",
                   "--import-mode=importlib", ROOT / "litesim3d_ros2/test/test_session_unit.py",
                   ROOT / "litesim3d_ros2/test/test_launch_units.py",
                   ROOT / "litesim3d_ros2/test/test_integration_expectations_unit.py",
                   ROOT / "scripts/quality/tests/test_prepare_xacro_unit.py",
                   ROOT / "scripts/quality/tests/test_convert_mid360_unit.py",
                   f"--junitxml={self.evidence / 'python-runtime-cases.xml'}"], 300, unit=True)

    def regression(self):
        runner = ROOT / "scripts/test_m3.sh"
        budget = int(subprocess.check_output(["bash", runner, "--timeout-budget"], env=self.env, text=True))
        self.stage("regression", ["bash", runner], budget)
        # These retained CLI cases use the real Config/Model/Simulator/sensor
        # pipeline. Calling main in-process does not make that an isolated unit.
        self.stage("core-cli-integration", [self.runtime / "build/litesim3d_core/core_cli_integration_tests",
                   f"--gtest_output=xml:{self.evidence / 'core-cli-integration-cases.xml'}"], 300)
        # Direct serializer units deliberately own no plugin registration.
        # Exercise the production DSO's registration/load/unload separately,
        # retaining its profiles only in the integration bucket.
        self.stage("livox-output-plugin-integration", [
                   self.runtime / "build/litesim3d_mid360/livox_output_plugin_integration",
                   f"--gtest_output=xml:{self.evidence / 'livox-output-plugin-integration-cases.xml'}"], 60)
        self.stage("controller-parameters-integration", [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider",
                   ROOT / "litesim3d_bringup/test/test_controller_parameters.py",
                   f"--junitxml={self.evidence / 'controller-parameters-cases.xml'}"], 300)
        # These packages intentionally have identically named test modules.
        for example in ("ackermann", "omni3", "omni4"):
            self.stage(example, [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider",
                       ROOT / "examples" / example / f"{example}_sim/test",
                       f"--junitxml={self.evidence / (example + '-cases.xml')}"],
                       int(self.env.get("LWS_TEST_SUITE_TIMEOUT", "300")))
        if self.name == "functional":
            # Tutorials assert real-time motion over a fixed wall interval.
            self.stage("tutorials", [sys.executable, ROOT / "scripts/checks/check_documented_tutorials.py",
                       "--output", self.evidence / "tutorials"], 300)
        self.stage("desktop", ["bash", ROOT / "scripts/test_desktop.sh"],
                   9000 if self.name == "asan" else 900, graphical=True)

    def execute(self):
        try:
            quality = ROOT / "scripts/quality/tests"
            selection = [quality]
            if self.unit_only:
                # These diagnostic/command-contract regressions use injected
                # results, not sanitizer runtimes, live DDS or a display.
                selection = [quality / module for module in (
                    "test_quality.py", "test_sanitizer_controls.py", "test_install_contract.py",
                    "test_run_checked_xml.py")]
                selection += ["-k", "not test_asan_executable_mapping_keeps_plugins_shared"]
            if not self.stage("coverage-tooling-unit" if self.unit_only else "quality-tests",
                       [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider",
                       *selection, f"--junitxml={self.evidence / 'quality-cases.xml'}"], installed=False):
                raise RuntimeError("Quality tooling tests failed")
            if self.name == "release":
                self.stage("versions", [sys.executable, ROOT / "scripts/checks/check_versions.py"], installed=False)
                self.stage("packages", [sys.executable, ROOT / "scripts/checks/check_release_packages.py",
                           "--work-dir", self.evidence / "packages", "--jobs", str(self.args.jobs),
                           "--continue-on-test-failure"], 4200, installed=False)
            elif self.build():
                self.native()
                if not self.unit_only:
                    self.lifecycle()
                if self.name != "tsan":
                    self.python_units()
                    if not self.unit_only:
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
                    self.stage("external-controller", ["bash", ROOT / "scripts/test_external_controller.sh",
                               self.evidence / "external-controller", self.prefix], 300)
                if self.name == "coverage":
                    self.stage("coverage-report", [sys.executable, ROOT / "scripts/quality/collect_coverage.py",
                               "--root", ROOT, "--prefix", self.prefix, "--build", self.runtime / "build",
                               "--evidence", self.evidence], 300, installed=not self.unit_only)
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
        passed = bool(not self.record.get("error") and self.record["stages"]
                      and all(stage["status"] == "PASS" for stage in self.record["stages"]))
        result = "PASS" if passed else "FAIL"
        self.record.update(scope_details(self.unit_only, result))
        self.record["status"] = "UNIT_ONLY_" + result if self.unit_only else result
        self.save()
        print(f"[{self.name}] {self.record['status']}: {self.evidence / 'result.json'}", flush=True)
        return passed



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("all", *PROFILES), default="all")
    parser.add_argument("--unit-only", action="store_true",
                        help="Coverage only: build every runtime/GUI target, run genuine units and report; not the full quality matrix")
    parser.add_argument("--output", type=Path, required=True, help="Outside the checkout; each profile has an isolated build")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--ros-prefix", type=Path, default=Path("/opt/ros/humble"))
    parser.add_argument("--rmw", default="rmw_fastrtps_cpp")
    args = parser.parse_args()
    if args.unit_only and args.profile != "coverage":
        parser.error("--unit-only is valid only with --profile coverage")
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
            try:
                results[profile] = current.execute()
            except Interrupted as error:
                current.record.update(scope_details(args.unit_only, "CANCELLED"))
                current.record.update(status="UNIT_ONLY_CANCELLED" if args.unit_only else "CANCELLED",
                                      error=f"Interrupted by signal {error.signum}")
                current.save()
                (args.output / "summary.json").write_text(json.dumps({
                    "status": "UNIT_ONLY_CANCELLED" if args.unit_only else "CANCELLED",
                    **scope_details(args.unit_only, "CANCELLED"),
                    "profiles": results, "cancelled_profile": profile,
                    "signal": error.signum}, indent=2) + "\n")
                return 128 + error.signum
            snapshots.append(current.record["source_snapshot"]["sha256"])
        final = source_snapshot(ROOT)
        same_source = set(snapshots + [final["sha256"]]) == {initial["sha256"]}
        passed = all(results.values()) and same_source
        status = "PASS" if passed else "FAIL"
        summary = {"status": "UNIT_ONLY_" + status if args.unit_only else status,
                   **scope_details(args.unit_only, status), "profiles": results,
                   "source_snapshot": initial, "same_source_across_profiles": same_source}
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        return int(not passed)


if __name__ == "__main__":
    raise SystemExit(main())
