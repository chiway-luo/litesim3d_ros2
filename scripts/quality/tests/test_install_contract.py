"""Disposable-install contracts without compiling or launching ROS processes.

The checker uses a tiny source fixture and fake subprocess results. The real
Linux build script is also run with a recording colcon stub to verify its
default test build and the packaging-only override at the command boundary.
"""
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys
from types import SimpleNamespace

import pytest


SCRIPTS = Path(__file__).resolve().parents[2]
CHECK_INSTALL = SCRIPTS / "checks/check_install.py"


@pytest.fixture
def install_probe(monkeypatch, tmp_path):
    """Run the actual checker, including copy, rename, reporting and exit code."""
    source = tmp_path / "source"
    source.mkdir()
    (source / "source-marker").write_text("disposable source\n")
    stage = tmp_path / "stage"
    stage.mkdir()
    output = tmp_path / "evidence"
    prefix = stage / "artifacts/install"
    calls = []

    def invoke(exit_codes=None):
        exit_codes = exit_codes or {}

        def fake_run(command, **kwargs):
            phase = "build" if not calls else command[3]
            # Capture the environment now: the checker updates it for runtime.
            calls.append((command, dict(kwargs, env=dict(kwargs["env"]))))
            if phase == "build":
                assert (stage / "project/source-marker").is_file()
                assert not (stage / "sources-unavailable").exists()
                prefix.mkdir(parents=True)
                (prefix / "setup.bash").write_text("# fake installed environment\n")
            else:
                assert not (stage / "project").exists()
                assert (stage / "sources-unavailable/source-marker").is_file()
            return subprocess.CompletedProcess(command, exit_codes.get(phase, 0),
                                               stdout="", stderr="")

        with monkeypatch.context() as patch:
            patch.syspath_prepend(str(SCRIPTS))
            import paths
            patch.setattr(paths, "REPO_ROOT", source)
            patch.setattr(paths, "EVIDENCE_DIR", output)
            patch.setenv("LWS_INSTALL_EVIDENCE", str(output))
            # An inherited headless setting must not weaken packaging coverage.
            patch.setenv("LWS_BUILD_GUI", "0")
            patch.setenv("LWS_BUILD_ROOT", str(tmp_path / "parent-build"))
            patch.setenv("LITESIM3D_ARTIFACTS_DIR", str(tmp_path / "parent-artifacts"))
            patch.setattr("tempfile.mkdtemp", lambda **kwargs: str(stage))
            patch.setattr(subprocess, "run", fake_run)
            with pytest.raises(SystemExit) as stopped:
                runpy.run_path(str(CHECK_INSTALL), run_name="__main__")
        return SimpleNamespace(code=stopped.value.code, calls=calls, source=source,
                               stage=stage, output=output, prefix=prefix,
                               record=json.loads((output / "install-check.json").read_text()))

    return invoke


def test_disposable_build_keeps_full_graph_and_timeout_but_omits_test_compilation(install_probe):
    result = install_probe()
    command, kwargs = result.calls[0]
    assert command == [
        sys.executable, str(result.source / "scripts/checks/run_checked.py"),
        "--timeout", "900", "--xml", str(result.output / "install-build-runner.xml"),
        "--log", str(result.output / "install-isolated-build.log"), "--",
        "bash", str(result.stage / "project/scripts/build.sh"), "-DBUILD_TESTING=OFF",
    ]
    assert kwargs["env"]["LWS_BUILD_GUI"] == "1"
    for name in ("LWS_BUILD_ROOT", "LITESIM3D_ARTIFACTS_DIR"):
        assert kwargs["env"][name] == str(result.stage / "artifacts")
    assert kwargs["env"]["PYTHONDONTWRITEBYTECODE"] == "1"
    assert result.code == 0
    assert result.record["status"] == "PASS"


def test_all_installed_checks_run_after_sources_are_hidden(install_probe):
    result = install_probe()
    assert len(result.calls) == 4
    launch, headless, mesh = [call[0] for call in result.calls[1:]]
    assert launch == [
        "bash", "-c", 'source "$1/setup.bash"; exec launch_test "$2" --junit-xml "$3"',
        "install-check", str(result.prefix),
        str(result.source / "litesim3d_ros2/test/launch_m1.py"),
        str(result.output / "install-launch.xml"),
    ]
    assert headless == [
        "bash", "-c", 'set -e; source "$1/setup.bash"; shift; exec "$@"',
        "install-headless", str(result.prefix), str(result.prefix / "bin/sim_headless"),
        str(result.prefix / "share/litesim3d_core/config/demo.yaml"), "200",
        str(result.output / "install-manifest.json"), ".4", "0",
    ]
    assert mesh == [
        "bash", "-c",
        'source "$1/setup.bash"; export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1; '
        'exec "$2" -m pytest -q "$3" --basetemp "$4" --junitxml "$5" -o "cache_dir=$6"',
        "install-mesh", str(result.prefix), sys.executable,
        str(result.source / "litesim3d_ros2/test/test_delivery.py")
        + "::test_T_URD_11_installed_mesh_resources",
        str(result.output / "install-mesh"), str(result.output / "install-mesh.xml"),
        str(result.output / "pytest-cache"),
    ]
    for _, kwargs in result.calls[1:]:
        assert kwargs["cwd"] == result.stage
        assert kwargs["timeout"] == 30
        assert kwargs["env"]["ROS_DOMAIN_ID"] == "7"
        assert kwargs["env"]["ROS_LOCALHOST_ONLY"] == "1"
        assert kwargs["env"]["ROS_LOG_DIR"] == str(result.output / "install-ros-logs")
    assert not (result.stage / "project").exists()
    assert (result.stage / "sources-unavailable/source-marker").is_file()
    assert result.record["source_hidden"] is True
    assert result.record["prefix"] == str(result.prefix)
    for phase in ("build", "launch", "headless", "mesh"):
        assert result.record[f"{phase}_exit"] == 0


def test_failed_build_stops_before_hiding_sources_or_running_installed_checks(install_probe):
    result = install_probe({"build": 124})
    assert result.code == 124
    assert len(result.calls) == 1
    assert (result.stage / "project/source-marker").is_file()
    assert not (result.stage / "sources-unavailable").exists()
    assert result.record == {"status": "FAIL", "phase": "build",
                             "stage": str(result.stage), "build_exit": 124}


@pytest.mark.parametrize("phase,field", [("install-check", "launch_exit"),
                                         ("install-headless", "headless_exit"),
                                         ("install-mesh", "mesh_exit")])
def test_each_installed_check_remains_required(install_probe, phase, field):
    result = install_probe({phase: 9})
    assert result.code == 1
    assert len(result.calls) == 4
    assert result.record["status"] == "FAIL"
    assert result.record["source_hidden"] is True
    assert result.record[field] == 9


@pytest.mark.skipif(sys.platform != "linux", reason="Ubuntu colcon invocation contract")
def test_real_build_script_defaults_on_and_forwards_packaging_override_last(install_probe, tmp_path):
    result = install_probe()
    build_command = result.calls[0][0]
    packaging_args = build_command[build_command.index("--") + 3:]
    assert packaging_args == ["-DBUILD_TESTING=OFF"]
    tools = tmp_path / "tools"
    tools.mkdir()
    colcon = tools / "colcon"
    colcon.write_text(
        f"#!{sys.executable}\nimport json, os, sys\n"
        "from pathlib import Path\n"
        "Path(os.environ['COLCON_RECORD']).write_text(json.dumps({"
        "'argv': sys.argv[1:], 'gui': os.environ.get('LWS_BUILD_GUI')}))\n")
    colcon.chmod(0o755)
    for name, overrides in (("default", []), ("packaging", packaging_args)):
        artifacts = tmp_path / name
        record = tmp_path / f"{name}-colcon.json"
        env = dict(os.environ, PATH=str(tools) + os.pathsep + os.environ["PATH"],
                   LITESIM3D_ARTIFACTS_DIR=str(artifacts), LWS_BUILD_ROOT=str(artifacts),
                   LWS_BUILD_GUI=result.calls[0][1]["env"]["LWS_BUILD_GUI"],
                   COLCON_RECORD=str(record), PYTHONDONTWRITEBYTECODE="1")
        built = subprocess.run(["bash", str(SCRIPTS / "build.sh"), *overrides],
                               env=env, capture_output=True, text=True, timeout=30)
        assert built.returncode == 0, built.stdout + built.stderr
        captured = json.loads(record.read_text())
        # No package selection/ignore flags: the runtime and GUI graph is intact.
        assert captured["gui"] == "1"
        assert captured["argv"] == [
            "--log-base", str(artifacts / "log"), "build", "--base-paths", str(SCRIPTS.parent),
            "--build-base", str(artifacts / "build"), "--install-base", str(artifacts / "install"),
            "--merge-install", "--parallel-workers", "2", "--cmake-args",
            "-DCMAKE_BUILD_TYPE=RelWithDebInfo", "-DBUILD_TESTING=ON", *overrides,
        ]
