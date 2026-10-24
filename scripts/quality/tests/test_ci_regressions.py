"""Regressions for process isolation, plugin hosts and simulation deadlines."""
import importlib.util
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time

import pytest

SCRIPTS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SCRIPTS))
from checks.run_checked import run
from quality.asan_host import prepare
from quality.sanitizer_control import asan_options

spec = importlib.util.spec_from_file_location("sim_timing", SCRIPTS.parent / "litesim3d_test_support/litesim3d_test_support/timing.py")
timing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(timing)


def test_simulation_deadline_is_independent_of_slow_wall_clock(monkeypatch):
    clocks = [0., 0.]
    monkeypatch.setattr(timing.time, "monotonic", lambda: clocks[1])
    def spin(_):
        clocks[0] += .1
        clocks[1] += 2.
    elapsed = timing.until_simulated(lambda: clocks[0] >= 1.1, lambda: clocks[0], spin, 2., wall_timeout=45)
    assert 1.1 <= elapsed < 1.3
    assert clocks[1] > 8  # The old wall-clock assertion failed before expiry.


def test_simulation_deadline_still_rejects_controller_that_never_stops(monkeypatch):
    clocks = [0., 0.]
    monkeypatch.setattr(timing.time, "monotonic", lambda: clocks[1])
    def spin(_):
        clocks[0] += .1
        clocks[1] += .1
    with pytest.raises(AssertionError, match="simulation deadline"):
        timing.until_simulated(lambda: False, lambda: clocks[0], spin, 2.)


def test_frozen_simulation_and_stale_stopped_feedback_fail(monkeypatch):
    wall = [0.]
    monkeypatch.setattr(timing.time, "monotonic", lambda: wall[0])
    def spin(_):
        wall[0] += 1.
    with pytest.raises(AssertionError, match="insufficient progress"):
        timing.until_simulated(lambda: True, lambda: 0., spin, 2., wall_timeout=3)


def test_parent_budget_covers_every_bounded_child_suite():
    env = dict(os.environ, LWS_TEST_CORE_TIMEOUT="1200", LWS_TEST_ROS_TIMEOUT="2400",
               LWS_TEST_SUITE_TIMEOUT="900", LWS_TEST_LAUNCH_SUITE_TIMEOUT="180")
    budget = int(subprocess.check_output(["bash", SCRIPTS / "test_m3.sh", "--timeout-budget"], env=env, text=True))
    assert budget >= 1200 + 2400 + 6 * 900 + 300 + 3 * 60 + 2 * 180


@pytest.mark.skipif(platform.system() != "Linux", reason="Linux process identity regression")
def test_nested_timeout_reaps_escaped_child_and_preserves_unrelated_process(tmp_path):
    child = tmp_path / "child.py"
    pidfile = tmp_path / "child.pid"
    child.write_text("import os, signal, time\nfrom pathlib import Path\n"
                     "signal.signal(signal.SIGINT, signal.SIG_IGN)\n"
                     "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
                     f"Path({str(pidfile)!r}).write_text(str(os.getpid()))\ntime.sleep(60)\n")
    parent = tmp_path / "parent.py"
    parent.write_text("import subprocess, sys, time\n"
                      f"subprocess.Popen([sys.executable, {str(child)!r}], start_new_session=True)\n"
                      "time.sleep(60)\n")
    unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    try:
        command = [sys.executable, str(SCRIPTS / "checks/run_checked.py"), "--timeout", "30",
                   "--xml", str(tmp_path / "inner.xml"), "--log", str(tmp_path / "inner.log"),
                   "--", sys.executable, str(parent)]
        assert run(command, 1.0, tmp_path / "outer.xml", tmp_path / "outer.log") == 124
        assert pidfile.exists()
        pid = int(pidfile.read_text())
        stat = Path(f"/proc/{pid}/stat")
        assert not stat.exists() or stat.read_text().rsplit(")", 1)[1].split()[0] == "Z"
        assert unrelated.poll() is None
    finally:
        unrelated.terminate()
        unrelated.wait(timeout=5)


@pytest.mark.skipif(platform.system() != "Linux", reason="Ubuntu CI cancellation regression")
def test_cancellation_stops_matrix_after_reaping_active_command(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    underlay = tmp_path / "underlay"
    underlay.mkdir()
    (underlay / "setup.bash").write_text("")
    pidfile = tmp_path / "child.pid"
    reached = tmp_path / "continued"
    child = f"import os,time;from pathlib import Path;Path({str(pidfile)!r}).write_text(str(os.getpid()));time.sleep(60)"
    driver = tmp_path / "driver.py"
    driver.write_text(
        "import sys,os\nfrom pathlib import Path\n"
        f"sys.path.insert(0,{str(SCRIPTS)!r})\nimport test_ubuntu as ci\n"
        f"ci.ROOT=Path({str(source)!r})\n"
        "def execute(self):\n"
        f" self.stage('blocking',[sys.executable,'-c',{child!r}],timeout=60,installed=False)\n"
        f" Path({str(reached)!r}).touch()\n return True\n"
        "ci.Profile.execute=execute\nos.environ['ROS_DISTRO']='humble'\n"
        "raise SystemExit(ci.main())\n")
    output = tmp_path / "output"
    with (tmp_path / "driver.log").open("w") as log:
        process = subprocess.Popen([sys.executable, str(driver), "--profile", "all", "--output", str(output),
                                    "--ros-prefix", str(underlay)], stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 10
            while not pidfile.exists() and process.poll() is None and time.monotonic() < deadline:
                time.sleep(.02)
            assert pidfile.exists()
            process.send_signal(signal.SIGTERM)
            assert process.wait(timeout=15) == 128 + signal.SIGTERM
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
    assert not Path(f"/proc/{pidfile.read_text()}").exists()
    assert not reached.exists() and not (output / "coverage").exists()
    summary = json.loads((output / "summary.json").read_text())
    assert summary['status'] == 'CANCELLED' and summary['cancelled_profile'] == 'functional'
    assert json.loads((output / "functional/latest.json").read_text())['stages'][0]['status'] == 'CANCELLED'


@pytest.mark.skipif(platform.system() != "Linux", reason="Clang ELF plugin runtime regression")
def test_asan_host_loads_plugin_and_detects_ub_and_leak_without_preloading_python(tmp_path):
    import shutil
    compiler = shutil.which("clang++-14")
    assert compiler
    underlay = tmp_path / "underlay"
    bindir = underlay / "lib/controller_manager"
    bindir.mkdir(parents=True)
    (underlay / "share/controller_manager").mkdir(parents=True)
    host = tmp_path / "host.cpp"
    host.write_text('#include <dlfcn.h>\n#include <cstdio>\nint main(int argc, char** argv){'
                    'void* h=dlopen(argv[1],RTLD_NOW);if(!h){puts(dlerror());return 7;}'
                    'auto f=(void(*)(int))dlsym(h,"probe");f(argc>2?argv[2][0]:0);dlclose(h);return 0;}')
    plugin = tmp_path / "plugin.cpp"
    plugin.write_text('#include <cstdlib>\n#include <climits>\n'
                      'extern "C" __attribute__((noinline)) void probe(int mode){'
                      'if(mode==108){volatile int* p=(int*)malloc(64);*p=7;}'
                      'if(mode==117){volatile int n=INT_MAX; n+=1;}}')
    binary = bindir / "ros2_control_node"
    library = tmp_path / "plugin.so"
    subprocess.run([compiler, str(host), "-ldl", "-o", str(binary)], check=True)
    subprocess.run([compiler, "-O0", "-g", "-shared", "-fPIC", "-fsanitize=address,undefined",
                    "-fno-sanitize-recover=all", str(plugin), "-o", str(library)], check=True)
    spawner = bindir / "spawner"
    spawner.write_text('#!/usr/bin/env python3\nimport os\nprint(os.environ.get("LD_PRELOAD", ""))\n')
    spawner.chmod(0o755)
    before = os.environ.get("LD_PRELOAD")
    overlay = tmp_path / "overlay"
    prepare(underlay, overlay, compiler)
    env = dict(os.environ)
    for name in ("ASAN_OPTIONS", "UBSAN_OPTIONS", "LSAN_OPTIONS", "LD_PRELOAD"):
        env.pop(name, None)
    env.update(ASAN_OPTIONS=asan_options(shutil.which("llvm-symbolizer-14")),
               UBSAN_OPTIONS="halt_on_error=1:print_stacktrace=1")
    unprepared = subprocess.run([str(binary), str(library)], env=env, capture_output=True, text=True)
    assert unprepared.returncode == 7 and "undefined symbol" in unprepared.stdout
    for mode, expected in (("", ""), ("leak", "Direct leak of 64 byte(s)"), ("undefined", "runtime error:")):
        result = subprocess.run([str(overlay / "lib/controller_manager/ros2_control_node"), str(library), mode],
                                env=env, capture_output=True, text=True, timeout=30)
        assert (result.returncode == 0) == (mode == ""), result.stdout + result.stderr
        assert expected in result.stdout + result.stderr
    result = subprocess.run([str(overlay / "lib/controller_manager/spawner")], env=env, capture_output=True, text=True)
    assert result.stdout.strip() == "" and result.returncode == 0
    assert os.environ.get("LD_PRELOAD") == before
