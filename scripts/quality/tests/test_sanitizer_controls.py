"""Sanitizer-control regressions; all compiler/probe subprocesses are faked.

These fixtures validate tooling, not this host's LSan/TSan capability.
"""
import json
from pathlib import Path
import subprocess
import sys

import pytest


QUALITY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(QUALITY))
import check_sanitizers
import control_plugin_control
import sanitizer_control


PTRACE_FATAL = """==25==LeakSanitizer has encountered a fatal error.
==25==HINT: For debugging, try setting environment variable LSAN_OPTIONS=verbosity=1:log_threads=1
==25==HINT: LeakSanitizer does not work under ptrace (strace, gdb, etc)
"""
LEAK = """=================================================================
==25==ERROR: LeakSanitizer: detected memory leaks

Direct leak of 64 byte(s) in 1 object(s) allocated from:
    #0 0x4a290d in malloc /compiler-rt/lib/asan/asan_malloc_linux.cpp:69:3
    #1 0x4d2591 in leak() /tmp/leak.cpp:2:69
    #2 0x4d2613 in main /tmp/leak.cpp:3:12

SUMMARY: AddressSanitizer: 64 byte(s) leaked in 1 allocation(s).
"""
SUPPRESSIONS = """-----------------------------------------------------
Suppressions used:
  count      bytes template
      7        352 pluginlib::ClassLoader<litesim3d::plugins::ChassisPlugin>::loadLibraryForClass
-----------------------------------------------------
"""
REGISTRATION = """==25==ERROR: LeakSanitizer: detected memory leaks

Direct leak of 112 byte(s) in 2 object(s) allocated from:
    #0 0x4a290d in malloc /compiler-rt/lib/asan/asan_malloc_linux.cpp:69:3
    #1 0x7fff2581 in class_loader::impl::loadLibrary(std::string const&, class_loader::ClassLoader*) /opt/ros/humble/lib/libclass_loader.so

Indirect leak of 240 byte(s) in 5 object(s) allocated from:
    #0 0x4a290d in malloc /compiler-rt/lib/asan/asan_malloc_linux.cpp:69:3
    #1 0x7fff2591 in std::string::_M_create(unsigned long&, unsigned long) /usr/lib/libstdc++.so

SUMMARY: AddressSanitizer: 352 byte(s) leaked in 7 allocation(s).
"""
UNDEFINED = "/tmp/undefined.cpp:2:47: runtime error: signed integer overflow: 2147483647 + 1 cannot be represented in type 'int'\n"
RACE = "WARNING: ThreadSanitizer: data race (pid=25)\nSUMMARY: ThreadSanitizer: data race /tmp/race.cpp:3:41 in write()\n"


def run_sanitizer(monkeypatch, tmp_path, leak=(1, LEAK), *, mode="asan", race=(66, RACE),
                  undefined=(1, UNDEFINED), plugin=False):
    """Exercise the real argument parsing, result aggregation and file writes."""
    output = tmp_path / "positive-control"
    argv = ["sanitizer_control.py", "--compiler", "fake-c++", "--mode", mode,
            "--output", str(output)]
    if plugin:
        argv += ["--plugin-probe", str(tmp_path / "fake-plugin")]
    monkeypatch.setattr(sys, "argv", argv)
    monkeypatch.setattr(sanitizer_control, "asan_link_options", lambda llvm_unwind=False: [])
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        executable = Path(command[0]).name
        if executable == "fake-c++":
            return subprocess.CompletedProcess(command, 0)
        response = {"leak": leak, "undefined": undefined, "race": race,
                    "fake-plugin": leak}[executable]
        if isinstance(response, Exception):
            raise response
        code, text = response
        assert kwargs["timeout"] == 30
        return subprocess.CompletedProcess(command, code, stdout=text.encode())

    monkeypatch.setattr(sanitizer_control.subprocess, "run", fake_run)
    code = sanitizer_control.main()
    result_path = output / ("plugin-result.json" if plugin else "result.json")
    return code, json.loads(result_path.read_text()), output, calls


def test_ptrace_fatal_is_not_the_intentional_leak(monkeypatch, tmp_path):
    # The original broad substring predicate accepted the actual local failure.
    assert 1 != 0 and "LeakSanitizer" in PTRACE_FATAL
    code, result, output, _ = run_sanitizer(monkeypatch, tmp_path, (1, PTRACE_FATAL))
    assert code == 1
    assert result["status"] == "FAIL"
    assert result["cases"]["leak"]["classification"] == "unsupported"
    assert result["cases"]["leak"]["detected"] is False
    assert result["cases"]["leak"]["passed"] is False
    assert result["cases"]["undefined"]["passed"] is True
    assert (output / "leak.log").read_text() == PTRACE_FATAL


@pytest.mark.parametrize("summary", ["AddressSanitizer", "LeakSanitizer"])
def test_complete_64_byte_control_passes(monkeypatch, tmp_path, summary):
    report = LEAK.replace("SUMMARY: AddressSanitizer", "SUMMARY: " + summary)
    code, result, _, calls = run_sanitizer(monkeypatch, tmp_path, (1, report))
    assert code == 0
    assert result["status"] == "PASS"
    assert result["cases"]["leak"] == {
        "exit_code": 1, "detected": True, "passed": True, "classification": "detected"}
    assert result["cases"]["undefined"]["passed"] is True
    for command, kwargs in calls:
        if command[0] != "fake-c++":
            options = kwargs["env"]["ASAN_OPTIONS"]
            assert options.startswith("detect_leaks=1:halt_on_error=1:")
            assert "fast_unwind_on_malloc=0:malloc_context_size=64:" in options


@pytest.mark.parametrize("returncode,report,classification", [
    (1, "LeakSanitizer is enabled\n", "not_detected"),
    (1, "", "not_detected"),
    (1, LEAK.split("SUMMARY:")[0], "incomplete_report"),
    (1, LEAK.replace("==25==ERROR: LeakSanitizer: detected memory leaks", ""), "not_detected"),
    (1, LEAK.replace("64 byte(s) leaked", "65 byte(s) leaked"), "incomplete_report"),
    (1, LEAK.replace("1 allocation(s)", "2 allocation(s)"), "incomplete_report"),
    (1, LEAK.replace("    #0", "    #9"), "incomplete_report"),
    (1, LEAK.replace("    #1", "    #7"), "incomplete_report"),
    (1, "\n".join(line for line in LEAK.splitlines() if not line.startswith(("    #1", "    #2"))), "incomplete_report"),
    (1, "\n".join(line for line in LEAK.splitlines() if not line.startswith("    #")), "incomplete_report"),
    (1, LEAK.replace("64 byte(s)", "128 byte(s)"), "unexpected_report"),
    (1, LEAK.replace("Direct leak", "Indirect leak"), "unexpected_report"),
    (1, LEAK.replace("1 object(s)", "2 object(s)").replace("1 allocation(s)", "2 allocation(s)"), "unexpected_report"),
    (1, LEAK + LEAK, "incomplete_report"),
    (1, LEAK + "Direct leak of 64 byte(s) in 1 object(s) allocated from:\n", "incomplete_report"),
    (1, LEAK + UNDEFINED, "unexpected_diagnostic"),
    (1, PTRACE_FATAL + LEAK, "unsupported"),
    (1, "==25==LeakSanitizer has encountered a fatal error.\n" + LEAK, "fatal"),
    (1, "AddressSanitizer:DEADLYSIGNAL\n" + LEAK, "crash"),
    (1, "AddressSanitizer: CHECK failed: sanitizer_common.cpp:1\n" + LEAK, "fatal"),
    (1, "ASan runtime does not come first in initial library list\n" + LEAK, "fatal"),
    (-11, LEAK, "signal"),
    (-6, LEAK, "signal"),
    (139, LEAK, "signal"),
    (0, LEAK, "zero_exit"),
])
def test_leak_control_rejects_invalid_evidence(monkeypatch, tmp_path, returncode, report, classification):
    code, result, _, _ = run_sanitizer(monkeypatch, tmp_path, (returncode, report))
    assert code == 1
    assert result["status"] == "FAIL"
    assert result["cases"]["leak"]["passed"] is False
    assert result["cases"]["leak"]["classification"] == classification


def test_timeout_replaces_stale_pass_and_keeps_partial_log(monkeypatch, tmp_path):
    run_sanitizer(monkeypatch, tmp_path)
    error = subprocess.TimeoutExpired("leak", 30, output=LEAK.encode())
    code, result, output, _ = run_sanitizer(monkeypatch, tmp_path, error)
    assert code == 1
    assert result["status"] == "FAIL"
    assert result["cases"]["leak"] == {
        "exit_code": None, "detected": False, "passed": False, "classification": "timeout"}
    assert (output / "leak.log").read_text() == LEAK


@pytest.mark.parametrize("plugin", [False, True])
def test_failed_compile_or_exec_cannot_leave_stale_pass(monkeypatch, tmp_path, plugin):
    _, _, output, _ = run_sanitizer(monkeypatch, tmp_path, (1, LEAK + SUPPRESSIONS), plugin=plugin)
    result_path = output / ("plugin-result.json" if plugin else "result.json")
    assert json.loads(result_path.read_text())["status"] == "PASS"

    def fail(*args, **kwargs):
        raise OSError("fixture compiler or probe unavailable")

    monkeypatch.setattr(sanitizer_control.subprocess, "run", fail)
    with pytest.raises(OSError, match="unavailable"):
        sanitizer_control.main()
    assert json.loads(result_path.read_text())["status"] == "FAIL"


@pytest.mark.parametrize("returncode,report,passed", [
    (1, UNDEFINED, True),
    (1, UNDEFINED + "SUMMARY: UndefinedBehaviorSanitizer: undefined-behavior /tmp/undefined.cpp:2:47 in \n", True),
    (1, "runtime error: misaligned address\n", False),
    (1, UNDEFINED + LEAK, False),
    (1, UNDEFINED + PTRACE_FATAL, False),
    (0, UNDEFINED, False),
    (-6, UNDEFINED, False),
])
def test_ub_control_requires_its_own_finding(monkeypatch, tmp_path, returncode, report, passed):
    code, result, _, _ = run_sanitizer(monkeypatch, tmp_path, undefined=(returncode, report))
    assert code == int(not passed)
    assert result["cases"]["undefined"]["passed"] is passed


@pytest.mark.parametrize("returncode,report,classification", [
    (66, RACE, "detected"),
    (66, RACE.split("SUMMARY:")[0], "incomplete_report"),
    (66, RACE.splitlines()[1], "incomplete_report"),
    (66, RACE.replace("data race (pid=25)", "data races disabled"), "incomplete_report"),
    (66, RACE + LEAK, "incomplete_report"),
    (66, "FATAL: ThreadSanitizer: unexpected memory mapping 0x123-0x456\n" + RACE, "unsupported"),
    (66, "FATAL: ThreadSanitizer: failed to mmap\n" + RACE, "fatal"),
    (0, RACE, "zero_exit"),
    (-11, RACE, "signal"),
])
def test_race_control_requires_complete_nonfatal_diagnostic(monkeypatch, tmp_path, returncode, report, classification):
    code, result, _, _ = run_sanitizer(monkeypatch, tmp_path, mode="tsan", race=(returncode, report))
    assert code == int(classification != "detected")
    assert result["cases"]["race"]["classification"] == classification


@pytest.mark.parametrize("suppression,passed", [
    (SUPPRESSIONS, True),
    ("Suppressions used:\n", False),
    ("", False),
    (SUPPRESSIONS.replace("ChassisPlugin", "SensorPlugin"), False),
    (SUPPRESSIONS.replace("      7", "      0"), False),
    (SUPPRESSIONS.replace("352", "0"), False),
    (SUPPRESSIONS.replace("loadLibraryForClass", "loadLibraryForClassExtra"), False),
    (SUPPRESSIONS.rsplit("-----------------------------------------------------", 1)[0], False),
])
def test_plugin_control_requires_real_matching_suppression(monkeypatch, tmp_path, suppression, passed):
    code, result, _, _ = run_sanitizer(monkeypatch, tmp_path, (1, LEAK + suppression), plugin=True)
    assert code == int(not passed)
    assert result["instance_leak_detected_with_registration_suppression"] is passed
    assert result["classification"] == ("detected" if passed else "missing_suppression")


@pytest.mark.parametrize("returncode,report", [
    (0, LEAK + SUPPRESSIONS),
    (-11, LEAK + SUPPRESSIONS),
    (1, PTRACE_FATAL + LEAK + SUPPRESSIONS),
    (1, LEAK.split("SUMMARY:")[0] + SUPPRESSIONS),
])
def test_plugin_control_cannot_accept_failure_or_incomplete_report(monkeypatch, tmp_path, returncode, report):
    code, result, _, _ = run_sanitizer(monkeypatch, tmp_path, (returncode, report), plugin=True)
    assert code == 1
    assert result["status"] == "FAIL"
    assert result["instance_leak_detected_with_registration_suppression"] is False


def registration_suppression(loader, kind):
    base = ("hardware_interface::SystemInterface" if kind == "hardware" else
            "controller_interface::ControllerInterface")
    rule = (f"pluginlib::ClassLoader<{base}>::loadLibraryForClass" if loader == "native" else
            "class_loader::impl::loadLibrary(")
    return SUPPRESSIONS.replace(sanitizer_control.PLUGIN_REGISTRATION_RULE, rule)


def run_control_plugins(monkeypatch, tmp_path, overrides=None):
    output = tmp_path / "control"
    monkeypatch.setattr(sys, "argv", ["control_plugin_control.py", "--compiler", "fake-c++",
                                     "--output", str(output)])
    monkeypatch.setattr(control_plugin_control.shutil, "which", lambda _: "/fake/llvm-symbolizer-14")
    monkeypatch.setattr(control_plugin_control, "runtime_library", lambda _: Path("/fake/libasan.so"))
    monkeypatch.setattr(control_plugin_control, "write_wrapper", lambda *args: None)
    calls = []

    def fake_run(command, **kwargs):
        calls.append(command)
        if command[0] == "cmake":
            return subprocess.CompletedProcess(command, 0)
        loader = "native" if Path(command[0]).name == "control_plugin_probe" else "stripped-host"
        kind = command[1]
        env = kwargs["env"]
        mode = ("leak" if env.get("LITESIM_PROBE_LEAK") else
                "clean" if env["LSAN_OPTIONS"] else "registration")
        suppression = registration_suppression(loader, kind)
        returncode, text = (overrides or {}).get(f"{loader}-{kind}-{mode}", {
            "registration": (1, REGISTRATION), "clean": (0, suppression), "leak": (1, LEAK + suppression)}[mode])
        return subprocess.CompletedProcess(command, returncode, stdout=text.encode())

    monkeypatch.setattr(control_plugin_control.subprocess, "run", fake_run)
    code = control_plugin_control.main()
    return code, json.loads((output / "result.json").read_text()), output, calls


def test_all_native_and_stripped_control_cases_pass_with_real_evidence(monkeypatch, tmp_path):
    code, result, _, calls = run_control_plugins(monkeypatch, tmp_path)
    assert code == 0
    assert result["status"] == "PASS"
    assert len(result["cases"]) == 12
    assert all(case["passed"] for case in result["cases"].values())
    assert len(calls) == 14  # Two fake CMake invocations and twelve fake probes.


def test_failed_control_plugin_build_cannot_leave_stale_pass(monkeypatch, tmp_path):
    _, _, output, _ = run_control_plugins(monkeypatch, tmp_path)

    def fail(command, **kwargs):
        raise subprocess.CalledProcessError(1, command)

    monkeypatch.setattr(control_plugin_control.subprocess, "run", fail)
    with pytest.raises(subprocess.CalledProcessError):
        control_plugin_control.main()
    assert json.loads((output / "result.json").read_text())["status"] == "FAIL"


@pytest.mark.parametrize("loader", ["native", "stripped-host"])
@pytest.mark.parametrize("kind", ["hardware", "controller"])
@pytest.mark.parametrize("mode", ["registration", "clean", "leak"])
def test_any_control_plugin_runtime_failure_fails_aggregate(monkeypatch, tmp_path, loader, kind, mode):
    name = f"{loader}-{kind}-{mode}"
    report = PTRACE_FATAL + (REGISTRATION if mode == "registration" else LEAK)
    report += registration_suppression(loader, kind)
    code, result, output, _ = run_control_plugins(monkeypatch, tmp_path, {name: (1, report)})
    assert code == 1
    assert result["status"] == "FAIL"
    assert result["cases"][name]["classification"] == "unsupported"
    assert sum(not case["passed"] for case in result["cases"].values()) == 1
    assert (output / f"{name}.log").read_text() == report


@pytest.mark.parametrize("mode,returncode,report,classification", [
    ("registration", 1, LEAK + "class_loader::impl::loadLibrary(\n", "unexpected_report"),
    ("registration", 1, REGISTRATION.split("SUMMARY:")[0], "incomplete_report"),
    ("registration", 0, REGISTRATION, "zero_exit"),
    ("registration", -11, REGISTRATION, "signal"),
    ("clean", 0, LEAK, "unexpected_diagnostic"),
    ("clean", 0, UNDEFINED, "unexpected_diagnostic"),
    ("clean", 0, "", "missing_suppression"),
    ("clean", 0, SUPPRESSIONS, "missing_suppression"),
    ("clean", 1, registration_suppression("native", "hardware"), "nonzero_exit"),
    ("leak", 1, LEAK + SUPPRESSIONS, "missing_suppression"),
    ("leak", 1, LEAK + "Suppressions used:\n", "missing_suppression"),
])
def test_control_plugin_requires_stage_specific_evidence(monkeypatch, tmp_path, mode, returncode, report, classification):
    name = f"native-hardware-{mode}"
    code, result, _, _ = run_control_plugins(monkeypatch, tmp_path, {name: (returncode, report)})
    assert code == 1
    assert result["cases"][name]["classification"] == classification


@pytest.mark.parametrize("diagnostic", [
    PTRACE_FATAL.splitlines()[0],
    PTRACE_FATAL.splitlines()[2],
    "FATAL: ThreadSanitizer: unexpected memory mapping 0x123-0x456",
    "AddressSanitizer:DEADLYSIGNAL",
    "AddressSanitizer: CHECK failed: sanitizer_common.cpp:1",
    "SUMMARY: LeakSanitizer: 64 byte(s) leaked in 1 allocation(s).",
    "ASan runtime does not come first in initial library list",
    UNDEFINED.strip(),
])
def test_child_log_audit_rejects_runtime_failures_and_findings(tmp_path, diagnostic):
    path = tmp_path / "child.log"
    path.write_text("normal startup\n" + diagnostic + "\n")
    assert check_sanitizers.findings(tmp_path) == [{"file": str(path), "line": 2, "message": diagnostic}]


def test_standalone_lsan_logs_are_audited_without_flagging_suppression_tables(tmp_path):
    (tmp_path / "lsan.25").write_text(PTRACE_FATAL)
    (tmp_path / "clean.log").write_text(SUPPRESSIONS)
    expected = tmp_path / "positive-control"
    expected.mkdir()
    (expected / "leak.log").write_text(LEAK)
    findings = check_sanitizers.findings(tmp_path)
    assert len(findings) == 2
    assert all(Path(item["file"]).name == "lsan.25" for item in findings)


def test_report_checker_fails_even_when_parent_and_control_statuses_pass(monkeypatch, tmp_path):
    evidence = tmp_path / "evidence"
    control = evidence / "positive-control"
    (control / "control").mkdir(parents=True)
    for relative in ("result.json", "plugin-result.json", "control/result.json"):
        (control / relative).write_text('{"status": "PASS"}')
    (evidence / "child.log").write_text(PTRACE_FATAL)
    monkeypatch.setattr(sys, "argv", ["check_sanitizers.py", "--root", str(tmp_path),
                                     "--build", str(tmp_path / "build"), "--evidence", str(evidence),
                                     "--mode", "asan"])
    monkeypatch.setattr(check_sanitizers, "inventory", lambda _: [])
    monkeypatch.setattr(check_sanitizers, "compilation_errors", lambda *args: [])
    assert check_sanitizers.main() == 1
    result = json.loads((evidence / "sanitizers.json").read_text())
    assert result["status"] == "FAIL"
    assert result["failures"] == ["2 sanitizer diagnostic(s), including child processes"]
    assert len(result["findings"]) == 2
