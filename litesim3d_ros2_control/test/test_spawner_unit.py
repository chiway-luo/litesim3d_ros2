"""Readiness state-machine units with injected clock/ROS boundaries.

No ROS context, publisher, transport, subprocess, or simulator is started.
"""
import argparse
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


def load():
    path = Path(__file__).resolve().parents[1] / "scripts/spawn_when_ready.py"
    spec = importlib.util.spec_from_file_location("spawner_readiness_unit", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def status(ready=False, state="running"):
    return SimpleNamespace(status=[SimpleNamespace(values=[
        SimpleNamespace(key="ready", value=str(ready).lower()),
        SimpleNamespace(key="state", value=state)])])


def clock(nanoseconds):
    sec, nanosec = divmod(nanoseconds, 1_000_000_000)
    return SimpleNamespace(clock=SimpleNamespace(sec=sec, nanosec=nanosec))


def boundary(monkeypatch, module, events, stopped=False):
    callbacks = {}
    calls = []

    class Context:
        def __init__(self):
            self.active = not stopped

        def ok(self):
            return self.active

        def shutdown(self):
            calls.append("context.shutdown")
            self.active = False

    class Node:
        def __init__(self, name, context):
            calls.append(("node", name))

        def create_subscription(self, message, topic, callback, qos):
            callbacks[topic] = callback

        def destroy_node(self):
            calls.append("node.destroy")

    class Executor:
        def __init__(self, context):
            self.context = context

        def add_node(self, node):
            calls.append("executor.add")

        def spin_once(self, timeout_sec):
            calls.append(("spin", timeout_sec))
            if events:
                event = events.pop(0)
                if isinstance(event, Exception):
                    raise event
                topic, message = event
                callbacks[topic](message)

        def shutdown(self):
            calls.append("executor.shutdown")

    now = [0.]

    def monotonic():
        now[0] += .01
        return now[0]

    monkeypatch.setattr(module, "Context", Context)
    monkeypatch.setattr(module, "Node", Node)
    monkeypatch.setattr(module, "SingleThreadedExecutor", Executor)
    monkeypatch.setattr(module.rclpy, "init", lambda **kwargs: calls.append("rclpy.init"))
    monkeypatch.setattr(module.time, "monotonic", monotonic)
    return calls


@pytest.mark.parametrize("value", ["0.001", "1", "3600"])
def test_positive_timeout_valid(value):
    assert load().positive_timeout(value) == float(value)


@pytest.mark.parametrize("value", ["0", "-1", "3600.1", "inf", "-inf", "nan"])
def test_positive_timeout_rejects_nonfinite_and_outside_boundaries(value):
    with pytest.raises(argparse.ArgumentTypeError, match="finite"):
        load().positive_timeout(value)


def test_clock_ignored_until_ready_repeats_do_not_advance_and_rewind_resets(monkeypatch):
    module = load()
    events = [("clock", clock(100)), ("status", status()), ("status", status(True)),
              ("clock", clock(10)), ("clock", clock(10)), ("clock", clock(5)),
              ("clock", clock(6))]
    calls = boundary(monkeypatch, module, events)
    assert module.wait_for_simulation(1, "status", "clock") is None
    assert not events
    assert len([call for call in calls if isinstance(call, tuple) and call[0] == "spin"]) == 7
    assert calls[-3:] == ["executor.shutdown", "node.destroy", "context.shutdown"]


def test_readiness_drop_discards_prior_clock_progress(monkeypatch):
    module = load()
    events = [("status", status(True)), ("clock", clock(100)),
              ("status", status(False)), ("clock", clock(101)),
              ("status", status(True)), ("clock", clock(1)), ("clock", clock(2))]
    calls = boundary(monkeypatch, module, events)
    module.wait_for_simulation(1, "status", "clock")
    assert not events
    assert calls[-3:] == ["executor.shutdown", "node.destroy", "context.shutdown"]


@pytest.mark.parametrize("events,expected", [
    ([("status", status(state="fault"))], "faulted"),
    ([], "not observed"),
    ([RuntimeError("executor failed")], "executor failed"),
])
def test_fault_timeout_and_dispatch_error_always_clean_up(monkeypatch, events, expected):
    module = load()
    calls = boundary(monkeypatch, module, list(events))
    with pytest.raises(RuntimeError, match=expected):
        module.wait_for_simulation(.04, "status", "clock")
    assert calls[-3:] == ["executor.shutdown", "node.destroy", "context.shutdown"]


def test_already_shutdown_context_is_not_shutdown_twice(monkeypatch):
    module = load()
    calls = boundary(monkeypatch, module, [], stopped=True)
    with pytest.raises(RuntimeError, match="not observed"):
        module.wait_for_simulation(.04, "status", "clock")
    assert calls[-2:] == ["executor.shutdown", "node.destroy"]
    assert "context.shutdown" not in calls
    assert not any(isinstance(call, tuple) and call[0] == "spin" for call in calls)


class ExecRecorded(BaseException):
    pass


@pytest.mark.parametrize("executable,package", [
    ("spawner", "controller_manager"),
    ("ros2_control_node", "litesim3d_ros2_control"),
])
@pytest.mark.parametrize("separator", [True, False])
def test_main_preserves_exact_forwarded_arguments(monkeypatch, executable, package, separator):
    module = load()
    forwarded = ["joint_state_broadcaster", "-c", "/private/manager", "--ros-args", "-r", "a:=b"]
    argv = ["spawn_when_ready.py", "--timeout", "5", "--status-topic", "/status",
            "--clock-topic", "/clock", "--executable", executable]
    argv += (["--"] if separator else []) + forwarded
    monkeypatch.setattr(module.sys, "argv", argv)
    waited = []
    monkeypatch.setattr(module, "wait_for_simulation", lambda *args: waited.append(args))
    monkeypatch.setattr(module, "get_package_prefix", lambda value: "/opt/test" if value == package else pytest.fail("wrong package"))
    recorded = []

    def execute(path, args):
        recorded.append((path, args))
        raise ExecRecorded()

    monkeypatch.setattr(module.os, "execv", execute)
    with pytest.raises(ExecRecorded):
        module.main()
    path = f"/opt/test/lib/{package}/{executable}"
    assert recorded == [(path, [path, *forwarded])]
    assert waited == [(5., "/status", "/clock")]


def test_main_requires_forwarded_arguments(monkeypatch, capsys):
    module = load()
    monkeypatch.setattr(module.sys, "argv", ["spawn_when_ready.py", "--"])
    with pytest.raises(SystemExit) as error:
        module.main()
    assert error.value.code == 2
    assert "provide stock controller_manager" in capsys.readouterr().err


@pytest.mark.parametrize("failing", ["wait", "lookup", "exec"])
def test_main_returns_failure_for_boundary_exceptions(monkeypatch, capsys, failing):
    module = load()
    monkeypatch.setattr(module.sys, "argv", ["spawn_when_ready.py", "--", "controller"])

    def fail(*args):
        raise RuntimeError("injected boundary failure")

    monkeypatch.setattr(module, "wait_for_simulation", fail if failing == "wait" else lambda *args: None)
    monkeypatch.setattr(module, "get_package_prefix", fail if failing == "lookup" else lambda *args: "/prefix")
    monkeypatch.setattr(module.os, "execv", fail)
    assert module.main() == 1
    assert "controller startup: injected boundary failure" in capsys.readouterr().err
