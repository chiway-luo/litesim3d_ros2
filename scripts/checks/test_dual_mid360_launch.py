"""Installed-example regression: paused startup, resume and bounded failure."""
import os
import signal
import subprocess
import time

import pytest
import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool


@pytest.fixture
def example(tmp_path, monkeypatch):
    # Select an unused localhost domain; never share a running simulator.
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-logs"))
    context = None
    node = None
    for domain in range(92, 99):
        monkeypatch.setenv("ROS_DOMAIN_ID", str(domain))
        context = Context()
        rclpy.init(args=[], context=context)
        node = Node("dual_mid360_startup_probe", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        if not node.count_publishers("/clock") and not any(
                name in {"litesim3d", "controller_manager"}
                for name, _ in node.get_node_names_and_namespaces()):
            break
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
    else:
        pytest.fail("no unused localhost domain in 92–98")

    processes = []

    def launch(timeout):
        log_path = tmp_path / "launch.log"
        with log_path.open("w") as log:
            process = subprocess.Popen([
                "ros2", "launch", "dual_mid360_bringup", "bringup.launch.py",
                "gui:=false", "rviz:=false", "paused:=true",
                f"controller_startup_timeout:={timeout}",
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process, log_path

    def wait(predicate, timeout=25):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if predicate():
                return
        pytest.fail("timed out waiting for the example")

    try:
        yield node, launch, wait
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def test_paused_startup_waits_then_activates_controllers(example):
    node, launch, wait = example
    process, log = launch(45)
    pause = node.create_client(SetBool, "/sim/pause")
    wait(pause.service_is_ready)
    # Exceed the stock spawner's controller-switch timeout while time is frozen.
    deadline = time.monotonic() + 12
    wait(lambda: time.monotonic() >= deadline, timeout=15)
    assert process.poll() is None, log.read_text()
    assert "controller_manager" not in [
        name for name, _ in node.get_node_names_and_namespaces()], log.read_text()
    resumed = pause.call_async(SetBool.Request(data=False))
    wait(resumed.done)
    assert resumed.result().success
    controllers = node.create_client(ListControllers, "/controller_manager/list_controllers")
    wait(controllers.service_is_ready)
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        result = controllers.call_async(ListControllers.Request())
        wait(result.done)
        states = {item.name: item.state for item in result.result().controller}
        if all(states.get(name) == "active" for name in
               ("joint_state_broadcaster", "diff_drive_controller")):
            break
    else:
        pytest.fail(log.read_text())
    assert process.poll() is None, log.read_text()


def test_paused_startup_fails_with_a_bounded_deadline(example):
    _, launch, _ = example
    process, log = launch(3)
    assert process.wait(timeout=30) != 0
    assert "advancing clock not observed" in log.read_text()
