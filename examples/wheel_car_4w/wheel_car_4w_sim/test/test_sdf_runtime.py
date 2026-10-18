"""Actual DDS coverage of SDF scene snapshots, wall-clock RTF and view-only lifecycle."""
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import pytest
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool
import yaml
from test_wheel_car_4w import Example


@pytest.fixture
def example(tmp_path):
    # RTF=0 can saturate a Python TF/IMU observer. This fixture measures wall
    # statistics and committed time; sensor and TF delivery have separate tests.
    car = Example(tmp_path, monitor_only=True)
    car.subscribe("stats", String, "/sim/stats")
    car.subscribe("scene_state", String, "/sim/scene_state")
    data = yaml.safe_load((car.share / "config/runtime.yaml").read_text())
    lidar = data["lidars"][0]
    lidar.update(columns=32, rings=1, vertical_angles_deg=[0], ring_delays_s=[0],
                 scan_mode="snapshot", publish_range_boundary=False, range_sigma_m=0)
    runtime = tmp_path / "runtime.yaml"
    runtime.write_text(yaml.safe_dump(data))
    yield car, runtime
    car.close()


@pytest.mark.parametrize("target", [.25, .5, 1, 2, 0])
def test_actual_rtf_matches_committed_clock_and_target(example, target):
    car, runtime = example
    # Rate-control precision uses the small reference scene, independently of
    # the more expensive office example (which may legitimately run below 1x).
    car.start(world="warehouse.sdf", runtime=str(runtime), real_time_factor=str(target), paused="false")
    samples = []
    scene_samples = []
    sub = car.node.create_subscription(String, "/sim/stats",
          lambda m: samples.append((time.monotonic(), json.loads(m.data))), qos_profile_sensor_data)
    scene_sub = car.node.create_subscription(String, "/sim/scene_state",
          lambda m: scene_samples.append((time.monotonic(), json.loads(m.data))), qos_profile_sensor_data)
    car.spin(10.4 if target in {.25, .5, 1} else 2.0)
    car.node.destroy_subscription(sub)
    car.node.destroy_subscription(scene_sub)
    assert len(samples) >= 5
    # Both slower wall timers must remain live under an always-ready RTF=0
    # physics timer, including when the executor is overloaded.
    assert len(scene_samples) >= 10
    assert scene_samples[-1][0] - scene_samples[0][0] >= 1.0
    assert scene_samples[-1][1]["stamp_ns"] > scene_samples[0][1]["stamp_ns"]
    end_wall, end = samples[-1]
    start_wall, start = min(samples[:-1], key=lambda pair: abs(end_wall - pair[0] - 1.0))
    measured = (end["stamp_ns"] - start["stamp_ns"]) * 1e-9 / (end_wall - start_wall)
    assert end["target_real_time_factor"] == target
    assert end["actual_real_time_factor"] == pytest.approx(measured, rel=.2, abs=.04)
    if target:
        assert end["actual_real_time_factor"] == pytest.approx(target, rel=.25, abs=.04)
        if target in {.25, .5, 1}:
            first_wall, first=samples[0]
            mean=(end["stamp_ns"]-first["stamp_ns"])*1e-9/(end_wall-first_wall)
            assert end_wall-first_wall>=9.8
            assert mean==pytest.approx(target,abs=.05)
    else:
        assert end["actual_real_time_factor"] > 0
    assert end["collision_budget_enabled"] is False
    state = json.loads(car.messages["scene_state"][-1].data)
    assert state["run_id"] == end["run_id"]
    assert "front_left_wheel_link" in state["link_poses"]
    (car.directory / "rtf-evidence.json").write_text(json.dumps({"target": target,
        "measured": measured, "reported": end["actual_real_time_factor"], "samples": samples}, indent=2))


def test_pause_keeps_wall_statistics_alive_and_clock_stops(example):
    car, runtime = example
    car.start(runtime=str(runtime), paused="false", collision_budget_enabled="true")
    client = car.node.create_client(SetBool, "/sim/pause")
    assert client.wait_for_service(timeout_sec=5)
    future = client.call_async(SetBool.Request(data=True))
    car.until(future.done)
    assert future.result().success
    car.until(lambda: car.messages["stats"] and json.loads(car.messages["stats"][-1].data)["state"] == "paused")
    before = json.loads(car.messages["stats"][-1].data)
    car.messages["stats"].clear()
    car.spin(.9)
    assert len(car.messages["stats"]) >= 3
    for m in car.messages["stats"]:
        value = json.loads(m.data)
        assert value["stamp_ns"] == before["stamp_ns"]
        assert value["actual_real_time_factor"] == 0
        assert value["collision_budget_enabled"] is True
    car.node.destroy_client(client)


@pytest.mark.skipif(os.environ.get("LWS_GUI_TESTS") != "1", reason="requires a real desktop graphics session")
def test_viewer_only_subscribes_and_can_close_without_stopping_simulation(example):
    car, runtime = example
    car.start(runtime=str(runtime), gui="true", paused="false", real_time_factor="0.5")
    publishers = car.node.get_publisher_names_and_types_by_node("litesim_viewer", "/")
    assert set(name for name, _ in publishers) <= {"/rosout", "/parameter_events"}
    subscriptions = car.node.get_subscriber_names_and_types_by_node("litesim_viewer", "/")
    assert {"/sim/scene_state", "/sim/stats"} <= set(name for name, _ in subscriptions)
    log = (car.directory / "launch.log").read_text()
    match = re.search(r"\[viewer-\d+\]: process started with pid \[(\d+)\]", log)
    assert match, log
    before = json.loads(car.messages["stats"][-1].data)["stamp_ns"]
    os.kill(int(match.group(1)), signal.SIGTERM)
    car.spin(.8)
    assert car.process.poll() is None
    assert json.loads(car.messages["stats"][-1].data)["stamp_ns"] > before
    # Reopen using only the latched ROS description, without launch's temp path.
    with (car.directory / "reconnect.log").open("w") as output:
        child=subprocess.Popen(["ros2","run","litesim3d_gui","viewer","--ros-args",
            "-p","exit_after_frames:=35","-p",f"capture_path:={car.directory / 'reconnected.png'}"],
            env=car.environment,stdout=output,stderr=subprocess.STDOUT)
        try:
            car.until(lambda: child.poll() is not None, timeout=15)
            assert child.returncode==0,(car.directory / "reconnect.log").read_text()
            assert (car.directory / "reconnected.png").is_file()
            from PyQt5.QtGui import QImage
            image=QImage(str(car.directory / "reconnected.png"))
            assert any(image.pixelColor(x,y).lightness()>40
                for y in range(100,image.height()-100,80) for x in range(100,image.width()-100,80)),"reconnected scene is black"
        finally:
            if child.poll() is None:
                child.terminate();child.wait(timeout=5)
