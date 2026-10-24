"""Real Ogre RGB/depth buffers through installed launch and DDS (desktop required)."""
import json
import hashlib
import math
import os
from pathlib import Path
import re
import signal
import statistics
import struct
import subprocess

import pytest
from sensor_msgs.msg import Image, CameraInfo
from litesim3d_interfaces.srv import Step
from std_msgs.msg import String
import yaml
import copy
import xml.etree.ElementTree as ET
from litesim3d_bringup.model import prepare_config, override
from test_wheel_car_4w import Example, stamp

pytestmark = pytest.mark.skipif(os.environ.get("LWS_GUI_TESTS") != "1",
                              reason="requires a real graphics backend and DDS")


@pytest.fixture
def camera_car(tmp_path):
    car = Example(tmp_path)
    yield car
    car.close(require_clean_children=True,
              expected_child_exits=getattr(car, "expected_child_exits", None))


def configuration(car, kind="rgb", width=320, height=180, rate=15):
    path = prepare_config(car.share / "config/runtime_twist.yaml", car.directory / "model")
    data = yaml.safe_load(path.read_text())
    model = ET.parse(data["robot"]["urdf"])
    override(model.find("./litesim3d/sensor[@name='roof_lidar']"), {"columns":32})
    camera = model.find("./litesim3d/sensor[@name='front_camera']")
    package = f"litesim3d_{kind}_camera"
    camera.set("class", package + "/" + {"rgb":"RgbCamera", "rgbd":"RgbdCamera", "depth":"DepthCamera"}[kind])
    camera.find("parameters").set("file", f"package://{package}/config/parameters.yaml")
    override(camera, {"enabled":True, "width":width, "height":height, "rate_hz":rate})
    model.write(data["robot"]["urdf"], encoding="unicode")
    return path


def subscribe(car, kind):
    for channel in ("color", "depth"):
        if kind == "rgbd" or kind == ("rgb" if channel == "color" else "depth"):
            car.subscribe(channel, Image, f"/camera/front/{channel}/image_raw", 100)
            car.subscribe(channel + "_info", CameraInfo, f"/camera/front/{channel}/camera_info", 100)


def steps(car, count, identity):
    client = car.node.create_client(Step, "/sim/step")
    assert client.wait_for_service(timeout_sec=5)
    future = client.call_async(Step.Request(request_id=identity, steps=count))
    car.until(future.done, timeout=35)
    assert future.result().success, future.result().message
    car.node.destroy_client(client)


def assert_no_camera_preview(car):
    assert not any(name.startswith("camera_view_") or name == "litesim_camera_view"
                   for name in car.node.get_node_names())
    assert not re.search(r"\[camera_view-\d+\]: process started", (car.directory / "launch.log").read_text())


def test_default_camera_publishes_rgb_without_preview_or_depth(camera_car):
    car = camera_car
    subscribe(car, "rgb")
    car.start(cameras="true", paused="true", reliable_sensors=True)
    assert_no_camera_preview(car)
    for suffix in ("image_raw", "camera_info"):
        assert car.node.count_publishers(f"/camera/front/depth/{suffix}") == 0
    car.until(lambda: json.loads(car.status().get("subscription_matches", "{}"))
        .get("/camera/front/color/image_raw", 0) == 1)
    steps(car, 120, "default-rgb")
    car.until(lambda: len(car.messages["color"]) == 9 and len(car.messages["color_info"]) == 9)
    expected = [1 + round(i * 1e9 / 15) for i in range(1, 10)]
    assert [stamp(m.header.stamp) for m in car.messages["color"]] == expected
    for image in car.messages["color"]:
        assert (image.width, image.height, image.encoding) == (640, 480, "rgb8")
    assert not any(name.startswith("/camera/front/depth/") for name, _ in car.node.get_topic_names_and_types())
    (car.directory / "camera-evidence.json").write_text(json.dumps({
        "type": "rgb", "width": 640, "height": 480, "rate_hz": 15,
        "timestamps_ns": expected, "depth_topics": [], "camera_preview_started": False}, indent=2))


@pytest.mark.parametrize("kind,width,height,rate", [("rgbd",320,180,15), ("rgb",160,120,7), ("depth",224,128,11), ("rgbd",640,480,30)])
def test_images_resolution_calibration_exact_stamps_and_pause(camera_car, kind, width, height, rate):
    car = camera_car
    runtime = configuration(car, kind, width, height, rate)
    subscribe(car, kind)
    # The launch override also exercises the disabled-by-default example path.
    data=yaml.safe_load(runtime.read_text());model=ET.parse(data["robot"]["urdf"])
    override(model.find("./litesim3d/sensor[@name='front_camera']"), {"enabled":False})
    model.write(data["robot"]["urdf"], encoding="unicode")
    car.start(runtime=str(runtime), cameras="true", paused="true", reliable_sensors=True)
    assert_no_camera_preview(car)
    channels = [c for c in ("color", "depth") if c in car.messages]
    for channel in set(("color", "depth")) - set(channels):
        for suffix in ("image_raw", "camera_info"):
            assert car.node.count_publishers(f"/camera/front/{channel}/{suffix}") == 0
    car.until(lambda: all(json.loads(car.status().get("subscription_matches", "{}"))
        .get(f"/camera/front/{channel}/image_raw", 0) >= 1 for channel in channels))
    count=100 if rate==30 else 200
    frames=rate//2 if rate==30 else rate
    # Drain each batch before advancing again. The publisher has KEEP_LAST(5)
    # history; reliable transport is not an unbounded burst recorder. At most
    # three frames per batch keeps this exact-payload test within that contract.
    requests = [(min(20, count - offset), f"sample-window-{offset}") for offset in range(0, count, 20)]
    completed = 0
    for batch, identity in requests:
        steps(car, batch, identity)
        completed += batch
        expected_count = completed * rate // 200  # This example uses 5 ms steps.
        car.until(lambda: all(len(car.messages[c]) == expected_count
            and len(car.messages[c+"_info"]) == expected_count for c in channels))
    car.until(lambda: all(len(car.messages[c]) == frames and len(car.messages[c+"_info"]) == frames for c in channels))
    expected = [1 + round(i * 1e9 / rate) for i in range(1, frames+1)]
    for channel in channels:
        assert [stamp(m.header.stamp) for m in car.messages[channel]] == expected
        assert [stamp(m.header.stamp) for m in car.messages[channel+"_info"]] == expected
        for image, info in zip(car.messages[channel], car.messages[channel+"_info"]):
            assert image.header == info.header
            assert image.header.frame_id == "camera_optical_frame"
            assert (image.width, image.height, info.width, info.height) == (width, height, width, height)
            assert image.encoding == ("rgb8" if channel == "color" else "32FC1")
            assert image.step == width * (3 if channel == "color" else 4)
            assert len(image.data) == height * image.step
            assert info.k[0] == pytest.approx(width/(2*math.tan(math.radians(35))))
            assert info.k[4] == info.k[0] and info.k[2] == (width-1)/2 and info.k[5] == (height-1)/2
            assert list(info.d) == [0.0]*5
        if channel == "color":
            image = car.messages[channel][-1]
            assert max(image.data) > 80 and len(set(image.data)) > 20
            from PyQt5.QtGui import QImage
            assert QImage(bytes(image.data), width, height, image.step, QImage.Format_RGB888).save(str(car.directory / "rgb.png"))
        else:
            values = struct.unpack("<" + "f"*(width*height), bytes(car.messages[channel][-1].data))
            finite = [v for v in values if math.isfinite(v)]
            assert len(finite) > width*height*.1 and all(.1 <= v < 20 for v in finite)
            assert max(finite)-min(finite) > .5
            (car.directory / "depth.f32").write_bytes(bytes(car.messages[channel][-1].data))
    car.spin(.2)
    for c in channels:car.messages[c].clear()
    car.spin(.5)
    assert all(not car.messages[c] for c in channels), "paused camera continued sampling"
    # A retry of an idempotent step must not republish old frames.
    for batch, identity in requests:
        steps(car, batch, identity)
    car.spin(.2)
    assert all(not car.messages[c] for c in channels)
    (car.directory / "camera-evidence.json").write_text(json.dumps({"type":kind,"width":width,"height":height,
        "rate_hz":rate,"timestamps_ns":expected,"pause":"PASS","step_retry":"PASS",
        "camera_preview_started":False}, indent=2))


def test_multiple_cameras_independent_rates_and_frame_prefix(camera_car):
    car=camera_car;runtime=configuration(car,kind="rgbd")
    data=yaml.safe_load(runtime.read_text())
    model=ET.parse(data["robot"]["urdf"])
    second=copy.deepcopy(model.find("./litesim3d/sensor[@name='front_camera']"))
    second.set("name","auxiliary");second.set("class","litesim3d_rgb_camera/RgbCamera")
    second.find("parameters").set("file","package://litesim3d_rgb_camera/config/parameters.yaml")
    override(second,{"topic":"camera/aux","rate_hz":7,"width":160,"height":120})
    model.find("litesim3d").append(second);model.write(data["robot"]["urdf"], encoding="unicode")
    params=yaml.safe_load((car.share/"config/ros.yaml").read_text())
    params["/litesim3d"]["ros__parameters"]["frame_prefix"]="rig/"
    ros=car.directory/"ros.yaml";ros.write_text(yaml.safe_dump(params))
    subscribe(car,"rgbd");car.subscribe("aux",Image,"/camera/aux/color/image_raw",100)
    car.start(runtime=str(runtime),params=str(ros),paused="true",reliable_sensors=True)
    assert_no_camera_preview(car)
    car.until(lambda:json.loads(car.status().get("subscription_matches","{}")).get("/camera/aux/color/image_raw",0)>=1)
    steps(car,200,"multi")
    car.until(lambda:len(car.messages["color"])==15 and len(car.messages["depth"])==15 and len(car.messages["aux"])==7)
    assert [stamp(m.header.stamp) for m in car.messages["aux"]]==[1+round(i*1e9/7) for i in range(1,8)]
    assert all(m.header.frame_id=="rig/camera_optical_frame" for key in ("color","depth","aux") for m in car.messages[key])
    assert car.buffer.can_transform("rig/base_link","rig/camera_optical_frame",__import__('rclpy').time.Time())


def test_camera_disable_override_does_not_start_renderer(camera_car):
    car=camera_car;runtime=configuration(car)
    car.start(runtime=str(runtime),cameras="false",paused="true")
    assert_no_camera_preview(car)
    assert "litesim_camera_renderer" not in car.node.get_node_names()
    assert car.node.count_publishers("/camera/front/color/image_raw")==0
    steps(car,40,"no-camera")
    car.until(lambda:bool(car.messages["cloud"]))


def render_error_samples(directory, kind):
    """Render co-located cameras so error checks use real ideal pixel values."""
    car = Example(directory)
    channels = ["color", "depth"] if kind == "rgbd" else ["color" if kind == "rgb" else "depth"]
    try:
        runtime = configuration(car, kind, width=160, height=120, rate=10)
        data = yaml.safe_load(runtime.read_text())
        model = ET.parse(data["robot"]["urdf"])
        original = model.find("./litesim3d/sensor[@name='front_camera']")
        errors = {}
        for variant in ("ideal", "bias", "noise"):
            name = "front_camera" if variant == "ideal" else "camera_" + variant
            sensor = original if variant == "ideal" else copy.deepcopy(original)
            sensor.set("name", name)
            override(sensor, {"topic": "camera/" + variant})
            if variant != "ideal":
                model.find("litesim3d").append(sensor)
            values = {}
            if "color" in channels:
                values.update(color_bias_dn=[0, 0, 0] if variant == "ideal" else [7, -5, 11],
                              color_sigma_dn=[2, 3, 4] if variant == "noise" else [0, 0, 0])
            if "depth" in channels:
                values.update(depth_bias_m=0 if variant == "ideal" else .05,
                              depth_sigma_m=.01 if variant == "noise" else 0)
            errors[name] = values
            for channel in channels:
                key = variant + "_" + channel
                topic = f"/camera/{variant}/{channel}"
                car.subscribe(key, Image, topic + "/image_raw", 100)
                car.subscribe(key + "_info", CameraInfo, topic + "/camera_info", 100)
        model.write(data["robot"]["urdf"], encoding="unicode")
        noise = directory / "noise.yaml"
        noise.write_text(yaml.safe_dump({"sensors": errors}))
        car.start(runtime=str(runtime), noise_config=str(noise), paused="true", reliable_sensors=True)
        topics = [f"/camera/{v}/{c}/image_raw" for v in ("ideal", "bias", "noise") for c in channels]
        car.until(lambda: all(json.loads(car.status().get("subscription_matches", "{}")).get(t, 0) >= 1
                             for t in topics))
        keys = [v + "_" + c for v in ("ideal", "bias", "noise") for c in channels]
        for frame in range(1, 4):
            steps(car, 20, f"render-errors-{frame}")
            car.until(lambda: all(len(car.messages[k]) == frame and len(car.messages[k+"_info"]) == frame
                                 for k in keys))
        samples = {key: list(car.messages[key]) for key in keys}
        for key, frames in samples.items():
            assert [stamp(m.header.stamp) for m in frames] == [100_000_001, 200_000_001, 300_000_001]
            for image, info in zip(frames, car.messages[key + "_info"]):
                assert image.header == info.header
                ideal_info = car.messages["ideal_" + key.rsplit("_", 1)[1] + "_info"][0]
                assert list(info.k) == list(ideal_info.k) and list(info.d) == list(ideal_info.d)
            if key.endswith("_color"):
                from PyQt5.QtGui import QImage
                image = frames[0]
                assert QImage(bytes(image.data), image.width, image.height, image.step,
                              QImage.Format_RGB888).save(str(directory / (key + ".png")))
        return samples
    finally:
        car.close(require_clean_children=True)


@pytest.mark.parametrize("kind", ["rgb", "depth", "rgbd"])
def test_rendered_noise_profiles_bias_statistics_and_seed_replay(tmp_path, kind):
    samples = render_error_samples(tmp_path / "first", kind)
    replay = render_error_samples(tmp_path / "replay", kind)
    evidence = {"kind": kind, "seed_replay": True, "statistics": {}, "sha256": {}}
    for key, frames in samples.items():
        assert [bytes(m.data) for m in frames] == [bytes(m.data) for m in replay[key]], key
        evidence["sha256"][key] = [hashlib.sha256(bytes(m.data)).hexdigest() for m in frames]
    for channel in ("color", "depth"):
        if "ideal_" + channel not in samples:
            continue
        ideal = samples["ideal_" + channel]
        biased = samples["bias_" + channel]
        noisy = samples["noise_" + channel]
        assert bytes(ideal[0].data) == bytes(ideal[1].data), "zero-error static rendering changed"
        assert bytes(noisy[0].data) != bytes(noisy[1].data), "noise must vary across timestamps"
        if channel == "color":
            raw = bytes(ideal[0].data)
            assert max(raw) > 80 and len(set(raw)) > 20, "renderer returned a blank image"
            bias = [7, -5, 11]
            assert bytes(biased[0].data) == bytes(max(0, min(255, v + bias[i % 3]))
                                               for i, v in enumerate(raw))
            for component, sigma in enumerate((2, 3, 4)):
                # Exclude clipping when estimating the Gaussian distribution.
                residuals = [float(m.data[i]) - raw[i] - bias[component] for m in noisy
                             for i in range(component, len(raw), 3) if 32 <= raw[i] <= 223]
                assert len(residuals) > 1000
                mean, deviation = statistics.mean(residuals), statistics.pstdev(residuals)
                assert abs(mean) < .15 and deviation == pytest.approx(sigma, rel=.08)
                evidence["statistics"][f"color_{component}"] = {"mean_dn": mean, "sigma_dn": deviation}
        else:
            def values(image):
                assert image.encoding == "32FC1" and not image.is_bigendian
                return struct.unpack("<" + "f" * (image.width * image.height), bytes(image.data))
            raw, shifted = values(ideal[0]), values(biased[0])
            assert sum(map(math.isfinite, raw)) > len(raw) * .1
            for value, actual in zip(raw, shifted):
                if math.isfinite(value) and .1 <= value + .05 <= 20:
                    assert actual == pytest.approx(value + .05, abs=2e-6)
                else:
                    assert math.isnan(actual)
            residuals = [value - raw[i] - .05 for m in noisy for i, value in enumerate(values(m))
                         if math.isfinite(raw[i]) and .2 < raw[i] < 19.8]
            assert len(residuals) > 1000 and all(map(math.isfinite, residuals))
            mean, deviation = statistics.mean(residuals), statistics.pstdev(residuals)
            assert abs(mean) < .0005 and deviation == pytest.approx(.01, rel=.05)
            evidence["statistics"]["depth"] = {"mean_m": mean, "sigma_m": deviation}
    (tmp_path / "camera-noise-evidence.json").write_text(json.dumps(evidence, indent=2))


def test_closing_previews_preserves_camera_data_and_renderer_failure_is_fatal(camera_car):
    car=camera_car;runtime=configuration(car,kind="rgbd",width=320,height=240,rate=10)
    subscribe(car,"rgbd");car.subscribe("stats",String,"/sim/stats")
    car.start(runtime=str(runtime), gui="true", camera_view="true", paused="false", real_time_factor="0.5", reliable_sensors=True)
    car.until(lambda:len(car.messages["color"])>=5, timeout=20)
    # A separately reopened preview must consume DDS without changing simulator state.
    with (car.directory/"preview.log").open("w") as log:
        preview=subprocess.Popen(["ros2","run","litesim3d_gui","camera_view","--ros-args",
            "-p","type:=rgbd","-p","rate_hz:=10.0","-p","exit_after_frames:=8","-p",f"capture_path:={car.directory/'preview.png'}"],
            env=car.environment,stdout=log,stderr=subprocess.STDOUT)
        try:
            car.until(lambda:preview.poll() is not None, timeout=20)
            assert preview.returncode==0,(car.directory/"preview.log").read_text()
            assert (car.directory/"preview.png").is_file()
        finally:
            if preview.poll() is None:preview.terminate();preview.wait(timeout=5)
    for executable in ("viewer", "camera_view"):
        match=re.search(r"\["+executable+r"-\d+\]: process started with pid \[(\d+)\]",(car.directory/"launch.log").read_text())
        assert match,executable
        os.kill(int(match.group(1)),signal.SIGTERM)
    before=stamp(car.messages["color"][-1].header.stamp)
    car.until(lambda:stamp(car.messages["color"][-1].header.stamp)>before+500_000_000,timeout=10)
    assert car.process.poll() is None
    stats=json.loads(car.messages["stats"][-1].data)
    assert 0 < stats["actual_real_time_factor"] <= .6
    publisher_types=dict(car.node.get_publisher_names_and_types_by_node("litesim_camera_renderer","/"))
    assert set(publisher_types) <= {"/rosout","/parameter_events"}
    match=re.search(r"\[camera_node-\d+\]: process started with pid \[(\d+)\]",(car.directory/"launch.log").read_text())
    assert match
    os.kill(int(match.group(1)),signal.SIGTERM)
    car.until(lambda:car.process.poll() is not None,timeout=12)
    assert car.process.returncode!=0
    # Either launch sees the renderer exit first and shuts the simulator down
    # cleanly, or the simulator observes the disconnect and fails explicitly.
    # Permit only that diagnosed fault on that exact PID; every other child
    # must still exit normally, and sanitizer log checks remain independent.
    output = (car.directory / "launch.log").read_text()
    failed_sim = re.search(r"\[sim_node-\d+\]: process has died \[pid (\d+), exit code (-?\d+)", output)
    if failed_sim:
        assert int(failed_sim[2]) == 1 and "simulation fault: camera renderer disconnected" in output
        car.expected_child_exits = {failed_sim[1]: 1}
    (car.directory/"camera-evidence.json").write_text(json.dumps({"preview_close":"PASS","renderer_failure":"PASS","stats":stats},indent=2))
