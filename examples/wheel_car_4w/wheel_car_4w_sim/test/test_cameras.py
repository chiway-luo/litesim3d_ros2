"""Real Ogre RGB/depth buffers through installed launch and DDS (desktop required)."""
import json
import math
import os
from pathlib import Path
import re
import signal
import struct
import subprocess

import pytest
from sensor_msgs.msg import Image, CameraInfo
from litesim3d_interfaces.srv import Step
from std_msgs.msg import String
import yaml
from test_wheel_car_4w import Example, stamp

pytestmark = pytest.mark.skipif(os.environ.get("LWS_GUI_TESTS") != "1",
                              reason="requires a real graphics backend and DDS")


@pytest.fixture
def camera_car(tmp_path):
    car = Example(tmp_path)
    yield car
    car.close()


def configuration(car, kind="rgb", width=320, height=180, rate=15):
    data = yaml.safe_load((car.share / "config/runtime.yaml").read_text())
    data["lidars"][0]["columns"] = 32
    data["cameras"][0].update(enabled=True, type=kind, width=width, height=height, rate_hz=rate)
    path = car.directory / "runtime.yaml"
    path.write_text(yaml.safe_dump(data))
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
    data=yaml.safe_load(runtime.read_text());data["cameras"][0]["enabled"]=False
    runtime.write_text(yaml.safe_dump(data))
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
    steps(car, count, "sample-window")
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
    steps(car, count, "sample-window");car.spin(.2)
    assert all(not car.messages[c] for c in channels)
    (car.directory / "camera-evidence.json").write_text(json.dumps({"type":kind,"width":width,"height":height,
        "rate_hz":rate,"timestamps_ns":expected,"pause":"PASS","step_retry":"PASS",
        "camera_preview_started":False}, indent=2))


def test_multiple_cameras_independent_rates_and_frame_prefix(camera_car):
    car=camera_car;runtime=configuration(car,kind="rgbd")
    data=yaml.safe_load(runtime.read_text())
    second=dict(data["cameras"][0],name="auxiliary",topic="camera/aux",type="rgb",rate_hz=7,width=160,height=120)
    data["cameras"].append(second);runtime.write_text(yaml.safe_dump(data))
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
    (car.directory/"camera-evidence.json").write_text(json.dumps({"preview_close":"PASS","renderer_failure":"PASS","stats":stats},indent=2))
