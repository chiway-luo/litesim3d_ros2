from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
from litesim3d_bringup.model import prepare_config, resolve


def fixture(tmp_path):
    source = tmp_path / "source"; source.mkdir()
    (source / "mechanical.urdf").write_text('<robot name="test"><link name="base_link"/></robot>')
    (source / "pattern.csv").write_text("time,azimuth,elevation\n0,0,0\n")
    (source / "lidar.yaml").write_text('lidar: {link: base_link, topic: scan, pattern_file: pattern.csv}\n')
    (source / "simulation.urdf.xacro").write_text('''<robot name="test" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:arg name="height" default="0.4"/>
      <xacro:include filename="mechanical.urdf"/>
      <link name="lidar_link"/>
      <joint name="mount" type="fixed"><parent link="base_link"/><child link="lidar_link"/><origin xyz="0 0 $(arg height)"/></joint>
      <litesim3d version="1"><sensor name="lidar" class="litesim3d_lidar/Lidar">
        <parameters file="lidar.yaml" section="lidar"/>
        <overrides>rate_hz: 5.0</overrides>
      </sensor></litesim3d>
    </robot>''')
    config = source / "config.yaml"
    config.write_text('robot: {urdf: simulation.urdf.xacro}\n')
    return source, config


def test_expand_once_preserves_source_and_relative_resources(tmp_path):
    source, config = fixture(tmp_path)
    before = {p.name:p.read_bytes() for p in source.iterdir()}
    result = prepare_config(config, tmp_path / "prepared", mappings={"height":.7})
    robot = ET.parse(yaml.safe_load(result.read_text())["robot"]["urdf"]).getroot()
    assert {l.get("name") for l in robot.findall("link")} == {"base_link", "lidar_link"}
    assert robot.find("joint/origin").get("xyz") == "0 0 0.7"
    values = yaml.safe_load(Path(robot.find("./litesim3d/sensor/parameters").get("file")).read_text())
    assert values["lidar"]["pattern_file"] == str(source / "pattern.csv")
    assert before == {p.name:p.read_bytes() for p in source.iterdir()}


def test_profile_replaces_scan_fields_and_keeps_bindings(tmp_path):
    _, config = fixture(tmp_path)
    profile = tmp_path / "profile.yaml"
    profile.write_text("lidar: {columns: 20000, rings: 1, scan_pattern: livox_mid360, point_layout: xyzirt_v1}\n")
    prepared = prepare_config(config, tmp_path / "prepared", lidar_profile=str(profile))
    robot = ET.parse(yaml.safe_load(prepared.read_text())["robot"]["urdf"]).getroot()
    values = yaml.safe_load(robot.find("./litesim3d/sensor/overrides").text)
    assert values["vertical_angles_deg"] == [0] and values["ring_delays_s"] == []
    assert "rings" not in values and "point_layout" not in values
    assert robot.find("./litesim3d/sensor").get("name") == "lidar"


def test_duplicates_and_invalid_override_are_not_silently_normalized(tmp_path):
    _, config = fixture(tmp_path)
    with pytest.raises(ValueError, match="requires a camera"):
        prepare_config(config, tmp_path / "bad-camera", cameras="true")
    with pytest.raises(ValueError, match="mapping of scalar"):
        prepare_config(config, tmp_path / "bad-args", mappings=["height"])
    with pytest.raises(ValueError, match="lidar_profile must be"):
        prepare_config(config, tmp_path / "missing-profile", lidar_profile=str(tmp_path / "absent.yaml"))
    config.write_text("robot: {urdf: simulation.urdf.xacro}\nrobot: {}\n")
    with pytest.raises(ValueError, match="duplicate YAML key"):
        prepare_config(config, tmp_path / "bad-duplicate")
    with pytest.raises(ValueError, match="invalid package resource"):
        resolve("package://litesim3d_core/../secret", tmp_path)


def test_noise_config_overrides_profile_without_changing_bindings_or_source(tmp_path):
    source, config = fixture(tmp_path)
    profile = tmp_path / "lidar.yaml"
    profile.write_text("lidar: {range_sigma_m: 0.02, columns: 16}\n")
    noise = tmp_path / "noise.yaml"
    noise.write_text("sensors: {lidar: {range_bias_m: 0.01, range_sigma_m: 0.0}}\n")
    before = {p.name: p.read_bytes() for p in source.iterdir()}
    prepared = prepare_config(config, tmp_path / "prepared", lidar_profile=str(profile), noise_config=str(noise))
    sensor = ET.parse(yaml.safe_load(prepared.read_text())["robot"]["urdf"]).find("./litesim3d/sensor")
    values = yaml.safe_load(sensor.find("overrides").text)
    assert values["range_sigma_m"] == 0 and values["range_bias_m"] == 0.01
    assert values["columns"] == 16 and values["rate_hz"] == 5
    assert sensor.get("name") == "lidar" and sensor.get("class") == "litesim3d_lidar/Lidar"
    assert before == {p.name: p.read_bytes() for p in source.iterdir()}


@pytest.mark.parametrize("document, message", [
    ("sensors: {missing: {range_sigma_m: 0.1}}", "unknown sensor"),
    ("sensors: {lidar: {topic: changed}}", "unsupported noise fields"),
    ("sensors: {lidar: {gyro_sigma: [0,0,0]}}", "unsupported noise fields"),
    ("sensors: {lidar: []}", "unsupported noise fields"),
    ("sensors: []", "must be a mapping"),
    ("simulation: {seed: 1}", "accepts only"),
    ("odometry: {linear_scale_error: [0.1, 0]}", "requires one standard"),
    ("sensors: {}\nsensors: {}", "duplicate YAML key"),
])
def test_noise_config_rejects_ambiguous_or_nonmeasurement_changes(tmp_path, document, message):
    _, config = fixture(tmp_path)
    noise = tmp_path / "noise.yaml"; noise.write_text(document)
    with pytest.raises(ValueError, match=message):
        prepare_config(config, tmp_path / "bad", noise_config=str(noise))


def test_noise_config_handles_odometry_and_all_five_sensor_kinds(tmp_path):
    from litesim3d_bringup.model import apply_noise_config
    robot = ET.fromstring("""<robot><litesim3d>
      <odometry name="encoders" class="litesim3d_omni_odometry/OmniOdometry"/>
      <sensor name="lidar" class="litesim3d_lidar/Lidar"/>
      <sensor name="imu" class="litesim3d_imu/Imu"/>
      <sensor name="rgb" class="litesim3d_rgb_camera/RgbCamera"/>
      <sensor name="depth" class="litesim3d_depth_camera/DepthCamera"/>
      <sensor name="rgbd" class="litesim3d_rgbd_camera/RgbdCamera"/>
    </litesim3d></robot>""")
    values = {
        "odometry": {"linear_scale_error": [0.02, 0], "angular_bias_radps": 0.01},
        "sensors": {
            "lidar": {"range_bias_m": 0.01, "range_sigma_m": 0.02, "dropout": 0.01},
            "imu": {"accel_bias": [0.1, 0, 0], "gyro_sigma": [0.01, 0.01, 0.01]},
            "rgb": {"color_sigma_dn": [1, 2, 3]},
            "depth": {"depth_sigma_m": 0.02},
            "rgbd": {"color_bias_dn": [1, 0, 0], "depth_bias_m": 0.01},
        },
    }
    noise = tmp_path / "noise.yaml"; noise.write_text(yaml.safe_dump(values))
    apply_noise_config({}, robot, noise, tmp_path)
    assert yaml.safe_load(robot.find("./litesim3d/odometry/overrides").text)["noise"] == values["odometry"]
    for sensor in robot.findall("./litesim3d/sensor"):
        assert yaml.safe_load(sensor.find("overrides").text) == values["sensors"][sensor.get("name")]
    # Legacy drone/SDF sensors have the same measurement fields.
    data = {"lidars": [{"name": "lidar"}], "imus": [{"name": "imu"}],
            "cameras": [{"name": kind, "type": kind} for kind in ("rgb", "depth", "rgbd")]}
    del values["odometry"]; noise.write_text(yaml.safe_dump(values))
    apply_noise_config(data, None, noise, tmp_path)
    for group in data.values():
        for sensor in group:
            assert all(sensor[key] == value for key, value in values["sensors"][sensor["name"]].items())


def test_mid360_preview_follows_resolved_binding_without_touching_other_lidars(tmp_path):
    from litesim3d_bringup.model import lidar_preview_topics
    _, config = fixture(tmp_path)
    profile = tmp_path / 'profile.yaml'
    profile.write_text('lidar: {scan_pattern: livox_mid360, columns: 20000, rings: 1}\n')
    prepared = prepare_config(config, tmp_path/'prepared', lidar_profile=str(profile))
    assert lidar_preview_topics(prepared, 'auto', 1) == {
        '/scan':'/livox/lidar/pointcloud', '/scan/range_boundary':'/livox/lidar/range_boundary'}
    assert lidar_preview_topics(prepared, 'livox', 0)['/scan'] == '/livox/lidar'
    assert lidar_preview_topics(prepared, 'native', 1) == {}
    native = prepare_config(config, tmp_path/'native')
    assert lidar_preview_topics(native, 'auto', 1) == {}
