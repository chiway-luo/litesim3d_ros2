from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
from litesim3d_bringup.model import prepare_config, resolve, ros_robot_description


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


@pytest.mark.parametrize("configured_urdf", ["", None, "outdated/missing.urdf"])
def test_launch_supplied_model_preserves_yaml_and_model_relative_resources(tmp_path, monkeypatch, configured_urdf):
    source, _ = fixture(tmp_path)
    installed = source.rename(tmp_path / "安装 模型")
    config_dir = tmp_path / "configuration"
    config_dir.mkdir()
    config = config_dir / "runtime.yaml"
    config.write_text(yaml.safe_dump({"robot": {"urdf": configured_urdf}}))
    before = config.read_bytes()
    # Launch may run from any directory; model resources belong to its share.
    monkeypatch.chdir(config_dir)
    result = prepare_config(config, tmp_path / "prepared", robot_urdf=installed / "simulation.urdf.xacro",
                            mappings={"height": .7})
    generated = Path(yaml.safe_load(result.read_text())["robot"]["urdf"])
    robot = ET.parse(generated).getroot()
    assert generated.parent == tmp_path / "prepared"
    assert {link.get("name") for link in robot.findall("link")} == {"base_link", "lidar_link"}
    assert robot.find("joint/origin").get("xyz") == "0 0 0.7"
    parameters = Path(robot.find("./litesim3d/sensor/parameters").get("file"))
    assert yaml.safe_load(parameters.read_text())["lidar"]["pattern_file"] == str(installed / "pattern.csv")
    assert config.read_bytes() == before


def test_invalid_launch_model_does_not_fall_back_to_yaml(tmp_path):
    _, config = fixture(tmp_path)
    with pytest.raises(ValueError, match="resource not found"):
        prepare_config(config, tmp_path / "prepared", robot_urdf=tmp_path / "missing.xacro")


def test_ros_mesh_urls_preserve_spaces_unicode_and_kinematics(tmp_path):
    source = tmp_path / "雷达 mesh"; source.mkdir()
    mesh, texture = source / "mid 360.dae", source / "body.png"
    mesh.touch(); texture.touch()
    root = ET.fromstring('''<robot name="mesh_robot"><link name="base_link">
      <visual><geometry><mesh/></geometry><material name="skin"><texture/></material></visual>
      <collision><geometry><mesh/></geometry></collision></link><link name="sensor"/>
      <joint name="mount" type="fixed"><parent link="base_link"/><child link="sensor"/>
      <origin xyz="0.32 0 0.55" rpy="0 0.261799 0"/></joint></robot>''')
    for node in root.findall(".//mesh"):
        node.set("filename", str(mesh))
    root.find(".//texture").set("filename", str(texture))
    urdf = source / "robot.urdf"
    ET.ElementTree(root).write(urdf)
    before = urdf.read_bytes()
    ros = ET.fromstring(ros_robot_description(urdf))
    assert all(node.get("filename") == mesh.as_uri() for node in ros.findall(".//mesh"))
    assert ros.find(".//texture").get("filename") == texture.as_uri()
    # XML printers may change indentation; all joint attributes and structure
    # must remain identical, including origin, parent and child bindings.
    assert ET.canonicalize(ET.tostring(ros.find("joint")), strip_text=True) == \
        ET.canonicalize(ET.tostring(root.find("joint")), strip_text=True)
    assert urdf.read_bytes() == before


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
    assert robot.find("./litesim3d/sensor").get("class") == "litesim3d_mid360/Mid360"


def test_mid360_class_supports_noise_preview_and_switching_back(tmp_path):
    from litesim3d_bringup.model import lidar_preview_topics
    source, config = fixture(tmp_path)
    model = source / "simulation.urdf.xacro"
    model.write_text(model.read_text().replace("litesim3d_lidar/Lidar", "litesim3d_mid360/Mid360"))
    noise = tmp_path / "noise.yaml"
    noise.write_text("sensors: {lidar: {range_sigma_m: 0.03}}\n")
    prepared = prepare_config(config, tmp_path / "mid360", noise_config=str(noise))
    robot = ET.parse(yaml.safe_load(prepared.read_text())["robot"]["urdf"]).getroot()
    sensor = robot.find("./litesim3d/sensor")
    assert yaml.safe_load(sensor.find("overrides").text)["range_sigma_m"] == .03
    assert lidar_preview_topics(prepared, "auto", 1)["/scan"] == "/livox/lidar/pointcloud"
    profile = tmp_path / "spinning.yaml"
    profile.write_text("lidar: {scan_pattern: spinning, columns: 16, rings: 1, pattern_file: ''}\n")
    restored = prepare_config(prepared, tmp_path / "spinning", lidar_profile=str(profile))
    sensor = ET.parse(yaml.safe_load(restored.read_text())["robot"]["urdf"]).find("./litesim3d/sensor")
    assert sensor.get("class") == "litesim3d_lidar/Lidar"
    assert lidar_preview_topics(restored, "auto", 1) == {}


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


@pytest.mark.parametrize("composed", [False, True])
@pytest.mark.parametrize("xfer_format", [0, 1])
def test_two_mid360_preview_topics_match_vendor_streams(tmp_path, composed, xfer_format):
    from litesim3d_bringup.model import lidar_preview_topics
    model = tmp_path / "robot.urdf"
    model.write_text('<robot name="test"><link name="base_link"/></robot>')
    bindings = [
        {"name": name, "link": "base_link", "topic": name + "/raw", "scan_pattern": pattern}
        for name, pattern in [("front", "livox_mid360"), ("rear", "livox_mid360"), ("roof", "spinning")]
    ]
    data = {"robot": {"urdf": str(model)}}
    if composed:
        root = ET.parse(model).getroot()
        extension = ET.SubElement(root, "litesim3d", version="1")
        for binding in bindings:
            name = binding["name"]
            sensor = ET.SubElement(extension, "sensor", name=name,
                attrib={"class": "litesim3d_mid360/Mid360" if name != "roof" else "litesim3d_lidar/Lidar"})
            # Names come from the declaration, never the parameter mapping.
            ET.SubElement(sensor, "overrides").text = yaml.safe_dump(
                {k: v for k, v in binding.items() if k != "name"})
        disabled = ET.SubElement(extension, "sensor", name="disabled",
            attrib={"class": "litesim3d_mid360/Mid360"})
        ET.SubElement(disabled, "overrides").text = "link: base_link\ntopic: unused\nenabled: false"
        ET.ElementTree(root).write(model)
    else:
        data["lidars"] = bindings
    config = tmp_path / "config.yaml"
    config.write_text(yaml.safe_dump(data))
    expected = {}
    for name in ("front", "rear"):
        stem = "/livox/" + name + "/lidar"
        expected["/" + name + "/raw"] = stem + ("/pointcloud" if xfer_format else "")
        expected["/" + name + "/raw/range_boundary"] = stem + "/range_boundary"
    assert lidar_preview_topics(config, "auto", xfer_format) == expected
    assert lidar_preview_topics(config, "native", xfer_format) == {}


def test_disabled_second_mid360_keeps_single_sensor_preview(tmp_path):
    from litesim3d_bringup.model import lidar_preview_topics
    model = tmp_path / "robot.urdf"
    model.write_text('''<robot name="test"><link name="base_link"/>
      <litesim3d version="1">
        <sensor name="front" class="litesim3d_mid360/Mid360"><overrides>topic: front/raw</overrides></sensor>
        <sensor name="rear" class="litesim3d_mid360/Mid360"><overrides>topic: rear/raw
enabled: false</overrides></sensor>
      </litesim3d></robot>''')
    config = tmp_path / "config.yaml"
    config.write_text(yaml.safe_dump({"robot": {"urdf": str(model)}}))
    assert lidar_preview_topics(config, "livox", 1) == {
        "/front/raw": "/livox/lidar/pointcloud",
        "/front/raw/range_boundary": "/livox/lidar/range_boundary"}
