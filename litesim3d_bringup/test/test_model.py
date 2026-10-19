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
