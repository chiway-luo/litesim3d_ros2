"""The shared launcher starts no RSP and preserves file parameters/remaps."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


def load_launch():
    path = Path(__file__).resolve().parents[1] / "launch/simulation.launch.py"
    spec = importlib.util.spec_from_file_location("simulation_launch_validation", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_invalid_paused_is_rejected_before_resource_loading(monkeypatch):
    module = load_launch()
    context = LaunchContext()
    context.launch_configurations["paused"] = "yes"
    def unexpected(*args, **kwargs):
        pytest.fail("invalid paused input reached resource preparation")
    monkeypatch.setattr(module, "prepare_config", unexpected)
    monkeypatch.setattr(module.subprocess, "run", unexpected)
    monkeypatch.setattr(module.tempfile, "mkdtemp", unexpected)
    with pytest.raises(ValueError, match="paused must be true or false"):
        module.setup(context)


@pytest.mark.parametrize("overrides, expected", [
    ({}, ("livox", 0, 7)),
    ({"lidar_interface": "native", "xfer_format": "1", "livox_lidar_id": "2"}, ("native", 1, 2)),
])
def test_no_rsp_file_parameters_and_remaps(monkeypatch, tmp_path, overrides, expected):
    module = load_launch()
    config = tmp_path / "runtime.yaml"
    config.write_text('robot:\n  urdf: "unused.urdf"\n')
    params = tmp_path / "ros.yaml"
    params.write_text(yaml.safe_dump({
        "/**": {"ros__parameters": {"use_sim_time": True}},
        "/litesim3d": {"ros__parameters": {"lidar_interface": "livox", "xfer_format": 0, "livox_lidar_id": 7}},
        "/external/kinematics": {"ros__parameters": {"publish_frequency": 100.0}},
    }))
    rviz = tmp_path / "test.rviz"
    rviz.write_text(yaml.safe_dump({"Visualization Manager": {"Global Options": {}, "Displays": []}}))
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(tmp_path))
    monkeypatch.setattr(module, "lidar_preview_topics", lambda *args: {})
    context = LaunchContext()
    context.launch_configurations.update(
        config=str(config), params=str(params), model_prepared="true", rviz="true", rviz_config=str(rviz),
        joint_states_topic="/custom/joints", tf_topic="/custom/tf", tf_static_topic="/custom/tf_static",
        robot_description_topic="/custom/model", **overrides)
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    nodes = {}
    node_class = module.Node
    def capture_node(**kwargs):
        nodes[kwargs["name"]] = kwargs
        return node_class(**kwargs)
    monkeypatch.setattr(module, "Node", capture_node)
    try:
        module.setup(context)
        assert set(nodes) == {"litesim3d", "rviz2"}
        settings = nodes["litesim3d"]["parameters"][0]
        assert tuple(settings[k] for k in ("lidar_interface", "xfer_format", "livox_lidar_id")) == expected
        assert not any("robot_state_publisher" in key for key in settings)
        assert ("joint_states", "/custom/joints") in nodes["litesim3d"]["remappings"]
        assert ("/tf", "/custom/tf") in nodes["litesim3d"]["remappings"]
        assert nodes["rviz2"]["remappings"] == [
            ("/tf", "/custom/tf"), ("/tf_static", "/custom/tf_static"),
            ("/robot_description", "/custom/model")]
        assert context.launch_configurations["prepared_config"] == str(config)
    finally:
        if "rviz2" in nodes:
            Path(nodes["rviz2"]["arguments"][1]).unlink(missing_ok=True)
