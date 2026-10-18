"""One entry point for the installed four-wheel URDF, simulator and RViz."""
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def resolve(uri, base):
    """Keep URDF resolution identical to the simulator's local/package paths."""
    if uri.startswith("package://"):
        package, relative = uri[10:].split("/", 1)
        return Path(get_package_share_directory(package)) / relative
    return (base / uri.removeprefix("file://")).resolve()


def boolean(value, name):
    value = value.strip().lower()
    if value not in {"true", "false"}:
        raise ValueError(f"{name} must be true or false")
    return value == "true"


def required_exit(event, context):
    # A dead simulator/RSP must stop the bringup with a nonzero exit status.
    # Ctrl+C on the whole launch is a normal, already-started shutdown.
    if not context.is_shutdown:
        raise RuntimeError(
            f"required process {event.process_name} exited unexpectedly ({event.returncode})"
        )


def remove_temporary_config(context, filename):
    Path(filename).unlink(missing_ok=True)


def profile_config(data, base, selection):
    """Replace one sensor's settings, preserving its URDF binding and topic."""
    if len(data.get("lidars", [])) != 1:
        raise ValueError("lidar_profile requires exactly one lidar in the example config")
    profiles = Path(get_package_share_directory("wheel_car_4w_sim")) / "config/lidars"
    source = profiles / (selection + ".yaml") if selection in {"vlp16", "mid360", "mid40"} else Path(selection).expanduser()
    if not source.is_file():
        raise ValueError("lidar_profile must be config, vlp16, mid360, mid40, or an existing YAML path")
    profile = yaml.safe_load(source.read_text())
    if not isinstance(profile, dict) or set(profile) != {"lidar"} or not isinstance(profile["lidar"], dict):
        raise ValueError("lidar profile must contain a single lidar mapping")
    if {"name", "link", "topic"} & profile["lidar"].keys():
        raise ValueError("lidar profile cannot replace name, link or topic bindings")
    binding = data["lidars"][0]
    data["lidars"] = [{**profile["lidar"], **{key: binding[key] for key in ("name", "link", "topic") if key in binding}}]
    if "pattern_file" in data["lidars"][0]:
        # A relative scan-table path belongs to the profile, not the generated temp YAML.
        data["lidars"][0]["pattern_file"] = str(resolve(data["lidars"][0]["pattern_file"], source.parent))
    # Generated YAML lives in a temporary directory: resolve all input paths
    # against the original config before relocating it, including scene meshes.
    data["robot"]["urdf"] = str(resolve(data["robot"]["urdf"], base))
    for entity in data.get("entities", []):
        for key in ("geometry", "visual", "collision", "sensor_geometry"):
            geometry = entity.get(key, {})
            if geometry.get("type") == "mesh":
                geometry["uri"] = str(resolve(geometry["uri"], base))
    return data


def setup(context):
    argument = lambda name: LaunchConfiguration(name).perform(context)
    config = Path(argument("config")).expanduser().resolve()
    parameters = Path(argument("params")).expanduser().resolve()
    data = yaml.safe_load(config.read_text())
    selection = argument("lidar_profile")
    document = yaml.safe_load(parameters.read_text())
    allowed = {"/**", "/litesim3d", "/robot_state_publisher", "/rviz2"}
    if not isinstance(document, dict) or set(document) - allowed:
        raise ValueError("params must contain only /** and the three documented node sections")

    def section(name):
        value = document.get(name, {"ros__parameters": {}})
        if not isinstance(value, dict) or set(value) != {"ros__parameters"} or not isinstance(value["ros__parameters"], dict):
            raise ValueError(f"invalid parameter section: {name}")
        return value["ros__parameters"]

    def node_parameters(name):
        values = {**section("/**"), **section(name)}
        if values.get("use_sim_time") is not True:
            raise ValueError(f"{name}: use_sim_time must be true for this simulation launch")
        return values

    sim_parameters = node_parameters("/litesim3d")
    rsp_parameters = node_parameters("/robot_state_publisher")
    rviz_parameters = node_parameters("/rviz2")
    sim_parameters["config"] = str(config)
    if argument("paused"):
        sim_parameters["start_paused"] = boolean(argument("paused"), "paused")
    if argument("manifest"):
        sim_parameters["manifest_path"] = str(Path(argument("manifest")).expanduser().resolve())
    prefix = sim_parameters.get("frame_prefix", "")
    if not isinstance(prefix, str) or (prefix and (prefix.startswith("/") or not prefix.endswith("/"))):
        raise ValueError("frame_prefix must be empty or relative and end in '/'")
    rsp_parameters["frame_prefix"] = prefix
    # Explicit string type prevents URDF XML/comments being interpreted as YAML.
    urdf = resolve(data["robot"]["urdf"], config.parent)
    rsp_parameters["robot_description"] = ParameterValue(urdf.read_text(), value_type=str)
    cleanup = []
    if selection != "config":
        data = profile_config(data, config.parent, selection)
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", prefix="wheel-car-lidar-", delete=False) as stream:
            yaml.safe_dump(data, stream, sort_keys=False)
            sim_parameters["config"] = stream.name
        cleanup.append(RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=remove_temporary_config, args=[stream.name])])))
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               name=sim_parameters.get("robot_state_publisher_name", "robot_state_publisher"),
               parameters=[rsp_parameters], output="screen")
    sim = Node(package="litesim3d_ros2", executable="sim_node", name="litesim3d",
               parameters=[sim_parameters], output="screen")
    actions = cleanup + [
        RegisterEventHandler(OnProcessExit(target_action=rsp, on_exit=required_exit)),
        RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=required_exit)),
        rsp, sim,
    ]
    if boolean(argument("rviz"), "rviz"):
        # Copy only the RViz presentation settings to a temporary file, syncing
        # the optional frame prefix without changing the installed template.
        presentation = yaml.safe_load(Path(argument("rviz_config")).expanduser().read_text())
        manager = presentation["Visualization Manager"]
        manager["Global Options"]["Fixed Frame"] = prefix + "odom"
        for display in manager["Displays"]:
            if display.get("Class") == "rviz_default_plugins/RobotModel":
                display["TF Prefix"] = prefix.rstrip("/")
        with tempfile.NamedTemporaryFile(mode="w", suffix=".rviz", prefix="wheel-car-4w-", delete=False) as stream:
            yaml.safe_dump(presentation, stream, sort_keys=False)
            rviz_file = stream.name
        actions.extend([
            RegisterEventHandler(OnShutdown(on_shutdown=[
                OpaqueFunction(function=remove_temporary_config, args=[rviz_file])])),
            Node(package="rviz2", executable="rviz2", name="rviz2",
                 arguments=["-d", rviz_file], parameters=[rviz_parameters], output="screen"),
        ])
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("wheel_car_4w_sim"))
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(share / "config/simulation.yaml"),
                              description="Robot, sensor, terrain and scene YAML"),
        DeclareLaunchArgument("params", default_value=str(share / "config/ros.yaml"),
                              description="ROS node parameters with inline explanations"),
        DeclareLaunchArgument("lidar_profile", default_value="config",
                              description="config / vlp16 / mid360 / mid40 / custom profile YAML path"),
        DeclareLaunchArgument("rviz", default_value="true", description="true/false: open RViz"),
        DeclareLaunchArgument("rviz_config", default_value=str(share / "rviz/wheel_car_4w.rviz")),
        DeclareLaunchArgument("paused", default_value="",
                              description="Empty uses ros.yaml; true/false overrides start_paused"),
        DeclareLaunchArgument("manifest", default_value="",
                              description="Optional output JSON path; empty uses ros.yaml"),
        OpaqueFunction(function=setup),
    ])
