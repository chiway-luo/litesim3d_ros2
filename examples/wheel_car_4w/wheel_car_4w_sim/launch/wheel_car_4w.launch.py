"""SDF resources + deterministic simulator + camera-only 3D viewer."""
from pathlib import Path
import tempfile
import os
import json
import shutil
import subprocess
import sys

import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
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


def legacy_setup(context):
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
    if context.launch_configurations.get("scene_description"):
        sim_parameters["scene_description"] = context.launch_configurations["scene_description"]
        sim_parameters["viewer_ready_file"] = context.launch_configurations.get("viewer_ready_file", "")
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


def cleanup_directory(context, directory):
    shutil.rmtree(directory, ignore_errors=True)
    Path(directory + "-runtime.yaml").unlink(missing_ok=True)


def viewer_exit(event, context, ready_file):
    # Closing a ready viewer is independent of simulation. A startup failure
    # must fail bringup, instead of silently accepting missing materials.
    if not context.is_shutdown and not Path(ready_file).is_file():
        raise RuntimeError(f"3D viewer failed before first rendered frames ({event.returncode})")


def setup(context):
    argument = lambda name: LaunchConfiguration(name).perform(context)
    if argument("config"):
        if argument("cameras"):
            raise ValueError("camera overrides require SDF mode (leave config empty)")
        # Explicit legacy configs retain the old deterministic regression path.
        return legacy_setup(context)
    share = Path(argument("assets_root")).expanduser().resolve()
    resource_path = os.pathsep.join(dict.fromkeys(filter(None, [str(share / "models"), str(share / "worlds"),
                                    *context.environment.get("GZ_SIM_RESOURCE_PATH", "").split(os.pathsep)])))
    world=Path(argument("world")).expanduser()
    if not world.is_absolute() and len(world.parts)==1:
        world=share / "worlds" / world
    environment = {**context.environment, "GZ_SIM_RESOURCE_PATH": resource_path}
    budget = boolean(argument("collision_budget_enabled"), "collision_budget_enabled")
    gui_enabled = boolean(argument("gui"), "gui")
    directory = Path(tempfile.mkdtemp(prefix="litesim-sdf-"))
    try:
        runtime = Path(argument("runtime")).expanduser().resolve()
        if argument("cameras"):
            enabled = boolean(argument("cameras"), "cameras")
            data = yaml.safe_load(runtime.read_text())
            if enabled and not data.get("cameras"):
                raise ValueError("cameras:=true requires camera definitions in runtime YAML")
            for camera in data.get("cameras", []):
                camera["enabled"] = enabled
            for lidar in data.get("lidars", []):
                if "pattern_file" in lidar:
                    lidar["pattern_file"] = str(resolve(lidar["pattern_file"], runtime.parent))
            runtime = Path(str(directory) + "-runtime.yaml")
            runtime.write_text(yaml.safe_dump(data, sort_keys=False))
        importer = Path(get_package_prefix("litesim3d_assets")) / "lib/litesim3d_assets/sdf_import"
        command = [str(importer), str(world.resolve()),
                   str(runtime), str(directory), str(budget).lower()]
        if argument("real_time_factor"):
            command.append(argument("real_time_factor"))
        result = subprocess.run(command, env=environment, text=True, capture_output=True, timeout=120)
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or "SDF import failed")
        print("LiteSim SDF: " + result.stdout.strip())
        context.launch_configurations["config"] = str(directory / "runtime.yaml")
        context.launch_configurations["scene_description"] = str(directory / "scene.json")
        ready = str(directory / "viewer.ready")
        context.launch_configurations["viewer_ready_file"] = ready if gui_enabled else ""
        actions = [SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
                   RegisterEventHandler(OnShutdown(on_shutdown=[
                       OpaqueFunction(function=cleanup_directory, args=[str(directory)])]))]
        actions += legacy_setup(context)
        description = json.loads((directory / "scene.json").read_text())
        cameras = [c for c in description.get("cameras", []) if c["enabled"]]
        if cameras:
            renderer = Node(package="litesim3d_gui", executable="camera_node", name="litesim_camera_renderer",
                            parameters=[{"scene_description": str(directory / "scene.json")}], output="screen")
            actions += [RegisterEventHandler(OnProcessExit(target_action=renderer, on_exit=required_exit)), renderer]
            if boolean(argument("camera_view"), "camera_view"):
                for camera in cameras:
                    actions.append(Node(package="litesim3d_gui", executable="camera_view",
                        name="camera_view_" + camera["name"], parameters=[{
                            key: camera[key] for key in ("topic", "type", "rate_hz", "near_clip_m", "far_clip_m")
                        }], output="screen"))
        if gui_enabled:
            if sys.platform == "darwin":
                metal = Path(get_package_prefix("litesim3d_gui")) / "lib/litesim3d_gui/ogre2"
                if (metal / "RenderSystem_Metal.dylib").is_file():
                    actions.append(SetEnvironmentVariable("OGRE2_RESOURCE_PATH", str(metal)))
            viewer = Node(package="litesim3d_gui", executable="viewer", name="litesim_viewer",
                          parameters=[{"scene_description": str(directory / "scene.json"), "ready_file": ready}],
                          output="screen")
            actions += [RegisterEventHandler(OnProcessExit(target_action=viewer,
                        on_exit=lambda event, ctx: viewer_exit(event, ctx, ready))), viewer]
        return actions
    except BaseException:
        cleanup_directory(context, str(directory))
        raise


def generate_launch_description():
    share = Path(get_package_share_directory("wheel_car_4w_sim"))
    models = Path(get_package_share_directory("litesim_models"))
    return LaunchDescription([
        DeclareLaunchArgument("assets_root", default_value=str(models)),
        DeclareLaunchArgument("world", default_value="warehouse.sdf"),
        DeclareLaunchArgument("runtime", default_value=str(share / "config/runtime.yaml")),
        DeclareLaunchArgument("gui", default_value="true", description="Read-only 3D window"),
        DeclareLaunchArgument("cameras", default_value="", description="Empty uses runtime; true/false enables/disables configured cameras"),
        DeclareLaunchArgument("camera_view", default_value="false", description="Optional RGB/depth preview; default publishes ROS topics only"),
        DeclareLaunchArgument("collision_budget_enabled", default_value="false"),
        DeclareLaunchArgument("real_time_factor", default_value="", description="Target rate; 0 means unpaced"),
        DeclareLaunchArgument("config", default_value="",
                              description="Optional legacy YAML; empty selects the SDF world"),
        DeclareLaunchArgument("params", default_value=str(share / "config/ros.yaml"),
                              description="ROS node parameters with inline explanations"),
        DeclareLaunchArgument("lidar_profile", default_value="config",
                              description="config / vlp16 / mid360 / mid40 / custom profile YAML path"),
        DeclareLaunchArgument("rviz", default_value="false", description="true/false: open RViz"),
        DeclareLaunchArgument("rviz_config", default_value=str(share / "rviz/wheel_car_4w.rviz")),
        DeclareLaunchArgument("paused", default_value="",
                              description="Empty uses ros.yaml; true/false overrides start_paused"),
        DeclareLaunchArgument("manifest", default_value="",
                              description="Optional output JSON path; empty uses ros.yaml"),
        OpaqueFunction(function=setup),
    ])
