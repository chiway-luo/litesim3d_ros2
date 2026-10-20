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
from litesim3d_bringup.model import prepare_config, resolve, boolean, lidar_preview_topics


def required_exit(event, context):
    # A dead simulator/RSP must stop the bringup with a nonzero exit status.
    # Ctrl+C on the whole launch is a normal, already-started shutdown.
    if not context.is_shutdown:
        raise RuntimeError(
            f"required process {event.process_name} exited unexpectedly ({event.returncode})"
        )


def remove_temporary_config(context, filename):
    Path(filename).unlink(missing_ok=True)


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
    sim_parameters["lidar_interface"] = argument("lidar_interface")
    sim_parameters["xfer_format"] = int(argument("xfer_format"))
    sim_parameters["livox_lidar_id"] = int(argument("livox_lidar_id"))
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
    cleanup = []
    if not context.launch_configurations.get("model_prepared"):
        directory = tempfile.mkdtemp(prefix="litesim-model-")
        try:
            config = prepare_config(config, directory, mappings=yaml.safe_load(argument("model_arguments")),
                                    lidar_profile=selection, profiles_dir=argument("lidar_profiles_dir"),
                                    noise_config=argument("noise_config"))
        except BaseException:
            shutil.rmtree(directory, ignore_errors=True)
            raise
        data = yaml.safe_load(config.read_text())
        sim_parameters["config"] = str(config)
        cleanup.append(RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=cleanup_directory, args=[directory])])))
    # The exact expanded XML is also loaded by the core and scene importer.
    urdf = resolve(data["robot"]["urdf"], config.parent)
    rsp_parameters["robot_description"] = ParameterValue(urdf.read_text(), value_type=str)
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
        topics = lidar_preview_topics(config, sim_parameters["lidar_interface"], sim_parameters["xfer_format"])
        for display in manager["Displays"]:
            if display.get("Class") == "rviz_default_plugins/RobotModel":
                display["TF Prefix"] = prefix.rstrip("/")
            if display.get("Class") == "rviz_default_plugins/PointCloud2":
                topic = display.get("Topic", {})
                if isinstance(topic, dict) and topic.get("Value") in topics:
                    topic["Value"] = topics[topic["Value"]]
                elif isinstance(topic, str) and topic in topics:
                    display["Topic"] = topics[topic]
        with tempfile.NamedTemporaryFile(mode="w", suffix=".rviz", prefix="litesim-rviz-", delete=False) as stream:
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
    for name in ("assets_root", "world", "runtime"):
        if not argument(name):
            raise ValueError(f"{name} is required in SDF mode")
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
        runtime = prepare_config(Path(argument("runtime")).expanduser(), directory / "model",
                                 mappings=yaml.safe_load(argument("model_arguments")), cameras=argument("cameras"),
                                 lidar_profile=argument("lidar_profile"), profiles_dir=argument("lidar_profiles_dir"),
                                 noise_config=argument("noise_config"))
        scene_directory = directory / "scene"
        importer = Path(get_package_prefix("litesim3d_assets")) / "lib/litesim3d_assets/sdf_import"
        command = [str(importer), str(world.resolve()),
                   str(runtime), str(scene_directory), str(budget).lower()]
        if argument("real_time_factor"):
            command.append(argument("real_time_factor"))
        result = subprocess.run(command, env=environment, text=True, capture_output=True, timeout=120)
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or "SDF import failed")
        print("LiteSim SDF: " + result.stdout.strip())
        context.launch_configurations["config"] = str(scene_directory / "runtime.yaml")
        context.launch_configurations["scene_description"] = str(scene_directory / "scene.json")
        context.launch_configurations["model_prepared"] = "true"
        ready = str(directory / "viewer.ready")
        context.launch_configurations["viewer_ready_file"] = ready if gui_enabled else ""
        actions = [SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
                   RegisterEventHandler(OnShutdown(on_shutdown=[
                       OpaqueFunction(function=cleanup_directory, args=[str(directory)])]))]
        actions += legacy_setup(context)
        description = json.loads((scene_directory / "scene.json").read_text())
        cameras = [c for c in description.get("cameras", []) if c["enabled"]]
        if cameras:
            renderer = Node(package="litesim3d_gui", executable="camera_node", name="litesim_camera_renderer",
                            parameters=[{"scene_description": str(scene_directory / "scene.json")}], output="screen")
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
                          parameters=[{"scene_description": str(scene_directory / "scene.json"), "ready_file": ready}],
                          output="screen")
            actions += [RegisterEventHandler(OnProcessExit(target_action=viewer,
                        on_exit=lambda event, ctx: viewer_exit(event, ctx, ready))), viewer]
        return actions
    except BaseException:
        cleanup_directory(context, str(directory))
        raise


def generate_launch_description():
    bridge = Path(get_package_share_directory("litesim3d_ros2"))
    return LaunchDescription([
        DeclareLaunchArgument("assets_root", default_value="", description="Installed model share directory; required in SDF mode"),
        DeclareLaunchArgument("world", default_value="", description="SDF filename under assets_root/worlds or absolute path"),
        DeclareLaunchArgument("runtime", default_value="", description="SDF sensor and drive bindings YAML"),
        DeclareLaunchArgument("model_arguments", default_value="{}", description="YAML mapping of Xacro arguments"),
        DeclareLaunchArgument("params", description="ROS parameter YAML"),
        DeclareLaunchArgument("gui", default_value="true", description="Read-only 3D window"),
        DeclareLaunchArgument("cameras", default_value="", description="Empty uses runtime; true/false overrides cameras"),
        DeclareLaunchArgument("camera_view", default_value="false"),
        DeclareLaunchArgument("collision_budget_enabled", default_value="false"),
        DeclareLaunchArgument("real_time_factor", default_value="", description="Target rate; 0 means unpaced"),
        DeclareLaunchArgument("config", default_value="", description="Optional legacy YAML; empty selects SDF mode"),
        DeclareLaunchArgument("lidar_profile", default_value="config"),
        DeclareLaunchArgument("noise_config", default_value="", description="Optional per-sensor and wheel odometry errors YAML"),
        DeclareLaunchArgument("lidar_interface", default_value="auto", description="auto selects official output only for Mid-360; native keeps xyzirt_v1"),
        DeclareLaunchArgument("xfer_format", default_value="1", description="Mid-360: 1 CustomMsg, 0 Livox PointCloud2"),
        DeclareLaunchArgument("livox_lidar_id", default_value="0"),
        DeclareLaunchArgument("lidar_profiles_dir", default_value="", description="Directory for named lidar profiles"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value=str(bridge / "rviz/demo.rviz")),
        DeclareLaunchArgument("paused", default_value="", description="Empty uses params; true/false overrides start_paused"),
        DeclareLaunchArgument("manifest", default_value="", description="Optional output JSON path"),
        OpaqueFunction(function=setup),
    ])
