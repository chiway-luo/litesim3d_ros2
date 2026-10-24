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
from litesim3d_bringup.model import prepare_config, boolean, lidar_preview_topics


def required_exit(event, context):
    # A dead simulator or renderer must stop the bringup with a nonzero exit status.
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
    selection = argument("lidar_profile")
    document = yaml.safe_load(parameters.read_text())
    if not isinstance(document, dict):
        raise ValueError("params must be a ROS parameter YAML mapping")
    # The caller may share this file with its own RSP or other nodes.
    # Only the simulator and RViz sections are consumed here.

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
    rviz_parameters = node_parameters("/rviz2")
    sim_parameters["config"] = str(config)
    for name, fallback, convert in (("lidar_interface", "auto", str), ("xfer_format", 1, int),
                                    ("livox_lidar_id", 0, int)):
        sim_parameters[name] = convert(argument(name) or sim_parameters.get(name, fallback))
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
    cleanup = []
    if not context.launch_configurations.get("model_prepared"):
        directory = tempfile.mkdtemp(prefix="litesim-model-")
        try:
            config = prepare_config(config, directory, robot_urdf=argument("robot_urdf"),
                                    mappings=yaml.safe_load(argument("model_arguments")),
                                    lidar_profile=selection, profiles_dir=argument("lidar_profiles_dir"),
                                    noise_config=argument("noise_config"))
        except BaseException:
            shutil.rmtree(directory, ignore_errors=True)
            raise
        sim_parameters["config"] = str(config)
        cleanup.append(RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=cleanup_directory, args=[directory])])))
    context.launch_configurations["prepared_config"] = str(config)
    actions = cleanup
    sim = Node(package="litesim3d_ros2", executable="sim_node", name="litesim3d",
               parameters=[sim_parameters], output="screen",
               remappings=[("joint_states", argument("joint_states_topic")),
                           ("sim/joint_commands", argument("joint_commands_topic")),
                           ("sim/joint_feedback", argument("joint_feedback_topic")),
                           ("/tf", argument("tf_topic")),
                           ("robot_description", "sim/robot_description")])
    actions += [
        RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=required_exit)),
        sim,
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
                 arguments=["-d", rviz_file], parameters=[rviz_parameters], output="screen",
                 remappings=[("/tf", argument("tf_topic")),
                             ("/tf_static", argument("tf_static_topic")),
                             ("/robot_description", argument("robot_description_topic"))]),
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
    # Reject invalid launch input before preparing models or starting the SDF
    # importer. Importing an instrumented scene can be expensive.
    if argument("paused"):
        boolean(argument("paused"), "paused")
    if argument("config"):
        if argument("cameras"):
            raise ValueError("camera overrides require SDF mode (leave config empty)")
        # Explicit legacy configs retain the old deterministic regression path.
        return legacy_setup(context)
    for name in ("assets_root", "world", "runtime"):
        if not argument(name):
            raise ValueError(f"{name} is required in SDF mode")
    share = Path(argument("assets_root")).expanduser().resolve()
    resource_path = os.pathsep.join(dict.fromkeys(filter(None, [os.path.join(share, "models"), os.path.join(share, "worlds"),
                                    *context.environment.get("GZ_SIM_RESOURCE_PATH", "").split(os.pathsep)])))
    world=Path(argument("world")).expanduser()
    if not world.is_absolute() and len(world.parts)==1:
        world=Path(os.path.join(share, "worlds", world))
    environment = {**context.environment, "GZ_SIM_RESOURCE_PATH": resource_path}
    budget = boolean(argument("collision_budget_enabled"), "collision_budget_enabled")
    gui_enabled = boolean(argument("gui"), "gui")
    directory = tempfile.mkdtemp(prefix="litesim-sdf-")
    try:
        runtime = prepare_config(Path(argument("runtime")).expanduser(), os.path.join(directory, "model"),
                                 robot_urdf=argument("robot_urdf"),
                                 mappings=yaml.safe_load(argument("model_arguments")), cameras=argument("cameras"),
                                 lidar_profile=argument("lidar_profile"), profiles_dir=argument("lidar_profiles_dir"),
                                 noise_config=argument("noise_config"))
        scene_directory = os.path.join(directory, "scene")
        importer = os.path.join(get_package_prefix("litesim3d_assets"), "lib", "litesim3d_assets", "sdf_import")
        command = [importer, str(world.resolve()),
                   str(runtime), scene_directory, str(budget).lower()]
        if argument("real_time_factor"):
            command.append(argument("real_time_factor"))
        result = subprocess.run(command, env=environment, text=True, capture_output=True, timeout=120)
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or "SDF import failed")
        print("LiteSim SDF: " + result.stdout.strip())
        context.launch_configurations["config"] = os.path.join(scene_directory, "runtime.yaml")
        context.launch_configurations["scene_description"] = os.path.join(scene_directory, "scene.json")
        context.launch_configurations["model_prepared"] = "true"
        ready = os.path.join(directory, "viewer.ready")
        context.launch_configurations["viewer_ready_file"] = ready if gui_enabled else ""
        actions = [SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
                   RegisterEventHandler(OnShutdown(on_shutdown=[
                       OpaqueFunction(function=cleanup_directory, args=[directory])]))]
        actions += legacy_setup(context)
        description = json.loads(Path(os.path.join(scene_directory, "scene.json")).read_text())
        cameras = [c for c in description.get("cameras", []) if c["enabled"]]
        if cameras:
            renderer = Node(package="litesim3d_gui", executable="camera_node", name="litesim_camera_renderer",
                            parameters=[{"scene_description": os.path.join(scene_directory, "scene.json")}], output="screen")
            actions += [RegisterEventHandler(OnProcessExit(target_action=renderer, on_exit=required_exit)), renderer]
            if boolean(argument("camera_view"), "camera_view"):
                for camera in cameras:
                    actions.append(Node(package="litesim3d_gui", executable="camera_view",
                        name="camera_view_" + camera["name"], parameters=[{
                            key: camera[key] for key in ("topic", "type", "rate_hz", "near_clip_m", "far_clip_m")
                        }], output="screen"))
        if gui_enabled:
            if sys.platform == "darwin":
                metal = os.path.join(get_package_prefix("litesim3d_gui"), "lib", "litesim3d_gui", "ogre2")
                if os.path.isfile(os.path.join(metal, "RenderSystem_Metal.dylib")):
                    actions.append(SetEnvironmentVariable("OGRE2_RESOURCE_PATH", metal))
            viewer = Node(package="litesim3d_gui", executable="viewer", name="litesim_viewer",
                          parameters=[{"scene_description": os.path.join(scene_directory, "scene.json"), "ready_file": ready}],
                          output="screen")
            actions += [RegisterEventHandler(OnProcessExit(target_action=viewer,
                        on_exit=lambda event, ctx: viewer_exit(event, ctx, ready))), viewer]
        return actions
    except BaseException:
        cleanup_directory(context, directory)
        raise


def generate_launch_description():
    bridge = get_package_share_directory("litesim3d_ros2")
    return LaunchDescription([
        DeclareLaunchArgument("assets_root", default_value="", description="Installed model share directory; required in SDF mode"),
        DeclareLaunchArgument("world", default_value="", description="SDF filename under assets_root/worlds or absolute path"),
        DeclareLaunchArgument("runtime", default_value="", description="SDF sensor and drive bindings YAML"),
        DeclareLaunchArgument("robot_urdf", default_value="", description="URDF/Xacro path supplied by robot bringup; overrides runtime/config robot.urdf"),
        DeclareLaunchArgument("model_arguments", default_value="{}", description="YAML mapping of Xacro arguments"),
        DeclareLaunchArgument("params", description="ROS parameter YAML"),
        DeclareLaunchArgument("joint_states_topic", default_value="joint_states", description="Simulator joint-state output; connect the caller's RSP to this topic"),
        DeclareLaunchArgument("joint_commands_topic", default_value="sim/joint_commands", description="ros2_control hardware command input"),
        DeclareLaunchArgument("joint_feedback_topic", default_value="sim/joint_feedback", description="ros2_control actual joint feedback output"),
        DeclareLaunchArgument("tf_topic", default_value="/tf", description="Odometry TF output and duplicate-publisher monitor; also used by RViz"),
        DeclareLaunchArgument("tf_static_topic", default_value="/tf_static", description="RViz static TF input; caller remaps its RSP separately"),
        DeclareLaunchArgument("robot_description_topic", default_value="/robot_description", description="RViz model input supplied by the caller"),
        DeclareLaunchArgument("gui", default_value="true", description="Read-only 3D window"),
        DeclareLaunchArgument("cameras", default_value="", description="Empty uses runtime; true/false overrides cameras"),
        DeclareLaunchArgument("camera_view", default_value="false"),
        DeclareLaunchArgument("collision_budget_enabled", default_value="false"),
        DeclareLaunchArgument("real_time_factor", default_value="", description="Target rate; 0 means unpaced"),
        DeclareLaunchArgument("config", default_value="", description="Optional legacy YAML; empty selects SDF mode"),
        DeclareLaunchArgument("lidar_profile", default_value="config"),
        DeclareLaunchArgument("noise_config", default_value="", description="Optional per-sensor and wheel odometry errors YAML"),
        DeclareLaunchArgument("lidar_interface", default_value="", description="Empty uses params (fallback auto); native/auto/livox overrides params"),
        DeclareLaunchArgument("xfer_format", default_value="", description="Empty uses params (fallback 1); 1 CustomMsg, 0 Livox PointCloud2"),
        DeclareLaunchArgument("livox_lidar_id", default_value="", description="Empty uses params (fallback 0)"),
        DeclareLaunchArgument("lidar_profiles_dir", default_value="", description="Directory for named lidar profiles"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(bridge, "rviz", "demo.rviz")),
        DeclareLaunchArgument("paused", default_value="", description="Empty uses params; true/false overrides start_paused"),
        DeclareLaunchArgument("manifest", default_value="", description="Optional output JSON path"),
        OpaqueFunction(function=setup),
    ])
