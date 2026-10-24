"""Installed multicopter example; example defaults for the shared bringup API."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from litesim3d_bringup.model import resolve, ros_robot_description


def required_exit(event, context):
    if not context.is_shutdown:
        raise RuntimeError(f"{event.process_name} exited unexpectedly ({event.returncode})")


def setup_rsp(context):
    with open(LaunchConfiguration("params").perform(context), encoding="utf-8") as stream:
        params = yaml.safe_load(stream)
    common = params.get("/**", {}).get("ros__parameters", {})
    settings = {**common, **params.get("/robot_state_publisher", {}).get("ros__parameters", {})}
    sim_settings = {**common, **params.get("/litesim3d", {}).get("ros__parameters", {})}
    settings["frame_prefix"] = sim_settings.get("frame_prefix", "")
    config = LaunchConfiguration("prepared_config").perform(context)
    with open(config, encoding="utf-8") as stream:
        runtime = yaml.safe_load(stream)
    model = resolve(runtime["robot"]["urdf"], os.path.dirname(config))
    settings["robot_description"] = ParameterValue(ros_robot_description(model), value_type=str)
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", parameters=[settings], output="screen",
        remappings=[("joint_states", LaunchConfiguration("joint_states_topic")),
                    ("/tf", LaunchConfiguration("tf_topic")),
                    ("/tf_static", LaunchConfiguration("tf_static_topic")),
                    ("robot_description", LaunchConfiguration("robot_description_topic"))])
    return [RegisterEventHandler(OnProcessExit(target_action=rsp, on_exit=required_exit)), rsp]


def generate_launch_description():
    share = get_package_share_directory('drone_3d_sim')
    bringup = get_package_share_directory('litesim3d_bringup')
    defaults = {
        'assets_root': get_package_share_directory('drone_3d_models'),
        'world': 'drone_arena.sdf',
        'runtime': os.path.join(share, 'config', 'runtime_twist.yaml'),
        'params': os.path.join(share, 'config', 'ros_twist.yaml'),
        'config': '', 'lidar_profile': 'config', 'noise_config': '',
        'lidar_profiles_dir': '',
        'rviz_config': os.path.join(get_package_share_directory('litesim3d_ros2'), 'rviz', 'demo.rviz'),
        'gui': 'true', 'rviz': 'false', 'cameras': '', 'camera_view': 'false',
        'collision_budget_enabled': 'false', 'paused': '', 'real_time_factor': '', 'manifest': '',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'simulation.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items()),
        OpaqueFunction(function=setup_rsp),
    ])
