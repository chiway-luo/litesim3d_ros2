"""Installed-resource M1 bringup; TF responsibility stays with RSP and bridge."""
from pathlib import Path
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve(uri, base):
    if uri.startswith("package://"):
        package, relative = uri[10:].split("/", 1)
        return Path(os.path.join(get_package_share_directory(package), relative))
    return Path(os.path.join(base, uri.removeprefix('file://'))).resolve()


def required_exit(event, context):
    # LaunchService propagates callback exceptions as nonzero and reaps siblings.
    # During an intentional launch shutdown, child exits are expected.
    if not context.is_shutdown:
        raise RuntimeError(f"required process {event.process_name} exited unexpectedly ({event.returncode})")


def boolean(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in {'true', 'false'}:
        raise ValueError(f"{name} must be true or false")
    return value == 'true'


def setup(context):
    config = Path(LaunchConfiguration("config").perform(context)).resolve()
    data = yaml.safe_load(config.read_text())
    urdf = resolve(data["robot"]["urdf"], config.parent)
    prefix = LaunchConfiguration("frame_prefix").perform(context)
    ns = LaunchConfiguration("namespace").perform(context)
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=ns,
               parameters=[{"robot_description": urdf.read_text(), "use_sim_time": True,
                            "frame_prefix": prefix, "publish_frequency": 200.0}], output="screen")
    sim = Node(package="litesim3d_ros2", executable="sim_node", namespace=ns,
               parameters=[{"config": str(config), "use_sim_time": True, "frame_prefix": prefix,
                            "start_paused": boolean(context, "paused"),
                            "manifest_path": LaunchConfiguration("manifest").perform(context)}], output="screen")
    rviz = Node(package="rviz2", executable="rviz2", namespace=ns,
                arguments=["-d", os.path.join(get_package_share_directory("litesim3d_ros2"), "rviz", "demo.rviz")],
                parameters=[{"use_sim_time": True}], condition=IfCondition(LaunchConfiguration("rviz")), output="screen")
    return [rsp, sim, rviz,
            RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=required_exit)),
            RegisterEventHandler(OnProcessExit(target_action=rsp, on_exit=required_exit))]


def generate_launch_description():
    default = os.path.join(get_package_share_directory("litesim3d_core"), "config", "demo.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("frame_prefix", default_value=""),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("manifest", default_value=""),
        OpaqueFunction(function=setup),
    ])
