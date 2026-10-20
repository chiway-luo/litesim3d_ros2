"""Installed ackermann example, using the common model/scene pipeline."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("ackermann_sim"))
    common = Path(get_package_share_directory("litesim3d_bringup"))
    defaults = {"assets_root": str(share), "world": "room.sdf",
                "runtime": str(share / "config/runtime.yaml"), "params": str(share / "config/ros.yaml"),
                "noise_config": "", "gui": "true", "rviz": "false", "cameras": "", "camera_view": "false",
                "paused": "", "real_time_factor": "", "manifest": "", "model_arguments": "{}"}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(common / "launch/simulation.launch.py")),
                                 launch_arguments={k: LaunchConfiguration(k) for k in defaults}.items()),
    ])
