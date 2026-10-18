"""Installed multicopter example, sharing the SDF/renderer bringup pipeline."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('drone_3d_sim'))
    bringup = Path(get_package_share_directory('wheel_car_4w_sim'))
    defaults = {
        'world': 'drone_arena.sdf',
        'runtime': str(share / 'config/runtime.yaml'),
        'params': str(share / 'config/ros.yaml'),
        'assets_root': get_package_share_directory('litesim_models'),
        'gui': 'true', 'rviz': 'false', 'cameras': '', 'camera_view': 'false',
        'collision_budget_enabled': 'false', 'paused': '', 'real_time_factor': '', 'manifest': '',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/wheel_car_4w.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items()),
    ])
