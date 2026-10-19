"""Installed four-wheel example; example defaults for the shared bringup API."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('wheel_car_4w_sim'))
    bringup = Path(get_package_share_directory('litesim3d_bringup'))
    defaults = {
        'assets_root': get_package_share_directory('wheel_car_4w_models'),
        'world': 'office.sdf',
        'runtime': str(share / 'config/runtime.yaml'),
        'params': str(share / 'config/ros.yaml'),
        'config': '', 'lidar_profile': 'config', 'model_arguments': '{}',
        'lidar_profiles_dir': str(share / 'config/lidars'),
        'rviz_config': str(share / 'rviz/wheel_car_4w.rviz'),
        'gui': 'true', 'rviz': 'false', 'cameras': '', 'camera_view': 'false',
        'collision_budget_enabled': 'false', 'paused': '', 'real_time_factor': '', 'manifest': '',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/simulation.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items()),
    ])
