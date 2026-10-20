"""Installed multicopter example; example defaults for the shared bringup API."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('drone_3d_sim'))
    bringup = Path(get_package_share_directory('litesim3d_bringup'))
    defaults = {
        'assets_root': get_package_share_directory('drone_3d_models'),
        'world': 'drone_arena.sdf',
        'runtime': str(share / 'config/runtime.yaml'),
        'params': str(share / 'config/ros.yaml'),
        'config': '', 'lidar_profile': 'config', 'noise_config': '',
        'lidar_profiles_dir': '',
        'rviz_config': str(Path(get_package_share_directory('litesim3d_ros2')) / 'rviz/demo.rviz'),
        'gui': 'true', 'rviz': 'false', 'cameras': '', 'camera_view': 'false',
        'collision_budget_enabled': 'false', 'paused': '', 'real_time_factor': '', 'manifest': '',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/simulation.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items()),
    ])
