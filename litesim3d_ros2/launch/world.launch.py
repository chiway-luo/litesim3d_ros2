"""One authoritative clock and a prefixed robot_state_publisher per robot."""
from pathlib import Path
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve(uri, base):
    if uri.startswith('package://'):
        package, relative = uri[10:].split('/', 1)
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
    path = Path(LaunchConfiguration('config').perform(context)).resolve()
    world = yaml.safe_load(path.read_text())
    source = resolve(world['config'], path.parent)
    config = yaml.safe_load(source.read_text())
    urdf = resolve(config['robot']['urdf'], source.parent).read_text()
    external = [r.strip() for r in LaunchConfiguration('external_odom_tf_robots').perform(context).split(',') if r.strip()]
    sim = Node(package='litesim3d_ros2', executable='sim_world_node', output='screen',
               parameters=[{'config': str(path), 'use_sim_time': True,
                            'start_paused': boolean(context, 'paused'),
                            'manifest_path': LaunchConfiguration('manifest').perform(context),
                            'cmd_mode': LaunchConfiguration('cmd_mode').perform(context),
                            'sensor_reliable': boolean(context, 'sensor_reliable'),
                            'publish_odom_tf': boolean(context, 'publish_odom_tf'),
                            **({'external_odom_tf_robots': external} if external else {})}])
    nodes = [sim]
    for robot in world['robots']:
        nodes.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                          namespace=robot['id'], output='screen', parameters=[{
                              'robot_description': urdf, 'frame_prefix': robot['id']+'/',
                              'use_sim_time': True, 'publish_frequency': 200.0}]))
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=required_exit)) for node in nodes]
    return nodes + handlers


def generate_launch_description():
    default = os.path.join(get_package_share_directory('litesim3d_core'), 'config', 'm3_world.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('manifest', default_value=''),
        DeclareLaunchArgument('cmd_mode', default_value='twist'),
        DeclareLaunchArgument('sensor_reliable', default_value='false'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
        DeclareLaunchArgument('external_odom_tf_robots', default_value=''),
        OpaqueFunction(function=setup),
    ])
