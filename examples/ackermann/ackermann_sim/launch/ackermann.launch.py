"""Example-owned RSP and ros2_control; simulator gets explicit model/parameter files."""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def required_exit(event, context):
    if not context.is_shutdown:
        raise RuntimeError(f"{event.process_name} exited unexpectedly ({event.returncode})")


def generate_launch_description():
    ld = LaunchDescription()
    share = get_package_share_directory("ackermann_sim")
    simulator_share = get_package_share_directory("litesim3d_bringup")
    runtime_file = os.path.join(share, "config", "runtime.yaml")
    ros_params = os.path.join(share, "config", "ros.yaml")
    rsp_params = os.path.join(share, "config", "rsp.yaml")
    controller_params = os.path.join(share, "config", "controllers.yaml")
    robot_xacro = os.path.join(share, "urdf", "robot.urdf.xacro")
    simulation_xacro = os.path.join(share, "urdf", "simulation.urdf.xacro")
    control_xacro = os.path.join(share, "urdf", "control.urdf.xacro")

    # 参数文件可由自己的 bringup 替换；普通仿真选项沿用公共 launch 的默认值。
    ld.add_action(DeclareLaunchArgument("runtime", default_value=runtime_file))
    ld.add_action(DeclareLaunchArgument("params", default_value=ros_params))
    ld.add_action(DeclareLaunchArgument("controllers", default_value=controller_params))
    ld.add_action(DeclareLaunchArgument("world", default_value="room.sdf"))
    ld.add_action(DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"))
    ld.add_action(DeclareLaunchArgument("tf_topic", default_value="/tf"))
    ld.add_action(DeclareLaunchArgument("tf_static_topic", default_value="/tf_static"))
    ld.add_action(DeclareLaunchArgument("robot_description_topic", default_value="/robot_description"))

    # 机械结构和传感器安装单独展开给 RSP；不含仿真插件、ros2_control 标签。
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[rsp_params, {
            "robot_description": ParameterValue(xacro.process_file(robot_xacro).toxml(), value_type=str),
        }],
        remappings=[
            ("joint_states", LaunchConfiguration("joint_states_topic")),
            ("/tf", LaunchConfiguration("tf_topic")),
            ("/tf_static", LaunchConfiguration("tf_static_topic")),
            ("robot_description", LaunchConfiguration("robot_description_topic")),
        ],
    )
    ld.add_action(RegisterEventHandler(OnProcessExit(target_action=rsp, on_exit=required_exit)))
    ld.add_action(rsp)

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(simulator_share, "launch", "simulation.launch.py")),
        launch_arguments={
            "assets_root": share,
            "world": LaunchConfiguration("world"),
            "runtime": LaunchConfiguration("runtime"),
            "params": LaunchConfiguration("params"),
            "robot_urdf": simulation_xacro,
        }.items(),
    )
    # 本案例选择仿真编码器里程计作为 /odom 与 odom→base_link 的唯一来源。
    # remap 仅作用于公共仿真 launch；控制器可替换而不影响里程计与 TF。
    ld.add_action(GroupAction([
        SetRemap(src="sim/odometry", dst="/odom"),
        simulation,
    ]))

    manager = Node(
        package="controller_manager", executable="ros2_control_node", output="screen",
        # 不设置全局 name remap，防止动态加载的控制器被一并改名。
        parameters=[LaunchConfiguration("controllers"), {
            "robot_description": ParameterValue(xacro.process_file(control_xacro).toxml(), value_type=str),
        }],
        remappings=[
            ("/ackermann_controller/reference_unstamped", "/cmd_vel"),
            ("/joint_states", LaunchConfiguration("joint_states_topic")),
        ],
    )
    ld.add_action(RegisterEventHandler(OnProcessExit(target_action=manager, on_exit=required_exit)))
    ld.add_action(manager)
    spawner = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["joint_state_broadcaster", "ackermann_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "60"],
    )
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=spawner,
        on_exit=lambda event, context: required_exit(event, context) if event.returncode else None)))
    ld.add_action(spawner)
    return ld
