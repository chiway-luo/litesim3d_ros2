"""Configure RSP and ros2_control in the example; pass configuration files to LiteSim."""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from litesim3d_bringup.controller_parameters import UrdfControllerParameters


def required_exit(event, context):
    if not context.is_shutdown:
        raise RuntimeError(f"{event.process_name} exited unexpectedly ({event.returncode})")


def generate_launch_description():
    ld = LaunchDescription()

    share = get_package_share_directory("dual_mid360_bringup")
    description_share = get_package_share_directory("dual_mid360_description")
    simulator_share = get_package_share_directory("litesim3d_bringup")
    ros_params = os.path.join(share, "config", "ros.yaml")  # 仿真节点与 RViz 的 ROS 参数。
    rsp_params = os.path.join(share, "config", "rsp.yaml")  # 本案例 RSP 的时间、TF 前缀与频率。
    controller_params = os.path.join(share, "config", "controllers.yaml")  # 控制管理器及各控制器参数。
    control_xacro = os.path.join(share, "urdf", "control.urdf.xacro")  # 机械结构及 ros2_control 硬件接口。
    runtime_file = os.path.join(share, "config", "runtime.yaml")  # 机器人、地形和场景绑定。
    simulation_xacro = os.path.join(share, "urdf", "simulation.urdf.xacro")  # 含仿真插件的组合模型。
    rviz_config = os.path.join(share, "rviz", "dual_mid360.rviz")  # 整车与双雷达的 RViz 显示配置。

    # worlds/ 下的场景文件名；改为 minimal.sdf 可使用最小场景模板。
    ld.add_action(DeclareLaunchArgument("world", default_value="workshop.sdf"))
    # 出生位姿等仿真节点参数从 ROS YAML 加载；支持替换为自己的参数文件。
    ld.add_action(DeclareLaunchArgument("params", default_value=ros_params))
    # 是否打开 LiteSim 三维窗口；无桌面环境可设为 false。
    ld.add_action(DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]))
    # 是否打开 RViz；与三维窗口开关相互独立。
    ld.add_action(DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]))
    # 墙钟秒：等待仿真就绪且时钟推进，再启动管理器和控制器加载器。
    ld.add_action(DeclareLaunchArgument("controller_startup_timeout", default_value="60"))
    # joint_state_broadcaster 发布、RSP 订阅的关节状态话题；跨命名空间时两端使用同一绝对话题名。
    ld.add_action(DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"))
    # 控制器 odom TF 与 RSP 活动关节 TF 共用的话题，公共 launch 同步配置 RViz。
    ld.add_action(DeclareLaunchArgument("tf_topic", default_value="/tf"))
    # RSP 固定关节 TF 话题，公共 launch 将 RViz 接到相同话题。
    ld.add_action(DeclareLaunchArgument("tf_static_topic", default_value="/tf_static"))
    # RSP 向 RViz 发布机械模型的话题；传入 XML 的参数名仍为 robot_description。
    ld.add_action(DeclareLaunchArgument("robot_description_topic", default_value="/robot_description"))

    # 机械 Xacro 单独展开，XML 内容传给 robot_state_publisher。
    description_xacro = os.path.join(description_share, "urdf", "rover.urdf.xacro")
    robot_description = xacro.process_file(description_xacro).toxml()
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",  # 案例自行命名，可改名或增加 namespace；仿真器不绑定它。
        output="screen",
        parameters=[rsp_params, {
            # 这里传机械 Xacro 展开后的 XML 字符串；组合仿真 Xacro 另交给公共 launch。
            "robot_description": ParameterValue(robot_description, value_type=str),
        }],
        # 下面四个选项也供公共 launch 读取，使 RSP、仿真与 RViz 使用一致的话题。
        remappings=[
            ("joint_states", LaunchConfiguration("joint_states_topic")),
            ("/tf", LaunchConfiguration("tf_topic")),
            ("/tf_static", LaunchConfiguration("tf_static_topic")),
            ("robot_description", LaunchConfiguration("robot_description_topic")),
        ],
    )
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=robot_state_publisher, on_exit=required_exit)))
    ld.add_action(robot_state_publisher)

    # 显式传入模型与参数文件；普通选项使用参数文件或公共 launch 默认值。
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulator_share, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "assets_root": share,  # 场景资源根目录，公共入口从其 worlds/ 下查找 world。
            "world": LaunchConfiguration("world"),  # 当前选择的 SDF 场景文件。
            "runtime": runtime_file,  # 显式传入 runtime.yaml；源文件中的 robot.urdf 保持空。
            "robot_urdf": simulation_xacro,  # 公共入口展开组合模型并写入临时运行配置。
            "params": LaunchConfiguration("params"),  # 加载出生位姿及其他仿真节点参数。
            "rviz_config": rviz_config,  # 显式传入 RViz 配置文件。
        }.items(),
    )
    ld.add_action(simulation)

    # 控制模型独立展开，控制器读取的参数文件显式列出。
    control_description = xacro.process_file(control_xacro).toxml()
    controller_manager = Node(
        package="litesim3d_ros2_control",
        executable="spawn_when_ready.py",
        arguments=["--executable", "ros2_control_node",
                   "--timeout", LaunchConfiguration("controller_startup_timeout"), "--"],
        # 保留管理器默认名；避免全局 __node remap 把动态加载的控制器也改名。
        output="screen",
        parameters=[UrdfControllerParameters(controller_params, control_description), {
            "robot_description": ParameterValue(control_description, value_type=str),
        }],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/diff_drive_controller/odom", "/odom"),
            ("/joint_states", LaunchConfiguration("joint_states_topic")),
            ("/tf", LaunchConfiguration("tf_topic")),
        ],
    )
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=controller_manager, on_exit=required_exit)))
    ld.add_action(controller_manager)
    # 管理器和 spawner 均等待仿真就绪及时间推进，避免暂停/慢启动时加载超时。
    # 默认只激活状态广播器和差速控制器。
    # 速度/位置直控控制器留给使用者按需加载与切换，避免争用轮关节。
    controller_spawner = Node(
        package="litesim3d_ros2_control",
        executable="spawn_when_ready.py",
        output="screen",
        arguments=["--timeout", LaunchConfiguration("controller_startup_timeout"), "--",
                   "joint_state_broadcaster", "diff_drive_controller",
                   "--controller-manager", "/controller_manager", "--controller-manager-timeout", "60"],
    )
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=controller_spawner,
        on_exit=lambda event, context: required_exit(event, context) if event.returncode else None)))
    ld.add_action(controller_spawner)

    return ld
