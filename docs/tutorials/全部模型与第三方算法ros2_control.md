# 全部模型与第三方算法 ros2_control

本教程覆盖仓库已有的差速、阿克曼、三轮/四轮普通全向和速度运动学无人机。各案例自行启动 RSP、controller_manager 与控制器，公共仿真 launch 只负责仿真。路径统一使用 os.path.join 和 get_package_share_directory，参数文件在 launch 中显式传入。

## 1. 克隆仓库与选择模型

```bash
git clone --branch humble https://github.com/chiway-luo/litesim3d_ros2.git
cd litesim3d_ros2
```

依赖与构建沿用 [安装说明](../usage/安装与构建.md)。已经构建后加载本工作空间的 install/setup.bash；下文命令都在已加载 Humble 和工作空间的终端中运行。

| 模型 / 插件 | ros2_control 资源 | 默认控制器与案例 |
| --- | --- | --- |
| DifferentialDrive | 独立轮关节 velocity，position/velocity 反馈 | diff_drive_controller；四轮车 |
| AckermannDrive | 后轮 velocity、前轮转向 position；前轮旋转仅状态 | ackermann_steering_controller；[阿克曼教程](阿克曼与全向轮ros2_control.md) |
| Omni3Drive / Omni4Drive | 三个或四个轮关节 velocity | OmniDriveController；[全向轮教程](阿克曼与全向轮ros2_control.md) |
| multicopter | body GPIO：机体系 XYZ 速度、偏航角速度；实际位姿/速度状态 | BodyVelocityController；下文无人机案例 |
| 其他独立可动关节 | position 或 velocity 指令，position/velocity 状态 | 可替换为轨迹、位置、速度或第三方控制器；同一关节不能同时申领两种模式 |
| DifferentialOdometry / AckermannOdometry / OmniOdometry | 根据实际编码器计算里程计，输出 sim/odometry | 案例可 remap 到 /odom；与控制器里程计择一发布 odom TF |
| Lidar / Mid360 | PointCloud2；Mid360 也支持 Livox 输出 | 控制后的真实采样位姿 → 传感器 → 算法 |
| Imu | sensor_msgs/Imu | IMU topic 可配置、remap |
| RgbCamera / DepthCamera / RgbdCamera | Image + CameraInfo | 图像、米制深度和内参按原接口消费 |

传感器插件是控制算法的数据来源，图像和点云继续使用对应 ROS 消息。执行机构通过 ros2_control 统一申领，传感器无需伪装成可写的标量关节。算法可以作为独立 ROS 节点向控制器发指令，也可以作为 ControllerInterface 插件直接读状态、写硬件接口。

## 2. 四轮车：原始机械模型与独立控制模型

`wheel_car_4w_description/urdf/wheel_car_4w.urdf` 保留原机械结构；rear_left/right 是独立后轮，front_left/right 在原 URDF 中 mimic 同侧后轮。`robot.urdf.xacro` 组合机械和相机安装给 RSP，`simulation.urdf.xacro` 组合传感器、底盘和里程计给仿真。控制模型单独传给 controller_manager：

<!-- source: examples/wheel_car_4w/wheel_car_4w_sim/urdf/control.urdf.xacro -->
```xml
<?xml version="1.0"?>
<robot name="wheel_car_4w" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find wheel_car_4w_description)/urdf/robot.urdf.xacro"/>
  <ros2_control name="LiteSimWheelCar" type="system">
    <hardware>
      <plugin>litesim3d_ros2_control/LiteSimSystem</plugin>
      <param name="command_topic">/sim/joint_commands</param>
      <param name="state_topic">/sim/joint_feedback</param>
      <param name="feedback_timeout">2.0</param>
      <param name="activation_timeout">30.0</param>
    </hardware>
    <!-- 后轮是独立驱动，原始 URDF 的前轮 mimic 仅导出状态。 -->
    <xacro:macro name="wheel" params="name driven">
      <joint name="${name}">
        <xacro:if value="${driven}">
          <command_interface name="velocity"/><command_interface name="position"/>
        </xacro:if>
        <state_interface name="position"/><state_interface name="velocity"/>
      </joint>
    </xacro:macro>
    <xacro:wheel name="rear_left_wheel_joint" driven="true"/>
    <xacro:wheel name="rear_right_wheel_joint" driven="true"/>
    <xacro:wheel name="front_left_wheel_joint" driven="false"/>
    <xacro:wheel name="front_right_wheel_joint" driven="false"/>
  </ros2_control>
</robot>
```

控制器参数中的轮半径 0.12 m、轮距 0.58 m 对应原机械模型，每侧只申领一个独立后轮。前轮由仿真返回 mimic 状态，不能重复申领。文件如下：

<!-- source: examples/wheel_car_4w/wheel_car_4w_sim/config/controllers.yaml -->
```yaml
# 由案例的 controller_manager 读取，公共仿真 launch 不读取此文件。
/controller_manager:
  ros__parameters:
    use_sim_time: true  # 与仿真 /clock 同步。
    update_rate: 100  # 控制器读取状态、计算指令、写入硬件的频率，Hz。
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster  # 唯一的公共关节状态发布者。
    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController  # 默认车体速度控制器。
    wheel_velocity_controller:
      type: forward_command_controller/ForwardCommandController  # 可选：第三方算法直接给各轮速度。
    wheel_position_controller:
      type: forward_command_controller/ForwardCommandController  # 可选：第三方算法直接给各轮位置。
/joint_state_broadcaster:
  ros__parameters:
    use_sim_time: true
    use_local_topics: false  # 在管理器命名空间下发布 joint_states，launch 可显式 remap。
/diff_drive_controller:
  ros__parameters:
    use_sim_time: true
    # 与 description/config/geometry.yaml 的驱动关节、轮径和轮距保持一致。
    left_wheel_names: [rear_left_wheel_joint]
    right_wheel_names: [rear_right_wheel_joint]
    wheel_radius: 0.12  # m，轮胎半径。
    wheel_separation: 0.58  # m，左右轮中心线距离。
    wheels_per_side: 1  # 每侧一个独立后轮，前轮 mimic。
    position_feedback: true  # 由仿真实际轮角计算里程计。
    open_loop: false  # 使用状态反馈，不从速度指令直接推算里程计。
    use_stamped_vel: false  # 订阅 Twist；launch 将 cmd_vel_unstamped remap 到 /cmd_vel。
    cmd_vel_timeout: 0.5  # 秒，外部算法应持续发送速度指令。
    publish_rate: 50.0  # Hz，发布 odom 和 odom TF。
    enable_odom_tf: false  # 本例由仿真编码器里程计发布 odom TF。
    tf_frame_prefix_enable: false  # 本例不自动给 frame 加控制器命名空间前缀。
    odom_frame_id: odom
    base_frame_id: base_link
    linear.x.has_velocity_limits: true
    linear.x.max_velocity: 0.8  # m/s。
    linear.x.min_velocity: -0.8
    linear.x.has_acceleration_limits: true
    linear.x.max_acceleration: 0.5  # 对称加减速上限，m/s²。
    linear.x.min_acceleration: -0.5
    angular.z.has_acceleration_limits: true
    angular.z.max_acceleration: 2.0  # rad/s²。
    angular.z.min_acceleration: -2.0
    angular.z.has_velocity_limits: true
    angular.z.max_velocity: 1.5  # rad/s。
    angular.z.min_velocity: -1.5
/wheel_velocity_controller:
  ros__parameters:
    use_sim_time: true
    joints: [rear_left_wheel_joint, rear_right_wheel_joint]
    interface_name: velocity  # Float64MultiArray 的顺序与 joints 一致，单位 rad/s。
/wheel_position_controller:
  ros__parameters:
    use_sim_time: true
    joints: [rear_left_wheel_joint, rear_right_wheel_joint]
    interface_name: position  # Float64MultiArray，单位 rad；连续关节角度不折返到 ±π。
```

案例用 ld 单独配置 RSP，再给公共 launch 传入 runtime.yaml、ros.yaml 和组合模型。controller_manager 单独读取 controllers.yaml：

<!-- source: examples/wheel_car_4w/wheel_car_4w_sim/launch/wheel_car_4w.launch.py -->
```python
"""Example-owned RSP and ros2_control; simulator gets explicit model/parameter files."""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def required_exit(event, context):
    if not context.is_shutdown:
        raise RuntimeError(f"{event.process_name} exited unexpectedly ({event.returncode})")


def select_rviz(context, share):
    if LaunchConfiguration("rviz_config").perform(context):
        return []
    filename = "mid360.rviz" if LaunchConfiguration("lidar_profile").perform(context) == "mid360" else "wheel_car_4w.rviz"
    return [SetLaunchConfiguration("rviz_config", os.path.join(share, "rviz", filename))]


def setup_rsp(context, rsp_params):
    robot_xacro = LaunchConfiguration("rsp_model").perform(context)
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
    return [RegisterEventHandler(OnProcessExit(target_action=rsp, on_exit=required_exit)), rsp]


def generate_launch_description():
    ld = LaunchDescription()
    share = get_package_share_directory("wheel_car_4w_sim")
    simulator_share = get_package_share_directory("litesim3d_bringup")
    runtime_file = os.path.join(share, "config", "runtime.yaml")
    ros_params = os.path.join(share, "config", "ros.yaml")
    rsp_params = os.path.join(share, "config", "rsp.yaml")
    controller_params = os.path.join(share, "config", "controllers.yaml")
    description_share = get_package_share_directory("wheel_car_4w_description")
    robot_xacro = os.path.join(description_share, "urdf", "robot.urdf.xacro")
    simulation_xacro = os.path.join(description_share, "urdf", "simulation.urdf.xacro")
    control_xacro = os.path.join(share, "urdf", "control.urdf.xacro")

    ld.add_action(DeclareLaunchArgument("lidar_profile", default_value="config"))
    ld.add_action(DeclareLaunchArgument("rviz_config", default_value=""))
    # 参数文件可由自己的 bringup 替换；普通仿真选项沿用公共 launch 的默认值。
    ld.add_action(DeclareLaunchArgument("rsp_model", default_value=robot_xacro))
    ld.add_action(DeclareLaunchArgument("robot_urdf", default_value=simulation_xacro))
    ld.add_action(DeclareLaunchArgument("runtime", default_value=runtime_file))
    ld.add_action(DeclareLaunchArgument("params", default_value=ros_params))
    ld.add_action(DeclareLaunchArgument("controllers", default_value=controller_params))
    ld.add_action(DeclareLaunchArgument("controller_startup_timeout", default_value="60"))
    ld.add_action(DeclareLaunchArgument("world", default_value="office.sdf"))
    ld.add_action(DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"))
    ld.add_action(DeclareLaunchArgument("tf_topic", default_value="/tf"))
    ld.add_action(DeclareLaunchArgument("tf_static_topic", default_value="/tf_static"))
    ld.add_action(DeclareLaunchArgument("robot_description_topic", default_value="/robot_description"))

    # 机械结构和传感器安装单独展开给 RSP；不含仿真插件、ros2_control 标签。
    ld.add_action(OpaqueFunction(function=setup_rsp, args=[rsp_params]))

    ld.add_action(OpaqueFunction(function=select_rviz, args=[share]))
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(simulator_share, "launch", "simulation.launch.py")),
        launch_arguments={
            "assets_root": get_package_share_directory("wheel_car_4w_models"),
            "world": LaunchConfiguration("world"),
            "runtime": LaunchConfiguration("runtime"),
            "params": LaunchConfiguration("params"),
            "robot_urdf": LaunchConfiguration("robot_urdf"),
            "lidar_profiles_dir": os.path.join(share, "config", "lidars"),
            "lidar_profile": LaunchConfiguration("lidar_profile"),
            "rviz_config": LaunchConfiguration("rviz_config"),
        }.items(),
    )
    # 本案例选择仿真编码器里程计作为 /odom 与 odom→base_link 的唯一来源。
    # remap 仅作用于公共仿真 launch；控制器可替换而不影响里程计与 TF。
    ld.add_action(GroupAction([
        SetRemap(src="sim/odometry", dst="/odom"),
        simulation,
    ]))

    manager = Node(
        package="litesim3d_ros2_control", executable="spawn_when_ready.py", output="screen",
        arguments=["--executable", "ros2_control_node",
                   "--timeout", LaunchConfiguration("controller_startup_timeout"), "--"],
        # 不设置全局 name remap，防止动态加载的控制器被一并改名。
        parameters=[LaunchConfiguration("controllers"), {
            "robot_description": ParameterValue(xacro.process_file(control_xacro).toxml(), value_type=str),
        }],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/joint_states", LaunchConfiguration("joint_states_topic")),
        ],
    )
    ld.add_action(RegisterEventHandler(OnProcessExit(target_action=manager, on_exit=required_exit)))
    ld.add_action(manager)
    spawner = Node(
        package="litesim3d_ros2_control", executable="spawn_when_ready.py", output="screen",
        arguments=["--timeout", LaunchConfiguration("controller_startup_timeout"), "--", "joint_state_broadcaster", "diff_drive_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "60"],
    )
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=spawner,
        on_exit=lambda event, context: required_exit(event, context) if event.returncode else None)))
    ld.add_action(spawner)
    return ld
```

启动并持续发送指令：

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py gui:=false
# 另一个终端，使用相同的 ROS_DOMAIN_ID 和工作空间。
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.2}}'
```

选择 Mid360 时加 `lidar_profile:=mid360`。传感器配置与控制器独立，换雷达不需要改变底盘接口。需要算法直接发轮速时，可加载参数文件中已有的 wheel_velocity_controller，并通过 controller_manager 在一次 switch 中停用 diff_drive_controller、启用 wheel_velocity_controller；指令顺序是左后轮、右后轮，单位 rad/s。ForwardCommandController 不具备上游指令超时，长期使用时应由自己的控制器实现输入超时。

## 3. 无人机：机体 GPIO 与传感器组合

此案例是保持水平的速度运动学模型，零速度悬停，具备限速、加减速和三维碰撞；没有推力、姿态飞控或自由落体模型。不能把 GPIO 速度当成电机转速接口。

world 使用 drone_3d_models/worlds/drone_arena.sdf，提供场景、出生位置 (-8, -8, 1.2) m、积分步长。机械模型由 robot.urdf.xacro 单独给 RSP；仿真组合模型配置已有传感器插件：

<!-- source: examples/drone_3d/drone_3d_sim/urdf/simulation.urdf.xacro -->
```xml
<?xml version="1.0"?>
<robot name="quadrotor" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="camera_type" default="rgb"/>
  <xacro:property name="camera_kind" value="$(arg camera_type)"/>
  <xacro:include filename="$(find drone_3d_sim)/urdf/robot.urdf.xacro"/>
  <litesim3d version="1">
    <sensor name="roof_lidar" class="litesim3d_lidar/Lidar">
      <parameters file="package://litesim3d_lidar/config/parameters.yaml"/>
      <parameters file="package://drone_3d_sim/config/sensors.yaml" section="roof_lidar"/>
    </sensor>
    <sensor name="body_imu" class="litesim3d_imu/Imu">
      <parameters file="package://litesim3d_imu/config/parameters.yaml"/>
      <parameters file="package://drone_3d_sim/config/sensors.yaml" section="body_imu"/>
    </sensor>
    <sensor name="front_camera" class="litesim3d_${camera_kind}_camera/${dict(rgb='RgbCamera', depth='DepthCamera', rgbd='RgbdCamera')[camera_kind]}">
      <parameters file="package://litesim3d_${camera_kind}_camera/config/parameters.yaml"/>
      <parameters file="package://drone_3d_sim/config/sensors.yaml" section="front_camera"/>
    </sensor>
  </litesim3d>
</robot>
```

<!-- source: examples/drone_3d/drone_3d_sim/config/sensors.yaml -->
```yaml
# 由 simulation.urdf.xacro 按实例加载，未列字段继承对应插件参数文件。
roof_lidar:
  link: lidar_link  # 机械模型内的安装坐标系。
  topic: points_raw  # PointCloud2 输出，支持 ROS remap。
  rate_hz: 10.0  # 每秒扫描帧数。
  columns: 256  # 每帧水平列数；线数由下方仰角数组决定。
  vertical_angles_deg: [-30, -25, -20, -15, -10, -5, -2, 0, 2, 5, 10, 15, 20, 25, 30, 40]
  min_range_m: 0.1  # 最近有效距离，m。
  max_range_m: 50.0  # 最大有效距离，m。
  horizontal_start_deg: -180.0  # 水平起始角，度。
  horizontal_end_deg: 180.0  # 水平结束角，度。
  scan_mode: rolling  # 按真实运动轨迹生成一帧内各点。
  self_filter: robot  # 过滤整个机器人自身几何。
  publish_range_boundary: true  # 另发显示用量程边界，不混入实际回波。
  range_sigma_m: 0.0  # 默认零测距噪声；可用 noise_config 覆盖。
body_imu:
  link: imu_link  # 按此 link 轴向输出。
  topic: imu/data_raw  # sensor_msgs/Imu；含重力比力，无姿态估计。
  rate_hz: 200.0  # Hz。
front_camera:
  enabled: false  # launch cameras:=true 可开启。
  link: camera_optical_frame  # 光学坐标：z 前、x 右、y 下。
  topic: camera/front  # 输出图像和 CameraInfo 的前缀。
  width: 640  # 横向像素数。
  height: 480  # 纵向像素数。
  rate_hz: 15.0  # Hz。
  horizontal_fov_deg: 80.0  # 水平视场角，度。
  near_clip_m: 0.1  # 最近裁剪面，m。
  far_clip_m: 40.0  # 最远裁剪面，m。
```

控制模型声明一个 body GPIO，提供机体系 linear.x/y/z.velocity 和 angular.z.velocity 指令。状态 position.xyz 使用世界坐标，orientation.xyzw 是世界中机体四元数；速度状态使用机体系。反馈来自实际运动，不是简单回显指令。该理想状态适合控制接口联调；需要定位误差的算法应订阅自己的估计器输出。

<!-- source: examples/drone_3d/drone_3d_sim/urdf/control.urdf.xacro -->
```xml
<?xml version="1.0"?>
<robot name="quadrotor" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find drone_3d_sim)/urdf/robot.urdf.xacro"/>
  <ros2_control name="LiteSimDrone" type="system">
    <hardware>
      <plugin>litesim3d_ros2_control/LiteSimSystem</plugin>
      <param name="command_topic">/sim/joint_commands</param>
      <param name="state_topic">/sim/joint_feedback</param>
      <param name="feedback_timeout">2.0</param>
      <param name="activation_timeout">30.0</param>
    </hardware>
    <!-- 机体系速度；不虚构电机转速或可动 URDF 关节。 -->
    <gpio name="body">
      <command_interface name="linear.x.velocity"/>
      <command_interface name="linear.y.velocity"/>
      <command_interface name="linear.z.velocity"/>
      <command_interface name="angular.z.velocity"/>
      <state_interface name="linear.x.velocity"/>
      <state_interface name="linear.y.velocity"/>
      <state_interface name="linear.z.velocity"/>
      <state_interface name="angular.z.velocity"/>
      <state_interface name="position.x"/>
      <state_interface name="position.y"/>
      <state_interface name="position.z"/>
      <state_interface name="orientation.x"/>
      <state_interface name="orientation.y"/>
      <state_interface name="orientation.z"/>
      <state_interface name="orientation.w"/>
    </gpio>
  </ros2_control>
</robot>
```

<!-- source: examples/drone_3d/drone_3d_sim/config/runtime.yaml -->
```yaml
# SDF 提供环境、出生位姿、步长和目标实时倍率；组合 Xacro 提供机器人。
schema_version: 1  # 配置格式版本。
units: m_rad_s  # m、rad、s。
simulation:
  control_mode: ros2_control  # 外部 controller_manager 接管；仿真不订阅 cmd_vel。
  seed: 42  # 可复现随机种子。
  odom_rate: 50.0  # 里程计输出频率，Hz。
  command_timeout: 0.5  # 硬件指令中断后停车，仿真秒。
  threads: 2  # 射线计算线程数。
  collision_budget_enabled: false  # 关闭计算量预算，碰撞仍启用。
robot:
  urdf: ""  # launch 动态传入仿真组合 Xacro。
  id: quadrotor  # 与 world 中的机器人模型名称一致。
  base_link: base_link  # 机体运动参考坐标系。
  motion_model: multicopter  # 水平姿态的三维速度运动学。
  sensor_geometry: visual  # 传感器使用视觉几何。
  drive:
    # max_v、acceleration、deceleration 限制 XYZ 合向量，非逐轴限幅。
    max_v: 2.0  # 合速度上限，m/s。
    max_w: 1.2  # 偏航角速度上限，rad/s。
    acceleration: 1.0  # 合加速度上限，m/s²。
    deceleration: 2.0  # 减速上限，m/s²。
    angular_acceleration: 2.0  # 偏航角加速度上限，rad/s²。
    unlimited_acceleration: false  # 启用加减速限制。
    support_links: []  # 飞行机体不使用支撑轮。
terrain:
  enabled: false  # 无地形贴合；地面仍参与碰撞。
odom:
  mode: ideal  # 当前飞行模型提供理想里程计。
entity_bindings:
  ground:
    support: false  # 作为碰撞地面，不按轮式支撑处理。
    reflectivity: 0.35  # 地面激光反射率。
```

<!-- source: examples/drone_3d/drone_3d_sim/config/controllers.yaml -->
```yaml
# 案例负责启动 controller_manager；公共仿真 launch 不读取本文件。
/controller_manager:
  ros__parameters:
    use_sim_time: true  # 控制周期采用仿真时间。
    update_rate: 100  # Hz。
    body_velocity_controller:
      type: litesim3d_ros2_control/BodyVelocityController
/body_velocity_controller:
  ros__parameters:
    use_sim_time: true
    body_name: body  # 对应 control.urdf.xacro 中 gpio 的 name。
    command_timeout: 0.5  # 秒；指令停止后输出零速度。
    max_linear_velocity: 2.0  # XYZ 速度向量的模长上限，m/s。
    max_angular_velocity: 1.2  # 偏航角速度上限，rad/s。
```

无人机没有活动关节，所以不启动 joint_state_broadcaster，RSP 仍独立发布固定安装 TF。launch 中显式给仿真、RSP、控制管理器传入各自配置：

<!-- source: examples/drone_3d/drone_3d_sim/launch/drone_3d.launch.py -->
```python
"""Example-owned RSP and ros2_control; simulator gets explicit model/parameter files."""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def required_exit(event, context):
    if not context.is_shutdown:
        raise RuntimeError(f"{event.process_name} exited unexpectedly ({event.returncode})")


def setup_rsp(context, rsp_params):
    robot_xacro = LaunchConfiguration("rsp_model").perform(context)
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
    return [RegisterEventHandler(OnProcessExit(target_action=rsp, on_exit=required_exit)), rsp]


def generate_launch_description():
    ld = LaunchDescription()
    share = get_package_share_directory("drone_3d_sim")
    simulator_share = get_package_share_directory("litesim3d_bringup")
    runtime_file = os.path.join(share, "config", "runtime.yaml")
    ros_params = os.path.join(share, "config", "ros.yaml")
    rsp_params = os.path.join(share, "config", "rsp.yaml")
    controller_params = os.path.join(share, "config", "controllers.yaml")
    robot_xacro = os.path.join(share, "urdf", "robot.urdf.xacro")
    simulation_xacro = os.path.join(share, "urdf", "simulation.urdf.xacro")
    control_xacro = os.path.join(share, "urdf", "control.urdf.xacro")

    # 参数文件可由自己的 bringup 替换；普通仿真选项沿用公共 launch 的默认值。
    ld.add_action(DeclareLaunchArgument("rsp_model", default_value=robot_xacro))
    ld.add_action(DeclareLaunchArgument("robot_urdf", default_value=simulation_xacro))
    ld.add_action(DeclareLaunchArgument("runtime", default_value=runtime_file))
    ld.add_action(DeclareLaunchArgument("params", default_value=ros_params))
    ld.add_action(DeclareLaunchArgument("controllers", default_value=controller_params))
    ld.add_action(DeclareLaunchArgument("controller_startup_timeout", default_value="60"))
    ld.add_action(DeclareLaunchArgument("world", default_value="drone_arena.sdf"))
    ld.add_action(DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"))
    ld.add_action(DeclareLaunchArgument("tf_topic", default_value="/tf"))
    ld.add_action(DeclareLaunchArgument("tf_static_topic", default_value="/tf_static"))
    ld.add_action(DeclareLaunchArgument("robot_description_topic", default_value="/robot_description"))

    # 机械结构和传感器安装单独展开给 RSP；不含仿真插件、ros2_control 标签。
    ld.add_action(OpaqueFunction(function=setup_rsp, args=[rsp_params]))

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(simulator_share, "launch", "simulation.launch.py")),
        launch_arguments={
            "assets_root": get_package_share_directory("drone_3d_models"),
            "world": LaunchConfiguration("world"),
            "runtime": LaunchConfiguration("runtime"),
            "params": LaunchConfiguration("params"),
            "robot_urdf": LaunchConfiguration("robot_urdf"),
        }.items(),
    )
    # 无人机选择仿真理想里程计作为 /odom 与 odom→base_link 的唯一来源。
    # remap 仅作用于公共仿真 launch；控制器可替换而不影响里程计与 TF。
    ld.add_action(GroupAction([
        SetRemap(src="sim/odometry", dst="/odom"),
        simulation,
    ]))

    manager = Node(
        package="litesim3d_ros2_control", executable="spawn_when_ready.py", output="screen",
        arguments=["--executable", "ros2_control_node",
                   "--timeout", LaunchConfiguration("controller_startup_timeout"), "--"],
        # 不设置全局 name remap，防止动态加载的控制器被一并改名。
        parameters=[LaunchConfiguration("controllers"), {
            "robot_description": ParameterValue(xacro.process_file(control_xacro).toxml(), value_type=str),
        }],
        remappings=[
            ("/body_velocity_controller/cmd_vel", "/cmd_vel"),
            ("/joint_states", LaunchConfiguration("joint_states_topic")),
        ],
    )
    ld.add_action(RegisterEventHandler(OnProcessExit(target_action=manager, on_exit=required_exit)))
    ld.add_action(manager)
    spawner = Node(
        package="litesim3d_ros2_control", executable="spawn_when_ready.py", output="screen",
        arguments=["--timeout", LaunchConfiguration("controller_startup_timeout"), "--", "body_velocity_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "60"],
    )
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=spawner,
        on_exit=lambda event, context: required_exit(event, context) if event.returncode else None)))
    ld.add_action(spawner)
    return ld
```

```bash
ros2 launch drone_3d_sim drone_3d.launch.py gui:=false
# 另一个终端：机体系 x 前、y 左、z 上；yaw 为 angular.z。
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.1, z: 0.3}, angular: {z: 0.2}}'
```

BodyVelocityController 的 command_timeout 管算法输入，runtime 的 simulation.command_timeout 管硬件报文中断；适配器 feedback_timeout 用墙钟检测反馈失联。正常暂停仍发送反馈，不触发失联。停用或重新激活控制器会清空旧指令。

相机默认关闭。三类相机使用同一安装位置，按需选择其中一个入口：

```bash
ros2 launch drone_3d_sim drone_3d.launch.py cameras:=true
ros2 launch drone_3d_sim drone_3d.launch.py cameras:=true model_arguments:='{"camera_type":"depth"}'
ros2 launch drone_3d_sim drone_3d.launch.py cameras:=true model_arguments:='{"camera_type":"rgbd"}'
```

RGB 发布 camera/front/color/image_raw，Depth 发布 camera/front/depth/image_raw，RGB-D 同时发布两者及配套 camera_info。传感器参数修改 sensors.yaml；世界与出生位置修改 world；控制器限速修改 controllers.yaml；机体物理运动限制修改 runtime.yaml。每次选择一个入口启动。

## 4. 独立构建第三方算法插件

仓库的 examples/external_plugins/litesim3d_example_altitude_controller 是独立 ros2_control 插件包，默认 COLCON_IGNORE。示例实现 P 定高控制：读取 body/position.z，以目标高度误差计算限幅垂直速度，同时将其他机体速度轴置零。插件只依赖 controller_interface、pluginlib、rclcpp、realtime_tools、std_msgs，不依赖仿真核心。

在已经加载仿真工作空间的终端，从仓库根目录复制到新的算法工作空间：

```bash
mkdir -p ~/litesim_algorithms_ws/src
cp -r examples/external_plugins/litesim3d_example_altitude_controller ~/litesim_algorithms_ws/src/
rm ~/litesim_algorithms_ws/src/litesim3d_example_altitude_controller/COLCON_IGNORE
cd ~/litesim_algorithms_ws
colcon build --merge-install
source install/setup.bash
```

实现文件 [controller.cpp](../../examples/external_plugins/litesim3d_example_altitude_controller/src/controller.cpp) 依次声明资源、加载参数、创建输入订阅、激活清空旧目标、在 update 读状态和写指令、停用归零。输入通过 RealtimeBuffer 交给 update；回调不直接写硬件。它处理非有限输入、输入超时和时钟倒退，激活代次防止停用前的回调恢复旧命令。

插件声明与构建文件可以直接作为自己的控制器包起点：

<!-- source: examples/external_plugins/litesim3d_example_altitude_controller/plugins.xml -->
```xml
<library path="example_altitude_controller">
  <class name="litesim3d_example_altitude_controller/AltitudeController" type="example::AltitudeController" base_class_type="controller_interface::ControllerInterface">
    <description>Example proportional altitude controller using body GPIO feedback.</description>
  </class>
</library>
```

<!-- source: examples/external_plugins/litesim3d_example_altitude_controller/CMakeLists.txt -->
```cmake
cmake_minimum_required(VERSION 3.16)
project(litesim3d_example_altitude_controller LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
set(DEPS controller_interface pluginlib rclcpp realtime_tools std_msgs)
foreach(dep ${DEPS})
  find_package(${dep} REQUIRED)
endforeach()
add_library(example_altitude_controller SHARED src/controller.cpp)
target_compile_features(example_altitude_controller PUBLIC cxx_std_17)
target_compile_options(example_altitude_controller PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(example_altitude_controller ${DEPS})
pluginlib_export_plugin_description_file(controller_interface plugins.xml)
install(TARGETS example_altitude_controller LIBRARY DESTINATION lib)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
```

该参数文件同时声明默认速度控制器和第三方控制器：

<!-- source: examples/external_plugins/litesim3d_example_altitude_controller/config/controllers.yaml -->
```yaml
# 案例负责启动 controller_manager；公共仿真 launch 不读取本文件。
/controller_manager:
  ros__parameters:
    use_sim_time: true  # 控制周期采用仿真时间。
    update_rate: 100  # Hz。
    altitude_controller:
      type: litesim3d_example_altitude_controller/AltitudeController  # 单独工作空间构建的算法插件。
    body_velocity_controller:
      type: litesim3d_ros2_control/BodyVelocityController
/body_velocity_controller:
  ros__parameters:
    use_sim_time: true
    body_name: body  # 对应 control.urdf.xacro 中 gpio 的 name。
    command_timeout: 0.5  # 秒；指令停止后输出零速度。
    max_linear_velocity: 2.0  # XYZ 速度向量的模长上限，m/s。
    max_angular_velocity: 1.2  # 偏航角速度上限，rad/s。
/altitude_controller:
  ros__parameters:
    use_sim_time: true
    body_name: body  # 硬件 GPIO 资源名。
    target_topic: /altitude_target  # Float64；可替换话题，不绑定仿真或 RSP 节点名。
    gain: 1.0  # P 增益，单位 1/s。
    max_vertical_velocity: 0.4  # m/s，饱和后的垂直速度。
    command_timeout: 0.5  # 仿真秒；目标高度需持续发布，超时悬停。
```

给现有案例显式传入这份参数文件：

```bash
ros2 launch drone_3d_sim drone_3d.launch.py gui:=false \
  controllers:="$(ros2 pkg prefix litesim3d_example_altitude_controller)/share/litesim3d_example_altitude_controller/config/controllers.yaml"
```

在另一个同时加载仿真与算法工作空间的终端，先加载为 inactive，再一次性交接资源：

```bash
ros2 run controller_manager spawner altitude_controller --inactive -c /controller_manager
ros2 control switch_controllers --deactivate body_velocity_controller --activate altitude_controller --strict
ros2 topic pub --rate 10 /altitude_target std_msgs/msg/Float64 '{data: 1.6}'
```

高度单位 m，以上目标是世界高度 1.6 m。默认出生高度为 1.2 m；该闭环不依赖 odom 初始高度的零点。停止持续发目标后，超时输出零速度并悬停。两个控制器不能同时申领同一速度接口。

切回默认速度控制器：

```bash
ros2 control switch_controllers --deactivate altitude_controller --activate body_velocity_controller --strict
```

新控制器激活后需重新发送指令，旧的上升或下降指令不会被重放。更换为自己的算法时保留资源名、状态单位、输入超时和生命周期处理，替换 update 内的控制律；算法如果只想输出高层速度，直接发布 /cmd_vel 即可。

## 5. 接到自己的 bringup

独立展开机械 Xacro 给 RSP，展开 control Xacro 给 controller_manager，将 simulation Xacro 和 runtime/ROS 参数文件传给公共 launch。配置文件里 robot.urdf 留空，文件路径在 launch 内动态查找。不要在公共仿真 launch 中加入 RSP、controller_manager 或 joint_state_publisher。

仿真内部端点通过 joint_commands_topic / joint_feedback_topic 显式 remap；硬件 command_topic / state_topic 填写改名后的端点。算法输入在控制器 Node 的 remappings 或控制器参数中配置；joint_states、tf、tf_static、robot_description 的 remap 由案例 RSP 和相应发布方配套设置。节点名与命名空间不构成接口契约。

odom 与 TF 的所有权要一致：本页四轮车、无人机选择仿真里程计，控制器不重复发布 odom TF。传感器话题可交给任意第三方感知、定位或导航节点，无需改变控制硬件插件。
