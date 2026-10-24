# 阿克曼与全向轮接入 ros2_control

阿克曼、三轮和四轮普通全向案例都使用外部 ros2_control。第三方算法可以发布车体速度，也可以通过控制器管理器切换到关节接口。公共仿真 launch 只启动仿真及可选显示，RSP、controller_manager、控制器和状态广播器由各自案例配置。

| 案例包 | 速度控制器 | 主动关节 | 被动关节 |
| --- | --- | --- | --- |
| ackermann_sim | ackermann_steering_controller/AckermannSteeringController | 后轮 velocity + 前轮转向 position | 前轮旋转，只读反馈 |
| omni3_sim | litesim3d_ros2_control/OmniDriveController | 三个轮 velocity | 无 |
| omni4_sim | litesim3d_ros2_control/OmniDriveController | 四个轮 velocity | 无 |

差速底盘见 [四轮车案例](全部模型与第三方算法ros2_control.md)。这里的全向轮是带被动滚子的普通全向轮；四轮采用角点切向 X 布局。

## 1. 克隆与文件分工

假设 Ubuntu 22.04 / ROS 2 Humble 开发环境已经准备好：

```bash
git clone --branch humble https://github.com/chiway-luo/litesim3d_ros2.git ~/chassis_ws/src/litesim3d_ros2
cd ~/chassis_ws
```

每个案例独立安装。例如 `src/litesim3d_ros2/examples/ackermann/ackermann_sim/`：

| 文件 | 配置内容 |
| --- | --- |
| worlds/room.sdf | 地面、障碍物、机器人出生位姿和仿真步长 |
| config/geometry.yaml | 轮位、轮径、转向几何，机械和仿真插件共用 |
| urdf/mechanical.urdf.xacro | 车体、车轮、转向关节和 URDF 限位 |
| urdf/sensor_mounts.xacro | 雷达、IMU、相机的安装 link 和固定关节 |
| urdf/robot.urdf.xacro | 机械结构与传感器安装，单独展开给 RSP |
| urdf/sensors.xacro | 引用安装结构并配置传感器插件 |
| urdf/simulation.urdf.xacro | 组合完整模型，选择仿真底盘和编码器里程计 |
| urdf/control.urdf.xacro | 机械模型加 ros2_control 硬件与关节接口 |
| config/runtime.yaml | control_mode、碰撞策略、地形和机器人身份 |
| config/ros.yaml、rsp.yaml | 仿真节点和 RSP 参数 |
| config/controllers.yaml | 管理器频率、控制器和 broadcaster 参数 |
| launch/ackermann.launch.py | 使用 ld 组装上述文件与节点 |

自己的工程可将机械与安装结构移入独立 description 包，将 world、runtime、控制器与 launch 放在自己的 bringup 包，再修改 `get_package_share_directory()` 使用的包名。

## 2. world 与传感器配置

`room.sdf` 的地面中心为 z=-0.05、厚度 0.1 m，因此顶面为 z=0。机器人 root 出生在地面投影，轮心高度等于轮半径。机器人 SDF model 只写身份及出生位姿，完整几何交给组合 URDF；环境实体要有 visual 和 collision。

<!-- source: examples/ackermann/ackermann_sim/worlds/room.sdf -->
```xml
<?xml version="1.0"?>
<sdf version="1.9"><world name="room">
  <physics name="timing" type="ignored" default="true"><max_step_size>0.01</max_step_size><real_time_factor>1</real_time_factor></physics>
  <scene><ambient>0.4 0.4 0.4 1</ambient><background>0.08 0.1 0.13 1</background></scene>
  <light name="sun" type="directional"><direction>-0.4 -0.3 -1</direction><diffuse>1 1 1 1</diffuse></light>
  <!-- 只提供机器人身份和 root 出生位姿；完整几何由组合 URDF 提供。 -->
  <model name="ackermann"><static>false</static><link name="spawn"/></model>
  <model name="ground"><static>true</static><pose>0 0 -0.05 0 0 0</pose><link name="floor">
    <visual name="floor"><geometry><box><size>20 20 0.1</size></box></geometry><material><diffuse>0.55 0.58 0.6 1</diffuse></material></visual>
    <collision name="floor"><geometry><box><size>20 20 0.1</size></box></geometry></collision>
  </link></model>
  <model name="wall"><static>true</static><pose>2.5 0 1 0 0 0</pose><link name="wall">
    <visual name="wall"><geometry><box><size>0.1 8 2</size></box></geometry><material><diffuse>0.8 0.6 0.3 1</diffuse></material></visual>
    <collision name="wall"><geometry><box><size>0.1 8 2</size></box></geometry></collision>
  </link></model>
  <model name="side_wall"><static>true</static><pose>0 2.5 1 0 0 0</pose><link name="wall">
    <visual name="wall"><geometry><box><size>8 0.1 2</size></box></geometry></visual>
    <collision name="wall"><geometry><box><size>8 0.1 2</size></box></geometry></collision>
  </link></model>
</world></sdf>
```

`runtime.robot.id` 必须等于 world 中机器人 model 的 name。三轮/四轮例子分别是 `omni3`、`omni4`。地面通过 entity_bindings.ground.support 参加轮式地形支撑。

安装位置写入 `sensor_mounts.xacro`，RSP 和仿真共用同一份 link/joint；传感器插件放在 `sensors.xacro`。雷达以 lidar_link 为测量坐标，IMU 为 imu_link，相机使用光学坐标。下面是阿克曼的传感器配置：

<!-- source: examples/ackermann/ackermann_sim/urdf/sensors.xacro -->
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="camera_enabled" default="false"/>
  <xacro:include filename="$(find ackermann_sim)/urdf/sensor_mounts.xacro"/>
  <litesim3d version="1">
    <sensor name="roof_lidar" class="litesim3d_lidar/Lidar">
      <parameters file="package://litesim3d_lidar/config/parameters.yaml"/>
      <overrides>columns: 64
max_range_m: 10.0
self_filter: robot</overrides>
    </sensor>
    <sensor name="body_imu" class="litesim3d_imu/Imu">
      <parameters file="package://litesim3d_imu/config/parameters.yaml"/>
      <overrides>rate_hz: 50.0</overrides>
    </sensor>
    <sensor name="front_camera" class="litesim3d_rgb_camera/RgbCamera">
      <parameters file="package://litesim3d_rgb_camera/config/parameters.yaml"/>
      <overrides>enabled: $(arg camera_enabled)
width: 160
height: 120</overrides>
    </sensor>
  </litesim3d>
</robot>
```

## 3. 声明 ros2_control 硬件和关节

阿克曼的后轮接 velocity；左右转向关节接 position。两个前轮旋转关节只声明状态接口，它们按实际行程滚动。位置与速度接口同时声明时仍只能由一个控制器占用其中一种。

<!-- source: examples/ackermann/ackermann_sim/urdf/control.urdf.xacro -->
```xml
<?xml version="1.0"?>
<robot name="ackermann" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find ackermann_sim)/urdf/robot.urdf.xacro"/>
  <ros2_control name="LiteSimSystem" type="system">
    <hardware>
      <plugin>litesim3d_ros2_control/LiteSimSystem</plugin>
      <!-- 两个内部话题须与仿真 remap 后的话题一致，可自行加机器人命名空间。 -->
      <param name="command_topic">/sim/joint_commands</param>
      <param name="state_topic">/sim/joint_feedback</param>
      <param name="feedback_timeout">2.0</param>
      <param name="activation_timeout">30.0</param>
    </hardware>
    <joint name="rear_left_wheel_joint">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="rear_right_wheel_joint">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="left_steering_joint">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="right_steering_joint">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <!-- 前轮被动滚动，只提供状态；不由控制器驱动。 -->
    <joint name="front_left_wheel_joint">
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <!-- 前轮被动滚动，只提供状态；不由控制器驱动。 -->
    <joint name="front_right_wheel_joint">
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
```

三轮全向声明三个独立轮；四轮全向按相同格式再加入 wheel_3_joint。每个主动轮提供 velocity 和可选 position，joint_state_broadcaster 读取真实 position/velocity。

<!-- source: examples/omni3/omni3_sim/urdf/control.urdf.xacro -->
```xml
<?xml version="1.0"?>
<robot name="omni3" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find omni3_sim)/urdf/robot.urdf.xacro"/>
  <ros2_control name="LiteSimSystem" type="system">
    <hardware>
      <plugin>litesim3d_ros2_control/LiteSimSystem</plugin>
      <!-- 两个内部话题须与仿真 remap 后的话题一致，可自行加机器人命名空间。 -->
      <param name="command_topic">/sim/joint_commands</param>
      <param name="state_topic">/sim/joint_feedback</param>
      <param name="feedback_timeout">2.0</param>
      <param name="activation_timeout">30.0</param>
    </hardware>
    <joint name="wheel_0_joint">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="wheel_1_joint">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="wheel_2_joint">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
```

所有主动关节在机械 URDF 中都必须有有限正的 velocity 限制。转向还需 lower/upper；本例为 ±0.6 rad。position 目标按关节速度上限逐步追踪，关节及车体碰撞会约束实际运动，反馈来自接受后的状态。

硬件 command_topic/state_topic 与仿真 joint_commands_topic/joint_feedback_topic 指向同一对话题即可；重命名或放到机器人命名空间时同时改两端，RSP 节点名不参与硬件连接。

## 4. 配置控制器

阿克曼使用 [Humble 标准控制器](https://control.ros.org/humble/doc/ros2_controllers/ackermann_steering_controller/doc/userdoc.html)。其数组顺序为右、左，front_wheels_names 填转向关节名称。geometry.yaml 与本文件的轴距、轮距、轮径应一致。

<!-- source: examples/ackermann/ackermann_sim/config/controllers.yaml -->
```yaml
/**:
  ros__parameters:
    use_sim_time: true  # 管理器和所有控制器使用 /clock。
/controller_manager:
  ros__parameters:
    update_rate: 100  # 控制循环频率，Hz。
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster  # 发布实际关节反馈。
    ackermann_controller:
      type: ackermann_steering_controller/AckermannSteeringController
    wheel_velocity_controller:
      type: forward_command_controller/ForwardCommandController  # 可选轮速直控，默认不加载。
    steering_position_controller:
      type: forward_command_controller/ForwardCommandController  # 可选转角直控，与轮速直控一起使用。
/joint_state_broadcaster:
  ros__parameters:
    use_local_topics: false  # 发布到全局 /joint_states，launch 可 remap。
/ackermann_controller:
  ros__parameters:
    # Humble 标准控制器数组顺序为右、左；转向接口填转向关节，不是前轮旋转关节。
    rear_wheels_names: [rear_right_wheel_joint, rear_left_wheel_joint]
    front_wheels_names: [right_steering_joint, left_steering_joint]
    front_steering: true  # 前轮转向、后轮驱动。
    # 下列尺寸与 geometry.yaml 保持一致，单位 m。
    wheelbase: 0.6
    front_wheel_track: 0.4
    rear_wheel_track: 0.4
    front_wheels_radius: 0.1
    rear_wheels_radius: 0.1
    position_feedback: true  # 后轮读取实际累计转角，转向读取实际角度。
    open_loop: false  # 里程计读取反馈，不把速度指令当作运动结果。
    use_stamped_vel: false  # reference_unstamped 接收 Twist，由 launch remap 至 /cmd_vel。
    reference_timeout: 0.5  # 算法停止发指令后停车，仿真秒。
    reduce_wheel_speed_until_steering_reached: true  # 大幅转向时先降低轮速。
    velocity_rolling_window_size: 1  # 不额外平滑编码器速度。
    enable_odom_tf: false  # 本案例 odom TF 已由仿真编码器里程计发布。
    odom_frame_id: odom  # 控制器自己的诊断里程计坐标系。
    base_frame_id: base_link  # 后轴中点的地面投影。
/wheel_velocity_controller:
  ros__parameters:
    joints: [rear_left_wheel_joint, rear_right_wheel_joint]  # Float64MultiArray.data 的顺序，rad/s。
    interface_name: velocity  # 与底盘控制器互斥，需原子切换。
/steering_position_controller:
  ros__parameters:
    joints: [left_steering_joint, right_steering_joint]  # data 的顺序，rad。
    interface_name: position  # 本车合法范围 [-0.6, 0.6]。
```

标准控制器接收 vx/wz，不支持原地旋转。算法应满足 `abs(wz/vx) <= tan(max_steering)/(wheelbase + track_width/2*tan(max_steering))`；本例约为 0.92849 m⁻¹（实际限幅应使用公式值）。零 vx 时也发送零 wz。越界的转向 position 指令会整包拒绝，并由仿真 command_timeout 停车；控制器不会代替上层算法做曲率限幅。

三轮全向控制器将 vx、vy、wz 换算成轮速；数组与 geometry.yaml 的 wheels 顺序一致。轮速超限时等比降低所有轮速，保持运动方向。

<!-- source: examples/omni3/omni3_sim/config/controllers.yaml -->
```yaml
/**:
  ros__parameters:
    use_sim_time: true  # 管理器和所有控制器使用 /clock。
/controller_manager:
  ros__parameters:
    update_rate: 100  # 控制循环频率，Hz。
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster  # 发布实际关节反馈。
    omni_controller:
      type: litesim3d_ros2_control/OmniDriveController
    wheel_velocity_controller:
      type: forward_command_controller/ForwardCommandController  # 可选轮速直控，默认不加载。
/joint_state_broadcaster:
  ros__parameters:
    use_local_topics: false  # 发布到全局 /joint_states，launch 可 remap。
/omni_controller:
  ros__parameters:
    # 数组索引一一对应；轮位、滚动方向和半径与 geometry.yaml 保持一致。
    wheel_joints: [wheel_0_joint, wheel_1_joint, wheel_2_joint]  # 独立轮关节。
    wheel_x: [0.3, -0.1499999999999999, -0.1500000000000001]  # 轮心 x，m。
    wheel_y: [0.0, 0.2598076211353316, -0.2598076211353315]  # 轮心 y，m。
    wheel_radius: [0.1, 0.1, 0.1]  # 有效半径，m。
    wheel_angle: [1.570796326794897, 3.665191429188092, 5.759586531581287]  # 正滚动方向相对 +x 的角度，rad。
    max_wheel_velocity: [30.0, 30.0, 30.0]  # 不超过机械 URDF 的 velocity，rad/s。
    max_linear_velocity: 1.0  # 平面速度向量的模长上限，m/s。
    max_angular_velocity: 1.0  # 绕 z 轴的角速度上限，rad/s。
    command_timeout: 0.5  # 算法停止发布 ~/cmd_vel 后停车，仿真秒。
/wheel_velocity_controller:
  ros__parameters:
    joints: [wheel_0_joint, wheel_1_joint, wheel_2_joint]  # Float64MultiArray.data 的顺序，rad/s。
    interface_name: velocity  # 与底盘控制器互斥，需原子切换。
```
四轮全向使用同一个控制器，换成下面四个轮的布局：

<!-- source: examples/omni4/omni4_sim/config/controllers.yaml -->
```yaml
/**:
  ros__parameters:
    use_sim_time: true  # 管理器和所有控制器使用 /clock。
/controller_manager:
  ros__parameters:
    update_rate: 100  # 控制循环频率，Hz。
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster  # 发布实际关节反馈。
    omni_controller:
      type: litesim3d_ros2_control/OmniDriveController
    wheel_velocity_controller:
      type: forward_command_controller/ForwardCommandController  # 可选轮速直控，默认不加载。
/joint_state_broadcaster:
  ros__parameters:
    use_local_topics: false  # 发布到全局 /joint_states，launch 可 remap。
/omni_controller:
  ros__parameters:
    # 数组索引一一对应；轮位、滚动方向和半径与 geometry.yaml 保持一致。
    wheel_joints: [wheel_0_joint, wheel_1_joint, wheel_2_joint, wheel_3_joint]  # 独立轮关节。
    wheel_x: [0.3, -0.3, -0.3, 0.2999999999999999]  # 轮心 x，m。
    wheel_y: [0.3, 0.3, -0.3, -0.3]  # 轮心 y，m。
    wheel_radius: [0.1, 0.1, 0.1, 0.1]  # 有效半径，m。
    wheel_angle: [2.356194490192345, 3.926990816987241, 5.497787143782138, 7.068583470577035]  # 正滚动方向相对 +x 的角度，rad。
    max_wheel_velocity: [30.0, 30.0, 30.0, 30.0]  # 不超过机械 URDF 的 velocity，rad/s。
    max_linear_velocity: 1.0  # 平面速度向量的模长上限，m/s。
    max_angular_velocity: 1.0  # 绕 z 轴的角速度上限，rad/s。
    command_timeout: 0.5  # 算法停止发布 ~/cmd_vel 后停车，仿真秒。
/wheel_velocity_controller:
  ros__parameters:
    joints: [wheel_0_joint, wheel_1_joint, wheel_2_joint, wheel_3_joint]  # Float64MultiArray.data 的顺序，rad/s。
    interface_name: velocity  # 与底盘控制器互斥，需原子切换。
```

全向控制器做逆运动学和速度限幅，不做路径跟踪或加速度规划。算法需要连续发布速度并自行规划平滑轨迹。两个控制器的算法超时均为 0.5 仿真秒；这与硬件失联的仿真 command_timeout 是不同层级。

## 5. runtime 和 ROS 参数

启用 ros2_control 后，仿真不订阅 cmd_vel，也不发布公共 joint_states。模型路径留空，由 launch 动态查找包资源并传入。

<!-- source: examples/ackermann/ackermann_sim/config/runtime.yaml -->
```yaml
# 场景和出生位姿来自 SDF；模型、底盘、里程计、传感器来自组合 Xacro。
schema_version: 1  # 运行配置格式版本。
units: m_rad_s  # 长度 m、角度 rad、时间 s。
simulation:
  # 由外部 controller_manager 写关节指令，仿真不直接订阅 cmd_vel。
  control_mode: ros2_control
  seed: 42  # 噪声与随机事件的可复现种子。
  odom_rate: 50.0  # 仿真编码器里程计输出频率，Hz。
  command_timeout: 0.5  # 硬件指令中断后的停车时间，仿真秒。
  threads: 2  # 传感器计算线程数。
robot:
  id: ackermann  # 必须与 world 中的机器人 model 名相同。
  urdf: ""  # launch 使用 get_package_share_directory 定位组合 Xacro 后传入。
  base_link: base_link  # 车体运动学参考坐标系。
  # true：碰撞时车体受阻，驱动轮和轮式里程计继续累计。
  continue_on_collision: true
  # 参与地形支撑的实际轮 link。
  support_links: [rear_left_wheel_link, rear_right_wheel_link, front_left_wheel_link, front_right_wheel_link]
terrain:
  enabled: true  # 使用场景支撑表面放置车体。
entity_bindings:
  ground:
    support: true  # ground 的 collision 作为可行驶地面。
```

三种案例均选择仿真编码器里程计作为 /odom 和 odom→base_link 的唯一来源；RSP 负责内部 link TF。这样第三方关节控制器接管时里程计仍可使用。阿克曼控制器自己的 odometry 保留在其私有话题，enable_odom_tf=false。

<!-- source: examples/ackermann/ackermann_sim/config/ros.yaml -->
```yaml
/**:
  ros__parameters:
    use_sim_time: true  # RSP、控制器和传感器使用仿真时钟。
/litesim3d:
  ros__parameters:
    publish_odom_tf: true  # 本案例由仿真编码器里程计发布 odom→base_link。
    external_odom_tf: false  # 未启用其他里程计 TF 来源。
    start_paused: false  # 启动后推进仿真；可通过 launch paused:=true 覆盖。
```

<!-- source: examples/ackermann/ackermann_sim/config/rsp.yaml -->
```yaml
/**:
  ros__parameters:
    use_sim_time: true  # 与仿真关节状态采用相同时间。
    publish_frequency: 200.0  # 活动关节 TF 的最大发布频率，Hz。
    frame_prefix: ""  # 修改时同步仿真帧配置。
```

## 6. 使用 ld 编写案例 launch

分别展开 robot.urdf.xacro 给 RSP、control.urdf.xacro 给 controller_manager；simulation.urdf.xacro 通过 robot_urdf 传给公共 launch 展开。以下代码显式列出 runtime、仿真 ROS、RSP 与控制器参数文件，普通仿真选项使用公共 launch 默认值。

<!-- source: examples/ackermann/ackermann_sim/launch/ackermann.launch.py -->
```python
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
```

[omni3.launch.py](../../examples/omni3/omni3_sim/launch/omni3.launch.py) 和 [omni4.launch.py](../../examples/omni4/omni4_sim/launch/omni4.launch.py) 使用同一结构：包名分别改为 omni3_sim/omni4_sim，spawner 激活 omni_controller，将 /omni_controller/cmd_vel remap 到 /cmd_vel。源码案例已完整配置。

RSP 的 joint_states、tf、tf_static、robot_description 都有显式 remap。修改 RSP 名称、命名空间时，保留对应话题连接即可。controller_manager 不使用全局 name remap，避免连带改变其动态加载的控制器节点名。

## 7. 构建和启动

在工作空间根目录：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --packages-up-to ackermann_sim omni3_sim omni4_sim
source install/setup.bash
```

每次选择一个案例；省略 gui:=false 打开三维窗口，rviz:=true 打开 RViz，cameras:=true 启用相机。

```bash
ros2 launch ackermann_sim ackermann.launch.py gui:=false
# 或
ros2 launch omni3_sim omni3.launch.py gui:=false
# 或
ros2 launch omni4_sim omni4.launch.py gui:=false
```

另一终端 source 同一安装环境，然后给阿克曼前进转弯：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.4}, angular: {z: 0.2}}'
```

全向案例横移或同时平移旋转：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {y: 0.3}}'
# 停止上一条发布后，发送复合运动。
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.2}, angular: {z: 0.2}}'
```

## 8. 第三方算法直接接管关节

若算法输出车体 Twist，直接发布 /cmd_vel。若算法输出每轮速度或转向角，先加载本案例预留的 ForwardCommandController，再在一次切换中停用旧控制器并启用新控制器。保持 joint_state_broadcaster 和 RSP 运行，不增加 joint_state_publisher。

阿克曼：

```bash
ros2 run controller_manager spawner wheel_velocity_controller steering_position_controller --inactive
ros2 control switch_controllers --deactivate ackermann_controller --activate wheel_velocity_controller steering_position_controller --strict
ros2 topic pub --once /steering_position_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.3, 0.25]}'
ros2 topic pub -r 20 /wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray '{data: [2.0, 2.0]}'
```

转角数组顺序为左、右，单位 rad；轮速数组为后左、后右，单位 rad/s。前轮旋转为只读反馈。恢复速度控制前停止上一条发布，再切换：

```bash
ros2 control switch_controllers --deactivate wheel_velocity_controller steering_position_controller --activate ackermann_controller --strict
```

全向案例：

```bash
ros2 run controller_manager spawner wheel_velocity_controller --inactive
ros2 control switch_controllers --deactivate omni_controller --activate wheel_velocity_controller --strict
# 三轮：按 wheel_0、wheel_1、wheel_2；这一组约为沿 +y 以 0.3 m/s 横移。
ros2 topic pub -r 20 /wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray '{data: [3.0, -1.5, -1.5]}'
# 四轮改发四个数：[2.12132, -2.12132, -2.12132, 2.12132]。
```

恢复时停用 wheel_velocity_controller、启用 omni_controller。自定义 ros2_control 控制器同样通过 controller_manager 声明并占用这些接口，无需改仿真器。

ForwardCommandController 会保持最近值，没有算法消息超时；停止算法前发送零轮速或停用它。需要自动超时停车的第三方关节控制器应自行实现消息看门狗。硬件通信继续时，仿真 command_timeout 不能替代算法超时。接口为运动学位置/速度控制，不提供 effort 或轮胎接触力动力学。
