# 全部模型与第三方算法 ros2_control

为现有模型接入 ros2_control，或用独立控制器替换默认算法。各案例自行创建 RSP、controller_manager 和控制器，再调用公共仿真 launch。

## 1. 准备环境与选择模型

完成[安装与构建](../usage/安装与构建.md)后，每个终端进入源码仓库根目录并执行：

```bash
source /opt/ros/humble/setup.bash
source "$(python3 scripts/paths.py install)/setup.bash"
export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1
```

标准 colcon 用户改为加载自己的 `install/setup.bash`。同一 domain 每次只运行一个案例；Ctrl+C 结束旧 launch 后再切换模型。

| 模型 / 插件 | ros2_control 资源 | 默认控制器与案例 |
| --- | --- | --- |
| DifferentialDrive | 独立轮关节 velocity，position/velocity 反馈 | diff_drive_controller；四轮车、双 MID360 |
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

完整文件：[urdf/control.urdf.xacro](../../examples/wheel_car_4w/wheel_car_4w_sim/urdf/control.urdf.xacro)。

机械模型的轮半径为 0.12 m、轮距为 0.58 m，launch 使用 `UrdfControllerParameters` 自动读取并填入控制器参数。源 YAML 只绑定左右后轮；前轮返回 mimic 状态，不能重复申领。

完整文件：[config/controllers.yaml](../../examples/wheel_car_4w/wheel_car_4w_sim/config/controllers.yaml)。

案例用 ld 单独配置 RSP，再给公共 launch 传入 runtime.yaml、ros.yaml 和组合模型。controller_manager 单独读取 controllers.yaml：

完整文件：[launch/wheel_car_4w.launch.py](../../examples/wheel_car_4w/wheel_car_4w_sim/launch/wheel_car_4w.launch.py)。

终端 A 启动：

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py gui:=false
```

终端 B 发送速度：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.2}}'
```

选择 Mid360 时加 `lidar_profile:=mid360`。传感器配置与控制器独立，换雷达不需要改变底盘接口。需要算法直接发轮速时，可加载参数文件中已有的 wheel_velocity_controller，并通过 controller_manager 在一次 switch 中停用 diff_drive_controller、启用 wheel_velocity_controller；指令顺序是左后轮、右后轮，单位 rad/s，发布端使用 RELIABLE QoS。ReliableForwardCommandController 不具备上游指令超时，长期使用时应由自己的控制器实现输入超时。

## 3. 无人机：机体 GPIO 与传感器组合

此案例是保持水平的速度运动学模型，零速度悬停，具备限速、加减速和三维碰撞；没有推力、姿态飞控或自由落体模型。不能把 GPIO 速度当成电机转速接口。

world 使用 drone_3d_models/worlds/drone_arena.sdf，提供场景和积分步长。出生位置 (-8, -8, 1.2) m 由 `config/ros.yaml` 的 `initial_pose_override` 加载。机械模型由 robot.urdf.xacro 单独给 RSP；仿真组合模型配置已有传感器插件：

完整文件：[urdf/simulation.urdf.xacro](../../examples/drone_3d/drone_3d_sim/urdf/simulation.urdf.xacro)。

完整文件：[config/sensors.yaml](../../examples/drone_3d/drone_3d_sim/config/sensors.yaml)。

控制模型声明一个 body GPIO，提供机体系 linear.x/y/z.velocity 和 angular.z.velocity 指令。状态 position.xyz 使用世界坐标，orientation.xyzw 是世界中机体四元数；速度状态使用机体系。反馈来自实际运动，不是简单回显指令。该理想状态适合控制接口联调；需要定位误差的算法应订阅自己的估计器输出。

完整文件：[urdf/control.urdf.xacro](../../examples/drone_3d/drone_3d_sim/urdf/control.urdf.xacro)。

完整文件：[config/runtime.yaml](../../examples/drone_3d/drone_3d_sim/config/runtime.yaml)。

完整文件：[config/controllers.yaml](../../examples/drone_3d/drone_3d_sim/config/controllers.yaml)。

无人机没有活动关节，所以不启动 joint_state_broadcaster，RSP 仍独立发布固定安装 TF。launch 中显式给仿真、RSP、控制管理器传入各自配置：

完整文件：[launch/drone_3d.launch.py](../../examples/drone_3d/drone_3d_sim/launch/drone_3d.launch.py)。

```bash
ros2 launch drone_3d_sim drone_3d.launch.py gui:=false
```

终端 B 发送速度，x 前、y 左、z 上，angular.z 为偏航：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.1, z: 0.3}, angular: {z: 0.2}}'
```

BodyVelocityController 的 command_timeout 管算法输入，runtime 的 simulation.command_timeout 管硬件报文中断；适配器 feedback_timeout 用墙钟检测反馈失联。正常暂停仍发送反馈，不触发失联。停用或重新激活控制器会清空旧指令。

相机默认关闭。三类相机使用同一安装位置，按需选择其中一个入口：

```bash
ros2 launch drone_3d_sim drone_3d.launch.py cameras:=true
ros2 launch drone_3d_sim drone_3d.launch.py cameras:=true model_arguments:='{"camera_type":"depth"}'
ros2 launch drone_3d_sim drone_3d.launch.py cameras:=true model_arguments:='{"camera_type":"rgbd"}'
```

RGB 发布 camera/front/color/image_raw，Depth 发布 camera/front/depth/image_raw，RGB-D 同时发布两者及配套 camera_info。传感器参数修改 sensors.yaml；场景修改 world，出生位姿修改 ros.yaml 的 initial_pose_override，并用 `params:=绝对路径` 加载；控制器限速修改 controllers.yaml；机体物理运动限制修改 runtime.yaml。每次选择一个入口启动。

## 4. 独立构建第三方算法插件

仓库的 examples/external_plugins/litesim3d_example_altitude_controller 是独立 ros2_control 插件包，默认 COLCON_IGNORE。示例实现 P 定高控制：读取 body/position.z，以目标高度误差计算限幅垂直速度，同时将其他机体速度轴置零。插件只依赖 controller_interface、pluginlib、rclcpp、realtime_tools、std_msgs，不依赖仿真核心。

停止已有仿真，在已加载仿真安装环境的终端，从仓库根目录复制到算法工作空间。以下目录应尚无同名插件包；已有该包时直接编辑并重新构建。

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

完整文件：[litesim3d_example_altitude_controller/plugins.xml](../../examples/external_plugins/litesim3d_example_altitude_controller/plugins.xml)。

完整文件：[litesim3d_example_altitude_controller/CMakeLists.txt](../../examples/external_plugins/litesim3d_example_altitude_controller/CMakeLists.txt)。

该参数文件同时声明默认速度控制器和第三方控制器：

完整文件：[config/controllers.yaml](../../examples/external_plugins/litesim3d_example_altitude_controller/config/controllers.yaml)。

给现有案例显式传入这份参数文件：

```bash
ros2 launch drone_3d_sim drone_3d.launch.py gui:=false \
  controllers:="$(ros2 pkg prefix litesim3d_example_altitude_controller)/share/litesim3d_example_altitude_controller/config/controllers.yaml"
```

在终端 B 加载第 1 节的环境和算法安装环境，先加载为 inactive，再交接资源：

```bash
source ~/litesim_algorithms_ws/install/setup.bash
ros2 run controller_manager spawner altitude_controller --inactive -c /controller_manager
ros2 control switch_controllers --deactivate body_velocity_controller --activate altitude_controller --strict
ros2 topic pub --rate 10 /altitude_target std_msgs/msg/Float64 '{data: 1.6}'
```

高度单位 m，以上目标是世界高度 1.6 m。默认出生高度为 1.2 m；该闭环不依赖 odom 初始高度的零点。停止持续发目标后，超时输出零速度并悬停。两个控制器不能同时申领同一速度接口。

Ctrl+C 停止目标高度发布后，切回默认速度控制器：

```bash
ros2 control switch_controllers --deactivate altitude_controller --activate body_velocity_controller --strict
```

新控制器激活后需重新发送指令，旧的上升或下降指令不会被重放。更换为自己的算法时保留资源名、状态单位、输入超时和生命周期处理，替换 update 内的控制律；算法如果只想输出高层速度，直接发布 /cmd_vel 即可。

## 5. 接到自己的 bringup

独立展开机械 Xacro 给 RSP，展开 control Xacro 给 controller_manager，将 simulation Xacro 和 runtime/ROS 参数文件传给公共 launch。配置文件里 robot.urdf 留空，文件路径在 launch 内动态查找。不要在公共仿真 launch 中加入 RSP、controller_manager 或 joint_state_publisher。

仿真内部端点通过 joint_commands_topic / joint_feedback_topic 显式 remap；硬件 command_topic / state_topic 填写改名后的端点。算法输入在控制器 Node 的 remappings 或控制器参数中配置；joint_states、tf、tf_static、robot_description 的 remap 由案例 RSP 和相应发布方配套设置。节点名与命名空间不构成接口契约。

odom 与 TF 的所有权要一致：本页四轮车、无人机选择仿真里程计，控制器不重复发布 odom TF；双 MID360 选择 diff_drive_controller，仿真关闭该 TF。传感器话题可交给任意第三方感知、定位或导航节点，无需改变控制硬件插件。
