# drone_3d：无人机三维仿真

本例采用 `robot.motion_model: multicopter`，默认由独立 controller_manager 加载 BodyVelocityController，经 LiteSimSystem 的机体 GPIO 控制三轴平移和偏航。机体保持水平，使用限速、加减速与三维几何碰撞约束；适合三维建图、避障与导航接口验证。当前不模拟电机推力、桨叶转动、空气动力学、姿态飞控或自由落体。初始位置为世界坐标 `(-8, -8, 1.2)` 米，零速度时悬停；IMU 仍包含重力比力。

## 目录

```text
drone_3d/
├── drone_3d_models/        # 无人机专用资源包
│   ├── models/            # quadrotor、aerial_course、textured_crate
│   └── worlds/            # drone_arena.sdf
└── drone_3d_sim/
    ├── config/            # runtime、传感器、controllers 和 RSP 参数
    ├── urdf/              # 分别给 RSP、仿真、控制管理器的模型
    ├── launch/            # 无人机启动入口
    └── test/              # ROS 与图形回归
```

## 启动

按仓库 [README](../../README.zh-CN.md) 安装依赖和构建，在同一个已 source 的 Humble 环境中执行：

```bash
source "$(python3 scripts/paths.py install)/setup.bash"
ros2 launch drone_3d_sim drone_3d.launch.py
```

默认打开只读三维窗口，只允许旋转、平移、缩放。碰撞预算和相机默认关闭，传感器偏置和噪声默认均为 0。需要观察点云和 TF 时追加 `rviz:=true`；内置 RViz 视图不显示 `/scene` 场景层。同一 ROS domain 每次只启动一个仿真世界。关闭观察窗口不会停止仿真；Ctrl+C 退出整个 launch。

复杂飞行场地为 24 × 24 米，包含不同高度的门框、立柱、桥梁、悬空梁、阶梯平台、货架和起降区。门框开口、桥下和桌下按实际几何保留通路；所有碰撞检测都启用。地图、机器人、贴图均由独立的 [`drone_3d_models`](drone_3d_models/README.md) 资源包安装，通过 ament share 和本次 launch 的 `GZ_SIM_RESOURCE_PATH` 定位，不读取小车资源目录。

```bash
# 无观察窗口；传感器和运动仿真继续运行。
ros2 launch drone_3d_sim drone_3d.launch.py gui:=false
# 目标半速，并启用碰撞复杂度上限（不会减少或跳过碰撞体）。
ros2 launch drone_3d_sim drone_3d.launch.py real_time_factor:=0.5 collision_budget_enabled:=true
```

以上启动命令择一运行。实际 RTF 由计算耗时决定，可低于目标。

## 三维运动和话题

在另一个加载相同 ROS 环境的终端发送：

```bash
# 从初始悬停位置上升；linear.z > 0 向上。
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {z: 0.4}}'
# 停止上条发布命令后，可沿机体前方和左方运动，同时偏航。
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3, y: 0.2}, angular: {z: 0.2}}'
```

`linear.x/y/z` 分别为机体前、左、上方向的 m/s；`angular.z` 为绕上轴的 rad/s。`angular.x/y` 非零或输入非有限值时，控制器输出停车目标。指令中断 0.5 **仿真秒**后，三轴和偏航速度减速至零并悬停。碰撞会停止全部速度；地板、侧墙、顶梁均参与防穿透检测。模型没有自动起降控制器。

| 数据 | 话题 / 坐标系 |
| --- | --- |
| 16 线三维雷达，256 列，10 Hz | `/points_raw`，`lidar_link` |
| IMU，200 Hz | `/imu/data_raw`，`imu_link` |
| 三维里程计，50 Hz | `/odom`，相对初始位姿；twist 包含 x/y/z 速度 |
| 世界位姿真值 | `/sim/ground_truth/quadrotor/odom` |
| 时钟与只读统计 | `/clock`、`/sim/status`、`/sim/stats` |
| TF | `odom -> base_link -> lidar_link / imu_link / camera_optical_frame` |

图形窗口根据世界真值显示模型。`odom` 从初始位姿开始为零，所以初始的世界高度 1.2 m 对应 odom.z=0。雷达运动畸变使用完整三维轨迹；相机和 IMU 使用采样时刻的三维位姿。暂停和单步沿用 [ROS 服务接口](../../docs/api/消息与服务.md)。

## 参数、普通相机与深度相机

- [`runtime.yaml`](drone_3d_sim/config/runtime.yaml)：control_mode、机体速度限制、硬件指令超时和运动类型。`drive.max_v` 是 XYZ 合速度上限，`acceleration` / `deceleration` 是合加速度上限，`max_w` / `angular_acceleration` 限制偏航；不需要轮关节或车轮参数。
- [`controllers.yaml`](drone_3d_sim/config/controllers.yaml)：默认 BodyVelocityController 的算法输入超时和限速。
- [`sensors.yaml`](drone_3d_sim/config/sensors.yaml)：传感器实例参数；类名在 simulation.urdf.xacro 中声明。
- [`rsp.yaml`](drone_3d_sim/config/rsp.yaml)：独立 RSP 参数，机械模型单独展开。
- [`ros.yaml`](drone_3d_sim/config/ros.yaml)：出生位姿 `initial_pose_override`、ROS 参数、QoS 和 TF。默认值为 `[-8.0, -8.0, 1.2, 0.0, 0.0, 0.0, 1.0]`，顺序是 `[x,y,z,qx,qy,qz,qw]`。使用 `params:=/绝对路径/ros.yaml` 加载自定义文件。
- [`drone_arena.sdf`](drone_3d_models/worlds/drone_arena.sdf)：地图、步长、默认目标 RTF 和观察相机初始视角。
- [`robot.urdf.xacro`](drone_3d_sim/urdf/robot.urdf.xacro)：默认组合模型的机械结构、安装位置和 visual/collision；保留原 SDF 同样的几何。SDF model 用于兼容入口，默认模型由 launch 显式传入。
- `terrain.enabled` 必须为 false，里程计目前只支持 `odom.mode: ideal`。所有配置在启动时读取，修改后重新启动。

相机默认禁用、类型为 `rgb`。开启普通相机只发布 640 × 480、15 Hz 的彩色图像及 CameraInfo，不打开摄像头窗口：

```bash
ros2 launch drone_3d_sim drone_3d.launch.py cameras:=true
```

需要深度数据时使用 `model_arguments:='{"camera_type":"rgbd"}'` 或 `model_arguments:='{"camera_type":"depth"}'`，并设置 `cameras:=true`。在 sensors.yaml 修改 width、height、rate_hz 控制分辨率与帧率。话题为 /camera/front/color/image_raw 和按类型启用的 /camera/front/depth/image_raw，分别为 rgb8 与 32FC1 米制光轴深度，并带同时间戳 CameraInfo。相机需要可用的渲染后端，详见 [SDF 与相机说明](../../docs/usage/SDF与独立窗口.md)。

独立 RSP、硬件 GPIO、参数文件和第三方定高控制器的构建、加载与切换见 [完整教程](../../docs/tutorials/全部模型与第三方算法ros2_control.md)。此模型没有活动关节，默认不启动 joint_state_broadcaster。旧直接 Twist 入口为 legacy_twist.launch.py。

修改地图时编辑本例的 SDF 世界，重新构建后启动。
