<h1 align="center">LiteSim 3D</h1>

<p align="center">
  <img src="litesim3d_gui/resources/icons/litesim3d.svg" width="128" height="128" alt="LiteSim 3D 图标">
</p>

<p align="center">面向机器人算法开发的轻量 ROS 2 三维仿真器</p>

<p align="center">
  <a href="README.md">English</a> · <strong>简体中文</strong>
</p>

<p align="center">
  <a href="#准备环境">准备环境</a> ·
  <a href="#快速开始">快速开始</a> ·
  <a href="#文档">文档</a> ·
  <a href="examples/wheel_car_4w/README.md">四轮小车示例</a> ·
  <a href="examples/drone_3d/README.md">无人机示例</a>
</p>

---

基于 C++17 的轻量三维运动学仿真器，在 CPU 上计算运动、碰撞、雷达和 IMU 数据，通过 ROS 2 发布点云、IMU、里程计、TF 和仿真时钟。四轮车默认从 SDF 导入场景，在独立三维窗口查看模型材质、贴图和实际仿真速率；也可无界面运行。

支持 URDF、差速/理想 Ackermann 底盘、速度控制无人机、三维几何碰撞、连续坡面、滚动扫描、测量噪声、动态障碍、1–4 台同构机器人和单机会话重置。随附可直接启动的四轮小车，以及 VLP-16、Mid-360、Mid-40 扫描预设。它采用运动学和几何约束，不模拟接触力、悬架、轮胎、电机推力或飞控动力学；雷达预设的建模范围见 [雷达说明](examples/wheel_car_4w/雷达配置说明.md)。

## 准备环境

以下命令使用 **Ubuntu 22.04、ROS 2 Humble、Bash**。先按 [ROS 2 官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 安装 Humble；随后安装本项目依赖：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y git build-essential cmake pkg-config \
  libeigen3-dev liburdfdom-dev libassimp-dev libfcl-dev libyaml-cpp-dev \
  libtinyxml2-dev libssl-dev nlohmann-json3-dev libpcl-dev libgtest-dev \
  libsdformat12-dev libignition-rendering6-dev qtbase5-dev libqt5svg5-dev \
  python3-colcon-common-extensions python3-pytest python3-yaml python3-psutil \
  ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-xacro \
  ros-humble-tf2-ros ros-humble-launch-testing ros-humble-ament-cmake-pytest
```

macOS 使用已配置的 RoboStack Humble 环境，激活后沿用下方 clone、构建和运行步骤；不要执行 Ubuntu 的 apt 命令。环境要求、标准 colcon 工作空间方式及安装路径说明见 [安装与构建](docs/usage/安装与构建.md)。平台实测范围单列在 [测试文档](docs/testing/README.md)。

## 快速开始

克隆仓库、构建并启动四轮小车示例：

```bash
git clone https://github.com/chiway-luo/litesim3d_ros2.git
cd litesim3d_ros2
bash scripts/build.sh
source "$(python3 scripts/paths.py install)/setup.bash"
export ROS_DOMAIN_ID=160 ROS_LOCALHOST_ONLY=1
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py
```

完整构建成功应显示 **9 packages finished**。启动后独立窗口显示四轮小车和场景，仅支持鼠标左键旋转、中键或右键平移、滚轮缩放；底部显示实际 RTF、目标 RTF、FPS 和预算状态。没有图形桌面时，把最后一条命令改为：

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py gui:=false
```

有可用图形后端时，开启普通 RGB 相机（默认 640×480 / 15 Hz，通过 ROS 2 话题输出彩色数据，不弹出摄像头窗口）：

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py cameras:=true
```

需要彩色与深度数据时，在 runtime 中显式设置相机 `type: rgbd`；`type: depth` 仅输出深度。详见 [相机配置](docs/usage/SDF与独立窗口.md#车载-rgb-与深度相机)。

在另一个 Bash 终端激活同一 ROS 环境，进入刚才 clone 的 `litesim3d_ros2` 目录，再加载安装环境并发送速度：

```bash
source /opt/ros/humble/setup.bash   # macOS 改为激活自己的 RoboStack Humble 环境
source "$(python3 scripts/paths.py install)/setup.bash"
export ROS_DOMAIN_ID=160 ROS_LOCALHOST_ONLY=1
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.2}}'
```

按 Ctrl+C 停止发送后，小车在 0.5 秒仿真时间的命令超时后按减速度停车；在 launch 终端按 Ctrl+C 退出仿真。每个 domain 同时运行一个仿真世界。

检查首帧点云，或切换雷达预设：

```bash
ros2 topic echo /points_raw --once --field header --qos-reliability best_effort
# 停止原 launch 后再运行：
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py lidar_profile:=mid360
```

小车默认进入带工位、会议室、接待区、走廊和机房的**办公区**。原小型场景可通过 `world:=warehouse.sdf` 选择。

退出小车 launch 后，可启动**无人机三维示例**：

```bash
ros2 launch drone_3d_sim drone_3d.launch.py
```

无人机初始悬停在起降区上方 1.2 米；24 × 24 米场地含高低门框、立柱、平台、货架和悬空障碍。`linear.x/y/z` 控制机体三轴平移，`angular.z` 控制偏航，机体保持水平。超时后减速悬停，碰撞停止全部速度；相机与碰撞预算默认关闭。控制指令、传感器话题、运动限制及 RGB/深度配置见 [无人机示例](examples/drone_3d/README.md)。

更多控制、参数、暂停/单步、录包和模型修改步骤见 [四轮小车示例](examples/wheel_car_4w/README.md)。

模型包位于 `examples/wheel_car_4w/litesim_models`，通过安装后的包 share 路径定位。碰撞预算默认关闭，碰撞检测保持启用；增加 `collision_budget_enabled:=true` 可启用复杂度检查。SDF 格式范围、材质兼容表和自定义场景用法见 [SDF 与独立窗口](docs/usage/SDF与独立窗口.md)。

| 办公区 / 四轮小车 | 三维飞行场地 / 无人机 |
| --- | --- |
| ![Office simulation](docs/images/office.png) | ![Drone simulation](docs/images/drone-arena.png) |

## 文档

| 入口 | 内容 |
| --- | --- |
| [使用文档](docs/usage/README.md) | 环境安装、运行案例、参数、模型接入、算法接入、多机器人与重置 |
| [测试文档](docs/testing/README.md) | 回归命令、验收规范、历史结果、证据位置与未验证项 |
| [开发设计](docs/design/开发方案.md) | 模块职责、需求与阶段规划；历史规划以当前使用手册为准 |

## 源码结构

```text
litesim3d_ros2/
├── litesim3d_core/          # 不依赖 ROS 运行时的 C++ 内核、资源与测试
├── litesim3d_assets/        # SDF 导入、资源校验与内部适配
├── litesim3d_gui/           # Qt/Ogre2 三维窗口、RGB/深度传感器与可选图像预览
├── litesim3d_interfaces/    # ROS 消息和服务定义
├── litesim3d_ros2/          # ROS 节点、launch、RViz、会话管理与集成测试
├── examples/wheel_car_4w/  # litesim_models、description、sim 三个示例包
├── examples/drone_3d/      # 无人机配置、launch 和集成测试
├── standalone/             # 不启用 ROS 桥接的 CMake 入口
├── scripts/                # 构建、测试、检查与报告工具
├── docs/{usage,testing,design}/
└── .github/workflows/      # Ubuntu Humble CI
```

三类核心包分开维护计算、ROS 类型和 ROS 运行时；示例独立成包，便于替换自己的机器人。构建产物和原始测试日志保存在仓库外，不需要下载历史日志才能运行。详见 [目录与迁移说明](docs/usage/目录与迁移.md)。
