<h1 align="center">LiteSim 3D</h1>

<p align="center">
  <img src="litesim3d_gui/resources/icons/litesim3d.svg" width="128" height="128" alt="LiteSim 3D 图标">
</p>

<p align="center">面向 Linux 与 ROS 2 的轻量机器人三维仿真器</p>

<p align="center">
  <a href="README.md">English</a> · <strong>简体中文</strong>
</p>

<p align="center">
  <a href="#主要功能">主要功能</a> ·
  <a href="#运行要求">运行要求</a> ·
  <a href="#快速开始">快速开始</a> ·
  <a href="#文档">文档</a> ·
  <a href="#开发与贡献">开发与贡献</a>
</p>

---

LiteSim 3D 是基于 C++17 的三维运动学仿真器，适合 ROS 2 机器人模型接入、传感器处理和算法联调。运动、几何碰撞、雷达和 IMU 在 CPU 上计算；可选三维查看器及 RGB/深度相机使用 Ogre2 渲染。

| 办公区四轮小车 | 三维飞行场地无人机 |
| --- | --- |
| ![办公区仿真](docs/images/office.png) | ![无人机仿真](docs/images/drone-arena.png) |

## 主要功能

- **机器人模型**：双轮/四轮差速、具有转向关节的阿克曼、三轮/四轮普通全向底盘，以及速度控制无人机。
- **场景**：组合 URDF/Xacro 模型、限定范围的 SDF 资源、固定障碍物、三维几何碰撞和地形约束。
- **传感器**：VLP-16、Mid-360、Mid-40 扫描预设，滚动扫描、测量噪声、IMU、编码器里程计和可选 RGB/深度图像。
- **ROS 2 接入**：传感器话题、TF、仿真时钟、暂停/单步，以及 1–4 台同构机器人的共享世界仿真。
- **插件扩展**：通过已安装插件替换底盘、里程计和传感器实现，无需重编核心。

仿真采用运动学和几何约束，不模拟接触力、悬架、轮胎动力学、电机推力或飞控动力学。当前开发版本及实测范围见[版本与验证状态](docs/版本与验证状态.md)。

<a id="开发环境"></a>
## 运行要求

| 组件 | 要求 |
| --- | --- |
| 操作系统与 ROS | Ubuntu 22.04 LTS + ROS 2 Humble |
| 构建工具 | C++17、CMake ≥ 3.20、colcon、Python ≥ 3.10、Bash |
| 可选三维窗口与相机 | Qt 5、Gazebo Rendering 6 / Ogre2，以及可用的图形环境 |

先按[安装依赖](docs/usage/安装与构建.md#安装依赖)准备 ROS 2 Humble、构建工具并初始化 rosdep。构建选项和安装路径见[安装与构建](docs/usage/安装与构建.md)。

## 快速开始

**想从自己的 description 搭起完整环境？** 从[双 MID-360 从零搭建教程](docs/tutorials/双MID360从零搭建.md)开始：从克隆开始，使用独立 description 与 bringup 包，逐步配置 world、URDF 传感器和 bringup.launch.py，再构建启动。

### 1. 获取源码并构建

准备好上述依赖后，在 Bash 终端执行：

```bash
source /opt/ros/humble/setup.bash
git clone --branch humble https://github.com/chiway-luo/litesim3d_ros2.git
cd litesim3d_ros2
rosdep install --from-paths . --ignore-src --rosdistro humble -y
bash scripts/build.sh
source "$(python3 scripts/paths.py install)/setup.bash"
```

确认所有包构建完成且没有失败，再继续。后续命令均从刚克隆的源码仓库根目录执行。

### 2. 启动办公区小车

在同一终端 A 执行：

```bash
export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py gui:=false rviz:=false cameras:=false lidar_profile:=vlp16 lidar_interface:=native
```

首次运行使用办公场景、VLP-16 和原生 PointCloud2，关闭窗口与相机。保持终端 A 运行；每个 ROS domain 同时运行一个仿真世界。

### 3. 验证数据与运动

新开终端 B，进入同一源码仓库根目录并执行：

```bash
source /opt/ros/humble/setup.bash
source "$(python3 scripts/paths.py install)/setup.bash"
export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1
python3 docs/examples/ros_client.py --drive --linear 0.1 --angular 0.0 --duration 8 --require-imu --require-motion
```

JSON 输出中的 `success: true` 和 `checks.motion: true` 表示传感器连通、已发送速度指令，并观测到至少 1 cm 的里程计位移。程序退出前发送零速度；在终端 A 按 Ctrl+C 退出仿真。

需要查看三维窗口时，停止 launch 后将第 2 步的 `gui:=false` 改为 `gui:=true` 再启动。逐项数据检查、手动控制和常见失败处理见[完整启动教程](docs/tutorials/启动与控制.md)。

测量误差默认均为 0。验证算法抗扰性时，可通过[传感器与轮式里程计误差配置](docs/usage/传感器配置.md)注入偏移、比例误差和随机噪声。

## 文档

按当前任务从[文档总入口](docs/README.md)选择；详细操作与接口说明在各自页面维护。

| 入口 | 内容 |
| --- | --- |
| [入门教程](docs/tutorials/README.md) | 双 MID-360 从零搭建、阿克曼/全向轮 ros2_control、启动与控制、添加 IMU、ROS 2 消费节点 |
| [使用手册](docs/usage/README.md) | 日常运行、模型与传感器配置、场景、多机、重置和排障 |
| [接口参考](docs/api/README.md) | 话题、消息/服务、QoS、参数、数据布局及 C++/插件契约 |
| [逐插件参数](docs/api/plugins/README.md) | 标准插件的类型、必填项、默认值、单位与约束 |
| [车型与扩展示例](examples/README.md) | 完整小车、无人机工程与外部插件 |
| [运行原理](docs/concepts/运行与坐标.md) | 配置流程、TF 发布责任、仿真时间与重置行为 |
| [版本与验证状态](docs/版本与验证状态.md) | 支持范围、实际验证记录和接口兼容约定 |

## 开发与贡献

开发入口包括[目录与包职责](docs/usage/目录与迁移.md)、[插件开发](docs/development/插件开发.md)、[测试指南](docs/testing/README.md)和[文档维护](docs/development/文档维护.md)。项目历史见[开发日志](CHANGELOG.rst)，版本号以 [VERSION](VERSION) 为准。

反馈问题时，请提供 Ubuntu/ROS 版本、仓库版本、启动命令、复现步骤和相关日志。提交改动时，请说明行为变化及实际执行的验证。维护者邮箱：[chiwayfpv@gmail.com](mailto:chiwayfpv@gmail.com)。

## 许可证

[Apache-2.0](LICENSE)。随附第三方材料保留各自的许可声明。

全部现有车型默认接入 ros2_control。四轮车、无人机及第三方算法插件的完整配置见 [统一接入教程](docs/tutorials/全部模型与第三方算法ros2_control.md)。
