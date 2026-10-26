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

LiteSim 3D 是面向 ROS 2 模型接入、传感器处理和算法联调的 C++17 三维运动学仿真器。运动、碰撞、雷达和 IMU 在 CPU 上计算；三维窗口及 RGB/深度相机使用 Ogre2 渲染。

| 办公区四轮小车 | 三维飞行场地无人机 |
| --- | --- |
| ![办公区仿真](docs/images/office.png) | ![无人机仿真](docs/images/drone-arena.png) |

## 主要功能

- 差速、阿克曼、三轮/四轮普通全向底盘，以及速度控制无人机；示例默认使用 ros2_control。
- URDF/Xacro 机器人、SDF 场景、固定障碍物、几何碰撞和地形约束。
- VLP-16、MID-360、MID-40 扫描预设，IMU、编码器里程计、RGB/深度相机与可配置测量误差。
- ROS 2 话题、TF、仿真时钟、暂停/单步，以及 1–4 台同构机器人的共享世界。
- 独立安装的底盘、里程计和传感器插件。

仿真采用运动学和几何约束，不模拟接触力、悬架、轮胎动力学、电机推力或飞控动力学。

<a id="开发环境"></a>
## 运行要求

Ubuntu 22.04 LTS、ROS 2 Humble、Bash、C++17、CMake ≥ 3.20、Python ≥ 3.10 和 colcon。三维窗口及相机另需 Qt 5、Gazebo Rendering 6 / Ogre2 和可用图形环境。

先完成[安装依赖](docs/usage/安装与构建.md#安装依赖)，再执行下面的步骤。

## 快速开始

### 1. 获取源码并构建

```bash
source /opt/ros/humble/setup.bash
git clone --branch humble https://github.com/chiway-luo/litesim3d_ros2.git
cd litesim3d_ros2
rosdep install --from-paths . --ignore-src --rosdistro humble -y
bash scripts/build.sh
source "$(python3 scripts/paths.py install)/setup.bash"
```

所有包构建完成且没有失败后，在同一源码目录继续。

### 2. 启动小车

终端 A：

```bash
export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py gui:=false rviz:=false cameras:=false lidar_profile:=vlp16 lidar_interface:=native
```

保持终端 A 运行。本步骤关闭窗口和相机，使用办公场景、VLP-16 和原生 PointCloud2。同一 domain 只运行一个仿真世界。

### 3. 发送速度并检查反馈

终端 B 进入同一源码目录，执行：

```bash
source /opt/ros/humble/setup.bash
source "$(python3 scripts/paths.py install)/setup.bash"
export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1
python3 docs/examples/ros_client.py --drive --linear 0.1 --angular 0.0 --duration 8 --require-imu --require-motion
```

JSON 中 `success: true`、`checks.motion: true` 表示数据连通且里程计位移至少 1 cm。程序退出前发送零速度；在终端 A 按 Ctrl+C 结束仿真。

需要三维窗口时，将第 2 步改为 `gui:=true` 后重新启动；需要 RViz 再加 `rviz:=true`。更多检查见[启动与控制](docs/tutorials/启动与控制.md)。

## 文档

| 要做什么 | 入口 |
| --- | --- |
| 安装依赖、选择构建方式 | [安装与构建](docs/usage/安装与构建.md) |
| 用自己的 description 搭建仿真 | [双 MID-360 从零搭建](docs/tutorials/双MID360从零搭建.md) |
| 接入 ros2_control 或第三方算法 | [统一控制教程](docs/tutorials/全部模型与第三方算法ros2_control.md) |
| 修改模型、场景、传感器和多机配置 | [使用手册](docs/usage/README.md) |
| 查询话题、消息、参数和插件接口 | [接口参考](docs/api/README.md)、[插件参数](docs/api/plugins/README.md) |
| 选择车型、理解 TF 和配置流 | [示例](examples/README.md)、[运行与坐标](docs/concepts/运行与坐标.md) |

全部页面见[文档目录](docs/README.md)。

## 开发与贡献

版本号见 [VERSION](VERSION)。源码、文档与安装包应使用同一版本。

反馈问题请附 Ubuntu/ROS 版本、仓库版本、启动命令、复现步骤和日志。维护者：[chiwayfpv@gmail.com](mailto:chiwayfpv@gmail.com)。

## 许可证

[Apache-2.0](LICENSE)。随附第三方材料保留各自的许可声明。
