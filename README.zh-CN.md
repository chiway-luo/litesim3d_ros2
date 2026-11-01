<h1 align="center">LiteSim 3D</h1>

<p align="center">
  <img src="litesim3d_gui/resources/icons/litesim3d.svg" width="128" height="128" alt="LiteSim 3D 图标">
</p>

<p align="center">面向 Linux 与 ROS 2 的轻量机器人三维仿真器</p>

<p align="center">
  <a href="#测试覆盖率"><img src="https://img.shields.io/badge/unit%20lines-88.21%25-orange?style=flat-square&amp;logo=github&amp;logoColor=white" alt="最近实测单元测试行覆盖率：88.21%"></a>
  <a href="#测试覆盖率"><img src="https://img.shields.io/badge/unit%20branches-82.58%25-orange?style=flat-square&amp;logo=github&amp;logoColor=white" alt="最近实测单元测试分支覆盖率：82.58%"></a>
  <a href="#测试覆盖率"><img src="https://img.shields.io/badge/unit%20functions-85.15%25-orange?style=flat-square&amp;logo=github&amp;logoColor=white" alt="最近实测单元测试函数覆盖率：85.15%"></a>
</p>
<p align="center"><sub>最近实测快照：2026-11-01 · <a href="#测试覆盖率">统计范围与状态</a></sub></p>

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

本机运行先完成[安装依赖](docs/usage/安装与构建.md#安装依赖)。使用[容器](docs/usage/容器运行.md)时，宿主机只需准备 Git 和 Docker。

## 快速开始

没有 ROS 环境时，可在仓库根目录运行 `docker compose up --build desktop`，通过浏览器使用三维窗口。完整步骤见[容器运行](docs/usage/容器运行.md)。下面是本机安装方式。

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
| 无需本机 ROS 环境，使用浏览器体验 | [容器运行](docs/usage/容器运行.md) |
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

### 测试覆盖率

徽章采用 `codex/cpp-runtime` 分支在 2026-11-01 的最近一次**单元测试**实测记录，环境为 Ubuntu 22.04 / ROS 2 Humble、x86_64，不计入集成测试命中。

| 指标 | 已覆盖 / 总数 | 覆盖率 |
| --- | --- | --- |
| 行 | 8,789 / 9,964 | 88.21% |
| 分支 | 5,295 / 6,412 | 82.58% |
| 函数 | 734 / 862 | 85.15% |

覆盖率门禁**尚未通过**：三项指标都要求严格大于 90%，另有一项源码映射待处理。这份快照早于可靠指令适配器等后续修复，当前源码尚未重跑覆盖率。详见[结果摘要](docs/coverage-summary.json)和[统计规则与复现步骤](scripts/quality/README.md)。

## 许可证

[Apache-2.0](LICENSE)。随附第三方材料保留各自的许可声明。
