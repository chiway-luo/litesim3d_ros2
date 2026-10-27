<h1 align="center">LiteSim 3D</h1>

<p align="center">
  <img src="litesim3d_gui/resources/icons/litesim3d.svg" width="128" height="128" alt="LiteSim 3D logo">
</p>

<p align="center">Lightweight 3D robotics simulation for ROS 2 on Linux</p>

<p align="center">
  <strong>English</strong> · <a href="README.zh-CN.md">简体中文</a>
</p>

<p align="center">
  <a href="#features">Features</a> ·
  <a href="#requirements">Requirements</a> ·
  <a href="#quick-start">Quick start</a> ·
  <a href="#documentation">Documentation</a> ·
  <a href="#development">Contributing</a>
</p>

---

LiteSim 3D is a C++17 kinematic simulator for ROS 2 model integration, sensor processing, and algorithm development. Motion, collisions, LiDAR, and IMU run on the CPU; the optional viewer and RGB/depth cameras use Ogre2.

| Four-wheel robot in an office | Drone in a 3D flight course |
| --- | --- |
| ![Office simulation](docs/images/office.png) | ![Drone simulation](docs/images/drone-arena.png) |

## Features

- Differential, Ackermann, three/four ordinary omni-wheel chassis, and velocity-controlled multicopters; examples use ros2_control by default.
- URDF/Xacro robots, SDF scenes, fixed obstacles, geometric collisions, and terrain constraints.
- VLP-16, MID-360, and MID-40 scan presets, IMU, encoder odometry, RGB/depth cameras, and configurable measurement errors.
- ROS 2 topics, TF, simulation time, pause/step control, and a shared world for one to four identical robots.
- Independently installed chassis, odometry, and sensor plugins.

The simulator uses kinematics and geometric constraints. It does not model contact forces, suspension, tire dynamics, motor thrust, or flight-controller dynamics.

<a id="development-environment"></a>
## Requirements

Ubuntu 22.04 LTS, ROS 2 Humble, Bash, C++17, CMake ≥ 3.20, Python ≥ 3.10, and colcon. The viewer and cameras also require Qt 5, Gazebo Rendering 6 / Ogre2, and a working graphics environment.

For a native installation, complete [dependency installation](docs/usage/安装与构建.md#安装依赖). The [container option](docs/usage/容器运行.md) only requires Git and Docker on the host.

## Quick start

Without a local ROS installation, run `docker compose up --build desktop` from the repository root and open [the browser desktop](http://localhost:6080/vnc.html?autoconnect=1&resize=scale). See the [container guide](docs/usage/容器运行.md) for setup and headless use. The steps below use a native installation.

### 1. Clone and build

```bash
source /opt/ros/humble/setup.bash
git clone --branch humble https://github.com/chiway-luo/litesim3d_ros2.git
cd litesim3d_ros2
rosdep install --from-paths . --ignore-src --rosdistro humble -y
bash scripts/build.sh
source "$(python3 scripts/paths.py install)/setup.bash"
```

Continue after all packages finish without failures. Run the remaining commands from this repository directory.

### 2. Start the robot

Terminal A:

```bash
export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py gui:=false rviz:=false cameras:=false lidar_profile:=vlp16 lidar_interface:=native
```

Keep terminal A running. This starts the office scene with VLP-16 and native PointCloud2, without windows or cameras. Run one simulation world per ROS domain.

### 3. Send velocity commands and check feedback

In terminal B, enter the same repository directory:

```bash
source /opt/ros/humble/setup.bash
source "$(python3 scripts/paths.py install)/setup.bash"
export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1
python3 docs/examples/ros_client.py --drive --linear 0.1 --angular 0.0 --duration 8 --require-imu --require-motion
```

JSON values `success: true` and `checks.motion: true` mean the client received data and observed at least 1 cm of odometry displacement. It sends zero velocity before exiting. Press Ctrl+C in terminal A to stop the simulation.

To open the viewer, restart step 2 with `gui:=true`; add `rviz:=true` for RViz. See the [first-run tutorial](docs/tutorials/启动与控制.md) for individual checks.

## Documentation

Detailed guides are in Chinese. The [Chinese README](README.zh-CN.md) follows the same quick-start path.

| Task | Guide |
| --- | --- |
| Try the simulator in a browser without installing ROS | [Container guide](docs/usage/容器运行.md) |
| Install dependencies and choose a build method | [Installation and build](docs/usage/安装与构建.md) |
| Set up a robot from a description package | [Dual MID-360 tutorial](docs/tutorials/双MID360从零搭建.md) |
| Connect ros2_control or a third-party algorithm | [Control integration](docs/tutorials/全部模型与第三方算法ros2_control.md) |
| Configure models, scenes, sensors, or multiple robots | [User manual](docs/usage/README.md) |
| Look up topics, messages, parameters, or plugin interfaces | [API reference](docs/api/README.md), [plugin parameters](docs/api/plugins/README.md) |
| Choose an example or understand TF and configuration | [Examples](examples/README.md), [runtime concepts](docs/concepts/运行与坐标.md) |

See the [documentation index](docs/README.md) for all pages.

## Development

The version is recorded in [VERSION](VERSION). Use source, documentation, and installed packages from the same version.

When reporting an issue, include Ubuntu/ROS versions, repository revision, launch command, reproduction steps, and relevant logs. Maintainer: [chiwayfpv@gmail.com](mailto:chiwayfpv@gmail.com).

## License

[Apache-2.0](LICENSE). Bundled third-party materials retain their own license notices.
