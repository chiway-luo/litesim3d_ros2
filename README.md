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
  <a href="#development-environment">Development environment</a> ·
  <a href="#quick-start">Quick start</a> ·
  <a href="#documentation">Documentation</a> ·
  <a href="#development">Development</a>
</p>

---

LiteSim 3D is a C++17 kinematic simulator for developing and testing robotics algorithms with ROS 2. It computes motion, geometric collisions, LiDAR, and IMU data on the CPU, and publishes sensor data, odometry, TF, and simulation time through ROS 2. An optional 3D viewer and RGB/depth cameras use Ogre2 rendering.

The documented development and runtime environment is **Ubuntu 22.04 + ROS 2 Humble**.

| Four-wheel robot in an office | Drone in a 3D flight course |
| --- | --- |
| ![Office simulation](docs/images/office.png) | ![Drone simulation](docs/images/drone-arena.png) |

## Features

- **Robot models:** two/four-wheel differential, articulated Ackermann, and three/four ordinary omni-wheel chassis plugins, plus compatible velocity-controlled multicopters; URDF/Xacro and supported SDF models.
- **Scenes and collisions:** fixed collidable obstacles, continuous slopes, and textured SDF scenes. Wheeled robots can optionally keep their encoders and odometry running while the chassis is blocked.
- **Sensors:** VLP-16, Mid-360, and Mid-40 scan presets; rolling scans, measurement noise, IMU, odometry, and optional RGB/depth cameras.
- **ROS 2 integration:** point clouds, images, TF, simulation time, pause/step control, and single-robot session resets.
- **Examples:** a four-wheel office robot, Ackermann and three/four omni-wheel robots, a drone flight course, and shared-world simulation for one to four identical robots.

The simulator uses kinematics and geometric constraints. It does not model contact forces, suspension, tire dynamics, motor thrust, or flight-controller dynamics. Sensor presets are simulation models; their scope is described in the [LiDAR guide](examples/wheel_car_4w/雷达配置说明.md).

Chassis, encoder odometry, and five sensor implementations are separate plugin packages. Install an external implementation and select it in the composed model without rebuilding the core. See [model composition and migration](docs/usage/模型组合与插件迁移.md) and [plugin development](docs/development/插件开发.md).

## Development environment

| Component | Requirement |
| --- | --- |
| Operating system | Linux: Ubuntu 22.04 LTS |
| ROS | ROS 2 Humble |
| Compiler and build tools | C++17, CMake ≥ 3.20, colcon |
| Python and shell | Python ≥ 3.10, Bash |
| Viewer and cameras | Qt 5, Gazebo Rendering 6 / Ogre2, and a working graphics environment |

Install ROS 2 Humble using the [official Ubuntu installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), then install the build tools and initialize rosdep. After cloning, rosdep installs the dependencies declared in each package:

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y git build-essential cmake pkg-config ninja-build \
  python3-colcon-common-extensions python3-rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update --rosdistro humble
```

See [Installation and build](docs/usage/安装与构建.md) for dependency details, standard colcon workspaces, and builds without the viewer. Other Linux distributions and ROS releases have not been validated. The [CI workflow](.github/workflows/m1.yml) targets Ubuntu 22.04 and Humble; results and validation limits are recorded in the [testing documentation](docs/testing/README.md).

## Quick start

### 1. Clone and build

Run these commands in a Bash terminal after installing the dependencies:

```bash
source /opt/ros/humble/setup.bash
git clone https://github.com/chiway-luo/litesim3d_ros2.git
cd litesim3d_ros2
rosdep install --from-paths . --ignore-src --rosdistro humble -y
bash scripts/build.sh
source "$(python3 scripts/paths.py install)/setup.bash"
```

A complete default build reports **29 packages finished**. Build artifacts are stored outside the source repository; `scripts/paths.py install` locates the generated install environment.

### 2. Launch the simulation

In the same terminal, start the four-wheel robot in the furnished office scene:

```bash
export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py
```

Drag with the left mouse button to orbit, the middle or right button to pan, and use the wheel to zoom. The status bar shows the actual and target real-time factor (RTF), viewer FPS, and collision-budget status.

For a terminal-only session, use this launch command instead:

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py gui:=false cameras:=false
```

### 3. Control the robot

Open another Bash terminal and enter the cloned `litesim3d_ros2` directory. Load the environment and publish velocity commands:

```bash
source /opt/ros/humble/setup.bash
source "$(python3 scripts/paths.py install)/setup.bash"
export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.2}}'
```

Press Ctrl+C to stop publishing. After the command timeout of 0.5 simulated seconds, the robot decelerates to a stop. Press Ctrl+C in the launch terminal to exit the simulation. All terminals must use the same `ROS_DOMAIN_ID`; run one simulation world per domain. `ROS_LOCALHOST_ONLY=1` keeps ROS communication on the local machine.

Inspect a point-cloud header:

```bash
ros2 topic echo /points_raw --once --field header --qos-reliability best_effort
```

## Examples and configuration

Stop the running simulation before starting another example in the same ROS domain.

| Example | Command |
| --- | --- |
| Four-wheel robot in the office | `ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py` |
| Four-wheel robot with Mid-360 | `ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py lidar_profile:=mid360` |
| Smaller warehouse scene | `ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py world:=warehouse.sdf` |
| Drone flight course | `ros2 launch drone_3d_sim drone_3d.launch.py` |

The drone starts hovering 1.2 m above its pad. `linear.x/y/z` commands body-frame translation and `angular.z` controls yaw; roll and pitch remain level. See the [four-wheel guide](examples/wheel_car_4w/README.md) and [drone guide](examples/drone_3d/README.md) for controls, topics, and model settings.

Cameras are disabled by default. With a working Ogre2 graphics backend, append `cameras:=true` to enable RGB output at **640 × 480, 15 Hz**. Set `type: rgbd` or `type: depth` in the runtime configuration for depth data. Cameras publish ROS topics without opening a separate preview window; they still require rendering when `gui:=false`. See [Camera configuration](docs/usage/SDF与独立窗口.md#车载-rgb-与深度相机).

Car assets are installed from [`wheel_car_4w_models`](examples/wheel_car_4w/wheel_car_4w_models/README.md), and drone assets from [`drone_3d_models`](examples/drone_3d/drone_3d_models/README.md). Each package contains its own models, worlds, and textures. Collision detection is enabled by default; optional complexity limits can be enabled with `collision_budget_enabled:=true`. See [SDF scenes and the viewer](docs/usage/SDF与独立窗口.md) for supported assets, materials, and custom scenes.

## Documentation

The [中文 README](README.zh-CN.md) covers the same setup and quick-start steps. Detailed guides are currently in Chinese.

| Guide | Contents |
| --- | --- |
| [Installation and build](docs/usage/安装与构建.md) | Linux dependencies, build options, and install paths |
| [User guide](docs/usage/README.md) | Setup, examples, parameters, models, algorithm integration, multiple robots, and resets |
| [Testing](docs/testing/README.md) | Regression commands, acceptance criteria, recorded results, evidence, and unverified areas |
| [Design](docs/design/开发方案.md) | Module responsibilities, requirements, and development phases; current usage guides take precedence over historical plans |

## Repository layout

```text
litesim3d_ros2/
├── VERSION                 # Project version (major.minor.patch)
├── litesim3d_core/          # C++ core, resources, and tests; pluginlib loading without DDS nodes
├── litesim3d_bringup/       # Shared launch pipeline; no dependency on example packages
├── litesim3d_test_support/  # Installed helpers for package-level DDS tests
├── litesim3d_assets/        # SDF import, resource validation, and internal adaptation
├── litesim3d_gui/           # Qt/Ogre2 viewer, RGB/depth sensors, and optional image previews
├── litesim3d_interfaces/    # ROS message and service definitions
├── litesim3d_ros2/          # ROS nodes, launch, RViz, sessions, and integration tests
├── examples/wheel_car_4w/  # wheel_car_4w_models, description, and sim example packages
├── examples/drone_3d/      # drone_3d_models and drone_3d_sim packages
├── standalone/             # CMake entry point without the ROS bridge
├── scripts/                # Build, test, validation, and reporting tools
├── docs/{usage,testing,design}/
└── .github/workflows/      # Ubuntu Humble CI
```

Build artifacts and raw test logs are stored outside the repository. See [Directory layout](docs/usage/目录与迁移.md) for package responsibilities and artifact locations.

## Development

The repository version is defined in [VERSION](VERSION), and all ROS package manifests must match it. Each package builds from its own `package.xml`; the core embeds that package version in the runtime manifest, so an independently released package does not require the repository root. When changing the version, update every `package.xml` to match and run the version check below. `scripts/build.sh` also runs this check before building.

From the repository root, after building and loading the ROS environment:

```bash
python3 scripts/checks/check_versions.py
python3 scripts/checks/check_repository_docs.py
bash scripts/test_m3.sh
```

The default regression suite does not cover desktop rendering, formal performance measurements, or long-duration stability. See [Testing](docs/testing/README.md) for those checks and their requirements.

When reporting an issue, include the Ubuntu and ROS versions, repository revision, launch command, reproduction steps, and relevant logs. For code changes, describe the behavior change and the checks actually performed.

The [development changelog](CHANGELOG.rst) records the project history and links to each package's `CHANGELOG.rst`. Changes for the planned 0.4.0 release remain under `Forthcoming` until release preparation is finalized.

LiteSim 3D uses the [Apache-2.0 license](LICENSE). Bundled third-party materials retain their own license notices. Maintainer contact: [chiwayfpv@gmail.com](mailto:chiwayfpv@gmail.com).
