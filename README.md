<h1 align="center">LiteSim 3D</h1>

<p align="center">
  <img src="litesim3d_gui/resources/icons/litesim3d.svg" width="128" height="128" alt="LiteSim 3D logo">
</p>

<p align="center">A lightweight ROS 2 simulator for 3D robotics algorithms</p>

<p align="center">
  <strong>English</strong> · <a href="README.zh-CN.md">简体中文</a>
</p>

<p align="center">
  <a href="#prerequisites">Prerequisites</a> ·
  <a href="#quick-start">Quick start</a> ·
  <a href="#documentation">Documentation</a> ·
  <a href="examples/wheel_car_4w/README.md">Four-wheel example</a> ·
  <a href="examples/drone_3d/README.md">Drone example</a>
</p>

---

LiteSim 3D is a lightweight, C++17 kinematic simulator. It computes motion, collisions, LiDAR, and IMU data on the CPU and publishes point clouds, IMU readings, odometry, TF, and simulation time through ROS 2. The four-wheel example loads its scene from SDF and displays materials, textures, and the actual simulation rate in a standalone 3D viewer. It can also run without a viewer.

Features include URDF support, differential-drive, ideal Ackermann, and velocity-controlled multicopter robots, 3D geometric collisions, continuous slopes, rolling scans, measurement noise, dynamic obstacles, one to four identical robots, and single-robot session resets. Ready-to-run office and complex aerial-course examples, plus VLP-16, Mid-360, and Mid-40 scan presets are included. The simulator uses kinematics and geometric constraints; contact forces, suspension, tire dynamics, motor thrust, and flight-controller dynamics are outside its scope. See the [LiDAR guide](examples/wheel_car_4w/雷达配置说明.md) for the modeling scope of each preset.

## Prerequisites

The commands below use **Ubuntu 22.04, ROS 2 Humble, and Bash**. Install Humble using the [official ROS 2 instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), then install the project dependencies:

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

On macOS, activate a configured RoboStack Humble environment and follow the clone, build, and run steps below. Skip the Ubuntu `apt` commands. See [Installation and build](docs/usage/安装与构建.md) for environment requirements, standard colcon workspaces, and install paths. The [testing documentation](docs/testing/README.md) records which platforms and features have actually been validated.

## Quick start

Clone the repository, build it, and launch the four-wheel example:

```bash
git clone https://github.com/chiway-luo/litesim3d_ros2.git
cd litesim3d_ros2
bash scripts/build.sh
source "$(python3 scripts/paths.py install)/setup.bash"
export ROS_DOMAIN_ID=160 ROS_LOCALHOST_ONLY=1
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py
```

A full successful build reports **9 packages finished**. The standalone viewer shows the robot and scene. Drag with the left mouse button to orbit, the middle or right button to pan, and use the wheel to zoom. The status bar shows the actual and target real-time factor (RTF), viewer FPS, and collision-budget status. Without a graphical desktop, replace the last command with:

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py gui:=false
```

With a working graphics backend, enable the ordinary RGB camera at **640 × 480, 15 Hz**. It publishes ROS 2 topics without opening a separate camera window:

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py cameras:=true
```

For both color and depth data, explicitly set the camera to `type: rgbd` in the runtime configuration. Use `type: depth` for depth only. See [Camera configuration](docs/usage/SDF与独立窗口.md#车载-rgb-与深度相机).

In another Bash terminal, activate the same ROS environment and enter the cloned `litesim3d_ros2` directory. Load the install environment and send velocity commands:

```bash
source /opt/ros/humble/setup.bash   # On macOS, activate your RoboStack Humble environment instead.
source "$(python3 scripts/paths.py install)/setup.bash"
export ROS_DOMAIN_ID=160 ROS_LOCALHOST_ONLY=1
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.2}}'
```

Press Ctrl+C to stop publishing commands. After the command timeout of 0.5 simulated seconds, the robot decelerates to a stop. Press Ctrl+C in the launch terminal to exit the simulation. Run only one simulation world per ROS domain at a time.

Inspect a point-cloud header or switch the LiDAR preset:

```bash
ros2 topic echo /points_raw --once --field header --qos-reliability best_effort
# Stop the previous launch before running this command:
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py lidar_profile:=mid360
```

For controls, parameters, pause/step operations, bag recording, and model changes, see the [four-wheel example](examples/wheel_car_4w/README.md).

The car now starts in a furnished **office** with workstations, meeting rooms, a reception area, corridors, and a server room. The original small scene remains available with `world:=warehouse.sdf`.

For the **drone example**, stop the car launch and run:

```bash
ros2 launch drone_3d_sim drone_3d.launch.py
```

The drone starts hovering 1.2 m above its pad in a 24 × 24 m flight course with elevated gates, pillars, platforms, shelves, and overhead obstacles. `linear.x/y/z` commands body-frame translation and `angular.z` controls yaw; roll and pitch remain level. The velocity watchdog brakes to a hover, and collisions stop all axes. Cameras and the collision budget remain disabled by default. See the [drone guide](examples/drone_3d/README.md) for commands, topics, limits, and RGB/depth camera settings.

Models live in the `examples/wheel_car_4w/litesim_models` resource package and are resolved through its installed package share directory. The collision budget is disabled by default; collision detection remains enabled. Add `collision_budget_enabled:=true` to enforce complexity limits. See [SDF and the standalone viewer](docs/usage/SDF与独立窗口.md) for supported SDF features, materials, and custom scenes.

| Office / four-wheel car | Aerial course / drone |
| --- | --- |
| ![Office simulation](docs/images/office.png) | ![Drone simulation](docs/images/drone-arena.png) |

## Documentation

The detailed guides below are currently in Chinese. The [中文 README](README.zh-CN.md) covers the same setup and quick-start steps as this page.

| Guide | Contents |
| --- | --- |
| [User guide](docs/usage/README.md) | Setup, examples, parameters, models, algorithm integration, multiple robots, and resets |
| [Testing](docs/testing/README.md) | Regression commands, acceptance criteria, recorded results, evidence, and unverified areas |
| [Design](docs/design/开发方案.md) | Module responsibilities, requirements, and development phases; current usage guides take precedence over historical plans |

## Repository layout

```text
litesim3d_ros2/
├── litesim3d_core/          # C++ core, resources, and tests; no ROS runtime dependency
├── litesim3d_assets/        # SDF import, resource validation, and internal adaptation
├── litesim3d_gui/           # Qt/Ogre2 viewer, RGB/depth sensors, and optional image previews
├── litesim3d_interfaces/    # ROS message and service definitions
├── litesim3d_ros2/          # ROS nodes, launch, RViz, sessions, and integration tests
├── examples/wheel_car_4w/  # litesim_models, description, and sim example packages
├── examples/drone_3d/      # Multicopter configuration, launch, and integration tests
├── standalone/             # CMake entry point without the ROS bridge
├── scripts/                # Build, test, validation, and reporting tools
├── docs/{usage,testing,design}/
└── .github/workflows/      # Ubuntu Humble CI
```

The core, interface, and ROS bridge packages separate computation, ROS types, and runtime integration. Example packages make it easier to substitute your own robot. Build artifacts and raw test logs are stored outside the repository; historical logs are not required to run the project. See [Directory layout and migration](docs/usage/目录与迁移.md).
