<h1 align="center">LiteSim 3D</h1>

<p align="center">
  <img src="litesim3d_gui/resources/icons/litesim3d.svg" width="128" height="128" alt="LiteSim 3D logo">
</p>

<p align="center">Lightweight 3D robotics simulation for ROS 2 on Linux</p>

<p align="center">
  <a href="#test-coverage"><img src="https://img.shields.io/badge/unit%20lines-88.21%25-orange?style=flat-square&amp;logo=github&amp;logoColor=white" alt="Last measured unit line coverage: 88.21%"></a>
  <a href="#test-coverage"><img src="https://img.shields.io/badge/unit%20branches-82.58%25-orange?style=flat-square&amp;logo=github&amp;logoColor=white" alt="Last measured unit branch coverage: 82.58%"></a>
  <a href="#test-coverage"><img src="https://img.shields.io/badge/unit%20functions-85.15%25-orange?style=flat-square&amp;logo=github&amp;logoColor=white" alt="Last measured unit function coverage: 85.15%"></a>
</p>
<p align="center"><sub>Last measured snapshot: 2026-11-01 · <a href="#test-coverage">Scope and status</a></sub></p>

<p align="center">
  <strong>English</strong> · <a href="README.zh-CN.md">简体中文</a>
</p>

<p align="center">
  <a href="#why-litesim-3d">Why LiteSim 3D</a> ·
  <a href="#features">Features</a> ·
  <a href="#compatibility">Compatibility</a> ·
  <a href="docs/tutorials/快速开始.md">Quick start</a> ·
  <a href="#documentation">Documentation</a> ·
  <a href="#development">Contributing</a>
</p>

---

LiteSim 3D is a C++17 kinematic simulator for ROS 2 model integration, sensor processing, and algorithm development. Motion, collisions, LiDAR, and IMU run on the CPU; the optional viewer and RGB/depth cameras use Ogre2.

| Four-wheel robot in an office | Drone in a 3D flight course |
| --- | --- |
| ![Office simulation](docs/images/office.png) | ![Drone simulation](docs/images/drone-arena.png) |

## Why LiteSim 3D

We built LiteSim 3D to make early robotics algorithm validation easier and faster. The aim is to reduce setup effort and the simulator's own resource use, leaving more computing resources for the algorithms being tested. We focus on making model integration, sensor processing, and control workflows easy to test repeatedly.

## Features

- Differential, Ackermann, three/four ordinary omni-wheel chassis, and velocity-controlled multicopters; examples use ros2_control by default.
- URDF/Xacro robots, SDF scenes, fixed obstacles, geometric collisions, and terrain constraints.
- VLP-16, MID-360, and MID-40 scan presets, IMU, encoder odometry, RGB/depth cameras, and configurable measurement errors.
- ROS 2 topics, TF, simulation time, pause/step control, and a shared world for one to four identical robots.
- Independently installed chassis, odometry, and sensor plugins.

The simulator uses kinematics and geometric constraints. It does not model contact forces, suspension, tire dynamics, motor thrust, or flight-controller dynamics.

<a id="development-environment"></a>
<a id="requirements"></a>
## Compatibility

**ROS 2 Humble is the only currently supported ROS 2 distribution.** The recommended platform is **Ubuntu 22.04 LTS**.

See [installation and build](docs/usage/安装与构建.md) for dependencies and build options. The [Docker guide](docs/usage/容器运行.md) covers hosts without a local ROS installation.

## Documentation

The quick start and detailed guides are in Chinese. A [Chinese README](README.zh-CN.md) is also available.

| Task | Guide |
| --- | --- |
| Clone, build, and run your first simulation | [Quick start](docs/tutorials/快速开始.md) |
| Try the simulator in a browser without installing ROS | [Container guide](docs/usage/容器运行.md) |
| Install dependencies and choose a build method | [Installation and build](docs/usage/安装与构建.md) |
| Set up a robot from a description package | [Dual MID-360 tutorial](docs/tutorials/双MID360从零搭建.md) |
| Connect ros2_control or a third-party algorithm | [Control integration](docs/tutorials/全部模型与第三方算法ros2_control.md) |
| Configure models, scenes, sensors, or multiple robots | [User manual](docs/usage/README.md) |
| Look up topics, messages, parameters, or plugin interfaces | [API reference](docs/api/README.md), [plugin parameters](docs/api/plugins/README.md) |
| Choose an example or understand TF and configuration | [Examples](examples/README.md), [runtime concepts](docs/concepts/运行与坐标.md) |

See the [documentation index](docs/README.md) for all pages.

## Development

The project is still at an early stage and is not yet mature. Features, stability, and documentation need further work. We welcome developers to try LiteSim 3D, contribute code, tests, or documentation, and report problems or suggestions through [Issues](https://github.com/chiway-luo/litesim3d_ros2/issues). [Pull requests](https://github.com/chiway-luo/litesim3d_ros2/pulls) are welcome. We will investigate and work to resolve reported issues as promptly as possible.

The version is recorded in [VERSION](VERSION). Use source, documentation, and installed packages from the same version.

When reporting an issue, include Ubuntu/ROS versions, repository revision, launch command, reproduction steps, and relevant logs. Maintainer: [chiwayfpv@gmail.com](mailto:chiwayfpv@gmail.com).

### Test coverage

The badges show the last **unit-only** measurement on `codex/cpp-runtime`, recorded on 2026-11-01 with Ubuntu 22.04 / ROS 2 Humble, x86_64. Integration-test hits are excluded.

| Metric | Covered / total | Coverage |
| --- | --- | --- |
| Lines | 8,789 / 9,964 | 88.21% |
| Branches | 5,295 / 6,412 | 82.58% |
| Functions | 734 / 862 | 85.15% |

The coverage gate **has not passed**: each metric must be strictly greater than 90%, and one source mapping remains unresolved. This snapshot predates the latest fixes, including the reliable-command adapter; current-source coverage has not been rerun. See the [recorded summary](docs/coverage-summary.json) and [coverage rules and reproduction steps](scripts/quality/README.md).

## License

[Apache-2.0](LICENSE). Bundled third-party materials retain their own license notices.
