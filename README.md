<h1 align="center">LiteSim 3D</h1>

<p align="center">
  <img src="litesim3d_gui/resources/icons/litesim3d.svg" width="128" height="128" alt="LiteSim 3D logo">
</p>

<p align="center">Lightweight 3D robotics simulation for ROS 2 on Linux</p>

<p align="center">
  <a href="#test-coverage"><img src="https://img.shields.io/badge/unit%20lines-97.45%25-brightgreen?style=flat-square&amp;logo=github&amp;logoColor=white" alt="Last measured unit line coverage: 97.45%"></a>
  <a href="#test-coverage"><img src="https://img.shields.io/badge/unit%20branches-95.56%25-brightgreen?style=flat-square&amp;logo=github&amp;logoColor=white" alt="Last measured unit branch coverage: 95.56%"></a>
  <a href="#test-coverage"><img src="https://img.shields.io/badge/unit%20functions-96.53%25-brightgreen?style=flat-square&amp;logo=github&amp;logoColor=white" alt="Last measured unit function coverage: 96.53%"></a>
</p>
<p align="center"><sub>Last measured snapshot: 2026-11-02 · <a href="#test-coverage">Scope and status</a></sub></p>

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

When reporting an issue, include Ubuntu/ROS versions, repository revision, launch command, reproduction steps, and relevant logs.

### Test coverage

The badges show the reviewed **unit-only** measurement from 2026-11-02 on Ubuntu 22.04 / ROS 2 Humble, x86_64. Run `20261102T155839Z-89961` measured source snapshot `21201f0f18e084d00ed428ee5bf46e1bfe30f7642e79417e58d095dc77172afc`, published in commit [`7f683956`](https://github.com/chiway-luo/litesim3d_ros2/commit/7f6839567f5fab84958337343fe718081e862a64). Integration-test hits are excluded.

| Metric | Covered / total | Coverage |
| --- | --- | --- |
| Lines | 9,723 / 9,977 | 97.45% |
| Branches | 6,129 / 6,414 | 95.56% |
| Functions | 834 / 864 | 96.53% |

The unit-only result is **UNIT_ONLY_PASS**: lines and functions are strictly greater than 90%, and branches are strictly greater than 95%. All aggregate and existing critical-core gates passed. The complete 102-source production inventory was retained, with no coverage denominator reductions.

**889 business unit tests and 146 tooling tests passed** across 33 runner stages, with no failures, errors, skipped or not-run cases in the selected unit suite. Integration, graphical integration, sanitizer controls, release packaging and the full quality matrix were not run in this measurement. The separate full CI run for `7f683956` reported release-packaging and TSAN failures. These badges do not certify a passing full CI run; check [GitHub Actions](https://github.com/chiway-luo/litesim3d_ros2/actions) for current status.

See the [recorded summary](docs/coverage-summary.json) and [coverage rules and reproduction steps](scripts/quality/README.md).

## License

[Apache-2.0](LICENSE). Bundled third-party materials retain their own license notices.
