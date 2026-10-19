# Humble 分包发布验证

验证日期：2026-10-19。目标版本：0.4.0；维护者邮箱：chiwayfpv@gmail.com；许可证：Apache-2.0。

## 本轮修正

| 原问题 | 修正 |
| --- | --- |
| 核心 CMake 读取仓库父目录的版本文件 | 从本包 `package.xml` 读取版本，生成并安装 `VERSION`；CMake 目标向使用方传递 C++17 要求 |
| 无人机引用小车 launch 和测试源码 | 提取 `litesim3d_bringup` 与 `litesim3d_test_support`，两个示例各自声明依赖 |
| SDF 测试由小车包跨源码目录注册 | 在 `litesim3d_assets` 注册，使用包内固定夹具；跨示例检查移至仓库 integration 套件 |
| 系统依赖键与实际 CMake 查找不一致 | 修正 `libssl-dev`、`nlohmann-json-dev`、`tinyxml2`、`pkg-config`，补齐 RMW、yaml-cpp 和测试依赖 |
| 整仓构建容易掩盖分包问题 | 增加独立包检查器与 CI job；每次仅复制一个源码包，前置模块通过已安装前缀提供 |
| 包拆出仓库后缺少许可证文件 | 每包保留 Apache-2.0 文本，GUI 的可选 Ogre 声明放回 GUI 包内 |
| 大分辨率相机一次单步批次超过 DDS 有界历史，精确接收断言不稳定 | 按最多 3 帧分批推进并收齐；保留全部 Image/CameraInfo 时间戳、标定、暂停和幂等重试检查 |

小车和无人机的现有启动命令保持有效。公共 bringup 不查询任何示例包的安装路径；模型、场景和参数由调用方传入。

## 环境与方法

通过 `ssh local_humble` 在 Ubuntu 22.04 主机上创建新的官方 `ros:humble-ros-base-jammy` 容器，架构 amd64。构建不使用宿主机 ROS 安装、旧项目构建树或其他线程的安装目录。

基准镜像摘要：`sha256:370191995451bb63f2e37bb2f6636a09f66ca810d0865703058361dc6ac70c87`。容器配置 6 个 CPU、18 GiB 内存上限，源码和构建工作区位于独立 tmpfs。

容器网络通过宿主机网络模式访问软件源，测试设置 `ROS_LOCALHOST_ONLY=1` 并使用专用 ROS domain。Ubuntu/ROS 包由 apt 验证签名与校验和。ROS 二进制下载使用清华镜像；镜像不提供的 `deb-src` 索引在此测试容器内关闭，签名校验保持启用。GitHub 原始文件连接受限，rosdep 的官方 YAML 与 Humble cache 通过本机 HTTPS 下载后原样传入，来源、大小和 SHA-256 保存在证据中；本地 index 仅将地址改为 `file://`。

验证分为：清单解析与 rosdep、逐包构建/安装/测试、安装后 C++ 使用方、跨包集成测试，以及 Bloom/Debian 打包与安装。Debian 打包使用 `nocheck`，测试结果由单独的包级运行提供，不能从打包成功推断测试通过。

最终文件核对发现首次 macOS 归档传输在 12 个包中带入 307 个 AppleDouble `._*` 文件。已核对其文件头并清理，分包复制与摘要计算也排除了这些元数据；所有实际源码逐文件 SHA-256 与测试输入一致。原始验证记录保留，通过 `source-normalization.json` 关联清理前后的输入摘要，Debian 包重新由干净源码完整生成。

## 验证结果

12 个包全部通过独立构建、安装和包级测试。源码包每次复制到唯一的 `current-source`，运行完本包测试后删除，再换下一个包。测试不用 `--symlink-install`；模型与启动流程从安装目录读取。

| 检查 | 结果 |
| --- | --- |
| package.xml、版本与模块依赖边界 | 12 包通过；版本全部 0.4.0；两个示例无交叉依赖 |
| rosdep install / check | 声明的系统与 ROS 依赖安装成功；check 无缺失项 |
| 核心 GTest | 162 通过 |
| SDF 资源包 pytest | 26 通过 |
| ROS 桥接 pytest / launch_testing | 74 + 4 通过 |
| 小车 / 无人机默认测试 | 19 + 2 通过；12 个图形项在此阶段跳过 |
| Xvfb / Mesa 软件渲染 | 12 个图形场景通过，覆盖 RGB、深度、RGB-D、查看器、预览与 RViz |
| 仓库跨包与工具集成 | 7 通过 |
| 安装后 C++ 使用方 | 普通 CMake 与独立 ament/colcon 包均编译、链接并执行 20 步成功 |
| Bloom / Debian | 12 个 `0.4.0-0jammy` amd64 包生成并安装成功；最终包不含 AppleDouble 文件 |
| 仅二进制独立运行 | 无源码、无测试辅助包；无人机仅 8 个项目包，小车仅 9 个项目包，均通过启动、DDS、运动和退出检查 |
| 动态库检查 | 22 个已安装 ELF 文件无缺失动态库 |

最终覆盖 **306 个不同测试用例**，不将 CTest 的 pytest 包装项或重跑重复计数。图形检查首轮为 11 通过、1 失败：640×480、30 Hz RGB-D 收到完整 RGB 和深度，但 CameraInfo 中间缺一帧。发布队列为 KEEP_LAST(5)，测试改为分批推进并收齐；修改涉及的 4 种参数组合全部重新通过。初次失败、诊断与修正后日志均保留，不以延长超时或删减断言代替修正。

普通 CMake 检查最初漏传 `/opt/ros/humble` 搜索前缀，补齐 `CMAKE_PREFIX_PATH` 后通过。独立 ament 使用方仅声明核心包依赖，通过 colcon 正常构建；该问题未通过修改核心依赖或硬编码路径来处理。安装后使用方式见 [安装与构建](../usage/安装与构建.md#在自己的包中使用核心库)。

可重复执行的包级入口：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src --rosdistro humble -y
python3 scripts/checks/check_versions.py
python3 scripts/checks/check_release_packages.py --work-dir /tmp/litesim-release-check --jobs 2
```

使用新的空工作目录。修复后可加 `--resume`，仅复用源码与依赖摘要均匹配的成功包。每次仍执行安装后 CMake 使用方检查。

二进制验证使用另一套全新官方 ROS base 容器：先安装无人机所需的 8 个项目包，运行后移除两个无人机包，再安装小车的 3 个示例包。每个阶段都检查另一套示例和 `litesim3d_test_support` 未安装。步进 60 步后，检查点云布局、IMU、里程计、关节与运动；两个 launch 都正常退出，退出码为 0。只使用 `/opt/ros/humble` 安装前缀，不挂载仓库或 colcon 安装目录。

本轮范围为 Ubuntu 22.04 / Humble **amd64** 容器。图形结果来自 Xvfb/Mesa 自动化，不代表物理桌面或真实设备验收；ARM64、正式性能与长时间稳定性未在本轮验证。新增 GitHub Actions job 已写入配置，本轮执行证据来自 SSH 环境中的本地容器。

## 证据与正式发布

原始日志、包清单、JUnit、源文件摘要和总结果 `verification.json` 位于源码仓库外的 `artifacts/evidence/humble-release-20261019/`。通过 `python3 scripts/paths.py evidence` 定位本机证据目录。安装包、安装说明和 `SHA256SUMS` 位于同一产物根目录的 `packages/humble-0.4.0-local-20261019/`。首次产物仅作为历史证据保留，最终包已重新生成并校验。历史报告不替代本轮结果。

正式发布还需冻结源码与各包发布日志、创建版本标签、执行 Bloom release 并完成 rosdistro 审核。临时 rosdep 映射只用于本地首次打包预检，不表示这些包已经进入官方索引。本轮没有创建标签、提交或推送发布仓库，也没有提交 rosdistro PR。
