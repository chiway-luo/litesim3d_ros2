# Humble ARM64 与 Lima 环境验证

验证日期：2026-10-19。目标版本：0.4.0；维护者邮箱：chiwayfpv@gmail.com；许可证：Apache-2.0。

后续已定位双重校时回拨触发 Fast DDS 缓存耗尽，完成 Humble 虚拟机配置修复、慢消费者连续五轮与完整 ROS 78 项复验，关闭当前环境的旧帧阻塞，见 [Fast DDS 旧帧与虚拟机时钟诊断](FastDDS旧帧与虚拟机时钟诊断.md)。下文保留首轮验证时的结果与判断，发布门槛以该后续报告为准。

两套环境已创建，Humble 的 12 包构建、Debian 打包和独立安装验证完成；该轮默认 Fast DDS 配置下的 306 个不同用例全部通过。当时仍因多次复现的慢消费者旧帧问题暂缓正式发布，一次完整通过不足以证明间歇性问题消失；后续根因、配置修复与加强后的复验见上方报告。UDP-only 仅用于对照测试，未更改虚拟机的全局 ROS 默认配置。

## 环境

本机使用 Homebrew 安装 Lima 2.2.1，通过 Apple Virtualization Framework（VZ）创建原生 ARM64 虚拟机。两台均为 4 vCPU、8 GiB 内存、32 GiB 稀疏磁盘。未挂载宿主机目录；源码和工具通过 Lima SSH 复制到 `/work`。

| 实例 | 系统与 ROS | 本轮范围 |
| --- | --- | --- |
| `humble-arm64` | Ubuntu 22.04.5 ARM64、ROS 2 Humble | 系统依赖、独立包构建/测试、Debian 打包、二进制安装与运行 |
| `lyrical-arm64` | Ubuntu 26.04.1 ARM64、ROS 2 Lyrical | ROS base、开发工具及 C++/Python 通信；不构建仿真器 |

Humble 使用 GCC 11 与 Python 3.10。构建采用 2 个并行编译任务，虚拟机本身仍分配 4 核。两个系统均使用官方 Ubuntu cloud image，并固定 SHA-256。26.04 镜像通过 USTC 镜像下载，完整 SHA-256 与 Canonical 原始镜像一致。ROS 软件源配置使用官方 `ros2-apt-source` 1.3.0，并校验发布文件摘要。

Humble 客体的 Ubuntu ARM 软件源使用清华镜像；二进制检查容器使用官方镜像原有的 `ports.ubuntu.com` 源，ROS 源保留 `packages.ros.org`。26.04 使用镜像原有的 Ubuntu archive/security 源。apt 的签名与包摘要检查保持启用。rosdep 使用保存了来源和摘要的官方索引快照；临时本项目映射只用于未发布包的 Bloom 预检。

## ARM64 首轮问题与修正

首轮核心 162 项全部通过，ROS 桥接的 74 项 pytest 中有 1 项超时：单次点云 PCL 校验未收到结果，探针进程正常退出。测试输入使用 best-effort，结果订阅使用 volatile；输入和结果端点分别发现，单次样本存在丢失窗口。

修正只影响 `test_EXT06_independent_voxel_centroids_and_invalid_vendor_input`：通过已有参数启用 reliable 输入，并使用 transient-local 订阅接收探针保留的结果。生产节点默认 QoS、点云布局与数值断言均未修改。仅修改结果订阅的中间尝试第 4 次仍失败；补齐输入可靠性后连续 10 次通过。初始失败、中间尝试、最终定向回归和源码摘要均保留。

QoS 行为依据 [ROS 2 Humble 官方说明](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)：transient-local 发布端配合 volatile 订阅端只交付新消息；双方使用 transient-local 才接收保留消息。

另一次完整回归中，延迟启动 RSP 用例触发了测试专用的 3 秒就绪期限：RSP 进程已启动，但模拟节点期限内未发现其 joint_states 订阅，按设计进入 fault 并以 1 退出。测试改为先观察初始化后的 `ready=false / step=0 / time_ns=1` 状态，再启动 RSP，使用产品正常的 10 秒启动预算；不再用固定睡眠和人为缩短的发现窗口。单独的 0.3 秒超时负例未修改。两种启动顺序及超时负例连续 5 轮、15 项通过。DDS 发现延迟的底层原因未进一步归因；没有将这次失败解释成 ARM64 算法错误。

第三次完整回归的慢消费者测试失败：生产端保持约 1.0 RTF，但慢订阅端十次取数只覆盖 4.3 秒。增加订阅匹配等待与按真实产帧数控制积压后，仍有失败；逐帧记录显示慢订阅端曾在生产端已到约 10 秒时回放约 5 秒的旧帧，不能归因于仿真速度不足。最终将慢消费者置于独立 ROS context，并按真实点云回调计数；全部原有时间跨度、32 MiB 内存增长、30 秒暂停和历史回放断言保留。该版本连续 3 轮定向检查通过，每轮约 40 秒。该定向版本在下一次完整回归中仍失败。之后只对测试进程通过 `FASTRTPS_DEFAULT_PROFILES_FILE=/work/fastdds-udp.xml` 关闭内置传输并启用 UDPv4，三轮对照通过。RMW 仍为 Fast DDS，系统与交互式 ROS 默认配置不变；底层机制未做确定归因。

低 domain 的 UDP 完整回归还暴露了两个 `/clock` 测试竞态：收到状态后立即读取尚未到达的时钟，触发空列表异常。已在需要时间戳的测试中等待首个实际时钟消息，不改变节点行为。

重复重置的消费者就绪超时有独立证据：消费者已进入 executor，但日志显示 `failed to send response (timeout): client will not receive response`。会话管理器原先只发送一次只读参数探针；现在在原有总时限内每秒重试，清理过期请求，仍要求真实响应并检查订阅匹配后才返回就绪。保留未启动 executor 的负例，不增加总超时。Humble 生产 C++ 仿真算法未改变，Python 会话管理器有上述修正。

另发现旧测试与使用示例的 domain 默认值位于 160–219。实测 Ubuntu 临时 UDP 端口为 32768–60999，这与部分 DDS 端口重叠。已将桥接测试池改为 10..59、示例测试池改为 60..99、检查脚本改为低位值、使用示例改为 70；未修改内核端口范围。依据 [ROS 2 domain 官方说明](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Domain-ID.rst)。旧帧问题在独立进程、低 domain 21 下仍复现，因此不能用这个配置问题解释全部现象。

查看器生命周期用例在仿真器就绪后立即查询查看器节点，ARM64 上遇到图发现尚未完成。已改为等待查看器的场景/统计订阅和实际统计样本，再执行原有只订阅、关闭后继续仿真及重连截图断言。相机、RViz 和查看器使用 Xvfb/Mesa 软件渲染。

仓库集成第一次运行缺少 Xacro。该依赖已在现有测试文档及 CI 中明确列出，是本次环境准备遗漏；补装 `ros-humble-xacro` 后重新执行集成检查。没有把仓库工具依赖加入核心或示例的运行包。

二进制容器首次使用清华 HTTPS 源时出现 TLS 握手失败，apt 未取得 Ubuntu 索引。原始日志已保留；重建一次性容器后恢复官方镜像自带软件源，并设置 `APT::Update::Error-Mode=any`，避免把索引获取失败误判为包依赖缺失。apt 的签名验证始终启用。

## 验证结果

12 个包完成独立构建与安装。该轮默认传输回归共 **306 个不同用例：306 个通过、0 个失败**，不重复计入重跑或 CTest 包装项。默认无图形测试中跳过的 12 项已通过 Xvfb/Mesa 单独执行，未把跳过或重跑计作额外通过项。

| 检查 | 结果 |
| --- | --- |
| 核心 GTest | 162 通过 |
| SDF 资源包 | 26 通过 |
| ROS 桥接 pytest / launch_testing | 78 通过，包含 100 次完整会话重置及未就绪消费者负例 |
| 小车 / 无人机默认测试 | 19 / 2 通过；图形项另行执行 |
| Xvfb / Mesa 图形检查 | 12 通过，无跳过 |
| 仓库集成 | 7 通过 |
| 已安装 SDK 使用方 | 普通 CMake、ament/colcon、Debian 安装前缀均编译、链接并执行 20 步成功 |
| 核心包单独安装 | 全新 ARM64 ROS base 容器只安装 1 个项目包与编译工具，外部 C++ 程序执行 20 步成功 |
| Debian 包 | 12 个 `0.4.0-0jammy` ARM64 包构建与安装成功；摘要核对通过，无 AppleDouble 文件 |
| 仅二进制运行 | 无源码、无测试辅助包；无人机 8 个项目包、小车 9 个项目包独立运行成功 |
| ELF / 动态库 | 21 个原生 AArch64 ELF 检查通过，无缺失动态库 |
| Lyrical 环境 | C++ 发布端到 Python 订阅端收到 3 条消息，正常退出；未安装仿真器 |

独立包验证每次仅复制当前包源码，依赖来自安装前缀，且不用 symlink install。Debian 打包阶段使用 `nocheck`，测试来自前述独立运行。二进制运行采用全新官方 ARM64 ROS base 容器，不挂载源码或 colcon 安装目录；两个示例分别核对另一示例与测试辅助包缺席，并检查 60 步后的点云、IMU、里程计、运动与正常退出。

安装包及 `SHA256SUMS` 位于仓库外的 `artifacts/packages/humble-0.4.0-arm64-20261019/`；同目录 README 提供按模块安装的命令。该轮归档源码与构建、测试、Debian 输入一致；构建后另补充报告、开发日志与 README 的版本读取说明。后续旧帧诊断加强了测试并增加只读检查器，本轮既有 Debian 包没有重新生成，详见后续报告的验证范围。


## 使用环境

两台实例均已停止，运行内存已释放，重新启动即可继续使用。宿主机实例目录实际占用：`humble-arm64` 约 8.4 GiB；`lyrical-arm64` 约 3.5 GiB。32 GiB 是每台稀疏虚拟盘的容量上限；上述占用不包含证据目录中的独立镜像缓存。

在 macOS 终端启动并进入所需实例：

```bash
limactl start humble-arm64
limactl shell --workdir=/work humble-arm64
```

Humble 客体的交互式 Bash 已加载 ROS。源码位于 `/work/src`，独立包验证结果位于 `/work/validation`，Debian 产物位于 `/work/debian-preflight`。

```bash
limactl start lyrical-arm64
limactl shell --workdir=/work lyrical-arm64
```

Lyrical 只准备环境，没有复制或构建仿真器源码。停止实例可释放运行内存，磁盘与环境仍保留：

```bash
limactl stop humble-arm64
limactl stop lyrical-arm64
```

## 证据与边界

原始证据位于仓库外 `artifacts/evidence/lima-arm64-20261019/`，包含 Lima 配置、镜像/软件源摘要、初始与最终源码归档、失败日志、JUnit、包元数据及虚拟机环境清单。通过 `python3 scripts/paths.py evidence` 定位本机证据根目录。

本轮为原生 ARM64 虚拟机中的软件验证。图形检查使用 Xvfb/Mesa，不代表真实桌面手动操作或设备验收；正式性能、8 小时长稳与外部导航闭环不在本轮范围。Lyrical 仿真器兼容性未验证。安装包属于本地发布预检，没有发布到 ROS 官方 apt，也未创建 Git 提交、标签或 rosdistro PR。
