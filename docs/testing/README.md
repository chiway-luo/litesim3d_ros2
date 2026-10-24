# 测试文档

阿克曼 TF 数量断言的后续调查、端点诊断与完整回归见 [阿克曼 TF 断言诊断](阿克曼TF断言诊断.md)。该记录包含启动失败后的 RSP 残留复现、进程组清理修复和从头执行的完整回归。

2026-10-23 的全部现有运动模型、独立第三方控制器及相机联调见 [全部模型 ros2_control 验证](全部模型ros2_control验证.md)。

2026-10-23 的阿克曼、三轮与四轮普通全向 ros2_control 联调见 [多底盘 ros2_control 验证](多底盘ros2_control验证.md)。

当前内置 RViz 视图不加载 `/scene` 场景图层，保留启动集成与点云显示。`scripts/test_desktop.sh` 验证 RViz、独立窗口及 RGB/Depth/RGB-D 实际渲染，包含相机误差与固定种子复现；`scripts/test_plugins.sh` 验证传感器/里程计计算契约。

2026-10-20 的实际渲染、误差统计和 RViz 截图见[测量误差、相机渲染与 RViz 验证](测量误差与相机RViz验证.md)。

当前能力摘要见[版本与验证状态](../版本与验证状态.md)。本轮文档、教程与站点验证见[文档规范化验证](文档规范化验证.md)。

2026-10-19 的插件架构、组合模型、外部插件及 29 包独立检查见 [插件架构完成验证](插件架构完成验证.md)。

重置后步进已执行但应答丢失的根因、修复与故障注入检查见 [重置后服务应答超时诊断](重置后服务应答超时诊断.md)。

慢消费者旧帧的根因、环境修复与复验结果见 [Fast DDS 旧帧与虚拟机时钟诊断](FastDDS旧帧与虚拟机时钟诊断.md)。

原生 ARM64 构建、Debian 安装与两套 Lima 环境见 [Humble ARM64 与 Lima 环境验证](HumbleARM64与Lima环境验证.md)。

本轮 0.4.0 的模块拆分、Ubuntu/Humble 逐包构建及 Debian 预检见 [Humble 分包发布验证](Humble分包发布验证.md)。

2026-10-19 的桌面补测、新无人机与办公区结果见 [无人机与办公场景验证](无人机与办公场景验证.md)。

2026-10-18 的 SDF 资源、独立窗口、碰撞预算和真实速率验收见 [SDF 实现验证](SDF实现验证.md)。

这里集中保存回归入口、验收门槛与历史记录。普通安装和运行不需要测试证据，见 [使用文档](../usage/README.md)。以下命令从源码仓库根目录、已激活 Humble 的 Bash 终端执行。

## 当前源码的检查与回归

本地 Ubuntu 和 [GitHub Actions](../../.github/workflows/m1.yml) 使用同一入口。完整提交前检查从仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
python3 scripts/test_ubuntu.py --profile all --output "$HOME/litesim-quality" --jobs 2
```

运行平台是 Ubuntu 22.04 / ROS 2 Humble。先按[安装与构建](../usage/安装与构建.md)安装 rosdep 声明依赖；测试工具还需要 `clang-14`、`llvm-14`、`libunwind-14-dev`、`python3-coverage`、`python3-pytest`、`dbus`、`xvfb`、`xauth` 和 `libgl1-mesa-dri`。输出目录必须在源码目录外，不同 profile 使用独立构建树。入口不会安装依赖或修改系统设置。

统一入口为每个图形阶段启动独立的 [D-Bus 会话](https://dbus.freedesktop.org/doc/dbus-run-session.1.html)和 Xvfb 显示，阶段退出时一同回收。这样 SSH 和无桌面登录会话的 CI 容器使用相同环境：Ubuntu 22.04 的 Qt 5.15.3 / libdbus 1.12.20 在缺少用户运行目录且尝试自动发现会话时，可由空 Qt 窗口复现 1032 字节退出泄漏；显式私有会话消除该触发条件，不增加 LSan 例外。工具回归会在无 `XDG_RUNTIME_DIR`、继承无效总线地址的环境中运行真实 ASan Qt 程序，验证总线可用、成功/失败退出码正确传递以及总线进程回收。

在 Docker 容器中运行这些检查时，使用 `docker run --init ...`，由 init 回收退出的子进程；CI 已配置同样的选项。仅用 `tail` 保活的容器不会回收被收养的孤儿进程，已退出的 D-Bus 进程会留下僵尸记录，使严格的回收检查失败。普通 Ubuntu 登录环境由系统 init 负责回收。

CI 在独立容器中安装测试工具。已有 Nav2、SLAM 或其他版本 LLVM 的 Ubuntu 开发环境，应把 LLVM 14 回溯库解压到独立目录：Ubuntu 的 `libunwind-14-dev` 与部分现有开发包冲突，直接安装可能连带移除导航依赖。以下方式只下载和解压，不修改系统包：

```bash
tools_dir="$HOME/litesim-quality-tools"
mkdir -p "$tools_dir/downloads" "$tools_dir/root"
(
  cd "$tools_dir/downloads"
  apt download libunwind-14 libunwind-14-dev
  for package in libunwind-14_*.deb libunwind-14-dev_*.deb; do
    dpkg-deb -x "$package" "$tools_dir/root"
  done
)
export LITESIM_LLVM_UNWIND_DIR="$tools_dir/root/usr/lib/llvm-14/lib"
export LD_LIBRARY_PATH="$LITESIM_LLVM_UNWIND_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

设置上述环境后，在同一终端运行统一入口。CMake、检测器阳性对照和工具回归都使用该目录；普通仿真运行不需要这些环境变量。

| Profile | 内容 |
| --- | --- |
| `functional` | 全包构建、内核与插件单元测试、同进程生命周期、ROS 回归、示例、教程、桌面渲染、RViz 独立性、隐藏源码安装及无可选插件补测 |
| `coverage` | 全包 Clang 插桩、Python 子进程采集、原生/ROS/示例/桌面回归，生成当前源码覆盖率和独立单元测试覆盖率 |
| `asan` | ASan + LSan + UBSan 全包插桩、泄漏/未定义行为阳性对照及原生/ROS/示例/桌面回归，检查子进程日志 |
| `tsan` | 独立 TSan 构建、数据竞争阳性对照、内核/插件单元测试与生命周期检查；不将未插桩 ROS/图形依赖当作全栈线程安全证明 |
| `release` | 每包独立构建、安装及其测试，验证 SDK 使用方 |

可用 `--profile coverage` 等单独诊断失败。每次都重新执行测试，日志和 JUnit 放在 `PROFILE/runs/时间-进程号/`，`latest.json` 指向本轮结果。结果包含源码逐文件哈希、架构、RMW 和命令；测试期间源码变化会使本轮失败。旧构建缓存只在构建身份一致时复用，旧通过结果不会复用。完整检查的五项必须全部通过，短测不替代 8 小时长稳。

ASan 的完整调用栈展开会显著增加高频分配用例的耗时，内核/插件测试墙钟预算为 1200 秒，ROS 套件为 2400 秒，整体回归预算由 `test_m3.sh --timeout-budget` 汇总各子套件上限并加 60 秒调度余量，当前 ASan 为 9900 秒，避免父进程提前截断尚未结束的子套件。两个 ROS 测试 Harness 的条件、服务与退出等待至少为 120 秒，以覆盖 SDF 导入、大型雷达模式的初始化与清理；示例套件为 900 秒，单个渲染程序和单批渲染请求为 900 秒。桌面测试单独将相机启动和条件等待设为 600 秒、ROS launch 的正常退出宽限设为 120 秒，桌面 pytest 套件为 6000 秒，整个桌面阶段为 9000 秒，CI 的 ASan 作业上限为 180 分钟。这些设置只由测试入口传入，普通运行的启动保护和故障测试保留原值；控制器停流检查以反馈消息的仿真时间判断 2 秒内停止，并另设有界墙钟存活保护；降速运行不能延长实际的仿真超时契约。controller_manager 切换请求显式设置墙钟等待（普通测试 10 秒、ASan 120 秒），避免 Humble 默认 1 秒在慢仿真下提前失败；暂停超过 1 秒再恢复的回归仍要求严格、原子的控制器交接。嵌套 runner 被取消时会回收自身启动的进程，包括另建 session 的子进程。RTF 用例在所有 profile 中检查实际时钟、统计准确性与持续推进；目标速度精度由普通和覆盖率构建检查，避免把 Sanitizer 分配回溯的性能开销误判为调速错误。

成功的内核校验不应为固定诊断文字分配堆内存。ASan 原生测试通过分配钩子验证该约束，并用真实分配确认钩子有效；失败校验仍须抛出原有异常和诊断，原 `std::string` 接口保留。Mid360 的大扫描表和传感器高频路径会放大此类无用分配，不能只靠延长启动等待掩盖。SDF 启动入口也须在导入资源之前拒绝非法的 `paused` 参数。

ASan 的独立 GUI/相机渲染进程使用 LLVM 14 的 `libunwind` 展开分配调用栈。Mesa 软件渲染的 JIT 注册异常栈时，GCC 回溯库可能在 `malloc` 中递归获取同一把锁，导致启动死锁；测试工具会实际运行一个程序，确认渲染构建的 `_Unwind_Backtrace` 来自 LLVM。无图形进程保留平台回溯库，避免额外开销；两种回溯器分别执行泄漏与未定义行为阳性对照。发布构建不依赖 LLVM 回溯库。

Linux x86_64 的 ASan 测试可执行文件使用 `-no-pie` 链接，避免 Clang 14 在较新内核的地址随机化下于 `main` 前发生分配器映射冲突。动态插件保留共享库形式，普通与发布构建保持原设置。入口不修改系统 ASLR，也不通过重试掩盖启动失败；工具回归会编译并运行真实的可执行文件与共享插件，检查 ELF 类型。

ASan 测试通过临时 ament overlay 为系统 `ros2_control_node` 设置同一 Clang 版本的共享运行库，仅在执行该原生宿主时注入。Python、spawner 和其他仿真可执行文件继续使用各自原有环境，系统 ROS 安装不改动。发行版宿主剥离了 pluginlib 模板符号，因此仅对该宿主追加 `class_loader::impl::loadLibrary` 注册路径例外；带符号和剥离符号的两套真实加载器均执行正常加载与构造函数泄漏对照，未匹配整个 controller_manager/hardware_interface 库。真实动态加载回归必须同时通过正常加载、UB 检出和实例泄漏检出。硬件插件只链接 `hardware_interface::hardware_interface`，避免把 fake/mock 示例插件直接链接进宿主、造成重复注册。

桌面阶段还运行 RGB、Depth、RGB-D 与真实 ros2_control 控制器同时工作的三项联调，检查实际 Ogre 图像、相机信息时间戳和运动反馈。普通 `functional` 阶段单独构建第三方高度控制器 overlay，检查冲突拒绝、原子交接和停流，并用 SHA-256 确认底层仿真器和硬件适配库未被改写。无显示或缺少 overlay 时跳过的用例因此均有对应的实际执行阶段。

Qt 程序在启动时创建一个 ROS executor，并在所有定时回调中复用，避免关闭期间反复构造 executor 引起异常。回调统一处理 context 在状态检查与 spin 之间关闭的竞态：正常关闭结束事件循环，ROS 仍有效时的异常保持失败。三项原生回归使用真实 Humble executor，覆盖关闭竞态、已关闭状态和真实异常传播，并接受 ASan/LSan 检查。私有测试 context 在节点和 executor 析构后显式清理子 context，并断言弱引用失效，解除 Humble GraphListener / GuardCondition 的循环引用，不增加泄漏抑制。相机用例记录并核对子进程退出码，避免图像断言通过后遗漏关闭失败。渲染器故障注入仅允许已记录断连诊断的仿真器 PID 以 1 退出，其余进程须正常退出，检测器日志仍独立检查。测试 Harness 先向仍运行的 launch 发送一次 SIGINT，由 launch 转发；只有退出的 launch 留有子进程时才向所属进程组补发信号，防止重复信号打断析构或 LSan 检查。覆盖率和 ASan 的示例套件外层预算均为 900 秒，控制器和仿真时间断言不变。

五种基础运动模型在仿真就绪、时钟推进后才执行原生管理器和 spawner，避免硬件提前激活后把慢启动误判为失联，也避免控制器列表切换一直等待更新线程。默认 `controller_startup_timeout` 为 60 秒，插桩测试按同一墙钟存活预算显式传入；这不修改仿真时钟、硬件反馈看门狗或控制器指令超时。真实 DDS 回归覆盖未就绪、时钟冻结、正常推进、仿真故障，以及两个原生程序的 ROS 参数和成功/失败退出码传播。

ASan 要求 `rclcpp >= 16.0.19`。该 Humble 版本包含[默认分配器修复](https://github.com/ros2/rclcpp/pull/3072)，旧版可在空节点启动时触发 `new-delete-type-mismatch`。入口在编译前记录实际版本并拒绝旧版，应更新测试用 ROS underlay，不能关闭该检测绕过问题。CI 显式安装更新后的 `ros-humble-rclcpp`，避免基础镜像保留旧版。

覆盖率清单从所有当前包自动生成，纳入运行时 C++、有可执行映射的头文件、Python、插件、GUI 和示例启动代码；生成消息、上游消息定义、测试支撑、测试程序和开发报告工具单独记录排除原因。漏掉新文件或未插桩编译会失败。LLVM 分母从所有编译对象的零命中映射固定提取，避免多个程序的同名函数或缺失执行数据缩小分母；命中按源码位置合并，安装副本与多个插件内的同一头文件不会重复计数。Python 未执行代码仍进入分母。未实例化的 C++ 模板/内联体没有 LLVM 映射，报告会明确这个工具边界。

`coverage/combined-coverage.json` 保存每文件与每包结果，`unit-coverage.json` 单列单元测试命中，`coverage-gate.json` 执行门槛：全部测试合计行覆盖率 ≥ 80%、分支 ≥ 70%；几何、模型、仿真器、传感器、地形、世界六个核心源文件必须仅凭单元测试逐文件达到 ≥ 90% / 80%。总结果按命中数与分母计算，不能把旧报告或端到端命中率当作当前单元测试覆盖率。

生命周期检查在同一仿真器进程完成 140 轮构建、运行、位姿重置、故障、销毁及失败初始化。40 轮预热后记录 RSS、句柄和线程；普通构建限制 RSS 增量 32 MiB，Sanitizer 构建记录 RSS 并由 LSan 检测泄漏，避免把隔离区误当泄漏。检测器必须先检出已知的故意错误，真实测试中任一子进程的检测报告均使 profile 失败。

ASan 始终开启 `detect_leaks=1`，阳性对照和真实测试统一使用 `fast_unwind_on_malloc=0:malloc_context_size=64`：Ubuntu x86_64 的部分依赖二进制省略帧指针，快速展开可能丢失库加载调用栈；Mesa 的深层分配路径还可能超出默认 30 帧，丢失限定例外所需的调用方。完整展开的额外开销只用于 ASan 检查。UBSan 与 TSan 遇错返回非零。Humble 二进制依赖有已复现的退出残留，[LSan 例外清单](../../scripts/quality/lsan.supp)只匹配指定的上游分配调用栈，实际使用的规则、分配数和字节数保存在 `sanitizers.json`。**通过含例外的门禁不等于整个依赖栈零泄漏**：

| 依赖路径 | 已知问题与检测边界 |
| --- | --- |
| pluginlib 加载注册 | [class_loader 元对象未释放](https://github.com/ros/class_loader/issues/131)。只匹配加载库期间的注册；另一个实际加载的测试插件在构造函数故意泄漏 64 字节，必须仍被检出。该插件仅在 ASan 测试构建生成。硬件和控制器基类各有独立的真实 pluginlib 对照，正常卸载仅应用注册例外，构造函数泄漏 64 字节必须仍失败 |
| ROS 消息类型支持 | [Humble 的 SharedLibrary 原始指针缓存](https://github.com/ros2/rosidl_typesupport/blob/humble/rosidl_typesupport_cpp/src/type_support_dispatch.hpp)在插件卸载后失去可达引用；二进制已剥离内部模板符号，因此限定 `librosidl_typesupport_cpp.so`，不匹配整个 rclcpp/rmw |
| Ignition Rendering 6 / Ogre2 | 6.6.4 的空场景创建、销毁、卸载独立程序仍保留场景/根节点/mesh factory 引用环；mesh factory 还缓存原始材质。例外覆盖引擎初始化、场景创建与网格导入的上游分配，这些路径的第三方泄漏不由本门禁证明已消除 |
| Mesa EGL / Fontconfig | 软件渲染的 EGL 初始化与 Qt 首次字体初始化有退出缓存；限定引擎加载和 `FcInit`，没有对整个 Qt/Ogre 库关闭检测 |

桌面用例实际渲染 OBJ、DAE、PBR、RGB/Depth/RGB-D，检查连续三次无效几何构造后引擎已卸载并能正常重建。慢消费者测试也保存 RSS 前后值：普通构建执行 32 MiB 上限，Sanitizer 构建记录隔离区影响并执行独立泄漏门禁，其他 DDS 队列和时序断言照常执行。8 小时长稳和第三方库源码插桩仍是独立待验项目。

组合模型的运行和测试需要 Xacro；Ubuntu 的 rosdep 会安装 `ros-humble-xacro`。底盘、里程计与传感器插件的契约测试见 `scripts/test_plugins.sh`，外部独立安装证明见 `scripts/test_external_plugin.sh`。

先构建并加载安装环境：

```bash
bash scripts/build.sh
source "$(python3 scripts/paths.py install)/setup.bash"
```

若使用标准 colcon 工作空间，先加载该工作空间 `install/setup.bash`，并设置 `LWS_INSTALL_PREFIX` 为它的绝对路径；构建必须启用 `BUILD_TESTING`，`build/` 与 merged `install/` 应为同级目录。

| 范围 | 命令 | 实际作用 |
| --- | --- | --- |
| 仓库文档 | `python3 scripts/checks/check_repository_docs.py` | 不依赖历史日志，检查仓库内链接与锚点、页面标题、个人路径及 Shell 语法 |
| 接口文档 | `python3 scripts/checks/check_api_docs.py` | 核对 IDL、声明参数名称及生成的插件参考表 |
| 文档检查器负例 | `python3 -m unittest discover -s scripts/checks -p test_documentation_checks.py` | 验证错误锚点和错误 Shell 能被拒绝 |
| 入门教程 | `python3 scripts/checks/check_documented_tutorials.py` | 执行教程原文命令，验证单机控制、IMU/TF 和消费节点负例 |
| 系统时钟跳变诊断 | `python3 scripts/checks/check_clock_stability.py --duration 60` | 只读观察系统时间相对单调时钟的跳变；用于排查 DDS 旧帧，不检查仿真 `/clock` |
| 完整功能回归 | `bash scripts/test_m3.sh` | 内核、插件契约、控制配置与硬件、M1/M2/M3 ROS、100 次会话重置、报告门禁负例、全部运动案例、bringup、SDF 场景、单机/多机 launch |
| M1 / M2 子集 | `bash scripts/test_m1.sh` / `bash scripts/test_m2.sh` | 按阶段选择测试，非完整新版本验收 |
| 逐包独立发布检查 | `python3 scripts/checks/check_release_packages.py --work-dir /tmp/litesim-release-check` | 每次只有当前包源码，依赖来自已安装前缀，并执行该包测试 |
| 安装后独立运行 | `python3 scripts/checks/check_m3_install.py` | 新目录构建、隐藏源码副本后验证安装资源和节点 |
| 真实桌面补测 | `bash scripts/test_desktop.sh` | 23 项桌面、相机和无人机用例，3 项 ROS 事件循环退出回归，以及独立模型/渲染检查；须有桌面与 Ogre 渲染后端 |
| 四轮示例 | 见 [四轮车与雷达测试](四轮车与雷达测试.md) | 真实 ROS 接口、配置、TF 和子进程退出 |

`test_m3.sh` 默认不启动 GUI、正式性能窗口或 8 小时长稳。测试失败返回非零；日志/JUnit 输出到 `evidence/m3/run-*`，每轮单独保存。`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 用于避开不兼容的自动加载插件，脚本会设置，不修改 Python 环境安装包。

正式性能测量需单独运行，避免同时构建或执行其他测试：

```bash
LWS_M3_PERFORMANCE=1 bash scripts/test_m3.sh
```

该选项增加 B3 的两种节拍、每种 5 轮、每轮 30 秒预热与 120 秒测量。性能阈值见 [M3 基线](M3基线.md)。长稳是单独的验收项目，不用短回归代替。

Linux 独立验收脚本为 `python3 scripts/checks/check_linux.py`，图形一致性检查需 Xvfb 或桌面会话；依赖和命令见 [CI](../../.github/workflows/m1.yml)。检测器构建可用 `SIM_SANITIZE=ON`（ASan/UBSan）或 `SIM_TSAN=ON`（独立 TSan），不可混用；`SIM_COVERAGE=ON` 用于 Clang 覆盖率构建。它们均需单独的构建目录与实际结果。

## 结果与能力边界

插桩构建的同步大步进允许 60 秒墙钟等待，普通 `functional` 仍使用原有 6 秒；四机器人 launch 的收帧和 SIGINT 退出等待在插桩构建中为 60 秒，普通构建仍为 15 秒 / 5 秒。四台雷达的 16×512 点、至少三帧、帧坐标及唯一时钟断言不变，不缩小负载或重试失败。固定墙钟运动窗口的教程在 `functional` 验证，插桩 profile 不用于性能判定。缺少 Mid360/Livox 的兼容性负例由 `functional` 在独立安装前缀执行，两项必须实际执行，不能以跳过代替通过；ROS underlay 若已安装这两个可选包，该检查会明确失败，需使用不含它们的干净 Humble 环境。

初始历史验收平台为 macOS arm64 / RoboStack Humble。历史报告按当时的源码指纹、版本和运行条件读取；同日后续改名、目录调整、雷达及重置修复不能自动继承全部历史结论。

- [近期修复与验证](近期修复与验证.md)：目录整理、四轮车/雷达和重置首帧问题的实测记录。
- [文档发布检查](文档发布检查.md)：本次通用上手流程与独立检出的验证结果。
- [M1 报告](M1测试报告.md)、[M2 报告](M2测试报告.md)、[M3 报告](M3测试报告.md)、[文档补全报告](文档对照与补全报告.md)：历史结果，不能作为当前提交全量复测的证明。
- 外部 SLAM/Nav2 闭环、8 小时长稳与真实设备验收仍未完成；Linux 的历史补充记录也不等于当前提交通过。远端 CI 以对应提交的 GitHub Actions 实际结果为准。

## 规范与基线

| 文档 | 用途 |
| --- | --- |
| [测试与验收规范](测试与验收规范.md) | 数值容差、覆盖率与验收要求 |
| [测试用例与需求追踪](测试用例与需求追踪.md) | 原始 140 条规格及需求映射 |
| [M1](M1基线.md)、[M2](M2基线.md)、[M3](M3基线.md) 基线 | 各阶段冻结范围与平台决定 |
| [测试报告模板](测试报告模板.md) | 新测试报告格式 |
| [文档检查记录](文档检查记录.md) | 早期文档审核历史 |

## 原始证据与历史报告生成

仓库只提交源码、文档与测试程序。大型日志、运行清单、JUnit、点云、覆盖率与旧构建树留在外部产物目录，不随 clone 下载。报告中的 `evidence/...` 是逻辑证据路径，不是仓库内超链接；查询本机位置：

```bash
python3 scripts/paths.py evidence
```

例如 `evidence/reset-first-frame-20261018/verification.json` 对应该输出目录下的 `reset-first-frame-20261018/verification.json`。新检出只能生成自己的运行证据，不能据缺失的历史文件声称旧结果已验证。GitHub Actions 新产生的日志作为对应 workflow run 的 artifact 下载。

`scripts/reports/report_*.py`、`check_docs.py` 和 `check_m2_docs.py` 是历史证据审计/报告工具，依赖完整的对应日志、固定门槛和源码指纹。它们不是首次运行步骤，也不能在空 clone 上代替新测试。生成器若缺少或发现过期证据，应失败而非生成通过结论。通用文档检查使用上表的 `check_repository_docs.py`。
