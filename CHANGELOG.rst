LiteSim 3D 开发日志
===================

更新时间：2026-10-19（Asia/Shanghai）。当前目标版本：**0.4.0**。

本日志根据 Git 提交、项目内实现/测试文档及可核对的历史证据回填。
M1、M2、M3 是开发里程碑，不是已发布的语义版本；本次回填所用 Git 历史没有版本标签。
各包将首次发布前的累计变更放在 ``Forthcoming``，不补造旧版本或发布日期。
开发日期来自当时的文档记录和 Git 提交；早期合并进初始提交的工作按阶段记录。

Forthcoming
-----------

2026-10-19：重置后的服务应答超时修复
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* 定位 Humble Fast DDS 服务发现竞态：请求已执行，但应答订阅者匹配超时，默认 rclcpp 只记录警告并丢弃应答。
* 单机与多机节点共用有界服务应答模块，只重发同一请求标识的已生成结果，不重新执行步进；限制应答保留时间、队列容量及每次定时回调的发送次数。
* 为 RCL 发送超时增加 Linux 故障注入回归，检查单机/多机不重复推进、超期清理、队列溢出拒绝、非超时错误传播和有待发应答时正常退出；保留原 6 秒客户端等待和 100 次重置检查。
* 直接声明 RCL 依赖，并将公共应答模块纳入运行清单的桥接实现指纹。

根因与验证证据见 `重置后服务应答超时诊断 <docs/testing/重置后服务应答超时诊断.md>`_。

2026-10-19：慢消费者旧帧根因、环境修复与回归加严
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* 通过无仿真器的独立进程复现、DDS 接收时间戳与缓存分支跟踪，定位 Lima/Ubuntu 反复校时回拨触发的 Fast DDS 接收缓存耗尽。
* 增加每次慢取样的帧龄上限与时钟跳变记录；修复前测试正确检出原跨度断言漏掉的 0.8/1.7 秒旧帧。
* 增加只读系统时钟稳定性检查器；经授权仅停用 Humble Lima 客体的重复 NTP 校时，保留宿主机校时，仿真实现与系统 DDS 库未改变。
* 默认 Fast DDS 下独立复现程序 3 轮、慢消费者 5 轮、完整 ROS 78 项均通过；约 507 秒回归期间无超阈值时钟跳变，关闭当前环境的旧帧阻塞。系统再次大幅回拨仍受上游缺陷影响。

根因、回退与验证范围见 `Fast DDS 旧帧与虚拟机时钟诊断 <docs/testing/FastDDS旧帧与虚拟机时钟诊断.md>`_。

2026-10-19：Humble ARM64 与本地 Lima 环境
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* 使用 4 核、8 GiB 内存、32 GiB 稀疏磁盘的 Ubuntu 22.04 ARM64 Lima 虚拟机，验证 12 包独立构建、Debian 打包与安装使用。
* 加固单次点云、延迟启动、首个时钟与慢消费者测试的 QoS、前置状态与观察器隔离；保留原始失败和逐帧证据。
* 会话管理器在原有总时限内重试未答复的只读就绪探针，并清理过期请求；收到真实响应后才能完成重置。C++ 仿真算法未修改。
* 默认 Fast DDS 配置下完成 306 个不同用例；完成无源码的无人机、小车和仅核心 SDK 三种二进制使用检查。历史失败日志保留，当时将间歇性旧帧问题列为发布阻塞。
* 首轮未修改全局默认传输，也未确定旧帧根因；后续定位、环境修复与加强的回归见本日志上方记录。
* 同配置准备 Ubuntu 26.04 / ROS 2 Lyrical，仅验证基础环境与通信，未构建仿真器。

范围、限制与证据见 `Humble ARM64 与 Lima 环境验证 <docs/testing/HumbleARM64与Lima环境验证.md>`_。

2026-10-19：逐包构建与公共模块拆分
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* 新增 ``litesim3d_bringup``，集中 SDF/旧式 YAML 的启动流程；小车和无人机仅提供各自默认配置，不再相互依赖。
* 新增 ``litesim3d_test_support``，以安装后的 Python 模块提供 DDS 和子进程测试辅助类。
* 核心版本改从本包 ``package.xml`` 读取；每包带有 Apache-2.0 许可证。仓库 ``VERSION`` 继续用于统一版本检查。
* SDF 导入器拥有自己的测试注册和固定夹具；跨包场景与仓库工具测试归入 integration 阶段。
* 修正 OpenSSL、JSON、tinyxml2、pkg-config、RMW 及测试工具依赖，构建文档和 CI 改由 rosdep 解析声明。
* 增加每次仅复制一个源码包、依赖已安装前缀的构建/测试检查器，防止整仓构建掩盖分包问题。
* 增加安装后 CMake 使用方检查，验证公开头文件、链接依赖与 C++17 要求；相机精确采样测试改为分批推进与收齐，遵守有界 DDS 历史并保留逐帧时间戳断言。
* 分包检查过滤 macOS AppleDouble 元数据和缓存文件，按实际源码内容计算摘要。

实现入口：``scripts/checks/check_release_packages.py``。Ubuntu 22.04 / Humble amd64
验证覆盖 12 包独立构建、306 个不同用例、12 个 Debian 包，以及两个示例的独立二进制运行。
范围和首次失败的修正记录见 `Humble 分包发布验证 <docs/testing/Humble分包发布验证.md>`_。

2026-10-19：模型资源拆分与发布资料
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* 将原 ``litesim_models`` 拆分为 ``wheel_car_4w_models`` 和 ``drone_3d_models``。
  小车和无人机分别保存模型、场景、网格与贴图；运行入口通过各自的 ament share 路径加载。
* 更新示例依赖、SDF 资源检查和场景生成器；原有几何与贴图迁移保持可追溯。
* 将 10 个包的维护者邮箱设为 ``chiwayfpv@gmail.com``，补齐仓库 Apache-2.0
  许可证和每包开发日志。当前版本由仓库的 VERSION 和各包 package.xml 声明。

依据：`小车资源说明 <examples/wheel_car_4w/wheel_car_4w_models/README.md>`_、
`无人机资源说明 <examples/drone_3d/drone_3d_models/README.md>`_、
`场景生成器 <scripts/fixtures/generate_example_worlds.py>`_。
拆分的历史本机证据为外部 ``evidence/model-separation-20261019/verification.json``：
10 包构建成功，回归 51 通过、12 个图形项跳过，未复测 Linux/桌面渲染。
这些是该次运行结果，不代表当前所有并行改动已重新验收。该阶段在本次回填时尚未提交。

2026-10-19：无人机、办公场景与观察测试
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* 增加保持水平的 multicopter 三轴速度与偏航运动学、合速度/加速度限幅、
  命令超时制动与悬停，以及地板、墙壁、顶板和动态障碍的几何碰撞约束。
* 增加无人机 launch、配置、飞行课程及四轮车办公区；场景保留门洞、桌下和桥下间隙。
* 扩展三维运动、TF/IMU/点云、相机深度变化及桌面测试。相机默认关闭，启用后默认 RGB；
  深度或 RGB-D 由配置显式选择。
* 修正 RViz 输出一致性检查：每批等待点云和里程计均收齐，
  检查里程计时间戳序列并保存失败时收到的时间戳，避免观察端积压造成误判。

提交：`a5886da <https://github.com/chiway-luo/litesim3d_ros2/commit/a5886daca775875afa6d579e643c89fa3c92c6b1>`_、
`0948f4c <https://github.com/chiway-luo/litesim3d_ros2/commit/0948f4cd4051904557492f319ce6a51433f91931>`_。
实现和当次验证范围见 `无人机与办公场景验证 <docs/testing/无人机与办公场景验证.md>`_。

2026-10-18 至 2026-10-19：Humble 与 Linux CI 修复
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* 修正 GCC 下空向量参数声明的重载选择，避免多机器人节点因静态类型参数未初始化退出。
* 为暂停后单步、大点云及会话重置检查补齐 DDS 匹配等待、观察节点图传播等待和 reliable
  传输测试；保留普通 best-effort 模式及不兼容 QoS 的失败检查。
* 修正 GitHub Actions job 环境配置；将构建、回归、隐藏源码后的安装检查与 RViz
  一致性检查分开执行，失败时继续收集独立证据。
* 修正不限速运行时的墙钟状态发布，以及高传感器负载下的测试回调公平性、
  DDS 启动观察和 RViz 退出观察。

相关提交：`a3ec1f3 <https://github.com/chiway-luo/litesim3d_ros2/commit/a3ec1f3e780b4e84fc3b9ea89d6d064a6b97244c>`_、
`63b8b1a <https://github.com/chiway-luo/litesim3d_ros2/commit/63b8b1a0d157265f11a2ceb18daf20a342c2aad1>`_、
`87340b8 <https://github.com/chiway-luo/litesim3d_ros2/commit/87340b8ddf858ae38df77aa5da635b84571ef3fe>`_、
`2a2b184 <https://github.com/chiway-luo/litesim3d_ros2/commit/2a2b184f3991c3a7bd830ce548b4b434f35d8028>`_、
`baf519e <https://github.com/chiway-luo/litesim3d_ros2/commit/baf519e2c1a9ef2749165f153264d26e4f67b17c>`_、
`b8ad2ec <https://github.com/chiway-luo/litesim3d_ros2/commit/b8ad2ec994db1c2d112ff414a2a87e1dacc1c1a3>`_、
`b297c3f <https://github.com/chiway-luo/litesim3d_ros2/commit/b297c3f43ec3b01a46ba072e96cef3d2d0f43435>`_、
`5527246 <https://github.com/chiway-luo/litesim3d_ros2/commit/5527246aacb4e9ec758c92c47ac0b438a603949e>`_。
排查记录见 `近期修复与验证 <docs/testing/近期修复与验证.md>`_。
CI 配置、修复提交和历史运行均不替代待发布提交的实际构建结果。

2026-10-18：SDF、独立查看器与相机
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* 增加 ``litesim3d_assets``、原共享模型资源包和 ``litesim3d_gui``；
  导入支持范围内的 SDF，解析模型资源、坐标变换与绑定，分别保留 visual/collision。
* 支持 OBJ/DAE 多材质、贴图和指定 PBR 资源；缺失资源、不支持的行为及非法绑定明确拒绝。
* 增加 Qt5/Ogre2 三维查看器、只影响观察相机的旋转/平移/缩放、
  静态场景重连、实际 RTF 显示和默认关闭的碰撞复杂度预算。
* 增加 RGB/depth/RGB-D 独立采样、渲染服务、Image/CameraInfo、
  光学坐标系、米制轴向深度、同步 RGB-D 与只读图像预览。
* 保持查看器、预览与相机采样生命周期独立；必需渲染器失败向 launch 传播。
  为本机图形运行补齐 Metal 后端选择与安装后相对库路径。

提交：`1289eea <https://github.com/chiway-luo/litesim3d_ros2/commit/1289eeabf77506bfb314116e6a6e71777b80483c>`_。
详细能力、历史测试轮次和限制见 `SDF 实现验证 <docs/testing/SDF实现验证.md>`_。

2026-10-18：工程整理、审查修复与会话重置
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* 工程统一为 ``litesim3d_ros2`` 独立多包仓库，构建/安装/证据移到仓库外；
  补齐中英文入口、可独立 clone 的说明及安装资源检查。
* 修正共享世界动态障碍停止状态对机器人遍历顺序的依赖、故障后仍推进、
  支撑采样未覆盖真实轮宽与声明支撑 link 的问题。
* 修正重置候选配置中相对 ``lidar.pattern_file`` 的重定位，
  并让覆盖率门禁拒绝缺失/重复源码和非法计数。
* 修正“节点被发现即就绪”的首帧竞态：等待参数服务/executor、
  当前 run_id、订阅类型/QoS 和 DDS 匹配；等待期间保持暂停。
* 增加延迟订阅、缺失订阅、无响应 executor、QoS 不兼容及百次重置回归。

初始仓库提交：`bdd758c <https://github.com/chiway-luo/litesim3d_ros2/commit/bdd758c4120b016f01a80b02276492ba6d93fca1>`_；
文档整理：`59037e5 <https://github.com/chiway-luo/litesim3d_ros2/commit/59037e51fb9b8e3c9a2be7fecd1652ce3bc63af6>`_。
早期审查修复已经包含在初始仓库快照中，不将其伪装为后续独立提交。
依据：`审查回归 <litesim3d_core/test/test_review_fixes.cpp>`_、
`近期修复与验证 <docs/testing/近期修复与验证.md>`_、
`文档发布检查 <docs/testing/文档发布检查.md>`_；
原审查证据为外部 ``evidence/review-fixes-20261018/result.json``。

2026-10-17 至 2026-10-18：M1–M3、四轮车与雷达
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* **M1**：独立 C++17 内核、URDF/网格加载、三维射线与几何碰撞、
  固定时间步、差速运动、PointCloud2、odom/TF、时钟、暂停/幂等单步与启动前位姿初始化。
* **M2**：连续坡面与多层支撑、逐点滚动扫描、噪声/丢点、raw IMU、
  独立里程计误差、协调会话重置、有界资源、评测与性能工具。
* **M3**：动态障碍、1–4 台同构机器人共享世界、理想 Ackermann、
  Ouster 字段适配与实际 PCL 消费；补齐 TwistStamped、QoS、TF 接管和子进程故障传播。
* 增加四轮车 description/sim 包、统一 launch、参数说明、车轮 mimic、
  雷达/IMU 安装帧、超时停车与 RViz 控制工具。
* 增加 spinning/spherical/rosette 模式、VLP-16/Mid-360/Mid-40 预设，
  将可选最远量程辅助点与原始无回波 NaN 槽分离。
* Mid-360 改用固定上游逐点方向表，保持 200,000 方向/秒及四秒有限周期，
  保留槽位、逐点时间、来源哈希和第三方许可。

依据：`M1 基线 <docs/testing/M1基线.md>`_、
`M2 基线 <docs/testing/M2基线.md>`_、
`M3 基线 <docs/testing/M3基线.md>`_、
`文档对照与补全 <docs/testing/文档对照与补全报告.md>`_、
`四轮车与雷达测试 <docs/testing/四轮车与雷达测试.md>`_、
`Mid-360 来源与范围 <litesim3d_core/resources/lidar_patterns/README.md>`_。

每包日志
--------

* `litesim3d_core <litesim3d_core/CHANGELOG.rst>`_
* `litesim3d_interfaces <litesim3d_interfaces/CHANGELOG.rst>`_
* `litesim3d_ros2 <litesim3d_ros2/CHANGELOG.rst>`_
* `litesim3d_assets <litesim3d_assets/CHANGELOG.rst>`_
* `litesim3d_gui <litesim3d_gui/CHANGELOG.rst>`_
* `wheel_car_4w_description <examples/wheel_car_4w/wheel_car_4w_description/CHANGELOG.rst>`_
* `wheel_car_4w_sim <examples/wheel_car_4w/wheel_car_4w_sim/CHANGELOG.rst>`_
* `wheel_car_4w_models <examples/wheel_car_4w/wheel_car_4w_models/CHANGELOG.rst>`_
* `drone_3d_sim <examples/drone_3d/drone_3d_sim/CHANGELOG.rst>`_
* `drone_3d_models <examples/drone_3d/drone_3d_models/CHANGELOG.rst>`_

验证与模型范围
--------------

历史报告只证明对应源码、平台和运行条件下的结果。本次回填没有重新执行仿真回归。
早期主要证据来自 macOS / RoboStack Humble；Linux 结果须对应具体 CI 提交。
默认跳过的图形项、独立桌面补测、短时性能窗口与长期稳定测试分别记录。

外部 SLAM/Nav2 完整闭环、八小时长稳和真实设备验收不计为已完成。
multicopter 是速度控制运动学，不包含飞控/推力动力学；
Mid-360 是有限仿真方向表，不是设备标定或厂家 UDP/CustomMsg。
当前入口和验证边界见 `测试文档 <docs/testing/README.md>`_。

许可证与联系
------------

项目许可证为 `Apache-2.0 <LICENSE>`_，维护者邮箱为 ``chiwayfpv@gmail.com``。
第三方材料保留各自声明：
`Livox 扫描表 MIT 许可 <litesim3d_core/resources/lidar_patterns/LICENSE.livox>`_、
`OGRE 许可 <scripts/macos_metal/LICENSE.OGRE>`_。
