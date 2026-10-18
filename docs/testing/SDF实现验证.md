# SDF 与独立三维窗口：实现验证

日期：2026-10-18。环境：本机 macOS arm64、RoboStack ROS 2 Humble、libsdformat12 12.8、Gazebo Rendering 6.6.3、Qt5 5.15.15、Ogre-Next 2.2.6 + 项目内构建的 Metal 插件。

已实现并验证本轮核心范围：默认 SDF 导入、`litesim_models` 安装资源包、只允许观察相机操作的独立窗口、默认关闭的单参数碰撞预算、真实实时因子显示。使用方法和明确支持范围见 [SDF 与独立窗口](../usage/SDF与独立窗口.md)。原 [设计方案](../design/SDF导入与独立三维窗口改造方案.md) 保留实施前目标，不能将其中全部规划项当作本次已交付能力。

## 实际执行结果

| 检查 | 结果 | 验证范围 |
| --- | --- | --- |
| 完整 colcon 构建 | PASS，8 包 | 包安装、导入器、ROS 节点、Qt/Ogre2 窗口及原示例 |
| C++ 内核 CTest | 148/148 PASS | 原几何、运动学、传感器、碰撞、多机，以及新增预算、实际速率和 DAE 轴向测试 |
| SDF 导入 pytest | 17/17 PASS | 安装资源、帧关系、轴向、缺贴图/坏绑定/不支持行为、预算两态、计时来源、DAE 和 PBR 资源清单 |
| ROS 桥接完整 pytest | 77/77 PASS | clock、点云、IMU、odom、TF、暂停/单步、会话重置、多机及原有错误路径 |
| 四轮小车原回归 | 13 PASS，1 SKIP | 默认新 SDF 入口、轮组、传感器、TF、预设和故障退出；旧 RViz 图形项跳过 |
| 新 SDF 运行 pytest | 7/7 PASS | 五档目标倍率、暂停统计、查看器无控制发布者、关闭后内核继续、完整场景重连；重连修复后再次单项通过 |
| 原生 Qt 输入与渲染 | OBJ、DAE、PBR 三组 PASS | 实际 Metal 绘制、纹理像素检查，旋转/平移/缩放/调整窗口，模型位姿逐项保持不变 |
| 资源复制安装与搬迁 | PASS | 单独构建 litesim_models 的测试副本，隐藏该副本源码，从新 share 路径和任意 cwd 加载全部 12 项模型资源 |
| 独立内核构建与运行 | PASS | standalone 新构建，未链接 Qt/Ogre/ROS；执行 200 步，输出 1 秒仿真、10 帧、327,680 点 |

这些是本机执行结果。资源测试隐藏的是隔离构建副本的源码目录，没有移动用户正在使用的源码。全量安装前缀跨机器迁移、完全移除图形开发包后的 ROS 构建不是本次验证范围。

## 实际速率证据

校准使用可承载的小雷达场景，通过独立 Python 单调墙钟与已提交时间戳计算。25%、50%、100% 各采样约 10.2 秒，误差门槛为 5 个百分点；200% 和不限速为短时测量。

| 目标 | 独立测量窗口 | 独立平均实际速率 |
| --- | --- | --- |
| 25% | 10.200 s | 25.00% |
| 50% | 10.199 s | 49.95% |
| 100% | 10.199 s | 99.96% |
| 200% | 1.601 s | 199.93% |
| 不限速 | 1.799 s | 约 1559% |

短时不限速结果仅属于这个校准场景，不代表默认高点数雷达或外部大型场景的性能承诺。实际值由约 1 秒滑动窗口统计，与独立测量的差异按运行测试门槛检查；没有将 FPS 或“相对目标完成率”当作 RTF。

窗口重连截图另外使用 50% 目标，确认底栏可直接显示低于 100% 的倍率。暂停时统计继续刷新且时间不推进。代码和单测对 run_id 变化、回跳重建统计基线；GUI 超过 2 秒无统计消息显示连接中断。

## 材质与图形检查

OBJ/MTL 和 DAE 样例具有独立的蓝色顶面材质与侧面棋盘纹理，实际截图确认材质槽没有被全局纯色覆盖。样例 OBJ 刻意不带法向量，回归覆盖法向量补齐。DAE 的共享索引、节点变换与坐标系使用 Assimp 显式适配，避免旧 gz-common 解码差异导致外观和计算几何不一致。

PBR 样例同时加载 albedo、normal、roughness、metalness、emissive、light map，使用确定性的普通/常量测试贴图，实际渲染及相机操作通过。该检查证明绑定、解码与渲染路径工作，不等于复杂 normal/AO、多 UV、透明排序、颜色空间或和 Gazebo 参考图的完整定量校准。AO 和 specular 工作流在本版明确拒绝。

macOS 初始 OpenGL 试验出现黑图，已改用项目内 Metal 后端。重开窗口通过入口脚本在加载动态库前选择该后端；没有改写系统或 RoboStack 安装。完整 Metal/GL 插件和 OGRE 许可文本安装在 GUI 包内，避免资源查找错误。

可重复执行的图形检查：

```bash
source "$(python3 scripts/paths.py install)/setup.bash"
python3 scripts/checks/check_sdf_viewer.py --output /tmp/litesim-visual-check
```

输出目录须不存在或为空，需要真实桌面图形会话；默认查找 merged install 相邻的测试二进制，也可显式传入 `--binary`。脚本保存三个材质场景的 initial/orbit/pan/zoom/resized 图像和逐项结果。Linux 可在确认 OpenGL 可用后使用虚拟显示执行，但本次没有把配置 CI 当作已运行证据。

## 与原设计目标的差异和未验证项

- 新 SDF 入口目前为单个受控机器人和静态环境。标准 SDF sensor、plane、submesh 选择、嵌套关节机器人、运动场景绑定和混合 SDF 多机未实现；对应输入明确拒绝，旧多机 YAML 入口保留。
- 预算采用固定拓扑的加载前保守检查；20,000 对是每次碰撞查询上界，256 MiB 是结构估算。未实现每个物理步的硬内存/耗时限制，也未做大型厂房峰值性能基准。
- 状态和统计使用版本为 1 的 `std_msgs/String` JSON；静态描述 reliable/transient-local，状态与统计 best-effort/depth 1。图形在 Qt 主线程读取最新快照；内核统计仍在原 ROS executor，极重单次步骤可能使心跳延迟。
- 完成同场景 GUI 重开。运行中直接更换成另一份 SDF 的 GUI 热重建、远程资产分发、在线 Fuel 下载、通用旧配置迁移工具不在本次交付。
- 本次 Ubuntu 图形运行、外部真实工厂模型、跨平台参考图、长时间浸泡和真实设备验收均未执行。原有 Linux 历史结果不替代新功能验收。

## 证据位置和回归入口

原始日志与截图放在工作空间 artifacts 的 `evidence/sdf-viewer-20261018`，不随 Git 提交；目录内 `summary.json`、`source-sha256.json`、XML、日志和图像给出实际结果。

```bash
python3 scripts/paths.py evidence
source "$(python3 scripts/paths.py install)/setup.bash"
bash scripts/test_m3.sh
# 图形相关运行测试需要桌面，且不能与同 domain 的其他世界同时运行
LWS_GUI_TESTS=1 python3 -m pytest -q examples/wheel_car_4w/wheel_car_4w_sim/test/test_sdf_runtime.py
```

`test_m3.sh` 已纳入 SDF 导入和运行测试；默认跳过需要桌面的 GUI 项，不会启动长时浸泡测试。

## 增补：RGB、深度和 RGB-D 传感器

同日后续实现增加相机配置、精确采样事件、独立渲染服务、ROS Image/CameraInfo 发布和只读 RGB/深度预览。此节记录相机加入后的实际回归；上方 148/17 等数字保留为前一轮 SDF/窗口证据，不与本轮重复累计。

| 检查 | 本轮结果 | 范围 |
| --- | --- | --- |
| 完整构建 | 8 包 PASS | 新接口生成、渲染器和预览安装、Mac 相对库路径 |
| 核心 CTest | 152/152 PASS | 包括独立相机采样率、暂停、绑定、参数验证和话题冲突 |
| SDF 资源测试 | 25/25 PASS | 相机光学帧、默认禁用、无效参数拒绝、纯 CPU 入口拒绝启用相机 |
| 原有 ROS 桥接 | 77/77 PASS，339.36 s | 包括雷达、IMU、TF、单步去重、完整会话重置和多机 |
| 普通四轮车回归 | 19 PASS，9 SKIP | 相机默认关闭下的旧功能与 RTF；其中 7 个相机项另在下列图形批次实际执行，其余为旧 RViz 和 SDF 查看器重连项 |
| 相机 ROS/桌面集成 | 7/7 PASS | 三种模式、320×180/15 Hz、160×120/7 Hz、224×128/11 Hz、640×480/30 Hz；双相机独立频率、TF 前缀、禁用覆盖和生命周期 |
| 原生 RGB-D 几何校准 | PASS | 米制轴向深度、离轴投影/配准、相机位姿变化、超量程 NaN |
| 原查看器材质/交互回归 | OBJ/DAE/PBR 三组 PASS | 真实渲染，缩放/平移/旋转只改变观察视角，不改变模型位姿 |
| 独立 CPU 内核重新构建/运行 | PASS | 200 步，1 秒仿真、10 帧雷达、327,680 点、0 接触，无 Qt/Ogre/ROS 渲染依赖 |

本轮去重后的自动测试为 280 项通过，另有原生相机校准和三组查看器图形检查。未将此前结果重复计数，也未将被跳过的图形项当作通过。

原生校准在已知几何场景中，正前方与离轴像素的墙面深度均为约 **2.900000 m**；相机前移 0.5 m 后测得 **2.400001 m**。离轴蓝色方块的 80 个彩色像素与深度表面、CameraInfo 重建位置一致，覆盖了方块正面和侧面；不会将欧氏射线长度误当成轴向深度。RGB、depth、rgbd 三种模式均检查实际缓冲和缺省通道为空；超出远裁剪面为 NaN。

集成测试逐帧核对 Image/CameraInfo 的尺寸、编码、步长、光学 frame、内参和仿真时间戳，验证暂停不出新帧、重复单步请求不重复发图。RGB-D 两通道同时间戳、同内参。双相机分别输出 15 Hz 和 7 Hz，TF 前缀正确。运行目标 50% 时，预览截图实际显示仿真 10.0 Hz / 墙钟 5.0 FPS。关闭三维查看器和图像预览后图像继续更新；终止必需的渲染进程则 launch 非零退出。

首轮发现 Mac shell 入口会丢失继承的 DYLD 搜索变量，新增渲染器无法加载项目接口库。已在安装产物写入项目相对 RPATH，后续实际 launch/DDS 集成全部通过。原生离轴方块检查也修正为同时接受正面与侧面的真实深度，未放宽到错误的背景墙。

本轮证据在工作空间 `artifacts/evidence/camera-sensors-20261018`，包括 summary/source 哈希、XML、日志、原始 float32 深度、RGB 和预览截图。可重复执行：

```bash
source "$(python3 scripts/paths.py install)/setup.bash"
LWS_GUI_TESTS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  examples/wheel_car_4w/wheel_car_4w_sim/test/test_cameras.py
```

原生相机校准可执行 merged build 中的 `litesim3d_gui/camera_sensor_test 输出目录`；macOS 需先把 `OGRE2_RESOURCE_PATH` 指到安装包 `lib/litesim3d_gui/ogre2`，与已安装入口脚本一致。输出目录用于保存校准 RGB 和 float32 深度。

限制：参数在启动时读取，未实现在线改分辨率/帧率；仅单机 SDF 入口支持相机，标准 SDF sensor 仍不执行。当前是同位姿 RGB-D 的理想针孔/深度模型，不是厂商双目、ToF、畸变或噪声仿真。Ubuntu 图形、纯无显示服务器的 GPU 部署、4K/多相机压力测试和长时间稳定性尚未执行；不能从本机通过推断上述验收已完成。

## 默认相机调整为普通 RGB（2026-10-18）

按用户确认的使用方式，内核 `CameraConfig`、省略 `type` 的 runtime、四轮车示例及单独打开的图像预览均默认 `rgb`。示例仍为 `enabled: false`；`cameras:=true` 只启用已配置相机，不切换类型。只有显式选择 `rgbd` 才输出彩色加深度，选择 `depth` 只输出深度。

此轮在本机 macOS / ROS 2 Humble / Ogre2 Metal 环境执行针对性回归，共 **22 项通过**，未重复运行上节的全部 280 项：

| 检查 | 结果 | 覆盖 |
| --- | --- | --- |
| colcon 构建 | 8 包通过 | 默认值与预览重新编译、安装 |
| Camera 内核测试 | 5/5 PASS | 默认仅彩色、显式模式、校验和精确采样 |
| 相机 SDF 导入测试 | 9/9 PASS | 示例关闭且类型 rgb，省略 type 仍为 rgb，非法配置拒绝 |
| 相机 ROS/桌面集成 | 8/8 PASS | 默认 RGB、显式 RGB/深度/RGB-D、独立频率、暂停、预览和渲染器生命周期 |

默认 launch 的真实 DDS 数据为 640×480 `rgb8`，在 0.6 秒仿真中收到 9 帧，时间戳符合 15 Hz；没有深度 Image/CameraInfo 话题。独立预览未传 `type` 时只订阅彩色图，能够显示图像、尺寸和频率并保存截图，没有等待深度数据。显式单通道模式也检查另一通道没有发布器；RGB-D 继续验证两路图像、内参和一致时间戳。

证据保存在工作空间 `artifacts/evidence/camera-rgb-default-20261018`，包含构建/测试日志、XML、默认 RGB 与显式 RGB-D 预览、逐帧数据检查记录及本轮源文件哈希。上节证据保持原样；本轮同样不代表 Ubuntu、真实相机或长期压力验证。

## 应用图标与相机默认仅发布话题（2026-10-18）

使用已确定的 LiteSim SVG 矢量图标，复用现有 Qt 资源接入并重新构建安装。源码与安装目录中的 SVG SHA-256 均为 `138bc293b01da0a86bdb0824ebd3e92b5eb1e062c3c14f5ce32499036aa39019`，安装的查看器包含 `qInitResources_app_icons`。没有替换图标设计或修改系统 Dock 固定项。

`camera_view` 的 launch 默认值改为 `false`。启用 RGB、depth、rgbd 相机均不自动启动图像预览，仍按类型输出 Image/CameraInfo。三维查看器由 `gui` 独立控制。可选预览工具保留，只有显式开启时才运行。

本轮本机验证：8 包构建通过；相机 ROS/图形集成 **8/8 PASS**，涵盖默认 RGB、三种显式模式、分辨率/频率/时间戳、双相机和生命周期。默认及三种模式检查 ROS 节点和 launch 进程记录均没有摄像头预览，同时核对真实话题数据。额外运行三维窗口与 RGB 相机，成功收到图像且预览未启动，验证结束后测试进程已退出。

证据位于工作空间 `artifacts/evidence/icon-camera-topics-20261018`。桌面工具无法绑定当前命令行启动的原生 Qt 进程，macOS Dock 外观目视检查未完成；Linux Dock/Wayland 未实测。本轮未重复执行全部内核和桥接回归。
