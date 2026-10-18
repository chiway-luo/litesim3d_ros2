# SDF 与独立三维窗口

四轮小车默认使用 SDF 世界和 `litesim_models` 资源包。运行时通过 ament 获取包 share 路径，launch 将 `models/`、`worlds/` 加入本次进程树的 `GZ_SIM_RESOURCE_PATH`，不修改 shell 或系统设置。

无人机的速度模型、复杂场景和传感器配置见 [无人机示例](../../examples/drone_3d/README.md)。

## 启动与查看

激活 ROS 2 Humble、构建并 source 安装环境后：

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py
# 无窗口
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py gui:=false
# 目标 50%，并启用碰撞预算
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py real_time_factor:=0.5 collision_budget_enabled:=true
```

每次只运行其中一个世界。窗口仅接受左键旋转、中键或右键平移、滚轮缩放，没有选取、拖动模型、控制机器人、暂停、重置等操作。底栏为只读信息。

关闭窗口后仿真继续。使用同一 ROS 环境和 `ROS_DOMAIN_ID` 可重新打开：

```bash
ros2 run litesim3d_gui viewer
```

查看器从 `/sim/scene_description` 获取完整场景，从 `/sim/scene_state` 更新世界真值位姿，从 `/sim/stats` 获取速率。不发布仿真命令、不调用控制服务。首次带窗口启动会等待渲染就绪；材质加载失败则启动失败。已有窗口关闭或崩溃不会终止内核。整个 launch 的 Ctrl+C 清理所有子进程和派生文件。

需要检查点云、TF、地图和轨迹时可显式加 `rviz:=true`，独立窗口不承担这些图层操作。

## 模型与参数的职责

```text
examples/wheel_car_4w/litesim_models/
├── models/<模型名>/
│   ├── model.config
│   ├── model.sdf
│   ├── meshes/
│   └── materials/textures/
└── worlds/{office,drone_arena,warehouse}.sdf
```

包使用 ament 安装到 `share/litesim_models`；`get_package_share_directory("litesim_models")` 依据安装索引定位，不依赖源码位于哪个 `src` 的旁边。资源可以搬迁后重新构建或重新 source。

SDF 提供 world/include、几何、link/joint、初始位姿、材质和灯光。可选 `<gui><camera><pose>` 指定世界坐标系下的观察初始视角，仅影响查看器；不引入其他交互控件。`<physics>` 的 `max_step_size`、`real_time_factor` 提供积分步长及默认目标速率；这里只使用计时参数，仍是运动学内核，不执行 Ode/Bullet 刚体求解。

`wheel_car_4w_sim/config/runtime.yaml` 提供底盘绑定、传感器扫描与噪声、ROS 话题、地形支持、`entity_bindings` 和 mimic 扩展。不能再次定义几何、初始位姿、步长或目标速率。固定、转动、滑动关节来自 SDF；派生 URDF 只用于现有内核和 RSP，完整视觉材质直接交给渲染器。

| launch 参数 | 默认和含义 |
| --- | --- |
| `assets_root` | `litesim_models` 包 share 路径；覆盖时目录下仍需有 models/worlds |
| `world` | `office.sdf`；单文件名相对 assets_root/worlds，也可指定显式文件路径 |
| `runtime` | 安装的 config/runtime.yaml |
| `gui` | true；false 关闭三维观察窗口，相机渲染由 cameras 配置独立决定 |
| `cameras` | 空表示沿用 runtime 中 enabled；true/false 只覆盖所有已定义相机的开关，不改变各自类型；示例类型默认 rgb |
| `camera_view` | false；相机默认只发布 ROS 2 数据，不弹出预览；true 显式打开可选预览 |
| `collision_budget_enabled` | false；唯一碰撞预算开关 |
| `real_time_factor` | 空表示使用 SDF；0 表示不限速，正数表示目标倍率 |
| `paused` | 沿用 ros.yaml；true/false 显式覆盖 |
| `params`、`lidar_profile`、`manifest` | 保留原示例的 ROS 参数、雷达预设和运行清单接口 |
| `rviz` | false；需要算法图层时显式开启 |
| `config` | 空；显式传入旧 YAML 时选择旧 URDF/YAML 回归入口，此时不使用 SDF 查看器 |

自定义世界示例：

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py \
  assets_root:=/绝对路径/litesim_models \
  world:=factory.sdf runtime:=/绝对路径/runtime.yaml
```

支持本地相对路径、`model://`、`file://`、`package://`；相对路径以声明资源的文件为基准。同名资源匹配多个不同目录时报错。Fuel 资产需预先下载到本地。导入记录源 SDF、网格、MTL、纹理等资源的 SHA-256；窗口核验资源未在导入后改变。

## 当前兼容范围

三维查看器使用 [LiteSim SVG 图标](../../litesim3d_gui/resources/icons/litesim3d.svg)（蓝色圆角底、白色立方体与青色扫描环）。矢量资源随程序编译，Qt 启用高清图标渲染；安装包另包含由同一 SVG 导出的 PNG 和 macOS ICNS。

| 内容 | 当前行为 |
| --- | --- |
| SDF 1.9、include、嵌套静态模型 | 使用 libsdformat12，解析 relative_to、attached_to、expressed_in |
| 单个受控机器人 + 静态场景 | 支持；robot.id 必须匹配世界模型名，未绑定动态模型报错 |
| box、sphere、cylinder、STL、OBJ、DAE | 支持正缩放；DAE 必须明确 Z_UP，保留节点变换和单位 |
| 多 visual/collision、多材质槽 | 独立保留；visual 不自动补作 collision；支持模型必须提供 collision |
| OBJ/MTL、DAE 漫反射纹理 | 保留每个材质槽、UV、颜色和纹理；缺失法向量按三角面补齐 |
| SDF 基础材质 | 颜色、透明度、自发光、双面、阴影；世界 ambient/background 和三种灯光 |
| SDF PBR metal | albedo、normal、roughness、metalness、emissive、light map 映射到 Ogre2；逐项验证范围见测试报告 |
| 不支持的视觉特性 | Classic material 脚本、任意 shader、specular 工作流、AO map、嵌入式图片明确拒绝；网格内非 diffuse 贴图需移到 SDF PBR 定义 |
| 其他不支持输入 | plane、heightmap、polyline、submesh 选取、骨骼 actor、外部 plugin、嵌套关节机器人、原生 SDF sensor：报错，不悄悄忽略 |

平面请在资产中**显式**改用所需范围的薄 box；原生 SDF 传感器需转换为 runtime 中的 LiteSim 参数。这不是任意 Gazebo 世界或动力学插件的直接替代。旧多机世界仍走原 `world.launch.py`，本次未扩展为多种 SDF 机器人的混合世界。

支持格式不代表全部资产已做视觉验收；文件能解析、贴图能解码、图形属性校准是不同验证层次。材质不支持或缺文件会报出具体资源，不以灰模作为成功结果。当前使用 Qt 支持的纹理图片解码格式；建议 PNG/JPEG。

## 碰撞预算与速率

预算关闭时，碰撞检测、防穿透和碰撞响应照常执行，仅不实施可选复杂度上限。开启时执行以下内部规则，超限在加载时拒绝，不删面、不丢弃碰撞体、不替换成包围盒：

| 检查 | 内部阈值 |
| --- | --- |
| 单碰撞 mesh | 10,000 三角面 |
| 世界碰撞 mesh（按实例） | 200,000 三角面 |
| 动态碰撞 mesh（按实例） | 20,000 三角面 |
| 单机器人碰撞体 | 64 个 |
| 世界碰撞体 | 1,024 个 |
| 固定拓扑潜在碰撞对 | 每次查询保守上界 20,000 对 |
| 碰撞结构估算内存 | 256 MiB；不是进程 RSS 硬上限 |

本版采用固定拓扑的加载预检，没有运行时添加模型，也没有实现通用的每步耗时/内存强制限额。预算不限制视觉网格，且不承诺总 CPU/GPU 占用上限。视觉和射线网格不会自动创建 FCL；同一资源确实被 collision 引用时才创建碰撞结构。

实际 RTF 为 `Δ已提交仿真时间 / Δ单调墙钟时间`，约 1 秒滚动窗口，5 Hz 发布。目标 50% 就显示接近 50%，实际 63% 不会被改写为 100%；FPS 单独计算。重计算超过周期会自然减慢，固定积分步长不变，不靠跳过碰撞或传感器事件追赶。

暂停显示 0%，统计仍按墙钟发布；run_id 或时间回跳重建基线。统计超过 2 秒没有更新显示连接中断。统计发布仍使用现有 ROS executor，极重单次计算可能延迟心跳；窗口会标识超时，并未另建可抢占物理步骤的线程。

## 构建与平台

新增依赖为 libsdformat12、Gazebo Rendering 6、Qt5 Widgets。Ubuntu 安装命令见仓库 README。macOS RoboStack 的 Ogre-Next 若缺 Metal，构建脚本按固定版本和 SHA-256 下载并在本项目产物目录编译 Metal 插件，不修改全局 ROS 安装；此适配目前锁定 Ogre-Next 2.2.6。第一次构建需要网络，后续使用经哈希校验的缓存。

无需图形包时使用 `LWS_BUILD_GUI=0 bash scripts/build.sh`，运行时加 `gui:=false cameras:=false`。单独内核构建保持不依赖 ROS、Qt 或 Ogre。启用 RGB/深度相机需要图形包和可用的 Ogre2 图形环境，即使关闭所有窗口也需要渲染后端。macOS 图形已本机实测；本次 Ubuntu 图形和外部工厂模型尚未实测，不能从既有 Linux 内核测试推断新 GUI 已通过。

完整证据与尚未验证项见 [SDF 实现验证](../testing/SDF实现验证.md)。

## 车载 RGB 与深度相机

当前小车 SDF 含 `camera_link` 和 `camera_optical_frame`，安装在车体前方上缘。相机参数放在 runtime；默认定义一台 640×480、15 Hz 的普通 RGB 相机，`enabled: false`。启用普通相机只生成彩色数据和内参，不创建深度渲染目标或深度发布器。所有相机类型默认只发布 ROS 2 话题，不弹出摄像头窗口。启动普通相机：

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py cameras:=true
# 只运行传感器，不打开三维观察或图像预览窗口
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py cameras:=true gui:=false
```

配置文件中 `cameras` 是列表，最多 8 项；名称和实际输出话题不能重复。`link` 必须是模型中已有的 **ROS 光学坐标系**，+z 向前、+x 向右、+y 向下。示例通过固定关节相对 camera_link 旋转 roll=-π/2、yaw=-π/2；发布帧与 RSP 的 TF 使用相同前缀。新增不同安装位置的相机时在 SDF 中增加对应 link/joint，再填入列表。

```yaml
cameras:
  - name: front_camera
    enabled: true
    type: rgb
    link: camera_optical_frame
    topic: camera/front
    width: 640
    height: 480
    rate_hz: 30.0
    horizontal_fov_deg: 70.0
    near_clip_m: 0.1
    far_clip_m: 20.0
```

| 参数 | 含义 |
| --- | --- |
| type | 默认 rgb：仅彩色；显式 depth：仅深度；显式 rgbd：同步、已对齐的彩色和深度 |
| enabled | 是否产生采样事件、创建渲染目标和发布器；false 不输出该相机数据 |
| width / height | 像素分辨率；每边 16–4096，单相机不超过 8,294,400 像素 |
| rate_hz | 每仿真秒的帧数，(0,240]；支持非整数和多个相机独立频率 |
| horizontal_fov_deg | 水平视场角，(0,180)°；方形像素、主点居中，自动生成内参 |
| near_clip_m / far_clip_m | 近、远裁剪面，0 < near < far，单位米 |
| topic | 相对话题前缀；下表以 camera/front 为例 |

所有启用相机合计不超过 16,588,800 像素，这是独立于碰撞预算的图像缓冲限制，不是 GPU 帧率承诺。分辨率、频率和模型复杂度增加会降低实际 RTF。参数在启动时读取；当前不支持 `ros2 param set` 在线更改采样率、分辨率或增删相机。编辑 runtime 后重新启动，用 `runtime:=/绝对路径/my_runtime.yaml` 选择文件，`cameras` 留空则保留逐台 enabled 设置。

需要“彩色图像加深度数据”的深度相机时，将该相机改为 `type: rgbd`；只需要深度图时选 `type: depth`。省略 `type` 仍按普通 `rgb` 处理。`cameras:=true` 只开启已配置的相机，不切换类型；不同相机可以分别选择类型。

| 话题 | 启用类型 | 消息 / 数据 |
| --- | --- | --- |
| /camera/front/color/image_raw | rgb / rgbd | sensor_msgs/Image，rgb8，带实际材质和贴图的彩色图 |
| /camera/front/color/camera_info | rgb / rgbd | sensor_msgs/CameraInfo，与彩色图同时间戳、同 optical frame |
| /camera/front/depth/image_raw | depth / rgbd | sensor_msgs/Image，32FC1，光轴 z 方向的米制深度；无有效深度为 NaN |
| /camera/front/depth/camera_info | depth / rgbd | sensor_msgs/CameraInfo，与深度图同时间戳、同 optical frame |

RGB-D 使用理想的同位姿、同分辨率、同内参双通道；两张图逐像素注册。深度表示依据 [ROS REP-118](https://reps.openrobotics.org/rep-0118/)，不是到相机中心的欧氏射线长度。K/P 的 fx=fy=width/(2 tan(horizontal_fov/2))，cx=(width−1)/2、cy=(height−1)/2，D 为 5 个零，R 为单位阵。当前是无畸变针孔模型，不模拟真实双目基线、ToF 多径、曝光、滚动快门或深度噪声；没有输出额外的深度点云或压缩图像。RGB/深度都观察完整 visual 场景，不受简化 collision 外形影响。

图像与 CameraInfo 默认 best effort、KEEP_LAST 5，可通过现有 `ros.yaml` 的 `sensor_reliable` 统一改为 reliable。仿真帧时刻由内核事件调度产生，机器人及关节使用该时刻的真值位姿；同一时刻的相机批次同步提交给独立渲染进程，完成后发布。最多一个批次在途，渲染慢会降低 RTF，不通过观察窗口的刷新频率补帧、偷跳帧或重复旧帧。暂停不再采样，单步和请求去重仍有效。渲染器意外退出或请求超时会使本次仿真失败，不把失联当作有效图像。

可以直接订阅 ROS 2 话题，在算法或自己的可视化程序中使用图像。检查默认普通相机的数据：

```bash
ros2 topic echo /camera/front/color/camera_info --once --qos-reliability best_effort
ros2 topic hz /camera/front/color/image_raw
```

独立只读预览作为可选调试工具保留，默认不运行。只有显式传入 `camera_view:=true` 才随 launch 启动，并按相机类型显示彩色图、深度伪彩色或两者。也可以手动执行 `ros2 run litesim3d_gui camera_view` 订阅普通相机；RGB-D 用 `--ros-args -p type:=rgbd`。预览参数只选择订阅和显示方式，不会启用或改变传感器类型。预览显示分辨率、配置频率、收到的仿真频率／墙钟 FPS、时间戳及状态。普通 RGB 预览不等待深度；深度伪彩色仅用于观察，ROS 数据仍是 float32 米。关闭预览或三维窗口不会停掉相机。`topic hz` 和预览墙钟 FPS 随 RTF 下降；例如 10 Hz 相机在实际 RTF=0.5 时约为现实每秒 5 帧。

当前相机接入限于单机 SDF launch。原生 SDF `<sensor>` / Gazebo 插件仍不执行；相机通过上述 runtime 字段声明。旧多机入口和独立 `sim_headless` 没有渲染后端，遇到启用相机会明确拒绝；关闭相机后仍可走原纯 CPU 路径。
