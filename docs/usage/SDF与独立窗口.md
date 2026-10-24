# SDF 与独立三维窗口

四轮小车使用 `wheel_car_4w_models` 资源包，无人机使用 `drone_3d_models` 资源包。两个包分别保存各自的 SDF 模型、世界、网格和贴图。运行时通过 ament 获取对应包的 share 路径，launch 将该包的 `models/`、`worlds/` 加入本次进程树的 `GZ_SIM_RESOURCE_PATH`，不修改 shell 或系统设置。

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

每次只运行其中一个世界。**左键拖动旋转视角，Shift + 左键拖动平移视角，滚轮缩放**。按住左键拖动时，按下 Shift 切换为平移，松开 Shift 恢复旋转；缩放无需按住 Shift。窗口没有选取、拖动模型、控制机器人、暂停、重置等操作。底栏为只读信息。

关闭窗口后仿真继续。使用同一 ROS 环境和 `ROS_DOMAIN_ID` 可重新打开：

```bash
ros2 run litesim3d_gui viewer
```

查看器从 `/sim/scene_description` 获取完整场景，从 `/sim/scene_state` 更新世界真值位姿，从 `/sim/stats` 获取速率。不发布仿真命令、不调用控制服务。首次带窗口启动会等待渲染就绪；材质加载失败则启动失败。已有窗口关闭或崩溃不会终止内核。整个 launch 的 Ctrl+C 清理所有子进程和派生文件。

需要检查点云、TF、地图和轨迹时可显式加 `rviz:=true`，独立窗口不承担这些图层操作。

## 模型与参数的职责

```text
examples/
├── wheel_car_4w/wheel_car_4w_models/
│   ├── models/{wheel_car_4w,office_layout,textured_crate,...}/
│   └── worlds/{office,warehouse}.sdf
└── drone_3d/drone_3d_models/
    ├── models/{quadrotor,aerial_course,textured_crate}/
    └── worlds/drone_arena.sdf
```

每个模型目录包含 `model.config`、`model.sdf`，按需附带 `meshes/` 和 `materials/textures/`。两个包各自携带 `textured_crate` 的网格和贴图副本，资源解析不依赖另一示例目录。

包使用 ament 分别安装到 `share/wheel_car_4w_models` 和 `share/drone_3d_models`；`get_package_share_directory()` 依据包名查询安装索引，不依赖源码位置。资源可以搬迁后重新构建或重新 source。以下参数表以小车为例；无人机默认 `assets_root` 为 `drone_3d_models` 的 share 路径，默认世界为 `drone_arena.sdf`。

SDF 提供 world/include、几何、link/joint、初始位姿、材质和灯光。可选 `<gui><camera><pose>` 指定世界坐标系下的观察初始视角，仅影响查看器；不引入其他交互控件。`<physics>` 的 `max_step_size`、`real_time_factor` 提供积分步长及默认目标速率；这里只使用计时参数，仍是运动学内核，不执行 Ode/Bullet 刚体求解。

`wheel_car_4w_sim/config/runtime.yaml` 中 robot.urdf 留空，由案例 launch 动态传入组合 `simulation.urdf.xacro`，并保留碰撞、地形和 `entity_bindings`。组合模型提供完整机器人结构、材质及插件；同名 SDF model 只提供 root 出生位姿。没有 `robot.urdf` 时保留 SDF 定义机器人和旧 YAML 绑定的兼容入口。两种方式都不能在 runtime 重复场景几何、初始位姿、步长或目标速率。完整规则见 [模型组合与插件迁移](模型组合与插件迁移.md)。

完整参数、默认值及通用入口与车型入口的区别见 [启动与节点参数](../api/启动与节点参数.md)。

自定义世界示例：

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py \
  assets_root:=/绝对路径/wheel_car_4w_models \
  world:=factory.sdf runtime:=/绝对路径/runtime.yaml
```

支持本地相对路径、`model://`、`file://`、`package://`；相对路径以声明资源的文件为基准。同名资源匹配多个不同目录时报错。Fuel 资产需预先下载到本地。导入记录源 SDF、网格、MTL、纹理等资源的 SHA-256；窗口核验资源未在导入后改变。

## 导入资源前检查

当前入口使用固定环境和一个受控机器人。先核对机器人 ID、资源文件、碰撞几何和材质；资源缺失或不支持的特性会在导入时报告具体原因。平面用所需范围的薄 box，原生 SDF 传感器转换为 LiteSim 声明；不能直接执行 Gazebo 动力学插件。

完整格式、材质、网格与拒绝项见 [SDF 兼容范围](../api/坐标与模型约定.md#sdf-资源兼容范围)。文件解析成功不等于完成视觉或动力学验收。

## 控制负载与观察速率

需要在加载时限制碰撞复杂度可开启 collision_budget_enabled；关闭时仍执行碰撞检测。超预算会拒绝加载，实际修改应简化自己模型的 collision，保留必要通行空间。阈值见 [碰撞预算](../api/坐标与模型约定.md#碰撞预算与速率)。

real_time_factor 只设目标节拍；实际速度由场景、传感器和渲染开销决定。查看器分别显示实际仿真倍率和显示 FPS，暂停显示 0%。出现超时或倍率下降时，按 [性能与排障](记录与排障.md) 检查。

## 构建与平台

开发与运行环境为 Ubuntu 22.04 + ROS 2 Humble。SDF 与图形模块依赖 libsdformat12、Gazebo Rendering 6、Qt 5 Widgets 和 Qt 5 SVG；依赖安装命令见[安装依赖](安装与构建.md#安装依赖)，构建方式见[安装与构建](安装与构建.md)。

无需图形包时使用 `LWS_BUILD_GUI=0 bash scripts/build.sh`，运行时加 `gui:=false cameras:=false`。单独内核不启动 ROS 节点、不链接 Qt/Ogre；插件加载使用 pluginlib 及已安装插件。启用 RGB/深度相机需要图形包和可用的 Ogre2 图形环境，即使关闭所有窗口也需要渲染后端。图形功能的验证需对应具体平台和渲染环境，内核测试通过不代表图形功能已完成验证。

完整证据与尚未验证项见 [SDF 实现验证](../testing/SDF实现验证.md)。

## 车载 RGB 与深度相机

当前小车的 `sensors.xacro` 定义 `camera_link` 和 `camera_optical_frame`，安装在车体前方上缘。相机声明引用 `litesim3d_rgb_camera/config/parameters.yaml`，默认一台 640×480、15 Hz 的 RGB 相机，实例覆盖 `enabled: false`。启用普通相机只生成彩色数据和内参，不创建深度渲染目标或深度发布器。所有相机类型默认只发布 ROS 2 话题，不弹出摄像头窗口。启动普通相机：

```bash
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py cameras:=true
# 只运行传感器，不打开三维观察或图像预览窗口
ros2 launch wheel_car_4w_sim wheel_car_4w.launch.py cameras:=true gui:=false
```

组合模型内每个相机使用独立 sensor 声明，最多 8 项；名称和实际输出话题不能重复。`link` 必须是模型中已有的 **ROS 光学坐标系**，+z 向前、+x 向右、+y 向下。示例通过固定关节相对 camera_link 旋转 roll=-π/2、yaw=-π/2；发布帧与 RSP 的 TF 使用相同前缀。新增安装位置时在 sensors.xacro 增加 link/joint，再声明对应插件。

```xml
<sensor name="front_camera" class="litesim3d_rgb_camera/RgbCamera">
  <parameters file="package://litesim3d_rgb_camera/config/parameters.yaml"/>
  <overrides>
link: camera_optical_frame
topic: camera/front
enabled: true
rate_hz: 30.0
  </overrides>
</sensor>
```

以上声明放在 `<litesim3d version="1">` 块中；完整参数、RGB/深度噪声与多实例接口见 [插件接口](../api/插件接口.md)。

类型、分辨率、帧率、视场和噪声参数见 [相机配置参考](../api/配置文件.md#cameras)。

所有启用相机合计不超过 16,588,800 像素，这是独立于碰撞预算的图像缓冲限制，不是 GPU 帧率承诺。分辨率、频率和模型复杂度增加会降低实际 RTF。参数在启动时读取；当前不支持 `ros2 param set` 在线更改采样率、分辨率或增删相机。编辑模型实例配置后启动新会话；用 `runtime:=/绝对路径/my_runtime.yaml robot_urdf:=/绝对路径/simulation.urdf.xacro` 传入运行参数与组合模型，`cameras` 留空则保留逐台 enabled 设置。

需要彩色加深度时，把类与参数文件一起替换为 `litesim3d_rgbd_camera/RgbdCamera` 和对应包配置；只需要深度时使用 `litesim3d_depth_camera/DepthCamera`。`cameras:=true` 只开启已声明的标准相机，不切换类型。旧 YAML 的 cameras 列表和 type 字段继续用于没有 sensor 扩展的兼容模型；两处同时声明会报冲突。

彩色、深度和 CameraInfo 的话题、编码、光学坐标、内参、噪声和无效值统一见 [相机数据接口](../api/传感器数据.md#camera) 与 [ROS 2 话题表](../api/ROS2接口.md)。组合模型的标准相机插件支持可配置噪声，默认偏差与噪声为零。

可以直接订阅 ROS 2 话题，在算法或自己的可视化程序中使用图像。检查默认普通相机的数据：

```bash
ros2 topic echo /camera/front/color/camera_info --once --qos-reliability best_effort
ros2 topic hz /camera/front/color/image_raw
```

独立只读预览作为可选调试工具保留，默认不运行。只有显式传入 `camera_view:=true` 才随 launch 启动，并按相机类型显示彩色图、深度伪彩色或两者。也可以手动执行 `ros2 run litesim3d_gui camera_view` 订阅普通相机；RGB-D 用 `--ros-args -p type:=rgbd`。预览参数只选择订阅和显示方式，不会启用或改变传感器类型。预览显示分辨率、配置频率、收到的仿真频率／墙钟 FPS、时间戳及状态。普通 RGB 预览不等待深度；深度伪彩色仅用于观察，ROS 数据仍是 float32 米。关闭预览或三维窗口不会停掉相机。`topic hz` 和预览墙钟 FPS 随 RTF 下降；例如 10 Hz 相机在实际 RTF=0.5 时约为现实每秒 5 帧。

当前相机接入限于单机 SDF launch。原生 SDF `<sensor>` / Gazebo 插件仍不执行；相机通过组合模型声明或兼容 runtime 字段配置。旧多机入口和独立 `sim_headless` 没有渲染后端，遇到启用相机会明确拒绝；关闭相机后仍可走原纯 CPU 路径。
