# Mid-360 仿真插件

此目录集中维护 Mid-360 传感器插件、Livox ROS 输出适配和官方消息接口。没有雷达硬件驱动、SDK、设备发现或设备控制功能。

```text
litesim3d_mid360/
├── litesim3d_mid360/             # ROS 包：采样插件与 ROS 输出插件
│   ├── src/plugin.cpp           # litesim3d_mid360/Mid360
│   ├── src/livox_outputs.cpp    # litesim3d_mid360/LivoxOutput
│   ├── config/parameters.yaml
│   ├── plugins.xml
│   └── output_plugins.xml
└── interfaces/livox_ros_driver2/ # ROS 包：原样的 CustomMsg / CustomPoint
```

外层是插件集合目录，不放 `package.xml`。两个 ROS 包由 colcon 分别发现和构建；这样消息仍叫 `livox_ros_driver2/msg/CustomMsg`，下游算法不需要改消息类型，且不会因嵌套在一个 ROS 包内而漏掉接口包。

## 使用

四轮小车的 `lidar_profile:=mid360` 会选择 `litesim3d_mid360/Mid360`。自定义模型可直接声明：

```xml
<litesim3d version="1">
  <sensor name="mid360" class="litesim3d_mid360/Mid360">
    <parameters file="package://litesim3d_mid360/config/parameters.yaml"/>
    <overrides>
link: lidar_link
topic: points_raw
    </overrides>
  </sensor>
</litesim3d>
```

默认使用滚动扫描、10 Hz、20000 个采样槽位、0.1–40 m 量程和 -7°–52° 垂直裁剪；`columns * rate_hz` 必须为 200000。完整字段见 [参数表](../../docs/api/plugins/litesim3d_mid360.md)。采样复用核心射线和时间计算；官方扫描表仍使用 `package://litesim3d_core/lidar_patterns/mid360.csv`，保持已有资源 URI 可用。

ROS 输出通过现有节点参数选择：

| 参数 | 行为 |
| --- | --- |
| `lidar_interface:=native` | 标准 `PointCloud2`；不加载 Livox 输出库 |
| `lidar_interface:=auto` | 发现 Mid-360 才动态加载本插件的输出库 |
| `lidar_interface:=livox` | 要求配置 Mid-360 并加载输出库 |
| `xfer_format:=1` | `livox/lidar` 为 `CustomMsg`，另发 `livox/lidar/pointcloud` |
| `xfer_format:=0` | `livox/lidar` 为 Livox 字段布局的 `PointCloud2` |

话题、QoS、逐点时间、多机器人命名空间、IMU 和 `livox_output` 诊断字段保持原有约定。每台机器人最多一个 Livox 输出；混合场景中的普通雷达继续走通用输出。旧 YAML 的 Mid-360 采样继续通过通用雷达插件兼容，选择 Livox 输出时同样加载这里的输出插件。

## 构建与依赖

从仓库根目录，在 ROS 2 Humble 环境中执行：

```bash
colcon build --packages-up-to litesim3d_mid360
source install/setup.bash
```

`litesim3d_ros2` 仅导出通用 [LidarOutputPlugin](../../litesim3d_ros2/include/litesim3d_ros2/lidar_output_plugin.hpp) 接口，不直接依赖或链接 `livox_ros_driver2`。仅构建 `--packages-up-to litesim3d_ros2` 不会构建此插件及消息包；使用普通雷达或旧 YAML 的 native 输出不需要本插件。显式声明 `litesim3d_mid360/Mid360` 的模型始终需要安装本插件。四轮小车示例声明运行依赖以保留其 Mid-360 预设。

工作空间已有官方 `livox_ros_driver2` 时，按 [消息包说明](interfaces/livox_ros_driver2/README.md) 跳过本目录内的同名接口包，并先加载官方包环境。不得在同一安装前缀覆盖两种分发。

迁移旧工作空间时使用新的 build/install 目录，避免旧安装内残留的消息或桥接库影响依赖验证。
