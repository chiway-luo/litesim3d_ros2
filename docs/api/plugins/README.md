# 标准插件参数

按实例选择插件，参数在启动时读取；修改后重新启动，不支持运行中热切换。
数值默认值来自随包参数文件，并由插件契约测试与实现的 `defaults()` 校对。标为必填的关节、link 和 topic 没有缺省值，文件中的绑定只是示例。轮组几何和机械限位自动读取完整 URDF，不在 YAML 中重复配置。
覆盖顺序、类生命周期和异常语义见[插件接口](../插件接口.md)。

| 实现包 | 类别 | 参数参考 |
| --- | --- | --- |
| `litesim3d_ackermann_drive` | `ChassisPlugin` | [字段表](litesim3d_ackermann_drive.md) |
| `litesim3d_ackermann_odometry` | `OdometryPlugin` | [字段表](litesim3d_ackermann_odometry.md) |
| `litesim3d_depth_camera` | `SensorPlugin` | [字段表](litesim3d_depth_camera.md) |
| `litesim3d_diff_drive` | `ChassisPlugin` | [字段表](litesim3d_diff_drive.md) |
| `litesim3d_diff_odometry` | `OdometryPlugin` | [字段表](litesim3d_diff_odometry.md) |
| `litesim3d_imu` | `SensorPlugin` | [字段表](litesim3d_imu.md) |
| `litesim3d_lidar` | `SensorPlugin` | [字段表](litesim3d_lidar.md) |
| `litesim3d_mid360` | `SensorPlugin` | [字段表](litesim3d_mid360.md) |
| `litesim3d_omni3_drive` | `ChassisPlugin` | [字段表](litesim3d_omni3_drive.md) |
| `litesim3d_omni4_drive` | `ChassisPlugin` | [字段表](litesim3d_omni4_drive.md) |
| `litesim3d_omni_odometry` | `OdometryPlugin` | [字段表](litesim3d_omni_odometry.md) |
| `litesim3d_rgb_camera` | `SensorPlugin` | [字段表](litesim3d_rgb_camera.md) |
| `litesim3d_rgbd_camera` | `SensorPlugin` | [字段表](litesim3d_rgbd_camera.md) |

维护时修改实现包的参数文件与代码，再运行 `python3 scripts/docs/generate_plugin_reference.py --write`。静态检查拒绝过期表格；运行期默认值由安装后的插件契约测试验证。
